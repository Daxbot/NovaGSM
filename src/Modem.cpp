/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file Modem.cpp
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 * @version 3.0
 */

#include <algorithm>
#include <cstdio>
#include <string.h>
#include <errno.h>
#include <stdarg.h>

#include "Modem.h"
#include "debug.h"

/**
 * @see GSM::State
 *
 * @dot
 * digraph {
 *     reset -> disabled            [ label = "disable()" ];
 *     reset -> init                [ label = "ATI" ];
 *     init -> reset                [ label = "reset()" ];
 *     init -> locked               [ label = "SIM locked" ];
 *     init -> offline              [ label = "SIM ready" ];
 *     locked -> reset              [ label = "reset()" ];
 *     locked -> offline            [ label = "unlock()" ];
 *     offline -> reset             [ label = "reset()" ];
 *     offline -> online            [ label = "" ];
 *     online -> reset              [ label = "reset()" ];
 *     online -> offline            [ label = "" ];
 *     online -> authenticating     [ label = "authenticate()" ];
 *     authenticating -> online     [ label = "error/\ntimeout" ];
 *     authenticating -> ready      [ label = "OK" ];
 *     ready -> reset               [ label = "reset()" ];
 *     ready -> offline             [ label = "lost\nsignal" ];
 *     ready -> handshaking         [ label = "connect()" ];
 *     handshaking -> ready         [ label = "error/\ntimeout" ];
 *     handshaking -> open          [ label = "OK" ];
 *     open -> reset                [ label = "reset()" ];
 *     open -> offline              [ label = "lost\nsignal" ];
 *     open -> ready                [ label = "disconnect()" ];
 * }
 * @enddot
 */

/** Maximum number of errors before reset. */
static constexpr int ERRORS_MAX = 10;

/** Buffer string search.
 *
 * @param [in] data buffer to search.
 * @param [in] target string to find.
 * @param [in] size maximum bytes to search.
 */
static void *memstr(void *data, const char *target, size_t size)
{
    char *p_data = static_cast<char*>(data);
    const size_t len = strlen(target);

    if(len <= size) {
        for(; data != (p_data-size); p_data++) {
            if(*p_data == target[0] && memcmp(p_data, target, len) == 0)
                return static_cast<void*>(p_data);
        }
    }

    return nullptr;
}

namespace GSM
{
    void Modem::process()
    {
        /**
         * The process method handles communication with the modem and
         * transitions between device states. If there is not a command
         * already awaiting a response then the next command in the buffer is
         * sent. Responses are handled based on the GSM::State.
         *
         * Any pending packets that exceed their timeout value are discarded
         * and the error counter is incremented. If the error counter exceeds
         * ERRORS_MAX the modem is assumed to be MIA and the driver returns to
         * State::reset.  The error counter is reset whenever a valid response
         * is received.
         * 
         * If the signal is lost the connection is assumed to be broken and the
         * driver returns to State::offline. The user must call authenticate()
         * and connect() again to re-establish communication.
         */

        uint32_t elapsed_ms = m_ctx->elapsed_ms();

        if(m_pending) {
            // Command pending - wait for response
            if((int32_t)(elapsed_ms - m_command_timer) > 0) {
                switch(m_device_state) {
                    case State::init:
                        GSM_WARN("Initialization timeout.\n");
                        m_pending = nullptr;
                        m_cmd_buffer.clear();
                        set_state(State::reset);
                        return;
                    case State::authenticating:
                        GSM_WARN("Authentication timeout.\n");
                        m_pending = nullptr;
                        m_cmd_buffer.clear();
                        set_state(State::online);
                        return;
                    case State::handshaking:
                        GSM_WARN("TCP connection timeout.\n");
                        m_pending = nullptr;
                        m_cmd_buffer.clear();
                        set_state(State::ready);
                        return;
                    case State::open:
                        GSM_WARN("Socket timeout\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_pending = nullptr;
                            m_cmd_buffer.pop();

                            if(f_sock_cb) {
                                if(m_sock_state == SocketState::cts)
                                    f_sock_cb(Event::tx_error, p_sock_cb_user);
                                else if(m_sock_state == SocketState::rtr)
                                    f_sock_cb(Event::rx_error, p_sock_cb_user);
                            }
                        }
                        return;
                    default:
                        GSM_VERBOSE("Command timeout\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_pending = nullptr;
                            m_cmd_buffer.pop();
                        }
                        return;
                 }
            }

            #if defined(GSM_ASYNC)
                // Asynchronous API
                m_response_size = m_ctx->rx_count_async();
            #else
                // Common API
                uint8_t *p_data = &m_response[m_response_size];
                int count = m_ctx->read(
                    p_data, (GSM_BUFFER_SIZE - m_response_size));

                if(count > 0) {
                    m_response_size += count;
                }
            #endif

            if(m_response_size > 4) {
                switch(m_device_state) {
                    case State::disabled:
                        /** State::disabled - modem is set to minimum
                         * functionality mode. Use reset to restore.
                         */
                        break;
                    case State::reset:
                        /** State::reset - Wait for the modem to respond to an
                         * ATI query and transition to State::init.
                         */
                        _process_reset();
                        break;
                    case State::init:
                        /** State::init - query the SIM card and transition to
                         * State::offline or State::locked.
                         */
                        _process_init();
                        break;
                    case State::locked:
                        /** State::locked - SIM is locked, call unlock() with
                         * the password to transition to State::offline.
                         */
                        _process_locked();
                        break;
                    case State::offline:
                        /** State::offline - wait for a signal and transition
                         * to State::online.
                         */
                        _process_offline();
                        break;
                    case State::online:
                        /** State::online - registered on network, wait for
                         * authenticate() to transition to
                         * State::authenticating.
                         */
                        _process_online();
                        break;
                    case State::authenticating:
                        /** State::authenticating - handle authenticate() and
                         * transition to State::ready on success.
                         */
                        _process_authenticating();
                        break;
                    case State::ready:
                        /** State::ready - authenticated with GPRS, wait for
                         * connect() to transition to State::handshaking.
                         */
                        _process_online();
                        break;
                    case State::handshaking:
                        /** State::handshaking - handle connect() and transition
                         * to State::open on success.
                         */
                        _process_handshaking();
                        break;
                    case State::open:
                        /** State::open - socket connection is established,
                         * handle disconnect() and data transfer.
                         */
                        _process_open();
                        break;
                }
            }
        }
        else if(!m_cmd_buffer.empty()) {
            // Send next command.
            m_pending = m_cmd_buffer.back();

            #if defined(GSM_ASYNC)
                m_ctx->rx_abort_async();
                m_ctx->receive_async(m_response, GSM_BUFFER_SIZE);
                m_ctx->send_async(m_pending->data, m_pending->size);
            #else
                m_ctx->write(m_pending->data, m_pending->size);
            #endif

            m_command_timer = elapsed_ms + m_pending->timeout_ms;
            m_response_size = 0;
        }
        else if((int32_t)(elapsed_ms - m_update_timer) > 0) {
            // Nothing queued - poll the modem
            m_update_timer = elapsed_ms + 20;
            poll_modem();
        }
    }

    int Modem::unlock(const void *pin, size_t pin_size)
    {
        if(pin == nullptr)
            return -EINVAL;

        // AT+CPIN=[pin] - enter pin
        return queue_command(DEFAULT_TIMEOUT_MS,
            "AT+CPIN=\"%.*s\"\r\n", pin_size, static_cast<const char*>(pin));
    }

    int Modem::authenticate(
        const void *apn,
        size_t apn_size,
        const void *user,
        size_t user_size,
        const void *pwd,
        size_t pwd_size)
    {
        switch(m_device_state) {
            // Invalid state, return error
            case State::disabled:
            case State::reset:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::authenticating:
                return -EALREADY;
            case State::handshaking:
            case State::ready:
            case State::open:
                return -EISCONN;

            // Continue
            case State::online:
                break;
        }

        m_apn[0] = '\0';
        m_user[0] = '\0';
        m_pwd[0] = '\0';

        if(apn && apn_size > 0)
            sprintf(m_apn, "%.*s", apn_size, static_cast<const char *>(apn));

        if(user && user_size > 0)
            sprintf(m_user, "%.*s", user_size, static_cast<const char *>(user));

        if(pwd && pwd_size > 0)
            sprintf(m_pwd, "%.*s", pwd_size, static_cast<const char *>(pwd));

        m_auth_state = 0;

        // AT+CIPSHUT - deactivate GPRS PDP context
        int ret = queue_command(65000, "AT+CIPSHUT\r\n");
        if(ret < 0)
            return ret;

        GSM_INFO("Authenticating...\n");
        set_state(State::authenticating);
        return 0;
    }

    int Modem::authenticate(const char *apn, const char *user, const char *pwd)
    {
        return authenticate(
            apn, (apn) ? strlen(apn) : 0,
            user, (user) ? strlen(user) : 0,
            pwd, (pwd) ? strlen(pwd) : 0);
    }

    int Modem::connect(const void *host, size_t host_size, int port)
    {
        if(host == nullptr)
            return -EINVAL;

        switch(m_device_state) {
            // Invalid state, return error
            case State::disabled:
            case State::reset:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::online:
            case State::authenticating:
                return -ENOTCONN;
            case State::handshaking:
                return -EALREADY;
            case State::open:
                return -EADDRINUSE;

            // Continue
            case State::ready:
                break;
        }

        int ret = queue_command(75000,
            "AT+CIPSTART=\"TCP\",\"%.*s\",%d\r\n",
            host_size, static_cast<const char*>(host), port);

        if(ret < 0)
            return ret;

        GSM_INFO("Handshaking...\n");
        set_state(State::handshaking);
        return 0;
    }

    int Modem::connect(const char *host, int port)
    {
        return connect(host, (host) ? strlen(host) : 0, port);
    }

    int Modem::disconnect()
    {
        switch(m_device_state) {
            // Invalid state, return error
            case State::disabled:
            case State::reset:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::online:
            case State::authenticating:
            case State::handshaking:
            case State::ready:
                return -ENOTSOCK;

            // Continue
            case State::open:
                break;
        }

        // AT+CIPCLOSE - close connection
        return queue_command(DEFAULT_TIMEOUT_MS, "AT+CIPCLOSE\r\n");
    }

    void Modem::reset()
    {
        GSM_INFO("Modem reset\n");

        #if defined(GSM_ASYNC)
        if(m_pending)
            m_ctx->rx_abort_async();
        #endif

        stop_send();
        stop_receive();
        m_cmd_buffer.clear();
        m_pending = nullptr;
        m_command_timer = 0;
        m_update_timer = 0;
        m_sock_state = SocketState::idle;
        m_rssi = 99;
        m_errors = 0;
        m_rx_buffer = nullptr;
        m_tx_buffer = nullptr;
        m_rx_available = 0;
        m_tx_available = 0;
        m_modem_id[0] = '\0';
        m_auth_state = 0;

        set_state(State::reset);
    }

    int Modem::disable(uint32_t timeout_ms)
    {
        reset();

        // AT+CFUN=0,1 - reset to minimum functionality mode
        int ret = queue_command(10000, "AT+CFUN=0\r\n");
        if(ret != 0)
            return ret;

        const uint32_t start = m_ctx->elapsed_ms();

        while(status() != State::disabled) {
            if(timeout_ms > 0) {
                if((uint32_t)(m_ctx->elapsed_ms() - start) > timeout_ms) {
                    GSM_WARN("Failed to disable modem - timeout.");
                    return -ETIME;
                }
            }

            if(!m_pending && m_cmd_buffer.empty()) {
                GSM_WARN("Failed to disable modem - no response.");
                return -ENODEV;
            }

            process();
        }

        return 0;
    }

    void Modem::receive(void *data, size_t size)
    {
        m_rx_buffer = static_cast<uint8_t*>(data);
        m_rx_size = size;
        m_rx_count = 0;
    }

    void Modem::stop_receive()
    {
        if(f_sock_cb && rx_busy())
            f_sock_cb(Event::rx_stopped, p_sock_cb_user);

        receive(nullptr, 0);
    }

    void Modem::send(const void *data, size_t size)
    {
        m_tx_buffer = static_cast<const uint8_t*>(data);
        m_tx_size = size;
        m_tx_count = 0;
    }

    void Modem::stop_send()
    {
        if(f_sock_cb && tx_busy())
            f_sock_cb(Event::tx_stopped, p_sock_cb_user);

        send(nullptr, 0);
    }

    int Modem::queue_command(uint32_t timeout, const char *command, ...)
    {
        va_list argp;

        if(m_cmd_buffer.full())
            return -ENOBUFS;

        command_t *cmd = m_cmd_buffer.front();
        char *p_char = reinterpret_cast<char*>(cmd->data);

        va_start(argp, command);
        cmd->size = vsnprintf(p_char, GSM_BUFFER_SIZE, command, argp);
        va_end(argp);

        if(cmd->size >= GSM_BUFFER_SIZE)
            return -EMSGSIZE;

        cmd->timeout_ms = timeout;
        m_cmd_buffer.push();
        return 0;
    }

    void Modem::poll_modem()
    {
        switch(m_device_state) {
            case State::reset:
                // ATI - device identification
                queue_command(1000, "ATI\r\n");
                break;
            case State::init:
            case State::locked:
                // AT+CPIN? - SIM status
                queue_command(5000, "AT+CPIN?\r\n");
                break;
            case State::offline:
            case State::online:
                // AT+CREG? - network registration status
                // AT+CSQ - signal quality report
                queue_command(1000, "AT+CREG?;+CSQ\r\n");
                break;
            case State::ready:
                // AT+CGATT? - state of GPRS attachment
                // AT+CSQ - signal quality report
                queue_command(75000, "AT+CGATT?;+CSQ\r\n");
            case State::open:
                // AT+CIPSTATUS - TCP connection status
                queue_command(1000, "AT+CIPSTATUS\r\n");
                m_sock_state = SocketState::idle;
                break;
            default:
                break;
        }
    }

    void Modem::_process_reset()
    {
        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "ATI", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);

            uint8_t *id_start = static_cast<uint8_t*>(
                memchr(p_data, '\n', size)) + 1;

            uint8_t *id_end = static_cast<uint8_t*>(
                memchr(id_start, '\r', size - (id_start-p_data)));

            // ATI\r\n%s\r\nOK\r\n
            // │       │ │
            // │       │ └ id_end
            // │       └ id_start
            // └ p_data

            const int id_size = (id_end - id_start);
            memcpy(m_modem_id, id_start, std::min(id_size, ID_SIZE));

            GSM_INFO("Modem is: %s\n", m_modem_id);

            // AT&F0 - reset to factory defaults
            // AT+CLTS=1 - enable local timestamps
            // AT+CFUN=1,1 - reset phone module
            // AT+CPIN? - SIM status
            queue_command(10000, "AT&F0;+CLTS=1;+CFUN=1,1;+CPIN?\r\n");

            GSM_INFO("Initializing...\n");
            set_state(State::init);
        }

        // Handle disable()
        if(memstr(m_response, "+CFUN=0", m_response_size) != nullptr)
            set_state(State::disabled);
    }

    void Modem::_process_init()
    {
        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CPIN:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            if(memstr(p_data, "\r\n", size) == nullptr)
                return;

            m_pending = nullptr;
            m_cmd_buffer.pop();
            m_errors = 0;

            if(memstr(p_data, "SIM PIN", size)
            || memstr(p_data, "SIM PUK", size)) {
                GSM_INFO("SIM locked\n");
                set_state(State::locked);
                return;
            }
            else if(memstr(p_data, "READY", size)) {
                GSM_INFO("SIM ready\n");
                set_state(State::offline);
                return;
            }
        }
    }

    void Modem::_process_locked()
    {
        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CPIN:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            if(memstr(p_data, "\r\n", size) == nullptr)
                return;

            m_pending = nullptr;
            m_cmd_buffer.pop();
            m_errors = 0;

            if(memstr(p_data, "READY", size)) {
                GSM_INFO("Modem ready\n");
                set_state(State::offline);
            }
        }
    }

    void Modem::_process_offline()
    {
        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CREG:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ',', size)) + 1;

            // +CREG: %d,%d,%s,%s\r\n
            // │         │
            // │         └ data
            // └ p_data

            const uint8_t status = strtol(data, nullptr, 10);
            if(status == 1 || status == 5) {
                GSM_INFO("Modem online\n");
                set_state(State::online);
            }
        }

        p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CSQ:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CSQ: %d,%d\r\n
            // │    │
            // │    └ data
            // └ p_data

            m_rssi = strtol(data, nullptr, 10);
        }
    }

    void Modem::_process_online()
    {
        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CREG:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ',', size)) + 1;

            // +CREG: %d,%d,%s,%s\r\n
            // │         │
            // │         └ data
            // └ p_data

            const uint8_t status = strtol(data, nullptr, 10);
            if(status != 1 && status != 5) {
                GSM_INFO("Modem offline\n");
                set_state(State::offline);
            }
        }

        p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CSQ:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CSQ: %d,%d\r\n
            // │    │
            // │    └ data
            // └ p_data

            m_rssi = strtol(data, nullptr, 10);
        }
    }

    void Modem::_process_authenticating()
    {
        int ret = 0;

        if(memstr(m_response, "ERROR", m_response_size)) {
            GSM_WARN("Authentication error.\n");
            m_pending = nullptr;
            m_cmd_buffer.pop();
            set_state(State::offline);
            return;
        }

        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        switch(m_auth_state++) {
            case 0:
                // AT+CGATT=0 - detach from GPRS service
                ret = queue_command(10000, "AT+CGATT=0\r\n");
                break;
            case 1:
                // AT+SAPBR=3,1,[tag],[value] - configure bearer
                ret = queue_command(DEFAULT_TIMEOUT_MS,
                    "AT+SAPBR=3,1,\"Contype\",\"GPRS\";"
                    "+SAPBR=3,1,\"APN\",\"%s\";"
                    "+SAPBR=3,1,\"USER\",\"%s\";"
                    "+SAPBR=3,1,\"PWD\",\"%s\";\r\n",
                    m_apn, m_user, m_pwd);
                break;
            case 2:
                // AT+CGDCONT=1,[type],[apn] - define GPRS PDP context
                ret = queue_command(DEFAULT_TIMEOUT_MS,
                    "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", m_apn);
                break;
            case 3:
                // AT+CGACT=1,1 - activate GPRS PDP context
                ret = queue_command(150000, "AT+CGACT=1,1\r\n");
                break;
            case 4:
                // AT+SAPBR=1,1 - open bearer
                ret = queue_command(85000, "AT+SAPBR=1,1\r\n");
                break;
            case 5:
                // AT+CGATT=1 - attach to GPRS service
                ret = queue_command(10000, "AT+CGATT=1\r\n");
                break;
            case 6:
                // AT+CIPMUX=0 - single IP mode
                // AT+CIPQSEND=1 - quick send mode
                // AT+CIPRXGET=1 - manual data mode
                ret = queue_command(DEFAULT_TIMEOUT_MS,
                    "AT+CIPMUX=0;+CIPQSEND=1;+CIPRXGET=1\r\n");
                break;
            case 7:
                // AT+CSTT=[apn],[user],[password] - set apn/user/password for GPRS PDP context
                ret = queue_command(DEFAULT_TIMEOUT_MS,
                    "AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", m_apn, m_user, m_pwd);
                break;
            case 8:
                // AT+CIICR - bring up wireless connection
                ret = queue_command(60000, "AT+CIICR\r\n");
                break;
            case 9:
                // AT+CIFSR - get local IP address
                // AT+CDNSCFG=[primary],[secondary] - configure DNS
                ret = queue_command(10000,
                    "AT+CIFSR;+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\";\r\n");
                break;
            case 10:
                GSM_INFO("Authentication success\n");
                set_state(State::ready);
                break;
            default:
                GSM_ERROR("Bad authentication state index\n");
                reset();
                break;
        }

        if(ret < 0) {
            GSM_ERROR("Failed to queue authentication command (%d).\n", ret);
            set_state(State::offline);
        }
    }

    void Modem::_process_ready()
    {
        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CGATT:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CGATT: %d\r\nOK\r\n
            // │      │
            // │      └ data
            // └ p_data

            const uint8_t status = strtol(data, nullptr, 10);
            if(status == 0) {
                GSM_WARN("Disconnected from GPRS.\n");
                set_state(State::offline);
            }
        }

        p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CSQ:", m_response_size));

        if(p_data) {
            const uint8_t size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CSQ: %d,%d\r\nOK\r\n
            // │    │
            // │    └ data
            // └ p_data

            m_rssi = strtol(data, nullptr, 10);
        }
    }

    void Modem::_process_handshaking()
    {
        if(memstr(m_response, "CONNECT OK", m_response_size)
        || memstr(m_response, "ALREADY CONNECT", m_response_size)) {
            GSM_INFO("TCP connection established\n");
            m_cmd_buffer.pop();
            m_pending = nullptr;
            m_errors = 0;
            set_state(State::open);
            return;
        }
        else if(memstr(m_response, "CONNECT FAIL", m_response_size)) {
            GSM_WARN("TCP connection failed.\n");
            m_cmd_buffer.pop();
            m_pending = nullptr;
            m_errors = 0;
            set_state(State::online);
            return;
        }
        else if(memstr(m_response, "+CGATT\r\n", m_response_size)) {
            m_cmd_buffer.pop();
            m_pending = nullptr;
            m_errors = 0;
        }
    }

    void Modem::_process_open()
    {
        if(memstr(m_response, "CLOSE OK", m_response_size)) {
            GSM_INFO("TCP socket disconnected\n");
            m_pending = nullptr;
            m_cmd_buffer.clear();
            m_errors = 0;

            stop_send();
            stop_receive();

            m_sock_state = SocketState::idle;
            set_state(State::online);
            return;
        }

        switch(m_sock_state) {
            case SocketState::idle:
                /** SocketState::idle - no active data transfer, handle send()
                 * and receive(). Also poll connection status.
                 */
                return _socket_idle();
            case SocketState::poll:
                /** SocketState::poll - waiting for modem to report status. */
                return _socket_poll();
            case SocketState::rtr:
                /** SocketState::rtr - ready to receive, data is being read. */
                return _socket_rtr();
            case SocketState::rts:
                /** SocketState::rts - ready to send, inform modem that data is
                 * about to be sent.
                 */
                return _socket_rts();
            case SocketState::cts:
                /** SocketState::cts - clear to send, data is being written. */
                return _socket_cts();
        }
    }

    void Modem::_socket_idle()
    {
        if(memstr(m_response, "STATE: CONNECT OK", m_response_size)) {
            if(m_rx_buffer && m_rx_count < m_rx_size && m_rx_available) {

                const int requested = (m_rx_size - m_rx_count);
                const int available = std::min(m_rx_available, SOCKET_MAX);
                const int count = std::min(requested, available);

                // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
                m_pending->size = sprintf(
                    reinterpret_cast<char*>(m_pending->data),
                    "AT+CIPRXGET=2,%d\r\n", count);

                // Request data
                #if defined(GSM_ASYNC)
                    m_ctx->rx_abort_async();
                    m_ctx->receive_async(m_response, GSM_BUFFER_SIZE);
                    m_ctx->send_async(m_pending->data, m_pending->size);
                #else
                    m_ctx->write(m_pending->data, m_pending->size);
                #endif

                GSM_VERBOSE("RTR %d bytes (%d)\n", count, m_rx_size-m_rx_count);
                m_command_timer = m_ctx->elapsed_ms() + DEFAULT_TIMEOUT_MS;
                m_response_size = 0;

                m_sock_state = SocketState::rtr;
                return;
            }

            if(m_tx_buffer && m_tx_count < m_tx_size && m_tx_available) {

                const int requested = (m_tx_size - m_tx_count);
                const int available = std::min(m_tx_available, SOCKET_MAX);
                const int count = std::min(requested, available);

                // AT+CIPSEND=[size] - indicate that data is about to be sent
                m_pending->size = sprintf(
                   reinterpret_cast<char*>(m_pending->data),
                   "AT+CIPSEND=%d\r\n", count
               );

                // Request data
                #if defined(GSM_ASYNC)
                    m_ctx->rx_abort_async();
                    m_ctx->receive_async(m_response, GSM_BUFFER_SIZE);
                    m_ctx->send_async(m_pending->data, m_pending->size);
                #else
                    m_ctx->write(m_pending->data, m_pending->size);
                #endif

                GSM_VERBOSE("RTS %d bytes (%d)\n", count, m_tx_size-m_tx_count);
                m_command_timer = m_ctx->elapsed_ms() + DEFAULT_TIMEOUT_MS;
                m_response_size = 0;

                m_sock_state = SocketState::rts;
                return;
            }

            m_pending = nullptr;
            m_cmd_buffer.pop();
            m_errors = 0;

            // AT+CIPRXGET=4 - query socket unread bytes
            // AT+CIPSEND? - query available size of tx buffer
            // AT+CSQ - signal quality report
            queue_command(1000, "AT+CIPRXGET=4;+CIPSEND?;+CSQ\r\n");

            m_sock_state = SocketState::poll;
            return;
        }
        else if(memstr(m_response, "STATE: TCP CLOSED", m_response_size)) {
            GSM_WARN("Server closed TCP socket.\n");
            m_pending = nullptr;
            m_cmd_buffer.clear();
            m_errors = 0;

            stop_send();
            stop_receive();

            m_sock_state = SocketState::idle;
            set_state(State::ready);
        }
    }

    void Modem::_socket_poll()
    {
        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CIPRXGET:", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ',', size)) + 1;

            // +CIPRXGET: 4,%d\r\nOK\r\n
            // │             │
            // │             └ data
            // └ p_data

            const int count = strtol(data, nullptr, 10);

            if(count > m_rx_available && f_sock_cb)
                f_sock_cb(Event::new_data, p_sock_cb_user);

            m_rx_available = count;
        }

        p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CIPSEND:", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CIPSEND: %d\r\nOK\r\n
            // │        │
            // │        └ data
            // └ p_data

            m_tx_available = strtol(data, nullptr, 10);
        }

        p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CSQ:", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);
            char const *data = static_cast<char*>(memchr(p_data, ':', size)) + 1;

            // +CSQ: %d,%d\r\nOK\r\n
            // │    │
            // │    └ data
            // └ p_data

            m_rssi = strtol(data, nullptr, 10);
        }
    }

    void Modem::_socket_rtr()
    {
        if(memstr(m_response, "ERROR", m_response_size)) {
            GSM_ERROR("Receive error\n");
            m_pending = nullptr;
            m_cmd_buffer.pop();
            if(f_sock_cb)
                f_sock_cb(Event::rx_error, p_sock_cb_user);

            m_sock_state = SocketState::idle;
            return;
        }

        if(memstr(m_response, "OK\r\n", m_response_size) == nullptr)
            return;

        m_pending = nullptr;
        m_cmd_buffer.pop();
        m_errors = 0;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "+CIPRXGET: 2,", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);

            uint8_t const *data_size = static_cast<const uint8_t*>(
                memchr(p_data, ',', size)) + 1;

            uint8_t const *data = static_cast<const uint8_t*>(
                memchr(data_size, '\n', size - (data_size - p_data))) + 1;

            // +CIPRXGET: 2,%d,%d,%s\r\n%s\r\nOK\r\n
            // │             │           │
            // │             │           └ data
            // │             └ data_size
            // └ p_data

            const int count = strtol(
                reinterpret_cast<char const *>(data_size), nullptr, 10);

            if(m_rx_buffer) {
                memcpy(m_rx_buffer+m_rx_count, data, count);

                GSM_INFO("Received %d bytes (%d)\n",
                    count, m_rx_size-m_rx_count-count);

                m_rx_count += count;
                m_rx_available -= count;
                if(m_rx_count == m_rx_size && f_sock_cb)
                    f_sock_cb(Event::rx_complete, p_sock_cb_user);
            }

            m_sock_state = SocketState::idle;
        }
    }

    void Modem::_socket_rts()
    {
        if(memstr(m_response, "ERROR", m_response_size)) {
            GSM_ERROR("Staging error\n");
            m_pending = nullptr;
            m_cmd_buffer.pop();
            if(f_sock_cb)
                f_sock_cb(Event::tx_error, p_sock_cb_user);

            m_sock_state = SocketState::idle;
            return;
        }

        if(memchr(m_response, '>', m_response_size) == nullptr)
            return;

        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "AT+CIPSEND=", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);
            char const *data_accept = static_cast<char*>(
                memchr(p_data, '=', size)) + 1;

            // +CIPSEND=%d\r\n>
            // │         │
            // │         └ data_accept
            // └ p_data

            const int count = strtol(data_accept, nullptr, 10);
            if(m_tx_buffer) {
                // Request data
                #if defined(GSM_ASYNC)
                    m_ctx->send_async(m_tx_buffer+m_tx_count, count);
                #else
                    m_ctx->write(m_tx_buffer+m_tx_count, count);
                #endif

                GSM_VERBOSE("CTS %d bytes (%d)\n", count, m_tx_size-m_tx_count);
            }
            else {
                // Send was canceled but modem is still expecting data,
                memset(m_pending->data, '\0', GSM_BUFFER_SIZE);

                #if defined(GSM_ASYNC)
                    m_ctx->send_async(m_pending->data, count);
                #else
                    m_ctx->write(m_pending->data, count);
                #endif
            }

            m_command_timer = m_ctx->elapsed_ms() + DEFAULT_TIMEOUT_MS;
            m_sock_state = SocketState::cts;
        }
    }

    void Modem::_socket_cts()
    {
        uint8_t *p_data = static_cast<uint8_t*>(
            memstr(m_response, "DATA ACCEPT:", m_response_size));

        if(p_data) {
            const int size = m_response_size - (p_data - m_response);
            if(memchr(p_data, '\n', size)) {
                m_pending = nullptr;
                m_cmd_buffer.pop();
                m_errors = 0;

                char const *data_accept = static_cast<char*>(
                    memchr(p_data, ':', size)) + 1;

                // +DATA ACCEPT:%d\r\n
                // │           │
                // │           └ data
                // └ p_data

                const int count = strtol(data_accept, nullptr, 10);

                GSM_INFO("Sent %d bytes (%d)\n",
                    count, m_tx_size-m_tx_count-count);

                m_tx_count += count;
                if(m_tx_count == m_tx_size && f_sock_cb)
                    f_sock_cb(Event::tx_complete, p_sock_cb_user);

                m_sock_state = SocketState::idle;
            }
        }
    }
}
