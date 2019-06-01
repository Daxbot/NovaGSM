/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file Modem.cpp
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 */

#include <algorithm>
#include <cstdio>
#include <string.h>
#include <errno.h>

#include "Modem.h"
#include "debug.h"

/**
 * @see GSM::State
 *
 * @dot
 * digraph {
 *     none -> init                 [ label = "ATI" ];
 *     init -> none                 [ label = "reset()" ];
 *     init -> locked               [ label = "SIM locked" ];
 *     init -> offline              [ label = "SIM ready" ];
 *     locked -> none               [ label = "reset()" ];
 *     locked -> offline            [ label = "unlock()" ];
 *     offline -> none              [ label = "reset()" ];
 *     offline -> online            [ label = "" ];
 *     online -> none               [ label = "reset()" ];
 *     online -> offline            [ label = "" ];
 *     online -> authenticating     [ label = "authenticate()" ];
 *     authenticating -> online     [ label = "error/\ntimeout" ];
 *     authenticating -> ready      [ label = "OK" ];
 *     ready -> none                [ label = "reset()" ];
 *     ready -> offline             [ label = "lost\nsignal" ];
 *     ready -> handshaking         [ label = "connect()" ];
 *     handshaking -> ready         [ label = "error/\ntimeout" ];
 *     handshaking -> open          [ label = "OK" ];
 *     open -> none                 [ label = "reset()" ];
 *     open -> offline              [ label = "lost\nsignal" ];
 *     open -> ready                [ label = "disconnect()" ];
 * }
 * @enddot
 */

/** Maximum size of socket data transfers.
 *
 * Each data chunk must be less than the actual
 * buffer size to account for protocol overhead.
 */
static constexpr size_t SOCKET_MAX = (GSM::BUFFER_SIZE - 64);

/** Maximum number of errors before reset. */
static constexpr size_t ERRORS_MAX = 10;

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
         * The process loop handles communication with the modem and transitions
         * between device states. If there is not a command already awaiting a 
         * response then the next command in the buffer is sent.  Responses are
         * handled based on the GSM::State.  Any m_pending packets that exceed their
         * timeout value are discarded and the error counter incremented.  
         *
         * If the error counter exceeds MAX_ERRORS the modem is assumed to be MIA and
         * the driver returns to State::none.  The error counter is reset whenever a
         * response is received.
         * 
         * If the signal is lost the connection is assumed to be broken and the driver
         * returns to State::offline.  The user must call authenticate() and connect() again
         * to re-establish.
         */

        uint32_t elapsed_ms = m_ctx->elapsed_ms();

        if(m_pending) {
            // If there is a command m_pending, check for a response.
            p_data = &m_pending->data[m_pending->size];
            const int read_count = m_ctx->read(p_data, (BUFFER_SIZE - m_pending->size));
            if(read_count > 0) {
                m_pending->size += read_count;
                if(m_pending->size < 4)
                    return;

                switch(m_device_state) {
                    case State::none:
                        /** State::none - wait for the modem to respond to an ATI query and transition to State::init. */
                        p_data = static_cast<uint8_t*>(memstr(m_pending->data, "ATI", m_pending->size));
                        if(p_data) {
                            const uint8_t size = m_pending->size - (p_data - m_pending->data);
                            if(memstr(p_data, "OK\r\n", size)) {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;

                                uint8_t *id_start = static_cast<uint8_t*>(memchr(p_data, '\n', size))+1;
                                uint8_t *id_end = static_cast<uint8_t*>(memchr(id_start, '\r', size - (id_start-p_data)));
                                snprintf(m_modem_id, std::min(static_cast<size_t>(id_end - id_start), ID_SIZE), "%s", reinterpret_cast<char*>(id_start));
                                GSM_INFO("Modem is: %s\n", m_modem_id);

                                // ATI\r\n%s\r\nOK\r\n
                                // │     │   │
                                // │     │   └ id_end
                                // │     └ id_start
                                // └ p_data


                                // AT&F0 - reset to factory defaults
                                command_t *command = m_cmd_buffer.front();
                                command->size = sprintf(reinterpret_cast<char*>(command->data), "AT&F0\r\n");
                                m_cmd_buffer.push();

                                // AT+CIURC=0 - disable unsolicited result codes
                                command = m_cmd_buffer.front();
                                command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIURC=0\r\n");
                                m_cmd_buffer.push();

                                // AT+CLTS=1 - enable local timestamps
                                command = m_cmd_buffer.front();
                                command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CLTS=1\r\n");
                                m_cmd_buffer.push();

                                // AT+CFUN=1,1 - reset phone module
                                command = m_cmd_buffer.front();
                                command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CFUN=1,1\r\n");
                                command->timeout_ms = 10000;
                                m_cmd_buffer.push();

                                set_state(State::init);
                            }
                        }
                        break;
                    case State::init:
                        /** State::init - query the SIM card and transition to State::offline or State::locked. */
                    case State::locked:
                        /** State::locked - SIM is locked, call unlock() with the password to transition to State::offline. */
                        if(memstr(m_pending->data, "+CPIN: SIM PIN", m_pending->size)
                        || memstr(m_pending->data, "+CPIN: SIM PUK", m_pending->size)) {
                            GSM_INFO("SIM locked\n");
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                            set_state(State::locked);
                        }
                        else if(memstr(m_pending->data, "+CPIN: READY", m_pending->size)) {
                            GSM_INFO("SIM ready\n");
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                            set_state(State::offline);
                        }
                        else if(memstr(m_pending->data, "OK\r\n", m_pending->size)) {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                        }
                        break;
                    case State::offline:
                        /** State::offline - wait for a signal and transition to State::online. */
                        p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CREG:", m_pending->size));
                        if(p_data) {
                            const uint8_t size = m_pending->size - (p_data - m_pending->data);
                            if(memstr(p_data, "OK\r\n", size)) {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;

                                char const *data = static_cast<char*>(memchr(p_data, ',', size));

                                // +CREG: %u,%u,%s,%s\r\nOK\r\n
                                // │        │
                                // │        └ data
                                // └ p_data

                                const uint8_t status = strtoul(data+1, nullptr, 0);
                                if(status == 1 || status == 5) {
                                    GSM_INFO("Modem online\n");
                                    set_state(State::online);
                                }
                                else {
                                    m_rssi = 99;
                                }
                            }
                        }
                        break;
                    case State::online:
                        /** State::online - registered on network, wait for authenticate() to transition to State::authenticating. */
                        p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CSQ:", m_pending->size));
                        if(p_data) {
                            const uint8_t size = m_pending->size - (p_data - m_pending->data);
                            if(memstr(p_data, "OK\r\n", size)) {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;

                                char const *data = static_cast<char*>(memchr(p_data, ':', size));

                                // +CSQ: %u,%u\r\nOK\r\n
                                // │   │
                                // │   └ data
                                // └ p_data

                                m_rssi = strtoul(data+1, nullptr, 0);
                                if(!m_rssi || m_rssi == 99) {
                                    GSM_INFO("Modem offline\n");
                                    set_state(State::offline);
                                }
                            }
                        }
                        break;
                    case State::authenticating:
                        /** State::authenticating - handle authenticate() and transition to State::ready on success. */
                        if(memstr(m_pending->data, "OK\r\n", m_pending->size)) {
                            if(memstr(m_pending->data, "+CIFSR", m_pending->size)) {
                                GSM_INFO("Authentication success\n");
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;
                                set_state(State::ready);
                            }
                            else {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;
                            }
                            break;
                        }
                        else if(memstr(m_pending->data, "ERROR", m_pending->size)) {
                            GSM_WARN("Authentication failed.\n");
                            m_cmd_buffer.clear();
                            m_pending = nullptr;
                            m_errors = 0;
                            set_state(State::online);
                        }
                        break;
                    case State::ready:
                        /** State::ready - ready to GPRS, wait for connect() to transition to State::handshaking. */
                        p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CGATT:", m_pending->size));
                        if(p_data) {
                            const uint8_t size = m_pending->size - (p_data - m_pending->data);
                            if(memstr(p_data, "OK\r\n", size)) {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;

                                char const *data = static_cast<char*>(memchr(p_data, ':', size));

                                // +CGATT: %u\r\nOK\r\n
                                // │     │
                                // │     └ data
                                // └ p_data

                                const uint8_t status = strtoul(data+1, nullptr, 0);

                                if(status) {
                                    // AT+CSQ - signal quality report
                                    command_t *command = m_cmd_buffer.front();
                                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CSQ\r\n");
                                    m_cmd_buffer.push();
                                }
                                else {
                                    GSM_WARN("Disconnected from GPRS.\n");
                                    set_state(State::online);
                                }
                            }
                            break;
                        }

                        p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CSQ:", m_pending->size));
                        if(p_data) {
                            const uint8_t size = m_pending->size - (p_data - m_pending->data);
                            if(memstr(p_data, "OK\r\n", size)) {
                                m_cmd_buffer.pop();
                                m_pending = nullptr;
                                m_errors = 0;

                                char const *data = static_cast<char*>(memchr(p_data, ':', size));

                                // +CSQ: %u,%u\r\nOK\r\n
                                // │   │
                                // │   └ data
                                // └ p_data

                                m_rssi = strtoul(data+1, nullptr, 0);
                                if(!m_rssi || m_rssi == 99) {
                                    GSM_WARN("Modem offline.\n");
                                    set_state(State::offline);
                                }
                            }
                            break;
                        }
                        break;
                    case State::handshaking:
                        /** State::handshaking - handle connect() and transition to State::open on success. */
                        if(memstr(m_pending->data, "CONNECT OK", m_pending->size)
                        || memstr(m_pending->data, "ALREADY CONNECT", m_pending->size)) {
                            GSM_INFO("TCP connection established\n");
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                            set_state(State::open);
                            break;
                        }
                        else if(memstr(m_pending->data, "CONNECT FAIL", m_pending->size)) {
                            GSM_WARN("TCP connection failed.\n");
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                            set_state(State::ready);
                            break;
                        }
                        else if(memstr(m_pending->data, "+CGATT:", m_pending->size)) {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                        }
                        else if(memstr(m_pending->data, "+CSQ:", m_pending->size)) {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                        }
                        break;
                    case State::open:
                        /** State::open - socket is established, handle write() and listen for incoming data */
                        if(memstr(m_pending->data, "CLOSE OK", m_pending->size)) {
                            GSM_INFO("TCP socket disconnected\n");
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            m_errors = 0;
                        }

                        socket_process();
                        break;
                }
            }
            else if((int32_t)(elapsed_ms - m_command_timer) > 0)
            {
                switch(m_device_state)
                {
                    case State::authenticating:
                        GSM_WARN("Authentication timeout.\n");
                        m_cmd_buffer.clear();
                        m_pending = nullptr;
                        set_state(State::online);
                        break;
                    case State::handshaking:
                        GSM_WARN("Handshaking timeout.\n");
                        m_cmd_buffer.clear();
                        m_pending = nullptr;
                        set_state(State::ready);
                        break;
                    case State::open:
                        GSM_WARN("Socket timeout\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;

                            if(f_sock_cb) {
                                if(m_socket_state == SocketState::cts)
                                    f_sock_cb(Event::tx_error, p_sock_cb_user);
                                else if(m_socket_state == SocketState::rtr)
                                    f_sock_cb(Event::rx_error, p_sock_cb_user);
                            }
                        }
                        break;
                    default:
                        GSM_WARN("Command timeout\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                        }
                        break;
                }
            }
        }
        else if(m_cmd_buffer.empty()) {
            command_t *command = nullptr;

            switch(m_device_state) {
                case State::none:
                    // ATI - device identification
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "ATI\r\n");
                    command->timeout_ms = 1000; // 1 seconds
                    m_cmd_buffer.push();
                    break;
                case State::init:
                case State::locked:
                    // AT+CPIN? - unlock status
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CPIN?\r\n");
                    command->timeout_ms = 5000; // 5 seconds
                    m_cmd_buffer.push();
                    break;
                case State::offline:
                    // AT+CREG - network registration status
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CREG?\r\n");
                    m_cmd_buffer.push();
                    break;
                case State::online:
                    // AT+CSQ - signal quality report
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CSQ\r\n");
                    m_cmd_buffer.push();
                    break;
                case State::authenticating:
                    GSM_ERROR("Authentication error.\n");
                    set_state(State::online);
                    break;
                case State::ready:
                    // AT+CGATT? - state of GPRS attachment
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CGATT?\r\n");
                    command->timeout_ms = 75000; // 75 seconds
                    m_cmd_buffer.push();
                    break;
                case State::handshaking:
                    GSM_ERROR("Handshaking error.\n");
                    set_state(State::ready);
                    break;
                case State::open:
                    // AT+CIPSTATUS - TCP connection status
                    command = m_cmd_buffer.front();
                    command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIPSTATUS\r\n");
                    command->timeout_ms = 1000; // 1 second
                    m_cmd_buffer.push();
                    m_socket_state = SocketState::idle;
                    break;
            }
        }
        else if((int32_t)(elapsed_ms - m_update_timer) > 0) {
            m_update_timer = elapsed_ms + 20;
            m_pending = m_cmd_buffer.back();

            m_ctx->write(m_pending->data, m_pending->size);
            m_command_timer = elapsed_ms + m_pending->timeout_ms;
            m_pending->size = 0;
        }
    }

    int Modem::unlock(const void *pin, uint8_t pin_size)
    {
        if(pin == nullptr)
            return -EINVAL;

        if(m_cmd_buffer.full())
            return -ENOBUFS;

        // AT+CPIN=[pin] - enter pin
        command_t *command = m_cmd_buffer.front();
        command->size = snprintf(reinterpret_cast<char*>(command->data), BUFFER_SIZE, "AT+CPIN=\"%.*s\"\r\n", pin_size, static_cast<const char*>(pin));
        m_cmd_buffer.push();

        return 0;
    }

    int Modem::authenticate(const void *apn, uint8_t apn_size, const void *user, uint8_t user_size, const void *pwd, uint8_t pwd_size)
    {
        if(apn == nullptr || user == nullptr || pwd == nullptr)
            return -EINVAL;

        switch(m_device_state) {
            // Invalid state, return error
            case State::none:
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

        if(m_cmd_buffer.size() + 7 >= POOL_SIZE)
            return -ENOBUFS;

        GSM_INFO("Authenticating...\n");
        set_state(State::authenticating);

        // AT+CIPSHUT - deactivate GPRS PDP context
        command_t *command = m_cmd_buffer.front();
        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIPSHUT\r\n");
        command->timeout_ms = 65000; // 65 seconds
        m_cmd_buffer.push();

        // AT+CGATT=0 - detach from GPRS service
        command = m_cmd_buffer.front();
        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CGATT=0\r\n");
        command->timeout_ms = 75000; // 75 seconds
        m_cmd_buffer.push();

        // AT+CGDCONT=1,[type],[apn] - define GPRS PDP context
        // AT+CGACT=1,1 - activate GPRS PDP context
        command = m_cmd_buffer.front();
        command->size = snprintf(
            reinterpret_cast<char*>(command->data), BUFFER_SIZE,
            "AT+CGDCONT=1,\"IP\",\"%.*s\";"
            "+CGACT=1,1\r\n",
            apn_size, static_cast<const char*>(apn));
        command->timeout_ms = 150000; // 150 second
        m_cmd_buffer.push();

        // AT+SAPBR=3,1,[tag],[value] - configure bearer
        // AT+SAPBR=1,1 - open bearer
        command = m_cmd_buffer.front();
        command->size = sprintf(
            reinterpret_cast<char*>(command->data),
            "AT+SAPBR=3,1,\"Contype\",\"GPRS\";"
            "+SAPBR=3,1,\"APN\",\"%.*s\";"
            "+SAPBR=3,1,\"USER\",\"%.*s\";"
            "+SAPBR=3,1,\"PWD\",\"%.*s\";"
            "+SAPBR=1,1\r\n",
            apn_size, static_cast<const char*>(apn),
            user_size, static_cast<const char*>(user),
            pwd_size, static_cast<const char*>(pwd));
        command->timeout_ms = 850000; // 85 seconds
        m_cmd_buffer.push();

        // AT+CGATT=1 - attach to GPRS service
        // AT+CIPMUX=0 - single IP mode
        // AT+CIPQSEND=1 - quick send mode
        // AT+CIPRXGET=1 - manual data mode
        // AT+CSTT=[apn],[user],[password] - set apn/user/password for GPRS PDP context
        command = m_cmd_buffer.front();
        command->size = snprintf(
            reinterpret_cast<char*>(command->data), BUFFER_SIZE,
            "AT+CGATT=1;"
            "+CIPMUX=0;"
            "+CIPQSEND=1;"
            "+CIPRXGET=1;"
            "+CSTT=\"%.*s\",\"%.*s\",\"%.*s\"\r\n",
            apn_size, static_cast<const char*>(apn),
            user_size, static_cast<const char*>(user),
            pwd_size, static_cast<const char*>(pwd));
        command->timeout_ms = 75000; // 75 seconds
        m_cmd_buffer.push();

        // AT+CIICR - bring up wireless connection
        command = m_cmd_buffer.front();
        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIICR\r\n");
        command->timeout_ms = 60000; // 60 seconds
        m_cmd_buffer.push();

        // AT+CIFSR - get local IP address
        // AT+CDNSCFG=[primary],[secondary] - configure DNS
        command = m_cmd_buffer.front();
        command->size = sprintf(
            reinterpret_cast<char*>(command->data),
            "AT+CIFSR;"
            "+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\";\r\n");
        command->timeout_ms = 10000; // 10 seconds
        m_cmd_buffer.push();

        return 0;
    }

    int Modem::authenticate(const char *apn, const char *user, const char *pwd)
    {
        return authenticate(apn, strlen(apn), user, strlen(user), pwd, strlen(pwd));
    }

    int Modem::connect(const void *host, uint8_t host_size, uint16_t port)
    {
        if(host == nullptr)
            return -EINVAL;

        switch(m_device_state) {
            // Invalid state, return error
            case State::none:
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

        if(m_cmd_buffer.full())
            return -ENOBUFS;

        GSM_INFO("Handshaking...\n");
        set_state(State::handshaking);

        // AT+CIPSTART=[type],[ip],[port] - start TCP/UDP connection to 'ip':'port'
        command_t *command = m_cmd_buffer.front();
        command->size = snprintf(
            reinterpret_cast<char*>(command->data), BUFFER_SIZE,
            "AT+CIPSTART=\"TCP\",\"%.*s\",%u\r\n",
            host_size, static_cast<const char*>(host), port);
        command->timeout_ms = 75000; // 75 seconds
        m_cmd_buffer.push();

        return 0;
    }

    int Modem::connect(const char *host, uint16_t port)
    {
        return connect(host, strlen(host), port);
    }

    int Modem::disconnect()
    {
        switch(m_device_state) {
            // Invalid state, return error
            case State::none:
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

        if(m_cmd_buffer.full())
            return -ENOBUFS;

        // AT+CIPCLOSE - close TCP/UDP connection
        command_t *command = m_cmd_buffer.front();
        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIPCLOSE\r\n");
        m_cmd_buffer.push();
        return 0;
    }

    void Modem::reset()
    {
        GSM_INFO("Connection reset\n");

        if(f_sock_cb) {
            if(m_tx_buffer && m_tx_size != m_tx_count)
                f_sock_cb(Event::tx_error, p_sock_cb_user);

            if(m_rx_buffer && m_rx_size != m_rx_count)
                f_sock_cb(Event::rx_error, p_sock_cb_user);
        }

        m_cmd_buffer.clear();
        m_pending = nullptr;
        m_command_timer = 0;
        m_update_timer = 0;
        m_device_state = State::none;
        m_socket_state = SocketState::idle;
        m_rssi = 99;
        m_errors = 0;
        m_rx_buffer = nullptr;
        m_tx_buffer = nullptr;
        m_rx_available = 0;
        m_tx_available = 0;
        m_modem_id[0] = '\0';
    }

    void Modem::receive(void *data,  uint32_t size)
    {
        m_rx_buffer = static_cast<uint8_t*>(data);
        m_rx_size = size;
        m_rx_count = 0;
    }

    void Modem::send(const void *data, uint32_t size)
    {
        m_tx_buffer = static_cast<const uint8_t*>(data);
        m_tx_size = size;
        m_tx_count = 0;
    }

    void Modem::socket_process()
    {
        switch(m_socket_state) {
            case SocketState::idle:
                /** SocketState::idle - no active data transfer, handle send() / receive() and poll connection status. */
                if(memstr(m_pending->data, "STATE: CONNECT OK", m_pending->size)) {
                    if(m_rx_buffer && m_rx_count < m_rx_size && m_rx_available) {
                        // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
                        const unsigned int count = std::min(SOCKET_MAX, std::min(m_rx_available, static_cast<size_t>(m_rx_size-m_rx_count)));
                        m_pending->size = sprintf(reinterpret_cast<char*>(m_pending->data), "AT+CIPRXGET=2,%u\r\n", count);
                        m_ctx->write(m_pending->data, m_pending->size);

                        GSM_VERBOSE("RTR %u bytes (%u)\n", count, m_rx_size-m_rx_count);

                        // Reset the buffer and use it for the response
                        m_command_timer = m_ctx->elapsed_ms() + m_pending->timeout_ms;
                        m_pending->size = 0;

                        m_socket_state = SocketState::rtr;
                    }
                    else if(m_tx_buffer && m_tx_count < m_tx_size && m_tx_available) {
                        // AT+CIPSEND=[size] - indicate that data is about to be sent
                        const unsigned int count = std::min(SOCKET_MAX, std::min(m_tx_available, static_cast<size_t>(m_tx_size-m_tx_count)));
                        m_pending->size = sprintf(reinterpret_cast<char*>(m_pending->data), "AT+CIPSEND=%u\r\n", count);
                        m_ctx->write(m_pending->data, m_pending->size);

                        GSM_VERBOSE("RTS %u bytes (%u)\n", count, m_rx_size-m_rx_count);

                        // Reset the buffer and use it for the response
                        m_command_timer = m_ctx->elapsed_ms() + m_pending->timeout_ms;
                        m_pending->size = 0;

                        m_socket_state = SocketState::rts;
                    }
                    else {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        // AT+CIPRXGET=4 - query socket unread bytes
                        command_t *command = m_cmd_buffer.front();
                        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIPRXGET=4\r\n");
                        m_cmd_buffer.push();

                        // AT+CIPSEND? - query available size of tx buffer
                        command = m_cmd_buffer.front();
                        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CIPSEND?\r\n");
                        m_cmd_buffer.push();

                        // AT+CSQ - signal quality report
                        command = m_cmd_buffer.front();
                        command->size = sprintf(reinterpret_cast<char*>(command->data), "AT+CSQ\r\n");
                        m_cmd_buffer.push();

                        m_socket_state = SocketState::poll;
                    }
                    break;
                }
                else if(memstr(m_pending->data, "STATE: TCP CLOSED", m_pending->size)) {
                    GSM_WARN("Server closed TCP socket.\n");
                    m_socket_state = SocketState::idle;

                    if(f_sock_cb) {
                        if(m_tx_buffer && m_tx_size != m_tx_count)
                            f_sock_cb(Event::tx_error, p_sock_cb_user);

                        if(m_rx_buffer && m_rx_size != m_rx_count)
                            f_sock_cb(Event::rx_error, p_sock_cb_user);
                    }

                    m_tx_buffer = nullptr;
                    m_rx_buffer = nullptr;

                    m_cmd_buffer.clear();
                    m_pending = nullptr;
                    m_errors = 0;
                    set_state(State::ready);
                    break;
                }
                break;
            case SocketState::poll:
                /** SocketState::poll - waiting for modem to report connection status. */
                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CIPRXGET:", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memstr(p_data, "OK\r\n", size)) {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        char const *data = static_cast<char*>(memchr(p_data, ',', size))+1;

                        // +CIPRXGET: 4,%u\r\nOK\r\n
                        // │             │
                        // │             └ data
                        // └ p_data

                        const unsigned int count = strtoul(data, nullptr, 0);

                        if(count > m_rx_available && f_sock_cb) {
                            m_rx_available = count;
                            f_sock_cb(Event::new_data, p_sock_cb_user);
                        }
                        else {
                            m_rx_available = count;
                        }

                    }
                    break;
                }

                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CIPSEND:", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memstr(p_data, "OK\r\n", size)) {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        char const *data = static_cast<char*>(memchr(p_data, ':', size))+1;

                        // +CIPSEND: %u\r\nOK\r\n
                        // │        │
                        // │        └ data
                        // └ p_data

                        m_tx_available = strtoul(data, nullptr, 0);
                    }
                    break;
                }

                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CSQ:", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memstr(p_data, "OK\r\n", size)) {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        char const *data = static_cast<char*>(memchr(p_data, ':', size))+1;

                        // +CSQ: %u,%u\r\nOK\r\n
                        // │    │
                        // │    └ data
                        // └ p_data

                        m_rssi = strtoul(data, nullptr, 0);
                        if(!m_rssi || m_rssi == 99) {
                            GSM_WARN("Modem offline.\n");
                            disconnect();
                        }
                    }
                    break;
                }
                break;
            case SocketState::rtr:
                /** SocketState::rtr - ready to receive, data is being read into the receive buffer. */
                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "+CIPRXGET: 2,", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memstr(p_data, "OK\r\n", size)) {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        uint8_t const *data_size = static_cast<const uint8_t*>(memchr(p_data, ',', size))+1;
                        uint8_t const *data = static_cast<const uint8_t*>(memchr(data_size, '\n', size - (data_size - p_data)))+1;

                        // +CIPRXGET: 2,%u,%u,%s\r\n%s\r\nOK\r\n
                        // │             │           │
                        // │             │           └ data
                        // │             └ data_size
                        // └ p_data

                        const unsigned int count = strtoul(reinterpret_cast<char const *>(data_size), nullptr, 0);
                        memcpy(m_rx_buffer+m_rx_count, data, count);
                        GSM_INFO("Received %u bytes (%u)\n", count, m_rx_size-m_rx_count-count);

                        m_rx_count += count;
                        m_rx_available = 0;
                        if(m_rx_count == m_rx_size && f_sock_cb)
                            f_sock_cb(Event::rx_complete, p_sock_cb_user);

                        m_errors = 0;
                        m_socket_state = SocketState::idle;
                    }
                    else if(memstr(p_data, "ERROR", size)) {
                        GSM_ERROR("Receive error\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            if(f_sock_cb)
                                f_sock_cb(Event::rx_error, p_sock_cb_user);

                            m_socket_state = SocketState::idle;
                        }
                    }
                }
                break;
            case SocketState::rts:
                /** SocketState::rts - ready to send, inform modem that data is about to be sent. */
                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "AT+CIPSEND=", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memstr(m_pending->data, "ERROR", m_pending->size)) {
                        GSM_ERROR("Staging error\n");
                        if(++m_errors >= ERRORS_MAX) {
                            reset();
                        }
                        else {
                            m_cmd_buffer.pop();
                            m_pending = nullptr;
                            if(f_sock_cb)
                                f_sock_cb(Event::tx_error, p_sock_cb_user);

                            m_socket_state = SocketState::idle;
                        }
                    }
                    else if(memchr(p_data, '>', size)) {
                        char const *data_accept = static_cast<char*>(memchr(p_data, '=', size))+1;

                        // +CIPSEND=%u\r\n>
                        // │         │
                        // │         └ data_accept
                        // └ p_data

                        const unsigned int count = strtoul(data_accept, nullptr, 0);
                        m_ctx->write(m_tx_buffer+m_tx_count, count);
                        GSM_VERBOSE("CTS %u bytes (%u)\n", count, m_tx_size-m_tx_count);

                        // Reset the buffer and use it for the response
                        m_command_timer = m_ctx->elapsed_ms() + m_pending->timeout_ms;
                        m_pending->size = 0;

                        m_socket_state = SocketState::cts;
                    }
                }
                break;
            case SocketState::cts:
                /** SocketState::cts - clear to send, data is being written from the send buffer. */
                p_data = static_cast<uint8_t*>(memstr(m_pending->data, "DATA ACCEPT:", m_pending->size));
                if(p_data) {
                    const size_t size = m_pending->size - (p_data - m_pending->data);
                    if(memchr(p_data, '\n', size)) {
                        m_cmd_buffer.pop();
                        m_pending = nullptr;

                        char const *data_accept = static_cast<char*>(memchr(p_data, ':', size))+1;

                        // +DATA ACCEPT:%u\r\n
                        // │           │
                        // │           └ data
                        // └ p_data

                        const unsigned int count = strtoul(data_accept, nullptr, 0);
                        GSM_INFO("Sent %u bytes (%u)\n", count, m_tx_size-m_tx_count-count);

                        m_tx_count += count;
                        if(m_tx_count == m_tx_size && f_sock_cb)
                            f_sock_cb(Event::tx_complete, p_sock_cb_user);

                        m_errors = 0;
                        m_socket_state = SocketState::idle;
                    }
                }
                break;
        }
    }

    void Modem::set_state(State state)
    {
        m_device_state = state;
        if(f_dev_cb)
            f_dev_cb(m_device_state, p_dev_cb_user);
    }
}
