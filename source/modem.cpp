/**
 * @file modem.cpp
 * @brief GSM/GPRS modem driver.
 * @author Wilkins White
 * @copyright 2019-2021 Nova Dynamics LLC
 */

#include <cassert>

#include <algorithm>
#include <cstdio>
#include <string.h>
#include <errno.h>
#include <modem.h>
#include <stdarg.h>

#include "debug.h"

/** How often to poll the modem. */
static constexpr int kPollingInterval = 20;

/**
 * @brief Buffer string search.
 * @param [in] data buffer to search.
 * @param [in] target string to find.
 * @param [in] size maximum bytes to search.
 */
static void *memstr(void *data, const char *target, int size)
{
    char *p_data = static_cast<char*>(data);
    const int len = strlen(target);

    if(len <= size) {
        for(; data != (p_data-size); p_data++) {
            if(*p_data == target[0] && memcmp(p_data, target, len) == 0)
                return static_cast<void*>(p_data);
        }
    }

    return nullptr;
}

namespace gsm
{
    Modem::~Modem()
    {
        // Free command queue
        while(cmd_buffer_.size() > 0) {
            free(cmd_buffer_.front());
            cmd_buffer_.pop_front();
        }
    }

    void Modem::process(int delta_ms)
    {
        elapsed_ms_ += delta_ms;

        if(pending_) {
            // Command pending - wait for response
            #if defined(NOVAGSM_ASYNC)
            response_size_ = ctx_->rx_count_async();
            #else
            uint8_t *p_data = &response_[response_size_];
            int count = ctx_->read(p_data, (kBufferSize - response_size_));
            if(count > 0)
                response_size_ += count;
            #endif

            // Parse response
            if(response_size_ > 0) {
                if(connected())
                    process_socket();
                else if(authenticating())
                    process_authentication();
                else if(handshaking())
                    process_handshaking();
                else
                    process_general();
            }

            // Handle timeout
            if(pending_ && (int)(elapsed_ms_ - command_timer_) > 0)
                process_timeout();
        }
        else if(cmd_buffer_.size() > 0) {
            pending_ = cmd_buffer_.front();
            cmd_buffer_.pop_front();

            // Send queued command
            #if defined(NOVAGSM_ASYNC)
                ctx_->rx_abort_async();
                ctx_->receive_async(response_, kBufferSize);
                ctx_->send_async(front->data, front->size);
            #else
                ctx_->write(pending_->data(), pending_->size());
            #endif

            command_timer_ = elapsed_ms_ + pending_->timeout();
            response_size_ = 0;
        }
        else if((int)(elapsed_ms_ - update_timer_) > 0) {
            // Nothing queued - poll the modem
            update_timer_ = elapsed_ms_ + kPollingInterval;
            poll_modem();
        }
    }

    void Modem::reinit()
    {
        stop_send();
        stop_receive();

        if(pending_) {
            #if defined(NOVAGSM_ASYNC)
            ctx_->rx_abort_async();
            #endif
            free_pending();
        }

        while(cmd_buffer_.size() > 0) {
            free(cmd_buffer_.front());
            cmd_buffer_.pop_front();
        }

        signal_ = 99;
        memset(ip_address_, '\0', sizeof(ip_address_));

        sock_state_ = SocketState::idle;
        rx_buffer_ = nullptr;
        tx_buffer_ = nullptr;
        rx_available_ = 0;
        tx_available_ = 0;
    }

    int Modem::disable()
    {
        reinit();

        // AT+CFUN=0 - minimum functionality mode
        Command *cmd = new (std::nothrow) Command(10000, "AT+CFUN=0\r\n");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        set_state(State::offline);
        return 0;
    }

    int Modem::reset()
    {
        reinit();

        // AT+CFUN=1,1 - reset phone module
        Command *cmd = new (std::nothrow) Command(10000, "AT+CFUN=1,1;\r\n");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        set_state(State::reset);
        return 0;
    }

    int Modem::unlock(const void *pin, int pin_size)
    {
        if(pin == nullptr)
            return -EINVAL;

        // AT+CPIN=[pin] - enter pin
        Command *cmd = new (std::nothrow) Command(kDefaultTimeout,
            "AT+CPIN=\"%.*s\"\r\n", pin_size, static_cast<const char*>(pin));

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        return result;
    }

    int Modem::authenticate(
        const void *apn,
        int apn_size,
        const void *user,
        int user_size,
        const void *pwd,
        int pwd_size)
    {
        // AT+CIPSHUT - reset GPRS context
        // AT+CIPMUX=0 - set single IP mode
        // AT+CIPRXGET=1 - set manual data receive
        // AT+CIPQSEND=1 - set quick send
        // AT+CSTT=[apn],[user],[pwd] - set apn/user/password for GPRS context
        Command *cmd = new (std::nothrow) Command(65000,
            "AT+CIPSHUT;+CIPMUX=0;+CIPRXGET=1;+CIPQSEND=1;"
            "+CSTT=\"%.*s\",\"%.*s\",\"%.*s\"\r\n",
            apn_size, static_cast<const char *>(apn),
            user_size, static_cast<const char *>(user),
            pwd_size, static_cast<const char *>(pwd));

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        // AT+CIICR - activate data connection
        // AT+CIFSR - get local IP address
        cmd = new (std::nothrow) Command(85000, "AT+CIICR;+CIFSR\r\n");
        if(cmd == nullptr)
            return -ENOMEM;

        result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_INFO("Authenticating...\n");
        set_state(State::authenticating);
        return 0;
    }

    int Modem::authenticate(
        const char *apn, const char *user, const char *pwd)
    {
        return authenticate(
            apn, (apn) ? strlen(apn) : 0,
            user, (user) ? strlen(user) : 0,
            pwd, (pwd) ? strlen(pwd) : 0);
    }

    int Modem::connect(const void *host, int host_size, int port)
    {
        if(host == nullptr)
            return -EINVAL;

        switch(device_state_) {
            // Invalid state, return error
            case State::offline:
                return -ENODEV;
            case State::reset:
            case State::locked:
            case State::searching:
                return -ENETUNREACH;
            case State::registered:
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

        // AT+CIPSTART=[mode],[host],[port] - start a new connection
        Command *cmd = new (std::nothrow) Command(75000,
            "AT+CIPSTART=\"TCP\",\"%.*s\",%d\r\n",
            host_size, static_cast<const char*>(host), port);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_INFO("Handshaking...\n");
        set_state(State::handshaking);
        return 0;
    }

    int Modem::connect(const char *host, int port)
    {
        return connect(host, (host) ? strlen(host) : 0, port);
    }

    int Modem::disconnect()
    {
        switch(device_state_) {
            // Invalid state, return error
            case State::offline:
                return -ENODEV;
            case State::reset:
            case State::locked:
            case State::searching:
                return -ENETUNREACH;
            case State::registered:
            case State::authenticating:
            case State::ready:
            case State::handshaking:
                return -ENOTSOCK;

            // Continue
            case State::open:
                break;
        }

        // AT+CIPCLOSE - close connection
        Command *cmd = new (std::nothrow) Command(kDefaultTimeout, "AT+CIPCLOSE\r\n");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        return result;
    }

    void Modem::receive(void *data, int size)
    {
        rx_buffer_ = static_cast<uint8_t*>(data);
        rx_size_ = size;
        rx_count_ = 0;
    }

    void Modem::stop_receive()
    {
        if(event_cb_ && rx_busy())
            event_cb_(Event::rx_stopped, event_cb_user_);

        receive(nullptr, 0);
    }

    void Modem::send(const void *data, int size)
    {
        tx_buffer_ = static_cast<const uint8_t*>(data);
        tx_size_ = size;
        tx_count_ = 0;
    }

    void Modem::stop_send()
    {
        if(event_cb_ && tx_busy())
            event_cb_(Event::tx_stopped, event_cb_user_);

        send(nullptr, 0);
    }

    void Modem::set_state(State state)
    {
        if(state == device_state_)
            return;

        device_state_ = state;
        if(state_cb_)
            state_cb_(device_state_, state_cb_user_);
    }

    void Modem::free_pending()
    {
        assert(pending_);
        delete pending_;
        pending_ = nullptr;
    }

    bool Modem::response_complete()
    {
        if(memstr(response_, "\r\nOK\r\n", response_size_))
            return true;

        if(memstr(response_, "\r\nERROR\r\n", response_size_))
            return true;

        return false;
    }

    int Modem::push_command(Command *cmd)
    {
        assert(cmd);
        if(cmd->size() >= kBufferSize)
            return -EMSGSIZE;

        cmd_buffer_.push_back(cmd);
        return 0;
    }

    int Modem::shift_command(Command *cmd)
    {
        assert(cmd);
        if(cmd->size() >= kBufferSize)
            return -EMSGSIZE;

        cmd_buffer_.push_front(cmd);
        return 0;
    }

    int Modem::poll_modem()
    {
        Command *cmd = nullptr;

        switch(device_state_) {
            case State::reset:
            case State::locked:
                // AT+CFUN? - power status
                // AT+CPIN? - SIM status
                cmd = new (std::nothrow) Command(5000, "AT+CFUN?;+CPIN?\r\n");
                break;
            case State::searching:
            case State::registered:
            case State::ready:
                // AT+CSQ - signal quality report
                // AT+CREG? - registration status
                cmd = new (std::nothrow) Command(1000, "AT+CSQ;+CREG?\r\n");
                break;
            case State::open:
                // AT+CSQ - signal quality report
                // AT+CIPSTATUS - TCP connection status
                // AT+CIPRXGET=4 - query socket unread bytes
                // AT+CIPSEND? - query available size of tx buffer
                cmd = new (std::nothrow) Command(1000,
                    "AT+CSQ;+CIPSTATUS;+CIPRXGET=4;+CIPSEND?\r\n");

                sock_state_ = SocketState::idle;
                break;
            default:
                return 0;
        }

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete  cmd;

        return result;
    }

    int Modem::socket_receive(int count)
    {
        if(count == 0)
            return 0;

        if(sock_state_ != SocketState::idle)
            return -EBUSY;

        // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
        Command *cmd = new (std::nothrow) Command(kDefaultTimeout,
            "AT+CIPRXGET=2,%d\r\n", count);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = shift_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_VERBOSE("RTR %d bytes (%d)\n", count, rx_size_-rx_count_);
        sock_state_ = SocketState::rtr;
        return 0;
    }

    int Modem::socket_send(int count)
    {
        if(count == 0)
            return 0;

        if(sock_state_ != SocketState::idle)
            return -EBUSY;

        // AT+CIPSEND=[size] - indicate that data is about to be sent
        Command *cmd = new (std::nothrow) Command(kDefaultTimeout,
            "AT+CIPSEND=%d\r\n", count);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = shift_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_VERBOSE("RTS %d bytes (%d)\n", count, (tx_size_ - tx_count_));
        sock_state_ = SocketState::rts;
        return 0;
    }

    void Modem::process_timeout()
    {
        switch(device_state_) {
            case State::authenticating:
                LOG_WARN("Authentication timeout.\n");
                if(event_cb_)
                    event_cb_(Event::auth_error, event_cb_user_);

                set_state(State::registered);
                break;
            case State::handshaking:
                LOG_WARN("TCP connection timeout.\n");
                if(event_cb_)
                    event_cb_(Event::conn_error, event_cb_user_);

                set_state(State::ready);
                break;
            case State::open:
                LOG_WARN("Socket timeout\n");
                if(event_cb_) {
                    if(sock_state_ == SocketState::cts)
                        event_cb_(Event::tx_error, event_cb_user_);
                    else if(sock_state_ == SocketState::rtr)
                        event_cb_(Event::rx_error, event_cb_user_);
                }
                break;
            default:
                LOG_WARN("Command timeout\n");
                if(event_cb_)
                    event_cb_(Event::timeout, event_cb_user_);
                break;
        }

        free_pending();
    }

    void Modem::process_general()
    {
        if(!response_complete())
            return;

        // Parse lines
        uint8_t *start = response_;
        int size = response_size_;

        while(size > 4) {
            uint8_t *end = static_cast<uint8_t*>(memchr(start, '\n', size));
            if(end == nullptr)
                break;

            int length = (end - start) + 1;
            if(length > 4) {
                if(memstr(start, "+CSQ:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CSQ: %d,%d\r\n
                    // │   │
                    // │   └ data
                    // └ start

                    signal_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
                else if(memstr(start, "+CREG:", length)) {
                    void *data = memchr(start, ',', length);

                    // +CREG: %d,%d,%s,%s\r\n
                    // │        │
                    // │        └ data
                    // └ start

                    const uint8_t status = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);

                    if(status == 1 || status == 5) {
                        if(device_state_ < State::registered) {
                            LOG_INFO("Registered\n");
                            set_state(State::registered);
                        }
                    }
                    else {
                        if(device_state_ >= State::registered) {
                            LOG_INFO("Searching for network\n");
                            set_state(State::searching);
                        }
                    }
                }
                else if(online()) {
                    if(memstr(start, "+CPIN: READY", length)) {
                        if(device_state_ < State::searching) {
                            LOG_INFO("Modem online\n");
                            set_state(State::searching);
                        }
                    }
                    else if(memstr(start, "+CPIN: SIM PUK", length)
                        || memstr(start, "+CPIN: SIM PIN", length)) {
                        LOG_INFO("SIM locked\n");
                        set_state(State::locked);
                    }
                    else if(memstr(start, "+CPIN: NOT READY", length)) {
                        LOG_INFO("Modem offline\n");
                        set_state(State::offline);
                    }
                    else if(memstr(start, "+CFUN:", length)) {
                        void *data = memchr(start, ':', length);

                        // +CFUN: %d\r\n
                        // │    │
                        // │    └ data
                        // └ start

                        const int status = strtol(
                            static_cast<char*>(data)+1, nullptr, 10);

                        if(status == 0) {
                            LOG_INFO("Modem offline\n");
                            set_state(State::offline);
                        }
                    }
                }
            }

            start = end + 1;
            size -= length;
        }

        free_pending();
    }

    void Modem::process_authentication()
    {
        /**
         * Clear any pending 'OK' responses. We know that the authentication
         * is successful once we get an IP address from AT+CIFSR.
         */
        if(memstr(response_, "\r\nOK\r\n", response_size_)) {
            free_pending();
            return;
        }

        // Parse lines
        uint8_t *start = response_;
        int size = response_size_;

        while(size > 4) {
            uint8_t *end = static_cast<uint8_t*>(memchr(start, '\n', size));
            if(end == nullptr)
                break;

            int length = (end - start) + 1;
            if(length > 4) {
                // Parse IP address from AT+CIFSR
                unsigned int a,b,c,d;
                char *data = reinterpret_cast<char *>(start);
                if(sscanf(data, "%3u.%3u.%3u.%3u", &a, &b, &c, &d) == 4) {
                    // Store IP address
                    snprintf(ip_address_, sizeof(ip_address_),
                        "%u.%u.%u.%u", a, b, c, d);

                    LOG_INFO("Connected to GPRS\n");
                    set_state(State::ready);
                    free_pending();
                }
            }

            start = end + 1;
            size -= length;
        }
    }

    void Modem::process_handshaking()
    {
        // Wait for 'OK'
        if(!memstr(response_, "\r\nOK\r\n", response_size_))
            return;

        // Parse lines
        uint8_t *start = response_;
        int size = response_size_;

        while(size > 4) {
            uint8_t *end = static_cast<uint8_t*>(memchr(start, '\n', size));
            if(end == nullptr)
                break;

            int length = (end - start) + 1;
            if(length > 4) {
                // Expected responses to AT+CIPSTART=...
                if(memstr(start, "CONNECT OK", length)) {
                    LOG_INFO("TCP socket connected\n");
                    sock_state_ = SocketState::idle;
                    set_state(State::open);
                    free_pending();
                    break;
                }
                else if(memstr(start, "ALREADY CONNECT", length)) {
                    set_state(State::open);
                    free_pending();
                    break;
                }
                else if(memstr(start, "CONNECT FAIL", length)) {
                    LOG_WARN("TCP connection failed\n");
                    set_state(State::ready);
                    free_pending();
                    break;
                }
            }

            start = end + 1;
            size -= length;
        }
    }

    void Modem::process_socket()
    {
        switch(sock_state_) {
            case SocketState::idle:
                // Idle
                socket_state_idle();
                break;
            case SocketState::rtr:
                // Ready to receive
                socket_state_rtr();
                break;
            case SocketState::rts:
                // Request to send
                socket_state_rts();
                break;
            case SocketState::cts:
                // Clear to send
                socket_state_cts();
                break;
        }
    }

    void Modem::socket_state_idle()
    {
        // Wait for 'OK'
        if(!memstr(response_, "\r\nOK\r\n", response_size_))
            return;

        // Parse lines
        uint8_t *start = response_;
        int size = response_size_;

        while(size > 4) {
            uint8_t *end = static_cast<uint8_t*>(memchr(start, '\n', size));
            if(end == nullptr)
                break;

            int length = (end - start);
            if(length >= 4) {
                if(memstr(start, "+CPIN: NOT READY", length)) {
                    LOG_INFO("Modem offline\n");

                    stop_send();
                    stop_receive();

                    set_state(State::offline);
                    break;
                }
                else if(memstr(start, "CLOSE OK", length)
                    || memstr(start, "TCP CLOSED", length)) {
                    LOG_INFO("TCP socket disconnected\n");

                    stop_send();
                    stop_receive();

                    set_state(State::ready);
                    break;
                }
                else if(memstr(start, "+CIPRXGET: 4", length)) {
                    void *data = memchr(start, ',', length);

                    // +CIPRXGET: 4,%d\r\nOK\r\n
                    // │           │
                    // │           └ data
                    // └ start

                    const int count = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);

                    if(count > rx_available_ && event_cb_)
                        event_cb_(Event::new_data, event_cb_user_);

                    rx_available_ = count;

                    const int requested = (rx_size_ - rx_count_);
                    const int available = std::min(rx_available_, kSocketMax);
                    if(rx_buffer_ && (requested > 0 || available > 0))
                        socket_receive(std::min(requested, available));
                }
                else if(memstr(start, "+CIPSEND:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CIPSEND: %d\r\nOK\r\n
                    // │       │
                    // │       └ data
                    // └ start

                    tx_available_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);

                    const int requested = (tx_size_ - tx_count_);
                    const int available = std::min(tx_available_, kSocketMax);
                    if(tx_buffer_ && (requested > 0 || available > 0))
                        socket_send(std::min(requested, available));
                }
                else if(memstr(start, "+CSQ:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CSQ: %d,%d\r\n
                    // │   │
                    // │   └ data
                    // └ start

                    signal_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
            }

            start = end + 1;
            size -= length;
        }

        free_pending();
    }

    void Modem::socket_state_rtr()
    {
        if(memstr(response_, "ERROR", response_size_)) {
            LOG_ERROR("Receive error: %.*s\n", response_size_, response_);
            free_pending();
            if(event_cb_)
                event_cb_(Event::rx_error, event_cb_user_);

            sock_state_ = SocketState::idle;
            return;
        }

        if(!memstr(response_, "\r\nOK\r\n", response_size_)) {
            return;
        }

        uint8_t *start = static_cast<uint8_t*>(
            memstr(response_, "+CIPRXGET: 2", response_size_));

        if(start) {
            int size = response_size_ - (start - response_);

            void *data = memchr(start, ',', size);

            // +CIPRXGET: 2,%d,%d,%s\r\n%s\r\nOK\r\n
            // │           │
            // │           └ data
            // └ start

            const int count = strtol(
                reinterpret_cast<char*>(data)+1, nullptr, 10);

            // Guard against partial read
            if(size < (count + 6))
                return;

            data = memchr(start, '\n', size);

            // +CIPRXGET: 2,%d,%d,%s\r\n%s\r\nOK\r\n
            // │                       │
            // │                       └ data
            // └ start

            LOG_INFO("Received %d bytes (%d)\n",
                    count, rx_size_ - rx_count_ - count);

            rx_available_ -= count;
            if(rx_buffer_) {
                memcpy(rx_buffer_ + rx_count_, data, count);

                rx_count_ += count;
                if(rx_count_ == rx_size_ && event_cb_)
                    event_cb_(Event::rx_complete, event_cb_user_);
            }

            sock_state_ = SocketState::idle;
            free_pending();
        }
    }

    void Modem::socket_state_rts()
    {
        if(memstr(response_, "ERROR", response_size_)) {
            LOG_ERROR("Staging error\n");
            free_pending();
            if(event_cb_)
                event_cb_(Event::tx_error, event_cb_user_);

            sock_state_ = SocketState::idle;
            return;
        }

        if(memchr(response_, '>', response_size_) == nullptr)
            return;

        uint8_t *start = static_cast<uint8_t*>(
            memstr(response_, "AT+CIPSEND=", response_size_));

        if(start) {
            const int size = response_size_ - (start - response_);

            void *data = memchr(start, '=', size);

            // +CIPSEND=%d\r\n>
            // │       │
            // │       └ data
            // └ start

            const int count = strtol(
                static_cast<char*>(data)+1, nullptr, 10);

            if(tx_buffer_) {
                // Send data
                const uint8_t *buffer = tx_buffer_ + tx_count_;
                #if defined(GSM_ASYNC)
                ctx_->send_async(buffer, count);
                #else
                ctx_->write(buffer, count);
                #endif

                LOG_VERBOSE("CTS %d bytes (%d)\n",
                    count, (tx_size_ - tx_count_));
            }
            else {
                /*
                 * The send was canceled, but the modem is still
                 * expecting data. Send zeroes until the buffer is
                 * full.
                 */
                uint8_t *buffer = static_cast<uint8_t*>(malloc(kBufferSize));
                memset(buffer, '\0', kBufferSize);

                #if defined(GSM_ASYNC)
                ctx_->send_async(buffer, kBufferSize);
                #else
                ctx_->write(buffer, kBufferSize);
                #endif
            }

            command_timer_ = elapsed_ms_ + kDefaultTimeout;
            sock_state_ = SocketState::cts;
        }
    }

    void Modem::socket_state_cts()
    {
        uint8_t *start = static_cast<uint8_t*>(
            memstr(response_, "DATA ACCEPT:", response_size_));

        if(start) {
            const int size = response_size_ - (start - response_);

            if(memchr(start, '\n', size)) {
                void *data = memchr(start, ':', size);

                // +DATA ACCEPT:%d\r\n
                // │           │
                // │           └ data
                // └ p_data

                const int count = strtol(
                    static_cast<char*>(data)+1, nullptr, 10);

                LOG_INFO("Sent %d bytes (%d)\n",
                    count, tx_size_ - tx_count_ - count);

                tx_count_ += count;
                if(tx_count_ == tx_size_ && event_cb_)
                    event_cb_(Event::tx_complete, event_cb_user_);

                sock_state_ = SocketState::idle;
                free_pending();
            }
        }
    }

    Modem::Command::Command(int timeout_ms, const char *command, ...)
        : timeout_(timeout_ms)
    {
        va_list argp;

        char *data = reinterpret_cast<char*>(data_);

        va_start(argp, command);
        size_ = vsnprintf(data, kBufferSize, command, argp);
        va_end(argp);
    }
}
