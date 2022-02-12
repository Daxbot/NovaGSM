/**
 * @file modem.cpp
 * @brief GSM/GPRS modem driver.
 * @author Wilkins White
 * @copyright 2019-2021 Nova Dynamics LLC
 */

#include <cassert>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <errno.h>

#include "debug.h"
#include "modem.h"

/** How often to poll the modem (ms). */
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
            delete cmd_buffer_.front();
            cmd_buffer_.pop_front();
        }
    }

    void Modem::set_state_callback(
        void (*func)(State state, void *user), void *user)
    {
        state_cb_ = func;
        state_cb_user_ = user;
    }

    void Modem::set_event_callback(
        void (*func)(Event event, void *user), void *user)
    {
        event_cb_ = func;
        event_cb_user_ = user;
    }

    void Modem::set_error_callback(
        void (*func)(int error, void *user), void *user)
    {
        error_cb_ = func;
        error_cb_user_ = user;
    }

    int Modem::configure(const char *apn)
    {
        switch(device_state_) {
            case State::reset:
            case State::probe:
                return -ENODEV;
            default:
                break;
        }

        if(apn == nullptr || strlen(apn) >= sizeof(apn_))
            return -EINVAL;

        sprintf(apn_, "%s", apn);

        // AT+CGDCONT=1,"IP",[apn] - Define PDP context
        Command *cmd = Command::create(kDefaultTimeout,
            "AT+CGDCONT=1,\"IP\",\"%s\"\r", apn_);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        return result;
    }

    void Modem::process(int delta_us)
    {
        elapsed_us_ += delta_us;

        if(pending_) {
            const int last_size = response_size_;

            // Command pending - wait for response
            uint8_t *p_data = &response_[response_size_];
            int count = ctx_->read(p_data, (kBufferSize - response_size_));
            if(count > 0)
                response_size_ += count;

            // Parse response
            if(response_size_ > last_size) {
                if(device_state_ == State::reset) {
                    // Wait for timeout
                }
                else if(device_state_ == State::probe) {
                    process_probe();
                }
                else if(connected()) {
                    process_socket();
                }
                else if(authenticating()) {
                    process_authentication();
                }
                else if(handshaking()) {
                    process_handshaking();
                }
                else if(closing()) {
                    process_close();
                }
                else {
                    process_general();
                }
            }
            else if((int)(elapsed_us_ - command_timer_) > 0) {
                process_timeout();
            }
        }
        else if(cmd_buffer_.size() > 0) {
            pending_ = cmd_buffer_.front();
            cmd_buffer_.pop_front();

            // Send queued command
            ctx_->write(pending_->data(), pending_->size());

            command_timer_ = elapsed_us_ + (pending_->timeout() * 1000);
            response_size_ = 0;
        }
        else if(!update_timer_ || (int)(elapsed_us_ - update_timer_) > 0) {
            // Nothing queued - poll the modem
            update_timer_ = elapsed_us_ + (kPollingInterval * 1000);
            poll_modem();
        }
    }

    void Modem::reinit()
    {
        stop_send();
        stop_receive();
        free_pending();

        while(cmd_buffer_.size() > 0) {
            delete cmd_buffer_.front();
            cmd_buffer_.pop_front();
        }

        signal_ = 99;
        service_ = 0;
        creg_ = 0;
        cgreg_ = 0;
        cereg_ = 0;

        memset(ip_address_, '\0', sizeof(ip_address_));

        device_state_ = State::probe;
        sock_state_ = SocketState::idle;
        rx_available_ = 0;
        tx_available_ = 0;
    }

    int Modem::disable()
    {
        reinit();

        // AT+CFUN=0 - minimum functionality mode
        Command *cmd = Command::create(10000, "AT+CFUN=0\r");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_INFO("Powering off modem...\n");
        set_state(State::offline);
        return 0;
    }

    int Modem::reset()
    {
        reinit();

        // AT+CFUN=1,1 - reset phone module
        Command *cmd = Command::create(10000, "AT+CFUN=1,1\r");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        LOG_INFO("Resetting modem...\n");
        set_state(State::reset);
        return 0;
    }

    int Modem::unlock(const char *pin)
    {
        if(pin == nullptr)
            return -EINVAL;

        // AT+CPIN=[pin] - enter pin
        Command *cmd = Command::create(kDefaultTimeout,
            "AT+CPIN=\"%s\"\r", pin);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        return result;
    }

    int Modem::authenticate(const char *user, const char *pwd)
    {
        if(apn_[0] == '\0')
            return -EINVAL;

        if(user == nullptr)
            user = "";

        if(pwd == nullptr)
            pwd = "";

        switch(device_state_) {
            // Invalid state, return error
            case State::reset:
            case State::probe:
                return -ENODEV;
            case State::init:
            case State::offline:
            case State::locked:
            case State::searching:
                return -ENETUNREACH;
            case State::authenticating:
                return -EALREADY;
            case State::handshaking:
            case State::open:
            case State::closing:
                return -EBUSY;

            // Continue
            case State::registered:
            case State::ready:
                break;
        }

        // AT+CIPSHUT - reset GPRS context
        // AT+CIPMUX=0 - set single IP mode
        // AT+CIPRXGET=1 - set manual data receive
        // AT+CIPQSEND=1 - set quick send
        // AT+CSTT=[apn],[user],[pwd] - set apn/user/password for GPRS context
        Command *cmd = Command::create(65000,
            "AT+CIPSHUT;+CIPMUX=0;+CIPRXGET=1;+CIPQSEND=1;"
            "+CSTT=\"%s\",\"%s\",\"%s\"\r", apn_, user, pwd);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        // AT+CIICR - activate data connection
        cmd = Command::create(85000, "AT+CIICR\r");

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

    int Modem::connect(const char *host, int port)
    {
        if(host == nullptr)
            return -EINVAL;

        switch(device_state_) {
            // Invalid state, return error
            case State::reset:
            case State::probe:
                return -ENODEV;
            case State::init:
            case State::offline:
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
            case State::closing:
                return -EBUSY;

            // Continue
            case State::ready:
                break;
        }

        // AT+CIPSTART=[mode],[host],[port] - start a new connection
        Command *cmd = Command::create(75000,
            "AT+CIPSTART=\"TCP\",\"%s\",%d\r", host, port);

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

    int Modem::disconnect()
    {
        switch(device_state_) {
            // Invalid state, return error
            case State::reset:
            case State::probe:
                return -ENODEV;
            case State::init:
            case State::offline:
            case State::locked:
            case State::searching:
                return -ENETUNREACH;
            case State::registered:
            case State::authenticating:
            case State::ready:
            case State::handshaking:
                return -ENOTSOCK;
            case State::closing:
                return -EALREADY;

            // Continue
            case State::open:
                break;
        }

        // AT+CIPCLOSE - close connection
        Command *cmd = Command::create(kDefaultTimeout, "AT+CIPCLOSE\r");
        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        LOG_INFO("Closing TCP connection...\n");
        set_state(State::closing);
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

        // Refresh update timer
        update_timer_ = elapsed_us_ + (kPollingInterval * 1000);

        // Change state
        device_state_ = state;
        if(state_cb_)
            state_cb_(device_state_, state_cb_user_);
    }

    void Modem::free_pending()
    {
        if(pending_) {
            delete pending_;
            pending_ = nullptr;
        }

        response_size_ = 0;
    }

    bool Modem::response_complete()
    {
        if(memstr(response_, "\r\nOK\r\n", response_size_))
            return true;

        if(memstr(response_, "\r\nERROR\r\n", response_size_))
            return true;

        // Find CME errors
        void *cme = memstr(response_, "+CME ERROR:", response_size_);
        if(cme != nullptr) {
            int length = (static_cast<uint8_t*>(cme) - response_) + 2;
            if(memchr(cme, '\n', length))
                return true;
        }

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
                set_state(State::probe);
                cmd = Command::create(1000, "AT\r");
                break;
            case State::probe:
                cmd = Command::create(1000, "AT\r");
                break;
            case State::init:
            case State::locked:
                // AT+CFUN? - power status
                // AT+CPIN? - SIM status
                cmd = Command::create(10000, "AT+CFUN?;+CPIN?\r");
                break;
            case State::searching:
            case State::registered:
            case State::ready:
                // AT+CSQ - signal quality report
                // AT+CREG? - network registration status
                // AT+CGREG? - GPRS registration status
                // AT+CEREG? - EPS registration status
                // AT+CGATT? - GPRS service status
                cmd = Command::create(1000,
                    "AT+CSQ;+CREG?;+CGREG?;+CEREG?;+CGATT?\r");

                break;
            case State::open:
                // AT+CSQ - signal quality report
                // AT+CIPRXGET=4 - query socket unread bytes
                // AT+CIPSEND? - query available size of tx buffer
                // AT+CIPSTATUS - TCP connection status
                cmd = Command::create(1000,
                    "AT+CSQ;+CIPRXGET=4;+CIPSEND?;+CIPSTATUS\r");

                sock_state_ = SocketState::idle;
                break;
            case State::authenticating:
                // AT+CIFSR - get local IP address
                cmd = Command::create(1000, "AT+CIFSR\r");
                break;
            default:
                return 0;
        }

        if(cmd == nullptr)
            return -ENOMEM;

        int result = push_command(cmd);
        if(result)
            delete cmd;

        return result;
    }

    int Modem::socket_receive(int count)
    {
        const int available = std::min(rx_available_, kSocketMax);
        if(count > available)
            count = available;

        if(count <= 0)
            return 0;

        if(sock_state_ != SocketState::idle)
            return -EBUSY;

        // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
        Command *cmd = Command::create(kDefaultTimeout,
            "AT+CIPRXGET=2,%d\r", count);

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
        const int available = std::min(tx_available_, kSocketMax);
        if(count > available)
            count = available;

        if(count <= 0)
            return 0;

        if(sock_state_ != SocketState::idle)
            return -EBUSY;

        // AT+CIPSEND=[size] - indicate that data is about to be sent
        Command *cmd = Command::create(645000,
            "AT+CIPSEND=%d\r", count);

        if(cmd == nullptr)
            return -ENOMEM;

        int result = shift_command(cmd);
        if(result) {
            delete cmd;
            return result;
        }

        tx_pending_ = count;

        LOG_VERBOSE("RTS %d bytes (%d)\n", tx_pending_, (tx_size_ - tx_count_));
        sock_state_ = SocketState::rts;
        return 0;
    }

    void Modem::process_timeout()
    {
        free_pending();

        switch(device_state_) {
            case State::reset:
            case State::probe:
                // Ignore
                break;
            case State::authenticating:
                LOG_WARN("Authentication timeout.\n");
                set_state(State::searching);
                if(event_cb_)
                    event_cb_(Event::auth_error, event_cb_user_);
                break;
            case State::handshaking:
                LOG_WARN("TCP connection timeout.\n");
                set_state(State::ready);
                if(event_cb_)
                    event_cb_(Event::conn_error, event_cb_user_);
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
            case State::closing:
                LOG_WARN("Close timeout\n");
                set_state(State::ready);
                break;
            default:
                LOG_WARN("Command timeout\n");
                if(event_cb_)
                    event_cb_(Event::timeout, event_cb_user_);
                break;
        }
    }

    void Modem::process_probe()
    {
        // Wait for an 'OK' response
        if(!memstr(response_, "\r\nOK\r\n", response_size_))
            return;

        free_pending();

        // ATE0 - disable echo
        // AT+CNMP - preferred mode selection
        Command *cmd = Command::create(1000, "ATE0;+CNMP=%d;+CMEE=1\r", mode_);
        if(cmd == nullptr)
            return;

        if(push_command(cmd) != 0) {
            delete cmd;
            return;
        }

        set_state(State::init);
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

                    // +CREG: %d,%d\r\n
                    // │        │
                    // │        └ data
                    // └ start

                    creg_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
                else if(memstr(start, "+CGREG:", length)) {
                    void *data = memchr(start, ',', length);

                    // +CGREG: %d,%d\r\n
                    // │         │
                    // │         └ data
                    // └ start

                    cgreg_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
                else if(memstr(start, "+CEREG:", length)) {
                    void *data = memchr(start, ',', length);

                    // +CEREG: %d,%d\r\n
                    // │         │
                    // │         └ data
                    // └ start

                    cereg_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
                else if(memstr(start, "+CGATT:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CGATT: %d\r\n
                    // │     │
                    // │     └ data
                    // └ start

                    service_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
                }
                else if(memstr(start, "+CPIN: READY", length)) {
                    if(device_state_ < State::searching) {
                        LOG_INFO("Modem online\n");
                        set_state(State::searching);
                    }
                }
                else if(memstr(start, "+CPIN: SIM PUK", length)
                    || memstr(start, "+CPIN: SIM PIN", length)) {
                    if(device_state_ != State::locked) {
                        LOG_INFO("SIM locked\n");
                        set_state(State::locked);
                    }
                }
                else if(memstr(start, "+CPIN: NOT READY", length)) {
                    if(device_state_ != State::offline) {
                        LOG_INFO("Modem offline\n");
                        set_state(State::offline);
                    }
                }
                else if(memstr(start, "+CFUN:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CFUN: %d\r\n
                    // │    │
                    // │    └ data
                    // └ start

                    const int status = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);

                    if(status == 0 && device_state_ != State::offline) {
                        LOG_INFO("Modem offline\n");
                        set_state(State::offline);
                    }
                }
                else if(memstr(start, "+CME ERROR:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CME ERROR: %d\r\n
                    // │         │
                    // │         └ data
                    // └ start

                    const int error = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);

                    if(error_cb_)
                        error_cb_(error, error_cb_user_);
                }
            }

            start = end + 1;
            size -= length;
        }

        bool reg = false;

        if(creg_ == 1 || creg_ == 5)
            reg = true;  // GSM (2G)

        if(cgreg_ == 1 || cgreg_ == 5)
            reg = true;  // GPRS (2G/3G)

        if(cereg_ == 1 || cereg_ == 5)
            reg = true; // EPS (3G/LTE)

        if(signal_ != 99 && service_ && reg) {
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

        free_pending();
    }

    void Modem::process_authentication()
    {
        /**
         * Clear any pending 'OK' responses. We know that the authentication
         * is successful once we get an IP address from AT+CIFSR.
         */
        if(memstr(response_, "\r\nOK\r\n", response_size_)
            || memstr(response_, "\r\nSHUT OK\r\n", response_size_)) {
            free_pending();
            return;
        }

        if(memstr(response_, "\r\nERROR\r\n", response_size_)) {
            LOG_INFO("Authentication error\n");
            set_state(State::registered);
            free_pending();
            if(event_cb_)
                event_cb_(Event::auth_error, event_cb_user_);
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
                    if(event_cb_)
                        event_cb_(Event::conn_error, event_cb_user_);
                    break;
                }
            }

            start = end + 1;
            size -= length;
        }
    }

    void Modem::process_close()
    {
        // Parse lines
        uint8_t *start = response_;
        int size = response_size_;

        while(size > 4) {
            uint8_t *end = static_cast<uint8_t*>(memchr(start, '\n', size));
            if(end == nullptr)
                break;

            int length = (end - start) + 1;
            if(length > 4) {
                // Expected responses to AT+CIPCLOSE
                if(memstr(start, "CLOSE OK", length)) {
                    LOG_INFO("TCP socket closed\n");
                    set_state(State::ready);
                    free_pending();
                }
                else if(memstr(start, "ERROR", length)) {
                    LOG_INFO("Error during close\n");
                    set_state(State::ready);
                    free_pending();
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
        if(!response_complete())
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
                else if(memstr(start, "TCP CLOSED", length)) {
                    LOG_INFO("TCP socket disconnected\n");

                    stop_send();
                    stop_receive();

                    sock_state_ = SocketState::idle;
                    set_state(State::ready);
                    break;
                }
                else if(memstr(start, "CONNECT OK", length)) {
                    const int tx_requested = (tx_buffer_)
                        ? (tx_size_ - tx_count_) : 0;

                    const int rx_requested = (rx_buffer_)
                        ? (rx_size_ - rx_count_) : 0;

                    if(tx_requested && tx_available_) {
                        socket_send(tx_requested);
                    }
                    else if(rx_requested && rx_available_) {
                        socket_receive(rx_requested);
                    }

                    free_pending();
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
                }
                else if(memstr(start, "+CIPSEND:", length)) {
                    void *data = memchr(start, ':', length);

                    // +CIPSEND: %d\r\nOK\r\n
                    // │       │
                    // │       └ data
                    // └ start

                    tx_available_ = strtol(
                        static_cast<char*>(data)+1, nullptr, 10);
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
                else if(memstr(start, "ALREADY CONNECT", length)) {
                    free_pending();
                }
            }

            start = end + 1;
            size -= length;
        }
    }

    void Modem::socket_state_rtr()
    {
        if(memstr(response_, "ERROR", response_size_)) {
            LOG_ERROR("Error during receive\n");
            sock_state_ = SocketState::idle;
            free_pending();
            if(event_cb_)
                event_cb_(Event::rx_error, event_cb_user_);

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
                uint8_t *destination = rx_buffer_ + rx_count_;
                memcpy(destination, static_cast<uint8_t*>(data)+1, count);

                rx_count_ += count;
                if(rx_count_ >= rx_size_ && event_cb_)
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
            sock_state_ = SocketState::idle;
            free_pending();
            if(event_cb_)
                event_cb_(Event::tx_error, event_cb_user_);

            return;
        }

        if(memchr(response_, '>', response_size_) == nullptr)
            return;

        if(tx_buffer_) {
            // Send data
            const uint8_t *buffer = tx_buffer_ + tx_count_;
            ctx_->write(buffer, tx_pending_);

            LOG_VERBOSE("CTS %d bytes (%d)\n",
                tx_pending_, (tx_size_ - tx_count_));
        }
        else {
            /*
             * The send was canceled, but the modem is still
             * expecting data. Send zeroes until the buffer is
             * full.
             */
            uint8_t *buffer = static_cast<uint8_t*>(malloc(kBufferSize));
            memset(buffer, '\0', kBufferSize);

            ctx_->write(buffer, tx_pending_);

            free(buffer);
        }

        command_timer_ = elapsed_us_ + (kDefaultTimeout * 1000);
        sock_state_ = SocketState::cts;
    }

    void Modem::socket_state_cts()
    {
        if(memstr(response_, "CLOSED", response_size_)) {
            LOG_ERROR("TCP socket disconnected\n");
            sock_state_ = SocketState::idle;
            set_state(State::ready);
            free_pending();
            if(event_cb_)
                event_cb_(Event::tx_error, event_cb_user_);
            return;
        }

        if(memstr(response_, "ERROR", response_size_)) {
            LOG_ERROR("Send error\n");
            sock_state_ = SocketState::idle;
            free_pending();
            if(event_cb_)
                event_cb_(Event::tx_error, event_cb_user_);
            return;
        }

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
                if(tx_count_ >= tx_size_ && event_cb_)
                    event_cb_(Event::tx_complete, event_cb_user_);

                sock_state_ = SocketState::idle;
                free_pending();
            }
        }
    }

    Modem::Command::~Command()
    {
        if(data_ != nullptr)
            free(data_);
    }

    Modem::Command *Modem::Command::create(int timeout_ms, const char *command, ...)
    {
        char *buffer = static_cast<char*>(malloc(kBufferSize));
        if(buffer == nullptr)
            return nullptr;

        va_list argp;
        va_start(argp, command);
        int size = vsnprintf(buffer, kBufferSize, command, argp);
        va_end(argp);

        uint8_t *data = static_cast<uint8_t*>(realloc(buffer, size));
        if(data == nullptr) {
            free(buffer);
            return nullptr;
        }

        return new (std::nothrow) Command(timeout_ms, data, size);
    }
}
