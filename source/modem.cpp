/**
 * @file modem.cpp
 * @brief GSM/GPRS modem driver.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
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
static constexpr uint32_t kPollingInterval = 20;

/** How long to wait for a 'RDY' response before resetting the modem (ms). */
static constexpr uint32_t kReadyTimeout = 30000;

namespace gsm {

Modem::Modem(context_t &context) :
        ctx(context)
{
    parser.set_parse_callback(parse_callback, this);
}

Modem::~Modem()
{
    // Free command queue
    while (cmd_buffer.size() > 0) {
        delete cmd_buffer.front();
        cmd_buffer.pop();
    }
}

void Modem::set_state_callback(
        void (*func)(State state, void *user), void *user)
{
    state_cb = func;
    state_cb_user = user;
}

void Modem::set_event_callback(
        void (*func)(Event event, void *user), void *user)
{
    event_cb = func;
    event_cb_user = user;
}

void Modem::set_error_callback(
        void (*func)(int error, void *user), void *user)
{
    error_cb = func;
    error_cb_user = user;
}

void Modem::process()
{
    if (next_state != device_state) {
        // Refresh the update timer
        update_timer = millis() + kPollingInterval;

        // Trigger the state change
        device_state = next_state;
        LOG_VERBOSE("State set to %d\n", device_state);
        emit_state(device_state);
    }

    if (pending) {
        // Command pending - wait for response
        int count = read(buffer, kBufferSize);
        if (count > 0) {
            parser.load(buffer, count);
        }
        else if ((int32_t) (millis() - command_timer) > 0) {
            handle_timeout();
        }
    }
    else if (cmd_buffer.size() > 0) {
        pending = cmd_buffer.front();
        cmd_buffer.pop();

        // Send queued command
        write(pending->data(), pending->size());

        command_timer = millis() + pending->timeout();
    }
    else if ((int32_t) (millis() - update_timer) > 0) {
        // Nothing queued - poll the modem
        update_timer = millis() + kPollingInterval;
        poll_modem();
    }

    if (device_state == State::reset) {
        // We should receive 'RDY' once the modem can accept commands.
        // If this takes too long we should try resetting it again.
        if (reset_timer == 0)
            reset_timer = millis() + kReadyTimeout;
        else if ((int32_t) (millis() - reset_timer) > 0)
            reset();
    }
}

int Modem::reset()
{
    // Clear any queued commands
    while (cmd_buffer.size() > 0) {
        delete cmd_buffer.front();
        cmd_buffer.pop();
    }

    // AT+CFUN=1,1 - reset phone module
    Command *cmd = new Command(1000, "+CFUN=1,1");
    if (cmd == nullptr)
        return -ENOMEM;

    int result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    // Reset cached values
    modem_csq = 99;
    modem_cgatt = 0;
    modem_creg = 0;
    modem_cgreg = 0;
    modem_cereg = 0;

    memset(modem_cifsr, '\0', sizeof(modem_cifsr));

    // Reset socket
    stop_send();
    stop_receive();

    modem_rx_available = 0;
    modem_tx_available = 0;

    LOG_INFO("Resetting modem...\n");
    set_state(State::reset);
    reset_timer = 0;
    return 0;
}

int Modem::configure(const char *apn, uint8_t mode)
{
    switch (next_state) {
    // Invalid state, return error
    case State::reset:
        return -ENODEV;
        // Continue
    case State::ready:
    case State::error:
    case State::searching:
    case State::authenticating:
    case State::handshaking:
    case State::open:
    case State::closing:
    case State::registered:
    case State::online:
        break;

    }

    if (apn == nullptr)
        return -EINVAL;

    Command *cmd = new Command(5000);
    if (cmd == nullptr)
        return -ENOMEM;

    char buffer[64];
    int size = 0;

    // AT+CMEE=1 - enable numeric error codes
    cmd->add("+CMEE=1");

    // AT+CNMP=[mode] - preferred mode selection
    size = snprintf(buffer, sizeof(buffer), "+CNMP=%d", mode);
    if (size < 0)
        return size;

    cmd->add(buffer, size);

    // AT+CGDCONT=1,"IP",[apn] - Define PDP context
    size = snprintf(buffer, sizeof(buffer),
            "+CGDCONT=1,\"IP\",\"%s\"", apn);

    if (size < 0)
        return size;

    cmd->add(buffer, size);

    int result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    set_state(State::searching);
    return 0;
}

int Modem::authenticate(const char *apn, const char *user, const char *pwd)
{
    if (apn == nullptr)
        return -EINVAL;

    switch (next_state) {
    // Invalid state, return error
    case State::reset:
        return -ENODEV;
    case State::ready:
    case State::error:
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
    case State::online:
        break;
    }

    Command *cmd = new Command(65000);
    if (cmd == nullptr)
        return -ENOMEM;

    char buffer[64];
    int size = 0;

    // AT+CIPSHUT - reset GPRS context
    cmd->add("+CIPSHUT");

    // AT+CIPMUX=0 - set single IP mode
    cmd->add("+CIPMUX=0");

    // AT+CIPRXGET=1 - set manual data receive
    cmd->add("+CIPRXGET=1");

    // AT+CIPATS=1,%d - set auto sending timer
    cmd->add("+CIPATS=1,1");

    // AT+CSTT=[apn],[user],[pwd] - set apn/user/password for GPRS context
    if (user == nullptr) {
        size = snprintf(buffer, sizeof(buffer),
                "+CSTT=\"%s\"", apn);
    }
    else if (pwd == nullptr) {
        size = snprintf(buffer, sizeof(buffer),
                "+CSTT=\"%s\",\"%s\"", apn, user);
    }
    else {
        size = snprintf(buffer, sizeof(buffer),
                "+CSTT=\"%s\",\"%s\",\"%s\"", apn, user, pwd);
    }

    if (size < 0)
        return size;

    cmd->add(buffer, size);

    int result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    // AT+CIICR - activate data connection
    cmd = new Command(85000, "+CIICR");
    if (cmd == nullptr)
        return -ENOMEM;

    result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    LOG_INFO("Authenticating\n");
    set_state(State::authenticating);
    return 0;
}

int Modem::connect(const char *host, unsigned int port)
{
    if (host == nullptr || port == 0)
        return -EINVAL;

    switch (next_state) {
    // Invalid state, return error
    case State::reset:
        return -ENODEV;
    case State::ready:
    case State::error:
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
    case State::online:
        break;
    }

    Command *cmd = new Command(75000);
    if (cmd == nullptr)
        return -ENOMEM;

    char buffer[64];
    int size = snprintf(buffer, sizeof(buffer),
            "+CIPSTART=\"TCP\",\"%s\",%d", host, port);

    if (size < 0)
        return size;

    // AT+CIPSTART=[mode],[host],[port] - start a new connection
    cmd->add(buffer, size);

    int result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    LOG_INFO("Handshaking\n");
    set_state(State::handshaking);
    return 0;
}

int Modem::close(bool quick)
{
    switch (next_state) {
    // Invalid state, return error
    case State::reset:
        return -ENODEV;
    case State::ready:
    case State::error:
    case State::searching:
        return -ENETUNREACH;
    case State::registered:
    case State::authenticating:
    case State::online:
    case State::handshaking:
        return -ENOTSOCK;
    case State::closing:
        return -EALREADY;

        // Continue
    case State::open:
        break;
    }

    Command *cmd = nullptr;
    if (quick) {
        // Kill the connection and return immediately
        cmd = new Command(kDefaultTimeout, "+CIPCLOSE=1");
    }
    else {
        // Wait for server to acknowledge the close request
        cmd = new Command(30000, "+CIPCLOSE");
    }

    if (cmd == nullptr)
        return -ENOMEM;

    int result = push_command(cmd);
    if (result)
        delete cmd;

    LOG_INFO("Closing TCP connection...\n");
    set_state(State::closing);
    return result;
}

void Modem::receive(void *data, size_t size)
{
    rx_buffer = static_cast<uint8_t*>(data);
    rx_size = size;
    rx_index = 0;
}

void Modem::stop_receive()
{
    const bool stopped = rx_busy();
    receive(nullptr, 0);
    if (stopped) {
        LOG_WARN("Receive interrupted\n");
        emit_event(Event::rx_complete);
    }
}

void Modem::send(const void *data, size_t size)
{
    tx_buffer = static_cast<const uint8_t*>(data);
    tx_size = size;
    tx_index = 0;
}

void Modem::stop_send()
{
    const bool stopped = tx_busy();
    send(nullptr, 0);
    if (stopped) {
        LOG_WARN("Send interrupted\n");
        emit_event(Event::tx_complete);
    }
}

void Modem::set_state(State state)
{
    next_state = state;
}

void Modem::free_pending()
{
    assert(pending != nullptr);
    delete pending;
    pending = nullptr;
}

int Modem::push_command(Command *cmd)
{
    if (cmd == nullptr)
        return -EINVAL;

    if (cmd->size() >= kBufferSize)
        return -EMSGSIZE;

    cmd_buffer.push(cmd);
    return 0;
}

int Modem::poll_modem()
{
    Command *cmd = nullptr;

    switch (status()) {
    case State::reset:
        case State::ready:
        cmd = new Command(1000); // AT
        update_timer = millis() + 1000;
        break;
    case State::searching:
    case State::registered:
    case State::online:
        cmd = new Command(10000);
        if (cmd != nullptr) {
            // AT+CSQ - signal quality report
            cmd->add("+CSQ");
            // AT+CREG? - network registration status
            cmd->add("+CREG?");
            // AT+CGREG? - GPRS registration status
            cmd->add("+CGREG?");
            // AT+CEREG? - EPS registration status
            cmd->add("+CEREG?");
            // AT+CGATT? - GPRS service status
            cmd->add("+CGATT?");
        }
        break;
    case State::authenticating:
        // AT+CIFSR - get local IP address
        cmd = new Command(1000, "+CIFSR");
        cifsr_flag = true;
        break;
    case State::open:
        return poll_socket();
    case State::handshaking:
    case State::error:
    case State::closing:
        return 0;
    }

    if (cmd == nullptr)
        return -ENOMEM;

    int result = push_command(cmd);
    if (result != 0) {
        delete cmd;
        return result;
    }

    return 0;
}

int Modem::poll_socket()
{
    const int rx_requested = (rx_buffer) ? (rx_size - rx_index) : 0;
    const int tx_requested = (tx_buffer) ? (tx_size - tx_index) : 0;

    if (rx_requested && modem_rx_available) {
        int result = socket_receive(rx_requested);
        if(result < 0)
            return result;

        // SocketState::receive will be set when CIPRXGET returns
    }
    else if (tx_requested && modem_tx_available) {
        int result = socket_send(tx_buffer + tx_index, tx_requested);
        if (result < 0)
            return result;

        sock_state = SocketState::send;
    }
    else {
        Command *cmd = new Command(1000);
        if (cmd == nullptr)
            return -ENOMEM;

        // AT+CSQ - signal quality report
        cmd->add("+CSQ");
        // AT+CIPRXGET=4 - query socket unread bytes
        cmd->add("+CIPRXGET=4");
        // AT+CIPSEND? - query available size of tx buffer
        cmd->add("+CIPSEND?");

        int result = push_command(cmd);
        if (result != 0) {
            delete cmd;
            return result;
        }
    }

    return 0;
}

int Modem::socket_receive(size_t size)
{
    if (sock_state != SocketState::command)
        return -EBUSY;

    const size_t available = std::min(modem_rx_available, kSocketMax);
    if (size > available)
        size = available;

    if (size <= 0)
        return 0;

    Command *cmd = new Command();
    if (cmd == nullptr)
        return -ENOMEM;

    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "+CIPRXGET=2,%d", size);
    if (len < 0)
        return len;

    // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
    cmd->add(buffer, len);

    int result = push_command(cmd);
    if (result) {
        delete cmd;
        return result;
    }

    LOG_VERBOSE("RTR %d bytes (%d)\n", size, modem_rx_available);
    return size;
}

int Modem::socket_send(const uint8_t *data, size_t size)
{
    if (sock_state != SocketState::command)
        return -EBUSY;

    const size_t available = std::min(modem_tx_available, kSocketMax);
    if (size > available)
        size = available;

    if (size > 0) {
        Command *cmd = new Command(1000);
        if (cmd == nullptr)
            return -ENOMEM;

        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "+CIPSEND=%d", size);
        if (len < 0)
            return len;

        // AT+CIPSEND=[size] - indicate that data is about to be sent
        cmd->add(buffer, len);

        int result = push_command(cmd);
        if (result) {
            delete cmd;
            return result;
        }

        // Send the data
        std::vector<uint8_t> payload(size);
        memcpy(payload.data(), data, size);

        cmd = new Command(10000, payload);
        if (cmd == nullptr)
            return -ENOMEM;

        result = push_command(cmd);
        if (result) {
            delete cmd;
            return result;
        }
    }

    LOG_VERBOSE("RTS %d bytes (%d)\n", size, (tx_size - tx_index));
    return size;
}

void Modem::handle_timeout()
{
    // Ignore timeouts for 'AT\r'
    const bool ignored = (pending->size() == 3);

    free_pending();

    if (ignored)
        return;

    switch (device_state) {
    case State::reset:
        case State::ready:
        break;
    case State::authenticating:
        LOG_WARN("Authentication timeout\n");
        set_state(State::searching);
        emit_event(Event::auth_error);
        break;
    case State::handshaking:
        LOG_WARN("TCP connection timeout\n");
        set_state(State::online);
        emit_event(Event::conn_error);
        break;
    case State::open:
        LOG_WARN("Socket timeout\n");
        emit_event(Event::sock_error);
        break;
    case State::closing:
        LOG_WARN("Close timeout\n");
        set_state(State::online);
        break;
    default:
        LOG_WARN("Command timeout\n");
        emit_event(Event::timeout);
        break;
    }
}

bool Modem::parse_urc(uint8_t *start, size_t size)
{
    if (size >= 12 && memcmp(start, "+CME ERROR:", 12) == 0) {
        // +CME ERROR: %d\r\n
        // │           │
        // │           └ start + 12
        // └ start

        const int error = strtoul(
                reinterpret_cast<char*>(start + 12), nullptr, 10);

        LOG_ERROR("+CME ERROR: %d", error);
        emit_error(error);
        return true;
    }
    else if (size >= 7 && memcmp(start, "+CPIN: ", 7) == 0) {
        // +CPIN: %s\r\n
        // │      │
        // │      └ start + 7
        // └ start

        start += 7;
        size -= 7;

        if (size >= 6 && memcmp(start, "READY\r", 6) == 0) {
            if (status() < State::searching)
                set_state(State::ready);
        }
        else if (size >= 13 && memcmp(start, "NOT INSERTED\r", 13) == 0) {
            LOG_ERROR("SIM card is not inserted\n");
            emit_event(Event::sim_error);
            set_state(State::error);
        }
        return true;
    }
    else if (size >= 7 && memcmp(start, "+CFUN: ", 7) == 0) {

        // +CFUN: %d\r\n
        // │      │
        // │      └ start + 7
        // └ start

        modem_cfun = strtoul(
                reinterpret_cast<char*>(start + 7), nullptr, 10);

        if (modem_cfun != 1) {
            LOG_WARN("Modem offline\n");
            set_state(State::error);
        }
        return true;
    }
    else if (size >= 12 && memcmp(start, "+PDP: DEACT\r", 12) == 0) {
        if (status() > State::registered) {
            set_state(State::registered);
        }
        return true;
    }

    return false;
}

void Modem::parse_general(uint8_t *start, size_t size)
{
    if (size >= 6 && memcmp(start, "+CSQ: ", 6) == 0) {
        // +CSQ: %d,%d\r\n
        // │     │
        // │     └ start + 6
        // └ start

        start += 6;
        size -= 6;

        modem_csq = strtoul(reinterpret_cast<char*>(start), nullptr, 10);
    }
    else if (size >= 7 && memcmp(start, "+CREG: ", 7) == 0) {
        // +CREG: %d,%d\r\n
        // │        │
        // │        └ data
        // └ start

        char *data = static_cast<char*>(memchr(start, ',', size));
        if (data != nullptr)
            modem_creg = strtoul(data + 1, nullptr, 10);
    }
    else if (size >= 8 && memcmp(start, "+CGREG: ", 8) == 0) {
        // +CGREG: %d,%d\r\n
        // │         │
        // │         └ data
        // └ start

        char *data = static_cast<char*>(memchr(start, ',', size));
        if (data != nullptr)
            modem_cgreg = strtoul(data + 1, nullptr, 10);
    }
    else if (size >= 8 && memcmp(start, "+CEREG: ", 8) == 0) {
        // +CEREG: %d,%d\r\n
        // │         │
        // │         └ data
        // └ start

        char *data = static_cast<char*>(memchr(start, ',', size));
        if (data != nullptr)
            modem_cereg = strtoul(data + 1, nullptr, 10);
    }
    else if (size >= 8 && memcmp(start, "+CGATT: ", 8) == 0) {
        // +CGATT: %d\r\n
        // │       │
        // │       └ start + 8
        // └ start

        start += 8;
        size -= 8;

        modem_cgatt = strtoul(reinterpret_cast<char*>(start), nullptr, 10);
    }

    bool reg = false;

    if (modem_creg == 1 || modem_creg == 5)
        reg = true;  // GSM (2G)

    if (modem_cgreg == 1 || modem_cgreg == 5)
        reg = true;  // GPRS (2G/3G)

    if (modem_cereg == 1 || modem_cereg == 5)
        reg = true; // EPS (3G/LTE)

    if (modem_cgatt && reg) {
        if (status() < State::registered) {
            LOG_INFO("Registered\n");
            set_state(State::registered);
        }
    }
    else {
        if (status() >= State::registered) {
            LOG_INFO("Searching for network\n");
            set_state(State::searching);
        }
    }
}

void Modem::parse_authentication(uint8_t *start, size_t size)
{
    /**
     * Clear any pending 'OK' responses. We know that the authentication
     * is successful once we get an IP address from AT+CIFSR.
     */
    if (size >= 3 && memcmp(start, "OK\r", 3) == 0) {
        free_pending();
    }
    else if (size >= 6 && memcmp(start, "ERROR\r", 6) == 0) {
        LOG_INFO("Authentication error\n");
        set_state(State::registered);
        free_pending();
        emit_event(Event::auth_error);
    }
    else if (cifsr_flag) {
        // Parse IP address from AT+CIFSR
        unsigned int a, b, c, d;
        char *data = reinterpret_cast<char*>(start);
        if (sscanf(data, "%3u.%3u.%3u.%3u", &a, &b, &c, &d) == 4) {
            // Store IP address
            snprintf(modem_cifsr, sizeof(modem_cifsr),
                    "%u.%u.%u.%u", a, b, c, d);

            LOG_INFO("Connected to GPRS\n");
            set_state(State::online);
            free_pending();
            cifsr_flag = false;
        }
    }
}

void Modem::parse_handshaking(uint8_t *start, size_t size)
{
    // Expected responses to AT+CIPSTART=...
    if (size >= 11 && memcmp(start, "CONNECT OK\r", 11) == 0) {
        LOG_INFO("TCP socket connected\n");
        sock_state = SocketState::command;
        set_state(State::open);
        free_pending();
    }
    else if (size >= 16 && memcmp(start, "ALREADY CONNECT\r", 16) == 0) {
        LOG_INFO("TCP socket reconnected\n");
        set_state(State::open);
        free_pending();
    }
    else if (size >= 13 && memcmp(start, "CONNECT FAIL\r", 13) == 0) {
        LOG_WARN("TCP connection failed\n");
        set_state(State::online);
        emit_event(Event::conn_error);
        free_pending();
    }
}

void Modem::parse_closing(uint8_t *start, size_t size)
{
    // Expected responses to AT+CIPCLOSE
    if (size >= 8 && memcmp(start, "CLOSE OK", 8) == 0) {
        LOG_INFO("TCP socket closed\n");
        set_state(State::online);
        free_pending();
    }
    else if (size >= 6 && memcmp(start, "ERROR\r", 6) == 0) {
        LOG_INFO("Error during close\n");
        set_state(State::online);
        free_pending();
    }
}

void Modem::parse_socket(uint8_t *start, size_t size)
{
    switch (sock_state) {
    case SocketState::command:
        parse_socket_command(start, size);
        break;
    case SocketState::receive:
        parse_socket_receive(start, size);
        break;
    case SocketState::send:
        parse_socket_send(start, size);
        break;
    }
}

void Modem::parse_socket_command(uint8_t *start, size_t size)
{
    if (size >= 3 && memcmp(start, "OK\r", 3) == 0) {
        free_pending();
    }
    else if (size >= 6 && memcmp(start, "ERROR\r", 6) == 0) {
        LOG_INFO("Socket error\n");
        free_pending();
        emit_event(Event::sock_error);
    }
    else if (size >= 11 && memcmp(start, "TCP CLOSED\r", 11) == 0) {
        LOG_INFO("TCP socket disconnected\n");

        stop_send();
        stop_receive();

        sock_state = SocketState::command;
        set_state(State::online);
    }
    else if (size >= 13 && memcmp(start, "+CIPRXGET: 4,", 13) == 0) {
        // +CIPRXGET: 4,%d\r\nOK\r\n
        // │            │
        // │            └ start + 13
        // └ start

        start += 13;
        size -= 13;

        const size_t count = strtoul(
                reinterpret_cast<char*>(start), nullptr, 10);

        if (count > modem_rx_available)
            emit_event(Event::new_data);

        modem_rx_available = count;
    }
    else if (size >= 13 && memcmp(start, "+CIPRXGET: 2,", 13) == 0) {
        // +CIPRXGET: 2,%d,%d,%s\r\n%s\r\n
        // │            │
        // │            └ start + 13
        // └ start

        start += 13;
        size -= 13;

        modem_rx_pending = strtoul(
                reinterpret_cast<char*>(start), nullptr, 10);

        modem_rx_available -= modem_rx_pending;
        sock_state = SocketState::receive;
    }
    else if (size >= 10 && memcmp(start, "+CIPSEND: ", 10) == 0) {
        // +CIPSEND: %d\r\nOK\r\n
        // │       │
        // │       └ data
        // └ start

        start += 10;
        size -= 10;

        modem_tx_available = strtoul(
                reinterpret_cast<char*>(start), nullptr, 10);


    }
}

void Modem::parse_socket_receive(uint8_t *start, size_t size)
{
    size_t count = modem_rx_pending;
    if (count > size)
        count = size;

    modem_rx_pending -= count;

    if (rx_buffer && rx_index < rx_size) {
        if (count > (rx_size - rx_index))
            count = (rx_size - rx_index);

        memcpy(rx_buffer + rx_index, start, count);
        rx_index += count;

        LOG_INFO("Received %d bytes\n", count);

        if (rx_index == rx_size)
            emit_event(Event::rx_complete);
    }
    else {
        LOG_WARN("Discarded %d bytes\n", count);
    }

    if (modem_rx_pending == 0)
        sock_state = SocketState::command;
}

void Modem::parse_socket_send(uint8_t *start, size_t size)
{
    if (size == 1 && *start == '>') {
        // Send prompt
        free_pending();
    }
    else if (size >= 3 && memcmp(start, "OK\r", 3) == 0) {
        free_pending();
    }
    else if (size >= 6 && memcmp(start, "ERROR\r", 6) == 0) {
        LOG_INFO("Socket error\n");
        free_pending();
        emit_event(Event::sock_error);
    }
    else if (size >= 8 && memcmp(start, "SEND OK\r", 8) == 0) {
        // Response to AT+CIPSEND
        const size_t count = pending->size();
        tx_index += count;

        LOG_INFO("Sent %d bytes\n", count);

        if (tx_index == tx_size)
            emit_event(Event::rx_complete);

        sock_state = SocketState::command;
        free_pending();
    }
    else if (size >= 10 && memcmp(start, "SEND FAIL\r", 10) == 0) {
        // Response to AT+CIPSEND
        sock_state = SocketState::command;
        emit_event(Event::sock_error);
        free_pending();
    }
}

void Modem::parse_callback(uint8_t *start, size_t size, void *user)
{
    Modem *ctx = static_cast<Modem*>(user);

#if (NOVAGSM_DEBUG >= NOVAGSM_DEBUG_TRACE)
    if(ctx->sock_state != SocketState::receive) {
        char dbg_buffer[128];
        size_t dbg_index = 0;

        for(size_t i = 0; i < size; ++i) {
            const char c = start[i];
            if(c == '\n')
                continue;

            if(c == '\r') {
                if((dbg_index + 2) >= sizeof(dbg_buffer))
                    break;

                dbg_buffer[dbg_index++] = '\\';
                dbg_buffer[dbg_index++] = 'r';
            }
            else {
                if((dbg_index + 1) >= sizeof(dbg_buffer))
                    break;

                dbg_buffer[dbg_index++] = c;
            }
        }

        dbg_buffer[dbg_index] = '\0';
        LOG_TRACE("%s\n", dbg_buffer);
    }
#endif

    // Discard echo
    if (size >= 2 && memcmp(start, "AT", 2) == 0) {
        if (ctx->status() != State::reset) {
            // ATE0 - disable echo
            Command *cmd = new Command(1000, "E0");
            if (ctx->push_command(cmd) != 0)
                delete cmd;
        }
        return;
    }

    // Unsolicited Result Codes
    if(ctx->parse_urc(start, size))
        return;

    // State specific responses
    switch (ctx->status()) {
    case State::authenticating:
        ctx->parse_authentication(start, size);
        break;
    case State::handshaking:
        ctx->parse_handshaking(start, size);
        break;
    case State::open:
        ctx->parse_socket(start, size);
        break;
    case State::closing:
        ctx->parse_closing(start, size);
        break;
    default:
        if (size >= 3 && memcmp(start, "OK\r", 3) == 0) {
            ctx->free_pending();
        }
        break;
    }

    // All other responses
    ctx->parse_general(start, size);
}

} // namespace gsm
