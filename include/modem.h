/**
 * @file modem.h
 * @brief GSM/GPRS modem driver.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
 */

#ifndef NOVAGSM_MODEM_H_
#define NOVAGSM_MODEM_H_

#include <cstdint>
#include <queue>

#include "command.h"
#include "parser.h"

/** Handles buffered communication through a GSM/GPRS modem. */
namespace gsm {

static_assert(kBufferSize > 64);

/**
 * @brief Maximum size of socket data transfers.
 *
 * Each data chunk must be less than the actual buffer size to account for
 * protocol overhead.
 */
constexpr size_t kSocketMax = (kBufferSize - 64);

/**
 * @brief Defines resources and callbacks used by the driver.
 *
 * The API expects a *non-blocking* buffered
 * implementation, e.g. unistd or Arduino.
 *
 *   https://linux.die.net/man/2/read
 *   https://linux.die.net/man/2/write
 */
typedef struct {
    /**
     * @brief Read 'size' bytes into 'data' from stream.
     *
     * @param [out] data - buffer to read into.
     * @param [in] size - number of bytes to read.
     * @return number of bytes read.
     */
    int (*read)(void *data, size_t size);

    /**
     * @brief Write 'size' bytes from 'data' to stream.
     *
     * @param [in] data - buffer to write.
     * @param [in] size - number of bytes to write.
     * @return number of bytes written.
     */
    int (*write)(const void *data, size_t size);

    /**
     * @brief Returns number of milliseconds elapsed since program start.
     */
    uint32_t (*millis)();
} context_t;

/**
 * @brief State of the modem.
 * @see set_state_callback().
 */
enum class State {
    reset, /**< 0x0 - Waiting for reset. */
    ready, /**< 0x1 - Modem is ready to receive AT commands. */
    error, /**< 0x2 - Modem is in an error state. */
    searching, /**< 0x3 - Searching for the network. */
    registered, /**< 0x4 - Network registers the modem. */
    authenticating, /**< 0x5 - Attempting to establish GPRS connection. */
    online, /**< 0x6 - Data connection active. */
    handshaking, /**< 0x7 - Attempting to establish TCP connection. */
    open, /**< 0x8 - TCP socket is open. */
    closing, /**< 0x9 - TCP socket is closing. */
};

/**
 * @brief Modem events.
 * @see set_event_callback().
 */
enum class Event {
    timeout, /**< A command timed out. */
    sim_error, /**< There is a problem with he SIM card. */
    auth_error, /**< An error occurred during authenticate(). */
    conn_error, /**< An error occurred during connect(). */
    sock_error, /**< An error occurred during a socket operation. */
    new_data, /**< New data available for read(). */
    rx_complete, /**< A read command has finished. */
    tx_complete, /**< A write command has finished. */
};

/** Class representing the connection with a GSM/GPRS modem. */
class Modem {
public:
    /**
     * @brief Constructor.
     *
     * @param [in] context - hardware specific callbacks for the driver.
     */
    Modem(context_t &context);

    /** Destructor. */
    ~Modem();

    /**
     * @brief Set a function to be called on state changes.
     *
     * @param [in] func - function to be called.
     * @param [in] user - pointer to be passed when 'func' is called.
     */
    void set_state_callback(
            void (*func)(State state, void *user), void *user = nullptr);

    /**
     * @brief Set a function to be called on a modem event.
     *
     * @param [in] func - function to be called.
     * @param [in] user - pointer to be passed when 'func' is called.
     */
    void set_event_callback(
            void (*func)(Event event, void *user), void *user = nullptr);

    /**
     * @brief Set a function to be called on a modem error (+CME ERROR).
     *
     * @param [in] func - function to be called.
     * @param [in] user - pointer to be passed when 'func' is called.
     */
    void set_error_callback(
            void (*func)(int error, void *user), void *user = nullptr);

    /**
     * @brief Handle communication with the modem.
     *
     * The process method handles communication with the modem and
     * controls the device state. If there is not a command
     * already awaiting a response then the next command in the buffer
     * is sent. Responses are handled based on the gsm::State.
     */
    void process();

    /**
     * @brief Reset the modem (+CFUN=1,1).
     *
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int reset();

    /**
     * @brief Configure the GPRS context.
     *
     * Must be in State::ready, transitions to State::registered.
     *
     * @param [in] apn - access point name.
     * @param [in] mode - CNMP mode (default LTE)
     * @return -EINVAL if 'apn' is null or larger than 63 bytes.
     * @return -ENODEV if the device is not responsive.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int configure(const char *apn, uint8_t mode = 38);

    /**
     * @brief Connect to GPRS.
     *
     * Must be in State::registered, transitions to State::online.
     *
     * @param [in] apn - access point name.
     * @param [in] user - access point user name.
     * @param [in] pwd - access point password.
     * @return -EINVAL if the GPRS context has not been configured.
     * @return -ENODEV if the device is not responsive.
     * @return -ENETUNREACH if the network is not available.
     * @return -EALREADY if authentication is already in progress.
     * @return -EBUSY if the socket is open.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int authenticate(
            const char *apn,
            const char *user = nullptr,
            const char *pwd = nullptr);

    /**
     * @brief Open a TCP socket.
     *
     * Must be in State::online, transitions to State::open.
     *
     * @param [in] host - server ip address.
     * @param [in] port - server port number.
     * @return -EINVAL if inputs are null.
     * @return -ENODEV if the device is not responsive.
     * @return -ENETUNREACH if the network is not available.
     * @return -ENOTCONN if GPRS is not connected.
     * @return -EALREADY if handshaking is already in progress.
     * @return -EADDRINUSE if a socket is already open.
     * @return -EBUSY disconnecting previous socket - try again.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int connect(const char *host, unsigned int port);

    /**
     * @brief Close TCP socket.
     *
     * @param [in] quick - if true, then close immediately.
     * @return -ENODEV if the device is not responsive.
     * @return -ENETUNREACH if the network is not available.
     * @return -ENOTSOCK if a connection is not established.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int close(bool quick = false);

    /**
     * @brief Start receiving data.
     *
     * Asynchronously receives up to 'size' bytes from the modem
     * and stores it in the buffer pointed to by 'data'.
     *
     * Function returns immediately and calls the socket
     * callback with the result.
     *
     * @param [out] data - buffer to read into.
     * @param [in] size - number of bytes to read.
     */
    void receive(void *data, size_t size);

    /**
     * @brief Cancel an ongoing receive() call.
     *
     * @warning if transfer is in progress the data will be lost.
     */
    void stop_receive();

    /**
     * @brief Stage bytes to the tx ring buffer.
     *
     * @param [in] data - buffer to write.
     * @param [in] size - number of bytes to write.
     */
    void send(const void *data, size_t size);

    /**
     * @brief Cancel an ongoing send() call.
     *
     * @warning if transfer is in progress '\0' will be sent for the
     * remaining bytes.
     */
    void stop_send();

    /**
     * @brief The number of bytes available to receive().
     */
    inline int rx_available() const
    {
        return modem_rx_available;
    }

    /**
     * @brief Number of bytes available to send().
     *
     * Buffers larger than this will be broken up and sent over
     * multiple transfers.
     */
    inline int tx_available() const
    {
        return modem_tx_available;
    }

    /**
     * @brief Poll the status of the last receive() call.
     *
     * @return true if receive is in progress.
     */
    inline bool rx_busy() const
    {
        return connected() && rx_buffer && rx_index < rx_size;
    }

    /**
     * @brief Poll the status of the last send() call.
     *
     * @return true if send is in progress.
     */
    inline bool tx_busy() const
    {
        return connected() && tx_buffer && tx_index < tx_size;
    }

    /**
     * @brief Poll the status of the last receive() call
     *
     * @return number of bytes received.
     */
    inline size_t rx_count() const
    {
        return (rx_buffer) ? rx_index : 0;
    }

    /**
     * @brief Poll the status of the last send() call
     *
     * @return number of bytes sent.
     */
    inline size_t tx_count() const
    {
        return (tx_buffer) ? tx_index : 0;
    }

    /**
     * @brief Return the device state.
     */
    inline State status() const
    {
        return next_state;
    }

    /**
     * @brief Returns true if the modem is registered on the network.
     */
    inline bool registered() const
    {
        return status() >= State::registered;
    }

    /**
     * @brief Returns true if authentication is in progress.
     */
    inline bool authenticating() const
    {
        return status() == State::authenticating;
    }

    /**
     * @brief Returns true if the modem is online.
     */
    inline bool online() const
    {
        return status() >= State::online;
    }

    /**
     * @brief Returns true if a connection attempt is in progress.
     */
    inline bool handshaking() const
    {
        return status() == State::handshaking;
    }

    /**
     * @brief Returns true if the connection is being closed.
     */
    inline bool closing() const
    {
        return status() == State::closing;
    }

    /**
     * @brief Returns true if a TCP connection is established.
     */
    inline bool connected() const
    {
        return status() == State::open;
    }

    /**
     * @brief Returns the value reported by [AT+CSQ].
     */
    inline uint8_t csq() const
    {
        return modem_csq;
    }

    /**
     * @brief Returns the value reported by [AT+CREG?]
     */
    inline uint8_t creg() const
    {
        return modem_creg;
    }

    /**
     * @brief Returns the value reported by [AT+CGREG?]
     */
    inline uint8_t cgreg() const
    {
        return modem_cgreg;
    }

    /**
     * @brief Returns the value reported by [AT+CGREG?]
     */
    inline uint8_t cereg() const
    {
        return modem_cereg;
    }

    /**
     * @brief Returns the value reported by [AT+CGATT?]
     */
    inline uint8_t cgatt() const
    {
        return modem_cgatt;
    }

    /**
     * @brief Returns the value reported by [AT+CIFSR]
     */
    inline const char *cifsr() const
    {
        return modem_cifsr;
    }

private:
    /** State of data transmission when socket is open. */
    enum class SocketState {
        command, /**< AT command state. */
        receive, /**< Receiving data. */
        send, /**< Sending data. */
    };

    /**
     * @brief Process a completed packet.
     *
     * @param [in] data - parsed packet.
     * @param [in] size - size of the parsed packet.
     * @param [in] user - private data.
     */
    static void parse_callback(uint8_t *data, size_t size, void *user);

    /**
     * @brief Update the device state and call user function.
     * @param [in] state - new device state.
     */
    void set_state(State state);

    /** Free the pending command. */
    void free_pending();

    /**
     * @brief Add a command to the end of the queue.
     *
     * @param [in] cmd - Command object.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int push_command(Command *cmd);

    /**
     * @brief Send a polling message based on the modem's state.
     *
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int poll_modem();

    /**
     * @brief Send a polling message based on the socket state.
     *
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int poll_socket();

    /**
     * @brief Read data from the socket.
     *
     * @param [in] count - the number of bytes to send.
     * @return -EBUSY if socket is not idle.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int socket_receive(size_t count);

    /**
     * @brief Write data to the socket.
     *
     * @param [in] data - buffer to send.
     * @param [in] count - the number of bytes to send.
     * @return -EBUSY if socket is not idle.
     * @return -ENOMEM if memory allocation failed.
     * @return -EMSGSIZE if buffer size exceeded.
     */
    int socket_send(const uint8_t *data, size_t count);

    /** Handle a command timeout. */
    void handle_timeout();

    /** Handle unsolicited result codes. */
    bool parse_urc(uint8_t *start, size_t size);

    /** Handle general command responses. */
    void parse_general(uint8_t *start, size_t size);

    /** Handle authenticate(). */
    void parse_authentication(uint8_t *start, size_t size);

    /** Handle connect(). */
    void parse_handshaking(uint8_t *start, size_t size);

    /** Handle disconnect(). */
    void parse_closing(uint8_t *start, size_t size);

    /** Handle socket. */
    void parse_socket(uint8_t *start, size_t size);

    /** Socket is idle. */
    void parse_socket_command(uint8_t *start, size_t size);

    /** Socket data is being read into the receive buffer. */
    void parse_socket_receive(uint8_t *start, size_t size);

    /** Data is being written from the send buffer. */
    void parse_socket_send(uint8_t *start, size_t size);

    /** Invoke the state callback .*/
    inline void emit_state(State state)
    {
        if (state_cb)
            state_cb(state, state_cb_user);

    }

    /** Invoke the event callback. */
    inline void emit_event(Event event)
    {
        if (event_cb)
            event_cb(event, event_cb_user);
    }

    /** Invoke the error callback. */
    inline void emit_error(int code)
    {
        if (error_cb)
            error_cb(code, event_cb_user);
    }

    inline uint32_t millis() const
    {
        return ctx.millis();
    }

    inline int read(void *data, size_t size) const
    {
        return ctx.read(data, size);
    }

    inline int write(const void *data, size_t size) const
    {
        return ctx.write(data, size);
    }

    /** Driver operating context. */
    const context_t &ctx;

    /** User function to call on state change event. */
    void (*state_cb)(State state, void *user) = nullptr;

    /** User private data for state change callback. */
    void *state_cb_user = nullptr;

    /** User function to call on event. */
    void (*event_cb)(Event event, void *user) = nullptr;

    /** User private data for event callback. */
    void *event_cb_user = nullptr;

    /** User function to call on +CME ERROR. */
    void (*error_cb)(int error, void *user) = nullptr;

    /** User private data for error callback. */
    void *error_cb_user = nullptr;

    /** Receive buffer. */
    uint8_t buffer[kBufferSize];

    /** Modem functional state reported by AT+CFUN? */
    uint8_t modem_cfun = 0;

    /** Signal value reported by AT+CSQ. */
    uint8_t modem_csq = 99;

    /** Registration value reported by AT+CREG? */
    uint8_t modem_creg = 0;

    /** Registration value reported by AT+CGREG? */
    uint8_t modem_cgreg = 0;

    /** Registration value reported by AT+CEREG? */
    uint8_t modem_cereg = 0;

    /** GPRS service status reported by AT+CGATT? */
    uint8_t modem_cgatt = 0;

    /** Local IP address reported by AT+CIFSR. */
    char modem_cifsr[32];

    /** The next valid line will be the CIFSR results. */
    bool cifsr_flag = false;

    /** Command queue. */
    std::queue<Command*> cmd_buffer;

    /** Most recent command awaiting response. */
    Command *pending = nullptr;

    /** Time the pending command will expire. */
    uint32_t command_timer = 0;

    /** Time of the next state update. */
    uint32_t update_timer = 0;

    /** How long to wait before sending the reset command. */
    uint32_t reset_timer = 0;

    /** State of the modem. */
    State device_state = State::reset;

    /** Next state of the modem. */
    State next_state = State::reset;

    /** State of data transmission through the socket. */
    SocketState sock_state = SocketState::command;

    /** Packet parser. */
    Parser parser;

    /** User buffer to send from. */
    const uint8_t *tx_buffer = nullptr;

    /** Size of 'tx_buffer'. */
    size_t tx_size = 0;

    /** Number of bytes that have been read from 'tx_buffer'. */
    size_t tx_index = 0;

    /** Tracks the space in the modem's tx buffer. */
    size_t modem_tx_available = 0;

    /** User buffer to receive into. */
    uint8_t *rx_buffer = nullptr;

    /** Size of 'rx_buffer'. */
    size_t rx_size = 0;

    /** Number of bytes that have been written to 'rx_buffer'. */
    size_t rx_index = 0;

    /** Tracks the number of bytes in the modem's rx buffer. */
    size_t modem_rx_available = 0;

    /** Number of bytes stage by rtr. */
    size_t modem_rx_pending = 0;
};

} // namespace gsm

#endif // NOVAGSM_MODEM_H_
