/**
 * @file modem.h
 * @brief GSM/GPRS modem driver.
 * @author Wilkins White
 * @copyright 2019-2021 Nova Dynamics LLC
 */

#ifndef NOVAGSM_MODEM_H_
#define NOVAGSM_MODEM_H_

#include <cstdint>
#include <deque>

/**@{*/
/** Allows user to specify buffer size with -DGSM_BUFFER_SIZE. */
#ifndef NOVAGSM_BUFFER_SIZE
#define NOVAGSM_BUFFER_SIZE 556
#endif
/**@}*/

#if NOVAGSM_BUFFER_SIZE < 256
#warning NOVAGSM_BUFFER_SIZE must be at least 256
#endif

/** Handles buffered communication through a GSM/GPRS modem. */
namespace gsm
{
    /**
     * @brief Maximum size of socket data transfers.
     *
     * Each data chunk must be less than the actual
     * buffer size to account for protocol overhead.
     */
    constexpr int kSocketMax = (NOVAGSM_BUFFER_SIZE - 64);

    /** How long to wait for a command response. */
    constexpr int kDefaultTimeout = 200;

    /**
     * @brief Defines resources and callbacks used by the driver.
     *
     * The user has two API options: common and asynchronous.
     *
     * The common API expects a non-blocking buffered
     * implementation, e.g. unistd or Arduino.
     *
     *   https://linux.die.net/man/2/read
     *   https://linux.die.net/man/2/write
     *
     * The asynchronous API expects a raw hardware implementation
     * where the buffer must stay allocated during the operation,
     * e.g. CMSIS USART drivers.
     *
     * https://www.keil.com/pack/doc/CMSIS/Driver/html/group__usart__interface__gr.html
     *
     * To use the common API provide callbacks for:
     *  1. read
     *  2. write
     *  3. elapsed_ms
     *
     * To use the asynchronous API define GSM_ASYNC and provide callbacks for:
     *  1. receive_async
     *  2. send_async
     *  3. rx_count_async
     *  4. rx_abort_async
     *  5. elapsed_ms
     */
    typedef struct {
        #if defined(NOVAGSM_ASYNC)
            // Asynchronous API
            int (*receive_async)(void *data, int size);         /**< Asynchronously receive 'size' bytes into 'data' from device. */
            int (*send_async)(const void *data, int size);      /**< Asynchronously send 'size' bytes from 'data' to device. */
            int (*rx_count_async)();                            /**< Return the number of bytes received during receive_async. */
            void (*rx_abort_async)();                           /**< Abort the ongoing asynchronous receive operation. */
        #else
            // Common API
            int (*read)(void *data, int size);                  /**< Read 'size' bytes into 'data' from stream. */
            int (*write)(const void *data, int size);           /**< Write 'size' bytes from 'data' to stream. */
        #endif

        // Both
        uint32_t (*elapsed_ms)();                               /**< Return the milliseconds elapsed since initialization. */
    } context_t;

    /**
     * @brief State of the modem.
     * @see process() for a functional description.
     */
    enum class State {
        offline,        /**< Low power mode. */
        reset,          /**< Modem is initializing. */
        locked,         /**< SIM is locked. */
        searching,      /**< Searching for the network. */
        registered,     /**< Network registers the modem. */
        authenticating, /**< Attempting to establish GPRS connection. */
        ready,          /**< Data connection active. */
        handshaking,    /**< Attempting to establish TCP connection. */
        open,           /**< TCP socket is open. */
    };

    /** Modem events. */
    enum class Event {
        timeout,            /**< A command timed out. */
        auth_error,         /**< An error occured during authenticate(). */
        conn_error,         /**< An error occured during connect(). */
        new_data,           /**< New data available for read(). */
        rx_complete,        /**< A read command has finished. */
        rx_stopped,         /**< A read command was stopped. */
        rx_error,           /**< A read command failed. */
        tx_complete,        /**< A write command has finished. */
        tx_stopped,         /**< A write command was stopped. */
        tx_error,           /**< A write command failed. */
    };

    /** Class representing the connection with a GSM/GPRS modem. */
    class Modem {
        public:
            /**
             * @brief Constructor.
             * @param [in] context hardware specific callbacks for the driver.
             */
            Modem(context_t *context);

            /** Destructor. */
            ~Modem();

            /** Handle communication with the modem. */
            void process();

            /**
             * @brief Set a function to be called on state changes.
             * @param [in] func function to be called.
             * @param [in] user pointer to be passed when 'func' is called.
             */
            inline void set_state_callback(
                void (*func)(State state, void *user), void *user=nullptr)
            {
                state_cb_ = func;
                state_cb_user_ = user;
            }

            /**
             * @brief Set a function to be called on a modem event.
             * @param [in] func function to be called.
             * @param [in] user pointer to be passed when 'func' is called.
             */
            inline void set_event_callback(
                void (*func)(Event event, void *user), void *user=nullptr)
            {
                event_cb_ = func;
                event_cb_user_ = user;
            }

            /**
             * @brief Configure the SIM card PIN.
             *
             * @param [in] pin SIM card pin
             * @param [in] pin_size length of 'pin'
             *
             * @return -EINVAL if inputs are null.
             */
            int unlock(const void *pin, int pin_size);

            /**
             * @brief Configure the Access Point Name (APN).
             *
             * @param [in] apn access point name.
             * @param [in] apn_size length of 'apn'.
             * @param [in] user user name.
             * @param [in] user_size length of 'user'.
             * @param [in] pwd password.
             * @param [in] pwd_size length of 'pwd'.
             */
            int authenticate(
                const void *apn, int apn_size,
                const void *user, int user_size,
                const void *pwd, int pwd_size);

            /**
             * @brief Configure the Access Point Name (APN).
             *
             * @param [in] apn access point name.
             * @param [in] user user name.
             * @param [in] pwd password.
             */
            int authenticate(
                const char *apn,
                const char *user=nullptr,
                const char *pwd=nullptr);

            /** Re-initialize the driver state. */
            void reinit();

            /** Reset the modem (+CFUN=1,1). */
            int reset();

            /** Enter low power mode (+CFUN=0). */
            int disable();

            /**
             * @brief Open a TCP socket.
             *
             * Must be in State::ready, transitions to State::open.
             *
             * @param [in] host server ip address.
             * @param [in] host_size length of 'host'.
             * @param [in] port server port number.
             * @return -EINVAL if inputs are null.
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTCONN if GPRS is not connected.
             * @return -EALREADY if handshaking is already in progress.
             * @return -EADDRINUSE if a socket is already open.
             */
            int connect(const void *host, int host_size, int port);

            /**
             * @brief Open a TCP socket.
             *
             * Must be in State::ready, transitions to State::open.
             *
             * @param [in] host server ip address.
             * @param [in] port server port number.
             * @return -EINVAL if inputs are null.
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTCONN if GPRS is not connected.
             * @return -EALREADY if handshaking is already in progress.
             * @return -EADDRINUSE if a socket is already open.
             */
            int connect(const char *host, int port);

            /**
             * @brief Close TCP socket.
             *
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTSOCK if a connection is not established.
             */
            int disconnect();

            /**
             * @brief Start receiving data.
             *
             * Asynchronously receives up to 'size' bytes from the modem
             * and stores it in the buffer pointed to by 'data'.
             *
             * Function returns immediately and calls the socket
             * callback with the result.
             *
             * @param [out] data buffer to read into.
             * @param [in] size number of bytes to read.
             *
             * @see set_socket_callback
             */
            void receive(void *data, int size);

            /**
             * @brief Cancel an ongoing receive() call.
             * @warning if the modem data transfer is in progress the data will
             * be lost.
             */
            void stop_receive();

            /**
             * @brief Stage bytes to the tx ring buffer.
             *
             * @param [in] data buffer to write.
             * @param [in] size number of bytes to write.
             */
            void send(const void *data, int size);

            /**
             * @brief Cancel an ongoing send() call.
             * @warning if the modem data transfer is in progress '\0' will be
             * sent for the remaining bytes.
             */
            void stop_send();

            /** Number of bytes available in the modem's buffer. */
            inline int rx_available()
            {
                return rx_available_;
            }

            /**
             * @brief Number of bytes available to send().
             *
             * Buffers larger than this will be automatically broken up
             * and sent over multiple transfers.
             */
            inline int tx_available()
            {
                return tx_available_;
            }

            /**
             * @brief Poll the status of the last receive() call.
             * @return true if receive is in progress.
             */
            inline bool rx_busy()
            {
                return rx_buffer_ && rx_count_ < rx_size_;
            }

            /**
             * @brief Poll the status of the last send() call.
             * @return true if send is in progress.
             */
            inline bool tx_busy()
            {
                return tx_buffer_ && tx_count_ < tx_size_;
            }

            /**
             * @brief Poll the status of the last receive() call
             * @return number of bytes received, or -1 on error
             */
            inline int rx_count()
            {
                return (rx_buffer_) ? rx_count_ : -1;
            }

            /**
             * @brief Poll the status of the last send() call
             * @return number of bytes sent, or -1 on error
             */
            inline int tx_count()
            {
                return (tx_buffer_) ? tx_count_ : -1;
            }

            /** Returns the device state. */
            inline State status()
            {
                return device_state_;
            }

            /** Returns true if the modem is online. */
            inline bool online()
            {
                return device_state_ != State::offline;
            }

            /** Returns true if the modem is registered on the network. */
            inline bool registered()
            {
                return device_state_ >= State::registered;
            }

            /** Returns true if a connection attempt is in progress. */
            inline bool authenticating()
            {
                return device_state_ == State::authenticating;
            }

            /** Returns true if a connection attempt is in progress. */
            inline bool handshaking()
            {
                return device_state_ == State::handshaking;
            }

           /** Returns true if a TCP connection is established. */
            inline bool connected()
            {
                return device_state_ == State::open;
            }

           /**
            * @brief Returns the value reported by AT+CSQ.
            * @warning value is encoded. Refer to modem documentation for
            * actual rssi value.
            */
           inline int signal()
           {
               return signal_;
           }

        private:
            class Command;

            /** State of data transmission when socket is open. */
            enum class SocketState {
                idle,   /**< Socket is idle. */
                rtr,    /**< Data is available and being read into the receive buffer. */
                rts,    /**< A send buffer is ready to be sent to the modem. */
                cts,    /**< Data is being written from the send buffer. */
            };

            /**
             * @brief Update the device state and call user function.
             * @param [in] state new device state.
             */
            void set_state(State state);

            /** Free the pending command. */
            void free_pending();

            /** Returns true if the response buffer is properly terminated. */
            bool response_complete();

            /**
             * @brief Add a command to the end of the queue.
             * @param [in] cmd Command object.
             */
            int push_command(Command *cmd);

            /**
             * @brief Add a command to the front of the queue.
             * @param [in] cmd Command object.
             */
            int shift_command(Command *cmd);

            /** Send a polling message based on the modem's state. */
            int poll_modem();

            /** Read data from the socket. */
            int socket_receive(int count);

            /** Write data to the socket. */
            int socket_send(int count);

            void process_timeout();
            void process_general();
            void process_authentication();
            void process_handshaking();
            void process_socket();

            void socket_state_idle();
            void socket_state_rtr();
            void socket_state_rts();
            void socket_state_cts();

            /** Driver operating context. */
            context_t const *ctx_;

            /** User function to call on state change event. */
            void (*state_cb_)(State state, void *user) = nullptr;

            /** User private data for state change callback. */
            void *state_cb_user_ = nullptr;

            /** User function to call on socket event. */
            void (*event_cb_)(Event event, void *user) = nullptr;

            /** User private data for socket callback. */
            void *event_cb_user_ = nullptr;

            /** Signal value reported by AT+CSQ. */
            int signal_ = 99;

            /** Local IP address reported by AT+CIFSR. */
            char ip_address_[32] = {};

            std::deque<Command*> cmd_buffer_;               /**< Command queue. */
            Command *pending_ = nullptr;                    /**< Most recent command awaiting response. */
            uint32_t command_timer_ = 0;                    /**< Time the pending command will expire. */
            uint32_t update_timer_ = 0;                     /**< Time of the next state update. */
            State device_state_ = State::reset;             /**< State of the modem. */
            SocketState sock_state_ = SocketState::idle;    /**< State of data transmission through the socket. */

            uint8_t response_[NOVAGSM_BUFFER_SIZE] = {};    /**< Modem response buffer. */
            int response_size_ = 0;                         /**< Size of modem response. */

            const uint8_t *tx_buffer_ = nullptr;            /**< User buffer to send from. */
            int tx_size_ = 0;                               /**< Size of 'tx_buffer_'. */
            int tx_count_ = 0;                              /**< Number of bytes that have been read from 'tx_buffer_'. */
            int tx_available_ = 0;                          /**< Tracks the space in the modem's tx buffer. */

            uint8_t *rx_buffer_ = nullptr;                  /**< User buffer to receive into. */
            int rx_size_ = 0;                               /**< Size of 'rx_buffer_'. */
            int rx_count_ = 0;                              /**< Number of bytes that have been written to 'rx_buffer_'. */
            int rx_available_ = 0;                          /**< Tracks the number of bytes in the modem's rx buffer. */
    };

    /** Modem command object. */
    class Modem::Command
    {
        public:
            /**
             * @brief Command constructor.
             * @param [in] timeout maximum time to wait for a response (ms).
             * @param [in] command standard c format string.
             * @param [in] ... arguments for command formatting.
             */
            Command(int timeout, const char *command, ...);

            /** Returns the timeout value (ms). */
            int timeout() const {
                return timeout_;
            }

            /** Returns the size of the payload string. */
            int size() const {
                return size_;
            }

            /** Returns the payload buffer. */
            const uint8_t *data() const {
                return data_;
            }

        private:
            int size_ = 0;                      /**< Size of payload. */
            int timeout_ = 0;                   /**< Response timeout (ms). */
            uint8_t data_[NOVAGSM_BUFFER_SIZE]; /**< Payload buffer. */
    };
}

#endif /* NOVAGSM_MODEM_H_ */
