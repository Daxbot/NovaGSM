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
     * @brief Maximum size of an AT command and response.
     *
     * This can be set with -DNOVAGSM_BUFFER_SIZE (default 556).
     */
    constexpr int kBufferSize = (NOVAGSM_BUFFER_SIZE);

    /**
     * @brief Maximum size of socket data transfers.
     *
     * Each data chunk must be less than the actual buffer size to account for
     * protocol overhead.
     */
    constexpr int kSocketMax = (kBufferSize - 64);

    /** How long to wait for a command response. */
    constexpr int kDefaultTimeout = 200;

    /**
     * @brief Defines resources and callbacks used by the driver.
     *
     * The user has two API options: common and asynchronous.
     *
     * The common API expects a *non-blocking* buffered
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
     *
     * To use the asynchronous API define GSM_ASYNC and provide callbacks for:
     *  1. receive_async
     *  2. send_async
     *  3. rx_count_async
     *  4. rx_abort_async
     */
    typedef struct {
        #if defined(NOVAGSM_ASYNC)
            /**
             * @brief Asynchronous receive.
             *
             * Receive 'size' bytes into 'data' from device.
             *
             * @param [out] data buffer to receive into.
             * @param [in] size number of bytes to receive.
             */
            int (*receive_async)(void *data, int size);

            /**
             * @brief Asynchronous send.
             *
             * Asynchronously send 'size' bytes from 'data' to device.
             *
             * @param [in] data buffer to send.
             * @param [in] size number of bytes to send.
             */
            int (*send_async)(const void *data, int size);

            /** Return the number of bytes received during receive_async(). */
            int (*rx_count_async)();

            /** Abort an ongoing asynchronous receive operation. */
            void (*rx_abort_async)();
        #else
            /**
             * @brief Common read.
             *
             * Read 'size' bytes into 'data' from stream.
             *
             * @param [out] data buffer to read into.
             * @param [in] size number of bytes to read.
             * @return number of bytes read.
             */
            int (*read)(void *data, int size);

            /**
             * @brief Common write.
             *
             * Write 'size' bytes from 'data' to stream.
             *
             * @param [in] data buffer to write.
             * @param [in] size number of bytes to write.
             * @return number of bytes written.
             */
            int (*write)(const void *data, int size);
        #endif
    } context_t;

    /**
     * @brief State of the modem.
     * @see set_state_callback().
     */
    enum class State {
        probe,          /**< Waiting for device. */
        init,           /**< Modem is initializing. */
        offline,        /**< Low power mode. */
        locked,         /**< SIM is locked. */
        searching,      /**< Searching for the network. */
        registered,     /**< Network registers the modem. */
        authenticating, /**< Attempting to establish GPRS connection. */
        ready,          /**< Data connection active. */
        handshaking,    /**< Attempting to establish TCP connection. */
        open,           /**< TCP socket is open. */
        closing,        /**< TCP socket is closing. */
    };

    /**
     * @brief Modem events.
     * @see set_event_callback().
     */
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
            Modem(context_t *context) : ctx_(context) {};

            /** Destructor. */
            ~Modem();

            /**
             * @brief Handle communication with the modem.
             *
             * The process method handles communication with the modem and
             * controls the device state. If there is not a command
             * already awaiting a response then the next command in the buffer
             * is sent. Responses are handled based on the gsm::State.
             *
             * @param [in] delta_us the number of microseconds that have
             * elapsed since the last call to process.
             */
            void process(int delta_us);

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

            /** Re-initialize the driver state. */
            void reinit();

            /**
             * @brief Reset the modem (+CFUN=1,1).
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int reset();

            /**
             * @brief Enter low power mode (+CFUN=0).
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int disable();

            /**
             * @brief Configure the SIM card PIN.
             * @param [in] pin SIM card pin
             * @param [in] pin_size length of 'pin'
             * @return -EINVAL if inputs are null.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int unlock(const void *pin, int pin_size);

            /**
             * @brief Configure the Access Point Name (APN).
             * @param [in] apn access point name.
             * @param [in] apn_size length of 'apn'.
             * @param [in] user user name.
             * @param [in] user_size length of 'user'.
             * @param [in] pwd password.
             * @param [in] pwd_size length of 'pwd'.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
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
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int authenticate(
                const char *apn,
                const char *user=nullptr,
                const char *pwd=nullptr);

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
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int connect(
                const void *host, int host_size, int port);

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
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int connect(const char *host, int port);

            /**
             * @brief Close TCP socket.
             *
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTSOCK if a connection is not established.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
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
             */
            void receive(void *data, int size);

            /**
             * @brief Cancel an ongoing receive() call.
             * @warning if transfer is in progress the data will be lost.
             */
            void stop_receive();

            /**
             * @brief Stage bytes to the tx ring buffer.
             * @param [in] data buffer to write.
             * @param [in] size number of bytes to write.
             */
            void send(const void *data, int size);

            /**
             * @brief Cancel an ongoing send() call.
             * @warning if transfer is in progress '\0' will be sent for the
             * remaining bytes.
             */
            void stop_send();

            /** Number of bytes available to receive(). */
            inline int rx_available()
            {
                return rx_available_;
            }

            /**
             * @brief Number of bytes available to send().
             *
             * Buffers larger than this will be broken up and sent over
             * multiple transfers.
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
             * @return number of bytes received or -1 if not receiving.
             */
            inline int rx_count()
            {
                return (rx_buffer_) ? rx_count_ : -1;
            }

            /**
             * @brief Poll the status of the last send() call
             * @return number of bytes sent or -1 if not sending.
             */
            inline int tx_count()
            {
                return (tx_buffer_) ? tx_count_ : -1;
            }

            /** Return the device state. */
            inline State status()
            {
                return device_state_;
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

            /** Returns true if the modem is ready for connect(). */
            inline bool ready()
            {
                return device_state_ == State::ready;
            }

            /** Returns true if a connection attempt is in progress. */
            inline bool handshaking()
            {
                return device_state_ == State::handshaking;
            }

            /** Returns true if the socket is being closed. */
            inline bool closing()
            {
                return device_state_ == State::closing;
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

           /** Returns the value reported by AT+CREG? */
           inline int creg()
           {
               return creg_;
           }

           /** Returns the value reported by AT+CGREG? */
           inline int cgreg()
           {
               return cgreg_;
           }

          /** Returns the value reported by AT+CGREG? */
          inline int cereg()
          {
              return cereg_;
          }

        private:
            class Command;

            /** State of data transmission when socket is open. */
            enum class SocketState {
                idle,   /**< Socket idle. */
                rtr,    /**< Socket request to receive. */
                rts,    /**< Socket request to send. */
                cts,    /**< Socket clear to send. */
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
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int push_command(Command *cmd);

            /**
             * @brief Add a command to the front of the queue.
             * @param [in] cmd Command object.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int shift_command(Command *cmd);

            /**
             * @brief Send a polling message based on the modem's state.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int poll_modem();

            /**
             * @brief Read data from the socket.
             * @return -EBUSY if socket is not idle.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int socket_receive(int count);

            /**
             * @brief Write data to the socket.
             * @return -EBUSY if socket is not idle.
             * @return -ENOMEM if memory allocation failed.
             * @return -EMSGSIZE if buffer size exceeded.
             */
            int socket_send(int count);

            /** Handle a command timeout. */
            void process_timeout();

            /** Handle device detection. */
            void process_probe();

            /** Handle general command responses. */
            void process_general();

            /** Handle authenticate(). */
            void process_authentication();

            /** Handle connect(). */
            void process_handshaking();

            /** Handle disconnect(). */
            void process_close();

            /** Handle socket. */
            void process_socket();

            /** Socket is idle. */
            void socket_state_idle();

            /** Socket data is available and being read. */
            void socket_state_rtr();

            /** A buffer is ready to be sent through the socket. */
            void socket_state_rts();

            /** Data is being written from the send buffer. */
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

            /** Registration value reported by AT+CREG? */
            int creg_ = 0;

            /** Registration value reported by AT+CGREG? */
            int cgreg_ = 0;

            /** Registration value reported by AT+CEREG? */
            int cereg_ = 0;

            /** GPRS service status reported by AT+CGATT? */
            int service_ = 0;

            /** Local IP address reported by AT+CIFSR. */
            char ip_address_[32] = {};

            /** Command queue. */
            std::deque<Command*> cmd_buffer_;

            /** Most recent command awaiting response. */
            Command *pending_ = nullptr;

            /** Tracks the time elapsed since initialization (ms). */
            unsigned int elapsed_us_ = 0;

            /** Time the pending command will expire. */
            unsigned int command_timer_ = 0;

            /** Time of the next state update. */
            unsigned int update_timer_ = 0;

            /** State of the modem. */
            State device_state_ = State::probe;

            /** State of data transmission through the socket. */
            SocketState sock_state_ = SocketState::idle;

            /** Modem response buffer. */
            uint8_t response_[kBufferSize] = {};

            /** Size of modem's response. */
            int response_size_ = 0;

            /** User buffer to send from. */
            const uint8_t *tx_buffer_ = nullptr;

            /** Size of 'tx_buffer_'. */
            int tx_size_ = 0;

            /** Number of bytes that have been read from 'tx_buffer_'. */
            int tx_count_ = 0;

            /** Tracks the space in the modem's tx buffer. */
            int tx_available_ = 0;

            /** Number of bytes staged by rts. */
            int tx_pending_ = 0;

            /** User buffer to receive into. */
            uint8_t *rx_buffer_ = nullptr;

            /** Size of 'rx_buffer_'. */
            int rx_size_ = 0;

            /** Number of bytes that have been written to 'rx_buffer_'. */
            int rx_count_ = 0;

            /** Tracks the number of bytes in the modem's rx buffer. */
            int rx_available_ = 0;
    };

    /** Modem command object. */
    class Modem::Command
    {
        public:
            /**
             * @brief Destructor.
             * Frees data buffer.
             */
            ~Command();

            /**
             * @brief Create a new Command object.
             * @param [in] timeout maximum time to wait for a response (ms).
             * @param [in] command standard c format string.
             * @param [in] ... arguments for command formatting.
             */
            static Command *create(int timeout, const char *command, ...);

            /** Returns the timeout value (ms). */
            int timeout() {
                return timeout_;
            }

            /** Returns the size of the payload string. */
            int size() {
                return size_;
            }

            /** Returns the payload buffer. */
            uint8_t *data() {
                return data_;
            }

        private:
            /**
             * @brief Constructor.
             * @param [in] timeout maximum time to wait for a response (ms).
             * @param [in] data pre-allocated data buffer.
             * @param [in] size length of 'data' buffer.
             */
             Command(int timeout, uint8_t *data, int size)
                 : timeout_(timeout), size_(size), data_(data) {};

            int timeout_ = 0;           /**< Response timeout. */
            int size_ = 0;              /**< Size of payload. */
            uint8_t *data_;             /**< Payload buffer. */
    };
}

#endif /* NOVAGSM_MODEM_H_ */
