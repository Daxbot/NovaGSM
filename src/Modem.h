/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file Modem.h
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 * @version 3.0
 */

#ifndef _GSM_MODEM_H_
#define _GSM_MODEM_H_

#include <stdint.h>
#include "Buffer.h"

/** Handles buffered communication through a GSM/GPRS modem. */
namespace GSM
{
    constexpr int MAJOR_VERSION = 3;    /**< Major version, increment for breaking changes. */
    constexpr int MINOR_VERSION = 0;    /**< Minor version, increment for non-breaking changes. */

    /** Library version. */
    constexpr int VERSION = (MAJOR_VERSION * 100) + MINOR_VERSION;

    /** Size of the buffer to store the modem ID. */
    constexpr int ID_SIZE = 20;

    /** Maximum size of socket data transfers.
     *
     * Each data chunk must be less than the actual
     * buffer size to account for protocol overhead.
     */
    constexpr int SOCKET_MAX = (GSM_BUFFER_SIZE - 64);

    /** Defines resources and callbacks used by the driver.
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
        #if defined(GSM_ASYNC)
            // Asynchronous API
            int32_t (*receive_async)(void *data, uint32_t size);    /**< Asynchronously receive 'size' bytes into 'data' from device. */
            int32_t (*send_async)(const void *data, uint32_t size); /**< Asynchronously send 'size' bytes from 'data' to device. */
            uint32_t (*rx_count_async)();                           /**< Return the number of bytes received during receive_async. */
            void (*rx_abort_async)();                               /**< Abort the ongoing asynchronous receive operation. */
        #else
            // Common API
            int (*read)(void *data, uint32_t size);                 /**< Read 'size' bytes into 'data' from stream. */
            int (*write)(const void *data, uint32_t size);          /**< Write 'size' bytes from 'data' to stream. */
        #endif

        // Both
        uint32_t (*elapsed_ms)();                               /**< Return the milliseconds elapsed since initialization. */
    } context_t;

    /** State of the device.
     *
     * @see process() for a functional description.
     */
    enum class State {
        reset,          /**< None. */
        init,           /**< SIM is being initialized. */
        locked,         /**< SIM is locked. */
        offline,        /**< No signal. */
        authenticating, /**< Attempting to authenticate with GPRS. */
        online,         /**< Network registers the modem. */
        handshaking,    /**< Attempting to establish TCP connection. */
        open,           /**< TCP socket is open. */
    };

    /** Socket callback event. */
    enum class Event {
        new_data,       /**< New data available for read(). */
        rx_complete,    /**< A read command has finished. */
        rx_stopped,     /**< A read command was stopped. */
        rx_error,       /**< A read command failed. */
        tx_complete,    /**< A write command has finished. */
        tx_stopped,     /**< A write command was stopped. */
        tx_error,       /**< A write command failed. */
    };

    /** Class representing the connection with a GSM/GPRS modem. */
    class Modem {
        public:
            /** Constructor.
             *
             * @param [in] context hardware specific callbacks for the driver
             */
            Modem(context_t *context)
                : m_ctx(context) {};

            /** Handle communication with the modem. */
            void process();

            /** Set a function to be called on state changes.
             *
             * @param [in] dev_cb function to be called.
             * @param [in] user pointer to be passed when dev_cb is called.
             */
            inline void set_device_callback(
                void (*dev_cb)(State state, void *user), void *user=nullptr)
            {
                f_dev_cb = dev_cb;
                p_dev_cb_user = user;
            }

            /** Set a function to be called on socket data event changes.
             *
             * @param [in] sock_cb function to be called.
             * @param [in] user pointer to be passed when the function is called.
             */
            inline void set_socket_callback(
                void (*sock_cb)(Event event, void *user), void *user=nullptr)
            {
                f_sock_cb = sock_cb;
                p_sock_cb_user = user;
            }

            /** Unlocks the SIM card.
             *
             * Must be in State::locked.
             *
             * @param [in] pin SIM card password
             * @param [in] pin_size length of 'pin'.
             * @return -EINVAL if inputs are null.
             * @return -ENOBUFS if command buffer is full.
             */
            int unlock(const void *pin, size_t pin_size);

            /** Connect to GPRS.
             *
             * Must be called in State::offline to transition to State::online.
             *
             * @param [in] apn GPRS access point name.
             * @param [in] apn_size length of 'apn'.
             * @param [in] user GPRS user name.
             * @param [in] user_size length of 'user'.
             * @param [in] pwd GPRS password.
             * @param [in] pwd_size length of 'pwd'.
             * @return -EINVAL if inputs are null.
             * @return -ENODEV if the device is not responding.
             * @return -ENETUNREACH if the network is not available.
             * @return -EALREADY if authentication is already in progress.
             * @return -EISCONN if already connected to GPRS.
             * @return -ENOBUFS if command buffer is full.
             */
            int authenticate(
                const void *apn, size_t apn_size,
                const void *user, size_t user_size,
                const void *pwd, size_t pwd_size);

            /** Connect to GPRS.
             *
             * Must be called in State::offline to transition to State::online.
             *
             * @param [in] apn GPRS access point name.
             * @param [in] user GPRS user name.
             * @param [in] pwd GPRS password.
             * @return -EINVAL if inputs are null.
             * @return -ENODEV if the device is not responding.
             * @return -ENETUNREACH if the network is not available.
             * @return -EALREADY if authentication is already in progress.
             * @return -EISCONN if already connected to GPRS.
             * @return -ENOBUFS if command buffer is full.
             */
            int authenticate(const char *apn, const char *user="", const char *pwd="");

            /** Open a TCP socket.
             *
             * Must be in State::connected, transitions to State::open.
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
             * @return -ENOBUFS if command buffer is full.
             */
            int connect(const void *host, size_t host_size, int port);

            /** Open a TCP socket.
             *
             * Must be in State::connected, transitions to State::open.
             *
             * @param [in] host server ip address.
             * @param [in] port server port number.
             * @return -EINVAL if inputs are null.
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTCONN if GPRS is not connected.
             * @return -EALREADY if handshaking is already in progress.
             * @return -EADDRINUSE if a socket is already open.
             * @return -ENOBUFS if command buffer is full.
             */
            int connect(const char *host, int port);

            /** Close TCP socket.
             *
             * @return -ENODEV if the device is not responsive.
             * @return -ENETUNREACH if the network is not available.
             * @return -ENOTSOCK if a connection is not established.
             * @return -ENOBUFS if command buffer is full.
             */
            int disconnect();

            /** Reinitialize driver.
             *
             * Resets the context and transitions to State::reset.
             */
            void reset();

            /** Start receiving data.
             *
             * Asynchronously receives up to 'size' bytes from the modem
             * and stores it in the buffer pointed to by 'data'.
             *
             * Function returns immediately and calls the socket
             * callback with the result.
             *
             * @param [out] data buffer to read into.
             * @param [in] size number of bytes to read.
             * @result Event::rx_complete if size bytes read.
             * @result Event::rx_error if the read fails.
             * @see set_socket_callback
             */
            void receive(void *data, size_t size);

            /** Cancel an ongoing receive() call.
             *
             * @warning if the modem data transfer is in progress the data will be lost.
             */
            void stop_receive();

            /** Stages bytes to the tx ring buffer.
             *
             * @param [in] data buffer to write.
             * @param [in] size number of bytes to write.
             */
            void send(const void *data, size_t size);

            /** Cancel an ongoing send() call.
             *
             * @warning if the modem data transfer is in progress '\0' will be sent
             * for the remaining bytes.
             */
            void stop_send();

            /** Number of bytes available in the modem's buffer. */
            inline int rx_available()
            {
                return m_rx_available;
            }

            /** Number of bytes available to send().
             *
             * Buffers larger than this will be automatically broken up
             * and sent over multiple transfers.
             */
            inline int tx_available()
            {
                return m_tx_available;
            }

            /** Poll the status of the last receive() call.
             *
             * @return true if receive is in progress.
             */
            inline bool rx_busy()
            {
                return m_rx_buffer && m_rx_count < m_rx_size;
            }

            /** Poll the status of the last send() call.
             *
             * @return true if send is in progress.
             */
            inline bool tx_busy()
            {
                return m_tx_buffer && m_tx_count < m_tx_size;
            }

            /** Poll the status of the last receive() call
             *
             * @return number of bytes received, or -1 on error
             */
            inline int rx_count()
            {
                return (m_rx_buffer) ? m_rx_count : -1;
            }

            /** Poll the status of the last send() call
             *
             * @return number of bytes sent, or -1 on error
             */
            inline int tx_count()
            {
                return (m_tx_buffer) ? m_tx_count : -1;
            }

            /** Returns the device state. */
           inline State status()
           {
               return m_device_state;
           }

           /** Returns if a TCP connection is established. */
           inline bool connected()
           {
               return m_device_state == State::open;
           }

           /** Returns the rssi value reported by AT+CSQ.
            *
            * @warning value is encoded. Refer to modem documentation for actual rssi values.
            */
           inline int signal()
           {
               return m_rssi;
           }
        protected:

        private:
            /** State of data transmission when socket is open. */
            enum class SocketState {
                idle,   /**< Socket is idle, connection status will be periodically polled. */
                poll,   /**< Socket status is being polled. */
                rtr,    /**< Data is available and being read into the receive buffer. */
                rts,    /**< A send buffer is ready to be sent to the modem. */
                cts,    /**< Data is being written from the send buffer. */
            };

            /** Queue a formatted command string.
             *
             * @param [in] timeout maximum time to wait for a response.
             * @param [in] command standard c format string.
             * @param [in] ... arguments for command formatting.
             */
            int queue_command(uint32_t timeout, const char *command, ...);

            /** Send an appropriate polling message based on the modem's state. */
            void poll_modem();

            /** State logic called by process.
             * @{
             */
            void _process_reset();
            void _process_init();
            void _process_locked();
            void _process_offline();
            void _process_authenticating();
            void _process_online();
            void _process_handshaking();
            void _process_open();
            /** @} */

            /** Socket logic called in State::open.
             * @{
             */
            void _socket_idle();
            void _socket_poll();
            void _socket_rtr();
            void _socket_rts();
            void _socket_cts();
            /** @} */

            /** Update the device state and call 'f_dev_cb'.
             *
             * @param [in] state new device state.
             */
            inline void set_state(State state)
            {
                m_device_state = state;
                if(f_dev_cb)
                    f_dev_cb(m_device_state, p_dev_cb_user);
            }

            /** Driver operating context. */
            context_t const *m_ctx;

            /**@{*/
            /** Called on state change. */
            void (*f_dev_cb)(State state, void *user) = nullptr;
            void *p_dev_cb_user = nullptr;
            /**@}*/

            /**@{*/
            /** Called on socket data event. */
            void (*f_sock_cb)(Event event, void *user) = nullptr;
            void *p_sock_cb_user = nullptr;
            /**@}*/

            char m_modem_id[ID_SIZE] = {};                  /**< Holds the response from ATI. */
            int m_rssi = 99;                                /**< Signal rssi value reported by AT+CSQ. */

            Buffer m_cmd_buffer;                            /**< Queued commands to send. */
            command_t *m_pending = nullptr;                 /**< Most recent command awaiting response. */
            uint32_t m_command_timer = 0;                   /**< Time the pending command will expire. */
            uint32_t m_update_timer = 0;                    /**< Time of the next state update. */
            State m_device_state = State::reset;            /**< State of the modem. */
            SocketState m_socket_state = SocketState::idle; /**< State of data transmission through the socket. */
            int m_auth_state = 0;                           /**< State of the authentication routine. */
            int m_errors = 0;                               /**< Timeout error counter.  If it exceeds MAX_ERRORS call reset(). */

            uint8_t m_response[GSM_BUFFER_SIZE] = {};       /**< Modem response buffer. */
            int m_response_size = 0;                        /**< Size of modem response. */

            const uint8_t *m_tx_buffer = nullptr;           /**< User buffer to send from. */
            int m_tx_size = 0;                              /**< Size of 'm_tx_buffer'. */
            int m_tx_count = 0;                             /**< Number of bytes that have been read from 'm_tx_buffer'. */
            int m_tx_available = 0;                         /**< Tracks the space in the modem's tx buffer. */

            uint8_t *m_rx_buffer = nullptr;                 /**< User buffer to receive into. */
            int m_rx_size = 0;                              /**< Size of 'm_rx_buffer'. */
            int m_rx_count = 0;                             /**< Number of bytes that have been written to 'm_rx_buffer'. */
            int m_rx_available = 0;                         /**< Tracks the number of bytes in the modem's rx buffer. */

            char m_apn[256] = {};                           /**< Holds the 'Access Point Name' for authenticate. */
            char m_user[266] = {};                          /**< Holds the 'Username' for authenticate. */
            char m_pwd[266] = {};                           /**< Holds the 'Password' for authenticate. */
    };
}

#endif /* _GSM_MODEM_H_ */
