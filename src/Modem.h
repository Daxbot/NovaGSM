/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file Modem.h
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 * @version 2.2
 */

#ifndef _GSM_MODEM_H_
#define _GSM_MODEM_H_

#include <stdint.h>
#include "Buffer.h"

/** Handles buffered communication through a GSM/GPRS modem. */
namespace GSM
{
    constexpr uint8_t MAJOR_VERSION = 2;    /**< Major version, increment for breaking changes. */
    constexpr uint8_t MINOR_VERSION = 2;    /**< Minor version, increment for non-breaking changes. */

    /** Library version. */
    constexpr uint16_t VERSION = (MAJOR_VERSION * 100) + MINOR_VERSION;

    /** Size of the buffer to store the modem ID. */
    constexpr size_t ID_SIZE = 20;

    /** Defines resources and callbacks used by the driver. */
    typedef struct {
        int (*read)(void *data, uint32_t size);         /**< Read 'size' bytes into 'data' from device. */
        int (*write)(const void *data, uint32_t size);  /**< Write 'size' bytes from 'data' to device. */
        uint32_t(*elapsed_ms)();                        /**< Return the milliseconds elapsed since initialization. */
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
        online,         /**< Network registers the modem. */
        authenticating, /**< Attempting to authenticate with GPRS. */
        ready,          /**< Connected to GPRS. */
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
                : m_ctx(context) {}

            /** Handle communication with the modem. */
            void process();

            /** Set a function to be called on state changes.
             *
             * @param [in] dev_cb function to be called.
             * @param [in] user pointer to be passed when dev_cb is called.
             */
            inline void set_device_callback(void (*dev_cb)(State state, void *user), void *user=nullptr)
            {
                f_dev_cb = dev_cb;
                p_dev_cb_user = user;
            }

            /** Set a function to be called on socket data event changes.
             *
             * @param [in] sock_cb function to be called.
             * @param [in] user pointer to be passed when the function is called.
             */
            inline void set_socket_callback(void (*sock_cb)(Event event, void *user), void *user=nullptr)
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
            int unlock(const void *pin, uint8_t pin_size);

            /** Connect to GPRS.
             *
             * Must be called in State::online to transition to State::ready.
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
            int authenticate(const void *apn, uint8_t apn_size, const void *user, uint8_t user_size, const void *pwd, uint8_t pwd_size);

            /** Connect to GPRS.
             *
             * Must be called in State::online to transition to State::ready.
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
            int authenticate(const char *apn="wholesale", const char *user="", const char *pwd="");

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
            int connect(const void *host, uint8_t host_size, uint16_t port);

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
            int connect(const char *host, uint16_t port);

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
            void receive(void *data, uint32_t size);

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
            void send(const void *data, uint32_t size);

            /** Cancel an ongoing send() call.
             *
             * @warning if the modem data transfer is in progress '\0' will be sent
             * for the remaining bytes.
             */
            void stop_send();

            /** Number of bytes available in the modem's buffer. */
            inline size_t rx_available()
            {
                return m_rx_available;
            }

            /** Number of bytes available to send().
             *
             * Buffers larger than this will be automatically broken up
             * and sent over multiple transfers.
             */
            inline size_t tx_available()
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

        private:
            /** State of data transmission when socket is open. */
            enum class SocketState {
                idle,   /**< Socket is idle, connection status will be periodically polled. */
                poll,   /**< Socket status is being polled. */
                rtr,    /**< Data is available and being read into the receive buffer. */
                rts,    /**< A send buffer is ready to be sent to the modem. */
                cts,    /**< Data is being written from the send buffer. */
            };

            /** Handle socket data transfer in State::open. */
            void socket_process();

            /** Update the device state and call 'f_dev_cb'. */
            void set_state(State state);

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

            Buffer m_cmd_buffer;                            /**< Queued commands to send. */
            command_t *m_pending = nullptr;                 /**< Most recent command awaiting response. */
            uint8_t *p_data = nullptr;                      /**< Current buffer position. */
            uint32_t m_command_timer = 0;                   /**< Time the pending command will expire. */
            uint32_t m_update_timer = 0;                    /**< Time of the next state update. */
            State m_device_state = State::reset;             /**< State of the modem. */
            SocketState m_socket_state = SocketState::idle; /**< State of data transmission through the socket. */
            uint8_t m_rssi = 99;                            /**< Signal rssi value reported by AT+CSQ. */
            uint8_t m_errors = 0;                           /**< Timeout error counter.  If it exceeds MAX_ERRORS call reset(). */
            char m_modem_id[ID_SIZE] = {};                  /**< Holds the response from ATI. */

            const uint8_t *m_tx_buffer = nullptr;           /**< User buffer to send from. */
            size_t m_tx_size = 0;                           /**< Size of 'm_tx_buffer'. */
            size_t m_tx_count = 0;                          /**< Number of bytes that have been read from 'm_tx_buffer'. */
            size_t m_tx_available = 0;                      /**< Tracks the space in the modem's tx buffer. */

            uint8_t *m_rx_buffer = nullptr;                 /**< User buffer to receive into. */
            size_t m_rx_size = 0;                           /**< Size of 'm_rx_buffer'. */
            size_t m_rx_count = 0;                          /**< Number of bytes that have been written to 'm_rx_buffer'. */
            size_t m_rx_available = 0;                      /**< Tracks the number of bytes in the modem's rx buffer. */
    };
}

#endif /* _GSM_MODEM_H_ */
