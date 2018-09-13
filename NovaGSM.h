/** Driver for estabilishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file NovaGSM.h
 * @author Wilkins White
 * @copyright 2018 Nova Dynamics LLC
 */

#ifndef NOVAGSM_H_
#define NOVAGSM_H_

#include <stdint.h>
#include <cstring>

/** Handles buffered communication through a GSM/GPRS modem. */
namespace GSM
{
    /** Defines resources and callbacks used by the drivers. */
    typedef struct {
        void (*uart_begin)(uint32_t baud);                  /**< Initialize the UART at baudrate. */
        size_t (*uart_available)();                         /**< Return the number of bytes that can be read from the UART. */
        size_t (*uart_read)(uint8_t *data, size_t size);    /**< Read 'size' bytes into 'data' from the UART. */
        void (*uart_write)(uint8_t* data, size_t size);     /**< Write 'size' bytes from 'data' to the UART. */
        void (*uart_clear)();                               /**< Discard pending read bytes; uart_available() should return 0. */
        void (*debug)(const char* data, size_t size);       /**< Debug write. */
        void *priv;                                         /**< Private data structure for driver use. */
    } context_t;

    /** State of the device. */
    enum class State : uint8_t {
        none,           /**< None. */
        init,           /**< SIM is being initialized. */
        locked,         /**< SIM is locked. */
        offline,        /**< No signal. */
        online,         /**< Network registers the modem. */
        authenticating, /**< Attempting to connect to GPRS. */
        connected,      /**< Connected to GPRS. */
        handshaking,    /**< Attempting to open TCP socket. */
        idle,           /**< TCP socket is open and idle. */
        busy,           /**< TCP socket is open and busy. */ 
    };

    /** Initialize the driver.
     *
     * @param [in] context driver operating context.
     */
    void init(void *context);

    /** Clean up driver.
     *
     * @param [in] context driver operating context.
     */
    void deinit(void *context);

    /** Handle UART communication with the modem.
     *
     * @param [in] context driver operating context.
     * @param [in] micros microseconds elapsed since initialization.
     */
    void process(void *context, uint32_t micros);

    /** Returns the modem state.
     * 
     * @param [in] ctx driver operating context.
     */
    State status(context_t *ctx);

    /** Returns the modem's rssi value as reported by AT+CSQ.
     * 
     * @param [in] ctx driver operating context.
     * @warning value is encoded. Refer to modem documentation for actual values.
     */
    uint8_t signal(context_t *ctx);

    /** Unlocks the SIM card.
     * 
     * Must be in State::locked.
     * 
     * @param [in] ctx driver operating context.
     * @param [in] pin SIM card password
     */
    int unlock(context_t *ctx, const char *pin);

    /** Reset modem to factory defaults.
     * 
     * @param [in] ctx driver operating context.
     */
    int reset(context_t *ctx);

    /** Connect to GPRS.
     * 
     * Must be in State::online, transitions to State::connected.
     * 
     * @param [in] ctx driver operating context.
     * @param [in] apn GPRS access point name (max 50 bytes).
     * @param [in] user GPRS user name (max 50 bytes).
     * @param [in] pwd GPRS password (max 50 bytes).
     */
    int connect(context_t *ctx, const char *apn, const char *user="", const char *pwd="");

    /** Disconnect from GPRS
     * 
     * If a socket is open call close() first.
     * 
     * @param [in] ctx driver operating context.
     */
    int disconnect(context_t *ctx);

    /** Open a TCP socket.
     * 
     * Must be in State::connected, transitions to State::idle.
     * 
     * @param [in] ctx driver operating context.
     * @param [in] host server ip address.
     * @param [in] port server port number.
     */
    int open(context_t *ctx, const char *host, uint16_t port);

    /** Close TCP socket.
     * 
     * @param [in] ctx driver operating context.
     */
    int close(context_t *ctx);

    /** Write bytes to the TCP socket.
     * 
     * Must be in State::idle or State::busy
     * 
     * @param [in] ctx driver operating context.
     * @param [in] data buffer to write.
     * @param [in] size number of bytes to write.
     */ 
    int write(context_t *ctx, uint8_t *data, uint16_t size);

    /** Read bytes from the rx ring buffer.
     * 
     * Must be in State::idle or State::busy
     * 
     * @param [in] ctx driver operating context.
     * @param [out] data buffer to read into.
     * @param [in] size number of bytes to read.
     */ 
    int read(context_t *ctx, uint8_t *data, uint16_t size);

    /** Number of bytes available to read
     * 
     * @param [in] ctx driver operating context.
     */
    uint16_t available(context_t *ctx);

    /** Overload to write strings to the TCP socket.
     * 
     * @param [in] ctx driver operating context.
     * @param [in] data string to write.
     * @param [in] size length of string.
     */ 
    inline int write(context_t *ctx, char *data, uint8_t size)
        { return write(ctx, (uint8_t *)data, size); };
}

#endif /* NOVAGSM_H_ */
