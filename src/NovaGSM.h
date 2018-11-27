/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file NovaGSM.h
 * @author Wilkins White
 * @copyright 2018 Nova Dynamics LLC
 */

#ifndef NOVAGSM_H_
#define NOVAGSM_H_

#include <stdint.h>
#include <string.h>

/** Handles buffered communication through a GSM/GPRS modem. */
namespace GSM
{
    /** Defines resources and callbacks used by the driver. */
    typedef struct {
        size_t (*available)();                          /**< Return the number of bytes that can be read from the device. */
        size_t (*read)(void *data, size_t size);        /**< Read 'size' bytes into 'data' from device. */
        void (*write)(const void *data, size_t size);   /**< Write 'size' bytes from 'data' to device. */
        uint32_t(*elapsed_ms)();                        /**< Return the milliseconds elapsed since initialization. */
        void *priv;                                     /**< Initialized by the driver for private use. */
    } context_t;

    /** State of the device.
     *
     * @see process() for a functional description.
     */
    enum class State : uint8_t {
        none,           /**< None. */
        init,           /**< SIM is being initialized. */
        locked,         /**< SIM is locked. */
        offline,        /**< No signal. */
        online,         /**< Network registers the modem. */
        authenticating, /**< Attempting to authenticate with GPRS. */
        ready,          /**< Connected to GPRS. */
        handshaking,    /**< Attempting to establish TCP connection. */
        open,           /**< TCP socket is open. */
    };

    /** Initialize the driver.
     *
     * @param [in] context driver operating context.
     * @warning assumes context can be statically cast to context_t;
     */
    void init(void *context);

    /** Clean up driver.
     *
     * @param [in] context driver operating context.
     * @warning assumes context can be statically cast to context_t;
     */
    void deinit(void *context);

    /** Handle communication with the modem.
     *
     * @param [in] context driver operating context.
     * @warning assumes context can be statically cast to context_t;
     */
    void process(void *context);

    /** Returns the device state.
     *
     * @param [in] ctx driver operating context.
     * @warning assumes ctx is not null.
     */
    State status(context_t *ctx);

    /** Returns if a GPRS connection is established.
     * 
     * @param [in] ctx driver operating context.
     * @warning assumes ctx is not null.
     */
    bool authenticated(context_t *ctx);

    /** Returns if a TCP connection is established.
     * 
     * @param [in] ctx driver operating context.
     * @warning assumes ctx is not null.
     */
    bool connected(context_t *ctx);

    /** Returns the rssi value reported by AT+CSQ.
     *
     * @param [in] ctx driver operating context.
     * @warning value is encoded. Refer to modem documentation for actual rssi values.
     * @warning assumes ctx is not null.
     */
    uint8_t signal(context_t *ctx);

     /** Unlocks the SIM card.
     *
     * Must be in State::locked.
     *
     * @param [in] ctx driver operating context.
     * @param [in] pin SIM card password
     * @return -EINVAL if inputs are null.
     * @return -ENOBUFS if command buffer is full.
     */
    int unlock(context_t *ctx, const char *pin);

    /** Connect to GPRS.
     *
     * Must be called in State::online to transition to State::ready.
     *
     * @param [in] ctx driver operating context.
     * @param [in] apn GPRS access point name (max 50 bytes).
     * @param [in] user GPRS user name (max 50 bytes).
     * @param [in] pwd GPRS password (max 50 bytes).
     * @return -EINVAL if inputs are null.
     * @return -ENODEV if the device is not responding.
     * @return -ENETUNREACH if the network is not available.
     * @return -EALREADY if authentication is already in progress.
     * @return -EISCONN if already connected to GPRS.
     * @return -ENOBUFS if command buffer is full.
     */
    int authenticate(context_t *ctx, const char *apn, const char *user="", const char *pwd="");

    /** Open a TCP socket.
     *
     * Must be in State::connected, transitions to State::open.
     *
     * @param [in] ctx driver operating context.
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
    int connect(context_t *ctx, const char *host, uint16_t port);

    /** Close TCP socket.
     *
     * @param [in] ctx driver operating context.
     * @return -EINVAL if ctx is null.
     * @return -ENODEV if the device is not responsive.
     * @return -ENETUNREACH if the network is not available.
     * @return -ENOTSOCK if a connection is not established.
     * @return -ENOBUFS if command buffer is full.
     */
    int disconnect(context_t *ctx);

    /** Discards unread data in the rx ring buffer.
     *
     * @param [in] ctx driver operating context.
     * @return -EINVAL if ctx is null.
     */
    int clear(context_t *ctx);

    /** Blocks until all bytes in the tx ring buffer have been sent or timeout.
     * 
     * @param [in] ctx driver operating context.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @return -EINVAL if ctx is null.
     * @return -ETIME if timeout is reached.
     */
    int flush(context_t *ctx, uint32_t timeout_ms=0);

    /** Reinitialize driver.
     *
     * Resets the context and transitions to State::none.
     *
     * @param [in] ctx driver operating context.
     * @return -EINVAL if ctx is null.
     */
    int reset(context_t *ctx);

    /** Number of bytes available to read().
     *
     * @param [in] ctx driver operating context.
     * @warning assumes ctx is not null.
     */
    uint16_t available(context_t *ctx);

    /** Read bytes from the rx ring buffer.
     *
     * @see available()
     * @param [in] ctx driver operating context.
     * @param [out] data buffer to read into.
     * @param [in] size number of bytes to read.
     * @warning assumes ctx is not null.
     */
    uint16_t read(context_t *ctx, uint8_t *data, uint16_t size);

    /** Read bytes from the rx ring buffer into a c-string.
     *
     * @param [in] ctx driver operating context.
     * @param [out] data buffer to read into.
     * @param [in] size number of bytes to read.
     * @warning assumes ctx is not null.
     */
    inline uint16_t read(context_t *ctx, char *data, uint16_t size)
        { return read(ctx, (uint8_t *)data, size); };
    
    /** Stages bytes to the tx ring buffer.
     *
     * @param [in] ctx driver operating context.
     * @param [in] data buffer to write.
     * @param [in] size number of bytes to write.
     * @warning assumes ctx is not null.
     */
    uint16_t write(context_t *ctx, const uint8_t *data, uint16_t size);

    /** Stages a c-string to the tx ring buffer.
     *
     * @param [in] ctx driver operating context.
     * @param [in] data string to write.
     * @param [in] size length of string.
     * @warning assumes ctx is not null.
     */
    inline uint16_t write(context_t *ctx, const char *data, uint16_t size)
        { return write(ctx, (uint8_t *)data, size); };

    /** Wait for GPRS connection to be established.
     *
     * @param [in] ctx driver operating context.
     * @param [in] apn GPRS access point name (max 50 bytes).
     * @param [in] user GPRS user name (max 50 bytes).
     * @param [in] pwd GPRS password (max 50 bytes).
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @return -EINVAL if inputs are null.
     * @return -ETIME if the connect times out.
     */
    int authenticate_sync(context_t *ctx, const char *apn, const char *user="", const char *pwd="", uint32_t timeout_ms=0);

    /** Wait for TCP connection to be established.
     *
     * Must call authenticate() first.
     *
     * @param [in] ctx driver operating context.
     * @param [in] host server ip address.
     * @param [in] port server port number.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @return -EINVAL if inputs are null.
     * @return -EPERM if authenticate() has not been called.
     * @return -ETIME if the connect times out.
     */
    int connect_sync(context_t *ctx, const char *host, uint16_t port, uint32_t timeout_ms=0);
    
    /** Synchronously reads bytes into a data buffer.
     *
     * Blocks until 'size' bytes are read or timeout.
     * 
     * @param [in] ctx driver operating context.
     * @param [out] data buffer to read into.
     * @param [in] size number of bytes to read.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @warning assumes ctx is not null.
     */
    uint16_t read_sync(context_t *ctx, uint8_t *data, uint16_t size, uint32_t timeout_ms=0);

    /** Synchronously reads bytes into a c-string.
     * 
     * Blocks until 'size' bytes are read or timeout.
     *
     * @param [in] ctx driver operating context.
     * @param [out] data buffer to read into.
     * @param [in] size number of bytes to read.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @warning assumes ctx is not null.
     */
    inline uint16_t read_sync(context_t *ctx, char *data, uint16_t size, uint32_t timeout_ms=0)
        { return read_sync(ctx, (uint8_t *)data, size, timeout_ms); };

    /** Synchronously stages bytes to the tx ring buffer.
     *
     * Blocks until 'size' bytes are staged or timeout.
     *
     * @param [in] ctx driver operating context.
     * @param [in] data buffer to write.
     * @param [in] size number of bytes to write.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @warning assumes ctx is not null.
     */
    uint16_t write_sync(context_t *ctx, const uint8_t *data, uint16_t size, uint32_t timeout_ms=0);

    /** Synchronously stages a c-string to the tx ring buffer.
     *
     * Blocks until 'size' bytes are staged or timeout.
     *
     * @param [in] ctx driver operating context.
     * @param [in] data string to write.
     * @param [in] size length of string.
     * @param [in] timeout_ms maximum milliseconds to wait (0 for infinite).
     * @warning assumes ctx is not null.
     */
    inline uint16_t write_sync(context_t *ctx, const char *data, uint16_t size, uint32_t timeout_ms=0)
        { return write_sync(ctx, (uint8_t *)data, size, timeout_ms); };
}

#endif /* NOVAGSM_H_ */
