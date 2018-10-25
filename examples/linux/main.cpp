#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "NovaGSM.h"
#include "time.h"

// GPRS credentials
constexpr char const *apn  = "wholesale";
constexpr char const *user = "";
constexpr char const *pwd = "";

// TCP credentials
constexpr char const *host = "127.0.0.1";
constexpr uint16_t port = 1883;

/** GSM::context_t functions. */
namespace
{
    /** Return the number of bytes that can be read from the device. */
    size_t ctx_available()
    {
        int n;
        ioctl(STDIN_FILENO, FIONREAD, &n);
        return (n > 0) ? n : 0;
    }

    /** Read 'size' bytes into 'data' from device.
     * 
     * @param [out] data buffer to read into.
     * @param [in] size maximum size of data buffer.
     * @return number of bytes read (<= size).
    */
    size_t ctx_read(void *data, size_t size)
    {
        return read(STDIN_FILENO, data, size);
    }

    /** Write 'size' bytes from 'data' to device.
     * 
     * @param [in] data buffer to write.
     * @param [in] size number of bytes to write.
    */
    void ctx_write(const void *data, size_t size)
    {
        write(STDIN_FILENO, data, size);
    }

    uint32_t ctx_millis()
    {
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        return (now.tv_sec*1e9 + (now.tv_nsec))/1e6;
    }
}

int main()
{
    GSM::context_t ctx;
    ctx.available = ctx_available;
    ctx.read = ctx_read;
    ctx.write = ctx_write;
    ctx.elapsed_ms = ctx_millis;

    // Initialize the driver
    GSM::init(&ctx);

    // Wait for GPRS connection (use authenticate() for async)
    GSM::authenticate_sync(&ctx, apn, user, pwd);

    // Wait for TCP connection (use connect() for async)
    GSM::connect_sync(&ctx, host, port);

    while(1)
    {
        if(GSM::connected(&ctx))
        {
            while(GSM::available(&ctx))
            {
                uint8_t c;
                GSM::read(&ctx, &c, 1);
                GSM::write(&ctx, &c, 1); // echo
                putchar(c);
            }
        }

        GSM::process(&ctx);
    }

    GSM::deinit(&ctx);
    return 0;
}

/** DEBUG print function
 * 
 * Only required when compiled with -DDEBUG flag
 */
void gsm_debug(const char *data, size_t size)
{
    fwrite(data, sizeof(char), size, stderr);
}