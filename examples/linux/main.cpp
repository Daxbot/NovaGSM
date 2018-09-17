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

int main()
{
    static struct timespec init, now;
    clock_gettime(CLOCK_REALTIME, &init);

    GSM::context_t ctx;
    ctx.available = ctx_available;
    ctx.read = ctx_read;
    ctx.write = ctx_write;

    GSM::init(&ctx);

    while(1)
    {
        switch(GSM::status(&ctx))
        {
            case GSM::State::none:
                // Modem has not been identified
                break;
            case GSM::State::init:
                // SIM is being initialized
                break;
            case GSM::State::locked:
                // SIM is locked!
                // GSM::unlock(&ctx, "1234");
                break;
            case GSM::State::offline:
                // Modem is offline
                break;
            case GSM::State::online:
                // Online!  Call connect() to authenticate with GPRS.
                GSM::connect(&ctx, apn, user, pwd);
                break;
            case GSM::State::authenticating:
                // Authentication in progress
                break;
            case GSM::State::connected:
                // Connected to GPRS!  Call open() to create a TCP socket.
                GSM::open(&ctx, host, port);
                break;
            case GSM::State::handshaking:
                // TCP handshaking in progress
                break;
            case GSM::State::busy:
            case GSM::State::idle:
                // Socket open!
                while(GSM::available(&ctx))
                {
                    uint8_t c;
                    GSM::read(&ctx, &c, 1);
                    GSM::write(&ctx, &c, 1); // echo
                    putchar(c);
                }
                break;
        }

        // Calculate elapsed time between calls.
        clock_gettime(CLOCK_REALTIME, &now);
        const size_t nanos = 
            ((now.tv_sec-init.tv_sec)*1e9 + (now.tv_nsec-init.tv_nsec));

        // Call process with the elapsed time
        GSM::process(&ctx, nanos/1000);
    }

    GSM::deinit(&ctx);
    return 0;
}

void gsm_debug(const char *data, size_t size)
{
    fwrite(data, sizeof(char), size, stderr);
}