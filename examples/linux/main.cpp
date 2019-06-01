#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "Modem.h"
#include "time.h"

// TCP credentials
constexpr char const *HOST = "127.0.0.1";
constexpr int PORT = 1883;

/** Read from STDIN. */
static int ctx_read(void *data, uint32_t size)
{
    return read(STDIN_FILENO, data, size);
}

/** Write to STDOUT. */
static int ctx_write(const void *data, uint32_t size)
{
    return write(STDOUT_FILENO, data, size);
}

/** Returns time since initialization. */
static uint32_t ctx_millis()
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (now.tv_sec*1e9 + (now.tv_nsec))/1e6;
}

/** Called on state changes. */
static void device_callback(GSM::State state, void *user)
{
    GSM::Modem *modem = static_cast<GSM::Modem*>(user);
    switch(state) {
        case GSM::State::offline:
            modem->disconnect();
            break;
        case GSM::State::online:
            modem->authenticate();
            break;
        case GSM::State::ready:
            modem->connect(HOST, PORT);
            break;
        default:
            break;
    }
}

void socket_callback(GSM::Event event, void *user)
{
    bool *flag = static_cast<bool*>(user);
    if(event == GSM::Event::rx_complete || event == GSM::Event::rx_error)
        *flag = true;
}

int main()
{
    GSM::context_t ctx;
    ctx.read = ctx_read;
    ctx.write = ctx_write;
    ctx.elapsed_ms = ctx_millis;

    // Initialize the driver
    GSM::Modem modem(&ctx);

    // Create a flag to indicate the read is done
    bool rx_complete = false;

    // Register callback
    modem.set_device_callback(device_callback, &modem);
    modem.set_socket_callback(socket_callback, &rx_complete);

    // Create a buffer to hold incoming data
    char buffer[100];

    while(1)
    {
        size_t count = modem.rx_available();
        if(count > 0) {
            rx_complete = false;
            modem.receive(buffer, sizeof(buffer));

            // Wait for async read
            while(!rx_complete) {}

            printf("got data: %.*s\n", modem.rx_count(), buffer);

        }

        modem.process();
    }

    return 0;
}

/** DEBUG print function
 * 
 * Only required when compiled with -DGSM_DEBUG flag
 */

void gsm_debug(int level, const char *file, int line, const char *str)
{
    const char *p, *basename;

    /* Extract basename from file */
    for(p = basename = file; *p != '\0'; p++) {
        if(*p == '/' || *p == '\\') {
            basename = p + 1;
        }
    }

    printf("%s:%04d: |%d| %s", basename, line, level, str);
}

