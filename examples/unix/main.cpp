#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "modem.h"

// File descriptor for the serial port.
int fd = -1;

/** Returns time since initialization. */
uint32_t millis()
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (now.tv_sec*1e9 + (now.tv_nsec))/1e6;
}

/**
 * @brief Read from the serial port.
 *
 * @param [in] data buffer to read into.
 * @param [in] size number of bytes to read.
 */
int read(void *data, int size)
{
    return ::read(fd, data, size);
}

/**
 * @brief Write to the serial port.
 *
 * @param [in] data buffer to write.
 * @param [in] size number of bytes to write.
 */
int write(const void *data, int size)
{
    return ::write(fd, data, size);
}

/**
 * @brief State change callback.
 *
 * @param [in] state new state of the modem.
 * @param [in] user private data.
 */
void device_callback(gsm::State state, void *user)
{
    gsm::Modem *modem = static_cast<gsm::Modem*>(user);
    switch(state) {
        case gsm::State::offline:
            /**
             * If the modem is in minimum functionality mode call reset().
             */
            modem->reset();
            break;
        case gsm::State::registered:
            /**
             * Once we are connected to the network call authenticate() to
             * activate the data connection.
             */
            modem->authenticate("hologram");
            break;
        case gsm::State::ready:
            /**
             * Once the GPRS initalization is done we can establish a TCP
             * connection.
             */
            modem->connect("cloudsocket.hologram.io", 9999);
            break;
        default:
            break;
    }
}

/** Application entry point. */
int main()
{
    // Open serial port
    fd = open("/dev/ttyUSB2", O_RDWR | O_NONBLOCK);
    if(fd < 0) {
        perror("Failed to open serial port");
        exit(1);
    }

    gsm::context_t ctx = {
        .read = read,
        .write = write,
        .elapsed_ms = millis,
    };

    // Initialize the driver
    gsm::Modem modem(&ctx);

    modem.set_state_callback(device_callback, &modem);

    while(1) {
        modem.process();
    }

    return 0;
}

/**
 * @brief Debug print function.
 *
 * Only required when the library is compiled with -DNOVAGSM_DEBUG flag
 *
 * @param [in] level the log level of the message.
 * @param [in] file where in the source the message originated.
 * @param [in] line where in 'file' the message originated.
 * @param [in] str message c-string
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

