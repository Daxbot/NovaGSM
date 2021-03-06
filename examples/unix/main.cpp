#include <cstdio>
#include <cstdlib>
#include <algorithm>
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
 * @return number of bytes read.
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
 * @return number of bytes written.
 */
int write(const void *data, int size)
{
    return ::write(fd, data, size);
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

    // Initialize the driver
    gsm::context_t ctx = {
        .read = read,
        .write = write,
        .elapsed_ms = millis,
    };

    gsm::Modem modem(&ctx);

    // Wait for registration
    while(!modem.registered())
        modem.process();

    // Activate data connection
    while(!modem.ready()) {
        if(!modem.authenticating())
            modem.authenticate("hologram", nullptr, nullptr, 10000);

        modem.process();
    }

    // Establish TCP connection
    while(!modem.connected()) {
        if(!modem.handshaking())
            modem.connect("www.httpbin.org", 80, 10000);

        modem.process();
    }

    // Send GET request
    char tx_buffer[64];
    int tx_count = snprintf(tx_buffer, sizeof(tx_buffer),
        "GET /ip HTTP/1.1\r\nHost: www.httpbin.org\r\n\r\n");

    modem.send(tx_buffer, tx_count);

    // Print response
    char rx_buffer[512];
    while(modem.connected()) {
        if(!modem.rx_busy()) {
            if(modem.rx_available()) {
                // Begin asynchronous receive
                int rx_count = std::min(
                    static_cast<size_t>(modem.rx_available()), sizeof(rx_buffer));

                modem.receive(rx_buffer, rx_count);
            }
            else if(modem.rx_count()) {
                // Receive completed - write data to stdout.
                fwrite(rx_buffer, modem.rx_count(), 1, stdout);
                fputc('\n', stdout);
                fflush(stdout);

                // Reset buffer
                modem.stop_receive();
                break;
            }
        }

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

    fprintf(stdout, "%s:%04d: |%d| %s", basename, line, level, str);
    fflush(stdout);
}

