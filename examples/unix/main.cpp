#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>

#include "modem.h"

// File descriptor for the serial port.
int fd = -1;

const char *data = "Lorem ipsum dolor sit amet, consectetur adipiscing elit,"
"sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad"
"minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea"
"commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit"
"esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat"
"non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";

/**
 * @brief Read from the serial port.
 *
 * @param [in] data buffer to read into.
 * @param [in] size number of bytes to read.
 * @return number of bytes read.
 */
int read(void *data, int size)
{
    int count = ::read(fd, data, size);
    if(count > 0) {
        fputs("RX: ", stdout);
        fwrite(data, count, 1, stdout);
        fflush(stdout);
    }
    return count;
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
    int count = ::write(fd, data, size);
    if(count > 0) {
        fputs("TX: ", stdout);
        fwrite(data, count, 1, stdout);
        fflush(stdout);
    }
    return count;
}

/** Application entry point. */
int main()
{
    struct timespec last;
    clock_gettime(CLOCK_REALTIME, &last);

    // Open serial port
    fd = open("/dev/ttyUSB2", O_RDWR | O_NONBLOCK);
    if(fd < 0) {
        perror("Failed to open serial port");
        exit(1);
    }

    // Configure port
    struct termios settings;
    tcgetattr(fd, &settings);
    cfsetospeed(&settings, B115200);
    cfmakeraw(&settings);
    tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
    tcflush(fd, TCOFLUSH);

    // Initialize the driver
    gsm::context_t ctx = {
        .read = read,
        .write = write,
    };

    gsm::Modem modem(&ctx);

    // Wait for registration
    while(!modem.registered()) {
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        const unsigned int delta_us = (now.tv_sec - last.tv_sec) * 1e6
            + (now.tv_nsec - last.tv_nsec) / 1e3;

        if(delta_us > 0) {
            memcpy(&last, &now, sizeof(struct timespec));
            modem.process(delta_us);
        }
    }

    // Activate data connection
    while(!modem.ready()) {
        if(!modem.authenticating())
            modem.authenticate("hologram");

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        const unsigned int delta_us = (now.tv_sec - last.tv_sec) * 1e6
            + (now.tv_nsec - last.tv_nsec) / 1e3;

        if(delta_us > 0) {
            memcpy(&last, &now, sizeof(struct timespec));
            modem.process(delta_us);
        }
    }

    // Establish TCP connection
    while(!modem.connected()) {
        if(!modem.handshaking())
            modem.connect("www.httpbin.org", 80);

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        const unsigned int delta_us = (now.tv_sec - last.tv_sec) * 1e6
            + (now.tv_nsec - last.tv_nsec) / 1e3;

        if(delta_us > 0) {
            memcpy(&last, &now, sizeof(struct timespec));
            modem.process(delta_us);
        }
    }

    // Send GET request
    char tx_buffer[1024];
    int tx_count = snprintf(tx_buffer, sizeof(tx_buffer),
        "GET /anything HTTP/1.1\r\n"
        "Host: www.httpbin.org\r\n"
        "data: \"%s\"\r\n\r\n",
        data);

    modem.send(tx_buffer, tx_count);

    // Print response
    char rx_buffer[1024];
    while(modem.connected()) {
        if(!modem.rx_busy()) {
            if(modem.rx_available()) {
                // Begin asynchronous receive
                int rx_count = std::min(
                    static_cast<size_t>(modem.rx_available()), sizeof(rx_buffer));

                modem.receive(rx_buffer, rx_count);
            }
            else if(modem.rx_count() > 0) {
                // Receive completed - write data to stdout.
                fwrite(rx_buffer, modem.rx_count(), 1, stdout);
                fputc('\n', stdout);
                fflush(stdout);

                // Reset buffer
                modem.stop_receive();
                break;
            }
        }

        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        const int delta_us = (now.tv_sec - last.tv_sec) * 1e6
            + (now.tv_nsec - last.tv_nsec) / 1e3;

        if(delta_us > 0) {
            memcpy(&last, &now, sizeof(struct timespec));
            modem.process(delta_us);
        }
    }

    close(fd);
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

