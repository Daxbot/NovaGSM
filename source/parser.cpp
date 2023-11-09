/**
 * @file parser.cpp
 * @brief AT response packet parser.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
 */

#include <cassert>
#include <cstring>
#include <cerrno>

#include "parser.h"

namespace gsm {

void Parser::load(const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        buffer[head++] = data[i];
        count += 1;

        if (head >= kBufferSize) {
            // Shift unprocessed bytes back to index 0
            memmove(buffer, buffer + tail, count);
            tail = 0;
            head = count;
            continue;
        }

        while (count > 0) {
            int result = try_parse(buffer + tail, count);
            if (result > 0) {
                // Successful parse
                tail += result;
                count -= result;
            }
            else if (result == -EINVAL) {
                // Invalid packet
                tail += 1;
                count -= 1;
            }
            else if (result == -EAGAIN) {
                // Need more data
                break;
            }
        }

        // Prepare for the next packet
        if (count == 0) {
            head = 0;
            tail = 0;
        }
    }
}

void Parser::set_parse_callback(parse_cb_t func, void *user)
{
    parse_cb = func;
    parse_cb_user = user;
}

int Parser::try_parse(uint8_t *data, size_t size)
{
    uint8_t *end = static_cast<uint8_t*>(memchr(data, '\n', size));
    if (end == nullptr) {
        if (*data == '>') {
            emit_data(data, 1);
            return 1;
        }
        else {
            return -EAGAIN;
        }
    }

    const size_t length = (end - data) + 1;
    if (length < 4)
        return -EINVAL;

    emit_data(data, length);

    return length;
}

} // namespace gsm
