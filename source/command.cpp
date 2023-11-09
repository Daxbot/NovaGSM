/**
 * @file command.h
 * @brief AT command buffer.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
 */

#include <cstdio>
#include <cstdarg>
#include <cstring>

#include "command.h"

namespace gsm {

Command::Command(uint32_t timeout, std::vector<uint8_t> &payload) :
        timeout_ms(timeout), payload(payload)
{
}

Command::Command(uint32_t timeout, const char *data) :
        timeout_ms(timeout)
{
    const size_t size = strlen(data);
    payload.reserve(size + 3);

    payload.push_back('A');
    payload.push_back('T');
    if (data != nullptr) {
        for (size_t i = 0; i < size; ++i)
            payload.push_back(data[i]);
    }
    payload.push_back('\r');
}

void Command::add(const void *data, size_t size)
{
    if (payload.size() == 3) {
        // 'AT\r' -> 'AT'
        payload.pop_back();
    }
    else {
        // Change the terminating '\r' character to a ';'
        payload.back() = ';';
    }

    // Append the data
    append(data, size);

    // Re-add the terminator
    payload.push_back('\r');
}

void Command::add(const char *data)
{
    add(data, strlen(data));
}

void Command::append(const void *data, size_t size)
{
    // Expand the vector to fit the new payload.
    size_t end = payload.size();
    payload.resize(end + size);

    // Append the new data
    memcpy(payload.data() + end, data, size);
}

} // namespace gsm
