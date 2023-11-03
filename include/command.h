/**
 * @file command.h
 * @brief AT command buffer.
 * @author Wilkins White
 * @copyright 2023 Nova Dynamics LLC
 */

#ifndef NOVAGSM_COMMAND_H_
#define NOVAGSM_COMMAND_H_

#include <cstdint>
#include <vector>

namespace gsm {

/** How long to wait for a command response. */
constexpr uint32_t kDefaultTimeout = 1000;

/** Modem command object. */
class Command
{
public:
    /**
     * @brief Constructor.
     *
     * @param [in] timeout - maximum time to wait for a response (ms).
     * @param [in] data - command payload buffer.
     */
    Command(uint32_t timeout = kDefaultTimeout, const char *data = nullptr);

    /**
     * @brief Constructor.
     *
     * @param [in] timeout - maximum time to wait for a response (ms).
     * @param [in] payload - command payload buffer.
     */
    Command(uint32_t timeout, std::vector<uint8_t> &payload);

    /**
     * @brief Add a command.
     *
     * @param [in] data - command to add.
     * @param [in] size - data size.
     */
    void add(const void *data, size_t size);

    /**
     * @brief Add a command.
     *
     * @param [in] data - command to add.
     */
    void add(const char *data);

    /**
     * @brief Append arbitrary data.
     *
     * @param [in] data - buffer to append.
     * @param [in] size - buffer size.
     */
    void append(const void *data, size_t size);

    /**
     * @brief Return the data pointer.
     */
    inline const uint8_t* data() const
    {
        return payload.data();
    }

    /**
     * @brief Return the payload size in bytes.
     */
    inline size_t size() const
    {
        return payload.size();
    }

    /**
     * @brief Return the command timeout in milliseconds.
     */
    inline uint32_t timeout() const
    {
        return timeout_ms;
    }

private:
    uint32_t timeout_ms; /**< Response timeout (ms). */
    std::vector<uint8_t> payload; /**< Command payload. */
};

} // namespace gsm

#endif // NOVAGSM_COMMAND_H_
