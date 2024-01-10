/**
 * @file parser.h
 * @brief AT response packet parser.
 * @author Wilkins White
 * @copyright 2024 Nova Dynamics LLC
 */

#ifndef NOVAGSM_PARSER_H_
#define NOVAGSM_PARSER_H_

/**@{*/
/** Allows user to specify buffer size with -DGSM_BUFFER_SIZE. */
#ifndef NOVAGSM_BUFFER_SIZE
#define NOVAGSM_BUFFER_SIZE 556
#endif
/**@}*/

#if NOVAGSM_BUFFER_SIZE < 256
#warning NOVAGSM_BUFFER_SIZE must be at least 256
#endif

#include <cstdint>

namespace gsm {

/**
 * @brief Maximum size of an AT command and response.
 *
 * This can be set with -DNOVAGSM_BUFFER_SIZE (default 556).
 */
constexpr size_t kBufferSize = (NOVAGSM_BUFFER_SIZE);


typedef void (*parse_cb_t)(uint8_t *data, size_t size, void *user);

class Parser {
public:
    /**
     * @brief Register a function to be called on a successful parse.
     *
     * @param [in] parse_cb - parse callback.
     * @param [in] user - private data.
     */
    void set_parse_callback(parse_cb_t func, void *user=nullptr);

    /**
     * @brief Load incoming data.
     *
     * @param [in] data - buffer.
     * @param [in] size - length of buffer.
     */
    void load(const uint8_t *data, size_t size);

private:
    /**
     * @brief Attempt to parse a packet.
     * @param [in] data - buffer.
     * @param [in] size - length of buffer.
     * @return number of processed bytes.
     * @return EAGAIN if more data is needed.
     * @return EINVAL if parsing fails.
     */
    int try_parse(uint8_t *data, size_t size);

    /**
     * @brief Invoke the parse callback.
     *
     * @param [in] data - parsed data.
     * @param [in] size - length of data.
     */
    inline void emit_data(uint8_t *data, size_t size)
    {
        if(parse_cb)
            parse_cb(data, size, parse_cb_user);
    }

    /** Function called on a successful parse. */
    parse_cb_t parse_cb = nullptr;

    /** User data passed to parse callback. */
    void *parse_cb_user = nullptr;

    uint8_t buffer[kBufferSize];    /**< Data buffer. */
    size_t count = 0;               /**< The number of bytes in the buffer. */
    size_t head = 0;                /**< Response write pointer. */
    size_t tail = 0;                /**< Response read pointer. */
};

} // namespace gsm

#endif // NOVAGSM_PARSER_H_
