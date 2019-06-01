/** Helper class for queuing commands.
 *
 * @file Buffer.h
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>

/**@{*/
/** Allows user to specify command buffer size with -DGSM_BUFFER_SIZE. */
#ifndef GSM_BUFFER_SIZE
#define GSM_BUFFER_SIZE 1024
#endif
/**@}*/

/**@{*/
/** Allows user to specify command pool size with -DGSM_POOL_SIZE. */
#ifndef GSM_POOL_SIZE
#define GSM_POOL_SIZE 10
#endif
/**@}*/

namespace GSM
{
    /** The maximum number of queued commands. */
    constexpr size_t POOL_SIZE = GSM_POOL_SIZE;

    /** The maximum size of a command. */
    constexpr size_t BUFFER_SIZE = GSM_BUFFER_SIZE;

    /** How long to wait for a command response. */
    constexpr size_t TIMEOUT_MS = 200;

    /** Represents an AT command to be sent. */
    typedef struct {
        size_t timeout_ms;              /**< How long to wait for a response (milliseconds). */
        size_t size;                    /**< Size of message data. */
        uint8_t data[BUFFER_SIZE];      /**< Message data. */
    } command_t;

    /** Ring buffer holding queued commands. */
    class Buffer {
        public:
            /** Gets the next free command_t struct from the buffer. */
            inline command_t *front() {
                command_t *packet = &pool[head++];
                if(head >= POOL_SIZE)
                    head = 0;

                packet->size = 0;
                packet->timeout_ms = TIMEOUT_MS;

                return packet;
            }

            /** Gets the next queued command_t struct. */
            inline command_t *back() {
                return &pool[tail];
            }

            /** Indicates that the HEAD command_t struct is ready to send. */
            inline void push() {
                if(count < POOL_SIZE)
                    count += 1;
            }

            /** Frees the TAIL command_t struct. */
            inline void pop() {
                if(count > 0) {
                    count -= 1;
                    if(++tail >= POOL_SIZE)
                        tail = 0;
                }
            }

            /** Frees all pending commands. */
            inline void clear() {
                while(count > 0)
                    pop();
            }

            /** Returns the number of elements in the buffer. */
            inline size_t size() {
                return count;
            }

            /** Returns if the buffer is empty. */
            inline bool empty() {
                return (size() == 0);
            }

            /** Returns if the buffer is full. */
            inline bool full() {
                return (size() >= POOL_SIZE);
            }

        private:
            size_t count = 0;               /**< Number of commands in the buffer. */
            size_t head = 0;                /**< Index of last pushed command. */
            size_t tail = 0;                /**< Index of last popped command. */
            command_t pool[POOL_SIZE];      /**< Pre-allocated command_t pool. */
    };
}

#endif /* BUFFER_H_ */
