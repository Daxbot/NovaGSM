/** Helper class for queuing commands.
 *
 * @file Buffer.h
 * @author Wilkins White
 * @copyright 2019 Nova Dynamics LLC
 * @version 2.0
 */

#ifndef _GSM_BUFFER_H_
#define _GSM_BUFFER_H_

#include <stdint.h>

/**@{*/
/** Allows user to specify buffer size with -DGSM_BUFFER_SIZE. */
#ifndef GSM_BUFFER_SIZE
#define GSM_BUFFER_SIZE 556
#endif
/**@}*/

#if GSM_BUFFER_SIZE < 256
#warning GSM_BUFFER_SIZE must be at least 256
#endif


/**@{*/
/** Allows user to specify command pool size with -DGSM_QUEUE_SIZE. */
#ifndef GSM_QUEUE_SIZE
#define GSM_QUEUE_SIZE 3
#endif
/**@}*/

#if GSM_QUEUE_SIZE < 2
#warning GSM_QUEUE_SIZE must be at least 2
#endif

namespace GSM
{
    /** How long to wait for a command response. */
    constexpr uint32_t DEFAULT_TIMEOUT_MS = 200;

    /** Represents an AT command to be sent. */
    typedef struct {
        int size;                       /**< Size of command. */
        uint32_t timeout_ms;            /**< Response timeout (msec). */
        uint8_t data[GSM_BUFFER_SIZE];  /**< Command buffer. */
    } command_t;

    /** Ring buffer holding queued commands. */
    class Buffer {
        public:
            /** Gets the next free command_t struct from the buffer. */
            inline command_t *front()
            {
                command_t *packet = &pool[head++];
                if(head >= GSM_QUEUE_SIZE)
                    head = 0;

                return packet;
            }

            /** Gets the next queued command_t struct. */
            inline command_t *back()
            {
                return &pool[tail];
            }

            /** Indicates that the HEAD command_t struct is ready to send. */
            inline void push()
            {
                if(count < GSM_QUEUE_SIZE)
                    count += 1;
            }

            /** Frees the TAIL command_t struct. */
            inline void pop()
            {
                if(count > 0) {
                    count -= 1;
                    if(++tail >= GSM_QUEUE_SIZE)
                        tail = 0;
                }
            }

            /** Frees all pending commands. */
            inline void clear()
            {
                while(count > 0)
                    pop();
            }

            /** Returns the number of elements in the buffer. */
            inline int size()
            {
                return count;
            }

            /** Returns if the buffer is empty. */
            inline bool empty()
            {
                return (size() == 0);
            }

            /** Returns if the buffer is full. */
            inline bool full()
            {
                return (size() >= GSM_QUEUE_SIZE);
            }

        private:
            int count = 0;  /**< Number of commands in the buffer. */
            int head = 0;   /**< Index of last pushed command. */
            int tail = 0;   /**< Index of last popped command. */

            /** Pre-allocated command_t pool. */
            command_t pool[GSM_QUEUE_SIZE] = {0};
    };
}

#endif /* _GSM_BUFFER_H_ */
