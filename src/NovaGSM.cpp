/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file NovaGSM.cpp
 * @author Wilkins White
 * @copyright 2018 Nova Dynamics LLC
 */

#include <algorithm>
#include <cstdio>
#include <errno.h>

#include "NovaGSM.h"

/**@{*/
/** Debug print. */
#ifdef DEBUG
    extern void gsm_debug(const char *data, size_t size);
    #define GSM_DEBUG gsm_debug
#else
    #define GSM_DEBUG(x,y) do{} while(0)
#endif
/**@}*/

/** Allows user to specify ring buffer size with -DGSM_BUFFER_SIZE. */
#ifndef GSM_BUFFER_SIZE
#define GSM_BUFFER_SIZE 1460
#endif

/**
 * @see GSM::State
 *
 * @dot
 * digraph {
 *     none -> init                 [ label = "ATI" ];
 *     init -> none                 [ label = "reset()" ];
 *     init -> locked               [ label = "SIM locked" ];
 *     init -> offline              [ label = "SIM ready" ];
 *     locked -> none               [ label = "reset()" ];
 *     locked -> offline            [ label = "unlock()" ];
 *     offline -> none              [ label = "reset()" ];
 *     offline -> online            [ label = "" ];
 *     online -> none               [ label = "reset()" ];
 *     online -> offline            [ label = "" ];
 *     online -> authenticating     [ label = "authenticate()" ];
 *     authenticating -> online     [ label = "error/\ntimeout" ];
 *     authenticating -> ready      [ label = "OK" ];
 *     ready -> none                [ label = "reset()" ];
 *     ready -> offline             [ label = "lost\nsignal" ];
 *     ready -> handshaking         [ label = "connect()" ];
 *     handshaking -> ready         [ label = "error/\ntimeout" ];
 *     handshaking -> open          [ label = "OK" ];
 *     open -> none                 [ label = "reset()" ];
 *     open -> offline              [ label = "lost\nsignal" ];
 *     open -> ready                [ label = "disconnect()" ];
 * }
 * @enddot
 */

namespace GSM
{
    namespace // Anonymous
    {
        constexpr uint32_t DEFAULT_TIMEOUT_MS = 200;        /**< Default command timeout milliseconds. */
        constexpr uint32_t SEND_PERIOD_MS = 20;             /**< Delay between subsequent commands sent to the modem. */
        constexpr uint16_t BUFFER_SIZE = GSM_BUFFER_SIZE;   /**< Size of the pending data ring buffers. */
        constexpr uint16_t COMMAND_SIZE = 200;              /**< Size of a command data buffer. */
        constexpr uint16_t ID_SIZE = 20;                    /**< Size of the modem ID string buffer. */
        constexpr uint8_t POOL_SIZE = 10;                   /**< Number of pre-allocated command_t structs in the command_buffer_t pool. */
        constexpr uint8_t MAX_ERRORS = 20;                  /**< Communication errors before modem is considered not connected. */

        /** State of data transmission when socket is open. */
        enum class SocketState {
            idle,
            query,
            stage,
            transmit,
            receive,
        };

        /** Represents an AT command to be sent. */
        typedef struct {
            uint32_t timeout_ms;            /**< How long to wait for a response (milliseconds). */
            uint8_t data[COMMAND_SIZE];     /**< Message data. */
            uint8_t size;                   /**< Size of message data. */
        } command_t;

        /** Ring buffer holding queued commands. */
        typedef struct {
            command_t *pending;             /**< Most recent command awaiting response. */
            command_t pool[POOL_SIZE];      /**< Pre-allocated command_t pool. */
            uint8_t count;                  /**< Number of commands in the buffer. */
            uint8_t head;                   /**< Index of last pushed command. */
            uint8_t tail;                   /**< Index of last popped command. */
            uint32_t timer;                 /**< Time the pending command will expire (ctx->milliseconds). */
        } command_buffer_t;

        /** Ring buffer to hold user data. */
        typedef struct {
            uint8_t data[BUFFER_SIZE];      /**< Pending bytes. */
            uint16_t count;                 /**< Count of pending bytes. */
            uint16_t head;                  /**< Index of next write. */
            uint16_t tail;                  /**< Index of next read. */
        } data_buffer_t;

        /** Private data structure stored in context_t. */
        typedef struct {
            command_buffer_t cmd_buffer;        /**< Queued commands to send. */
            data_buffer_t tx_buffer;            /**< Outgoing user data. */
            data_buffer_t rx_buffer;            /**< Incoming user data. */
            uint32_t update_timer;              /**< Time of the next state update. */
            State state;                        /**< State of the modem. */
            SocketState socket_state;           /**< State of data transmission through the socket. */
            uint8_t signal;                     /**< Signal rssi value reported by AT+CSQ. */
            uint8_t errors;                     /**< Timeout error counter.  If it exceeds MAX_ERRORS call reset(). */
            char id[ID_SIZE];                   /**< Identification string reported by ATI. */
        } modem_t;

        /** Gets the next free command_t struct from the buffer.
         * 
         * @param [in] buffer context command buffer.
         */
        inline command_t *command_front(command_buffer_t *buffer)
        {
            command_t *packet = &buffer->pool[buffer->head++];
            if(buffer->head >= POOL_SIZE)
                buffer->head = 0;

            packet->size = 0;
            packet->timeout_ms = DEFAULT_TIMEOUT_MS;

            return packet;
        }

        /** Indicates that the HEAD command_t struct is ready to send.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void command_push(command_buffer_t *buffer)
        {
            buffer->count += 1;
        }

        /** Frees the TAIL command_t struct.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void command_pop(command_buffer_t *buffer)
        {
            buffer->pending = nullptr;
            buffer->count -= 1;
            if(++buffer->tail >= POOL_SIZE)
                buffer->tail = 0;
        }

        /** Frees all pending commands.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void command_clear(command_buffer_t *buffer)
        {
            while(buffer->count > 0)
                command_pop(buffer);
        }

        /** Buffer string search.
         * 
         * @param [in] data buffer to search.
         * @param [in] target string to find.
         * @param [in] size maximum bytes to search.
         */
        void *memstr(void *data, const char *target, size_t size)
        {
            char *pch = static_cast<char *>(data);
            const size_t len = strlen(target);

            if(len <= size)
            {
                for(; data != (pch-size); pch++)
                {
                    if(*pch == target[0] && memcmp(pch, target, len) == 0)
                        return static_cast<void *>(pch);
                }
            }
            
            return nullptr;
        }
    }

    void init(void *context)
    {
        context_t *ctx = static_cast<context_t*>(context);
        ctx->priv = static_cast<void*>(new modem_t);
        reset(ctx);
    }

    void deinit(void *context)
    {
        context_t *ctx = static_cast<context_t*>(context);
        delete static_cast<modem_t*>(ctx->priv);
    }

    void process(void *context)
    {
        context_t *ctx = static_cast<context_t*>(context);
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_buffer_t *cmd_buffer = &modem->cmd_buffer;

        /**
         * The process loop handles communication with the modem and transitions
         * between device states. If there is not a command already awaiting a 
         * response then the next command in the buffer is sent.  Responses are
         * handled based on the GSM::State.  Any pending packets that exceed their
         * timeout value are discarded and the error counter incremented.  
         *
         * If the error counter exceeds MAX_ERRORS the modem is assumed to be MIA and
         * the driver returns to State::none.  The error counter is reset whenever a
         * response is received.
         * 
         * If the signal is lost the connection is assumed to be broken and the driver
         * returns to State::offline.  The user must call authenticate() and connect() again
         * to re-establish.
         */

        uint32_t elapsed_ms = ctx->elapsed_ms();

        if(cmd_buffer->pending)
        {
            // If there is a command pending, check for a response.
            const size_t available = ctx->available();
            if(available)
            {
                command_t *pending = cmd_buffer->pending;
                uint8_t *pch = &pending->data[pending->size];
                const size_t count = ctx->read(pch, available);;

                pending->size += count;
                if(pending->size < 4)
                    return;

                // Reset error counter
                modem->errors = 0;

                switch(modem->state)
                {
                    case State::none:
                        /** State::none - wait for the modem to respond to an ATI query and transition to State::init. */
                        pch = static_cast<uint8_t *>(memstr(pending->data, "ATI", pending->size));
                        if(pch)
                        {
                            const uint8_t size = pending->size - (pch - pending->data);
                            if(memstr(pch, "OK\r\n", size))
                            {
                                uint8_t *data_start = static_cast<uint8_t *>(memchr(pch, '\n', size));
                                uint8_t *data_end = static_cast<uint8_t *>(memchr(data_start+1, '\r', size - (data_start-pch)));

                                // ATI\r\n%s\r\nOK\r\n
                                // │     │   │
                                // │     │   └ data_end
                                // │     └ data_start
                                // └ pch

                                uint8_t data_size = (data_end - data_start);
                                if(data_size > ID_SIZE)
                                    data_size = ID_SIZE;

                                memcpy(modem->id, data_start, data_size);

                                GSM_DEBUG("Modem is: ", 10);
                                GSM_DEBUG(modem->id, data_size);
                                GSM_DEBUG("\r\n", 2);

                                // AT&F0 - reset to factory defaults
                                command_t *command = command_front(cmd_buffer);
                                command->size = sprintf((char*)command->data, "AT&F0\r\n");
                                command_push(cmd_buffer);

                                // AT+CIURC=0 - disable unsolicited result codes
                                command = command_front(cmd_buffer);
                                command->size = sprintf((char*)command->data, "AT+CIURC=0\r\n");
                                command_push(cmd_buffer);

                                // AT+CLTS=1 - enable local timestamps
                                command = command_front(cmd_buffer);
                                command->size = sprintf((char*)command->data, "AT+CLTS=1\r\n");
                                command_push(cmd_buffer);

                                // AT+CFUN=1,1 - reset phone module
                                command = command_front(cmd_buffer);
                                command->size = sprintf((char*)command->data, "AT+CFUN=1,1\r\n");
                                command->timeout_ms = 10000;
                                command_push(cmd_buffer);

                                modem->state = State::init;
                                command_pop(cmd_buffer);
                            }
                        }
                        break;
                    case State::init:
                        /** State::init - query the SIM card and transition to State::offline or State::locked. */
                    case State::locked:
                        /** State::locked - SIM is locked, call unlock() with the password to transition to State::offline. */
                        if(memstr(pending->data, "OK\r\n", pending->size))
                        {
                            // +CPIN: %s\r\nOK\r\n
                            if(memstr(pending->data, "+CPIN: SIM PIN", pending->size)
                            || memstr(pending->data, "+CPIN: SIM PUK", pending->size))
                            {
                                GSM_DEBUG("SIM locked.\r\n", 13);
                                modem->state = State::locked;
                            }
                            else if(memstr(pending->data, "+CPIN: READY", pending->size))
                            {
                                GSM_DEBUG("SIM ready.\r\n", 12);
                                modem->state = State::offline;
                            }
                            command_pop(cmd_buffer);
                        }
                        break;
                    case State::offline:
                        /** State::offline - wait for a signal and transition to State::online. */
                        pch = static_cast<uint8_t *>(memstr(pending->data, "+CREG:", pending->size));
                        if(pch)
                        {
                            const uint8_t size = pending->size - (pch - pending->data);
                            if(memstr(pch, "OK\r\n", size))
                            {
                                char const *data = static_cast<char *>(memchr(pch, ',', size));

                                // +CREG: %d,%d,%s,%s\r\nOK\r\n
                                // │        │
                                // │        └ data
                                // └ pch

                                const uint8_t status = strtoul(data+1, nullptr, 0);
                                if(status == 1 || status == 5)
                                {
                                    GSM_DEBUG("Modem online.\r\n", 15);
                                    modem->state = State::online;
                                }
                                else modem->signal = 99;

                                command_pop(cmd_buffer);
                            }
                        }
                        break;
                    case State::online:
                        /** State::online - registered on network, wait for authenticate() to transition to State::authenticating. */
                        pch = static_cast<uint8_t *>(memstr(pending->data, "+CSQ:", pending->size));
                        if(pch)
                        {
                            const uint8_t size = pending->size - (pch - pending->data);
                            if(memstr(pch, "OK\r\n", size))
                            {
                                char const *data = static_cast<char *>(memchr(pch, ':', size));

                                // +CSQ: %d,%d\r\nOK\r\n
                                // │   │
                                // │   └ data
                                // └ pch

                                modem->signal = strtoul(data+1, nullptr, 0);
                                if(!modem->signal || modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                command_pop(cmd_buffer);
                            }
                        }
                        break;
                    case State::authenticating:
                        /** State::authenticating - handle authenticate() and transition to State::ready on success. */
                        if(memstr(pending->data, "OK\r\n", pending->size))
                        {
                            if(memstr(pending->data, "+CIFSR", pending->size))
                            {
                                GSM_DEBUG("Authentication success.\r\n", 25);
                                modem->state = State::ready;
                            }

                            command_pop(cmd_buffer);
                            break;
                        }
                        else if(memstr(pending->data, "ERROR", pending->size))
                        {
                            GSM_DEBUG("Authentication failed.\r\n", 24);
                            modem->state = State::online;
                            command_clear(cmd_buffer);
                        }
                        break;
                    case State::ready:
                        /** State::ready - ready to GPRS, wait for connect() to transition to State::handshaking. */
                        pch = static_cast<uint8_t *>(memstr(pending->data, "+CGATT:", pending->size));
                        if(pch)
                        {
                            const uint8_t size = pending->size - (pch - pending->data);
                            if(memstr(pch, "OK\r\n", size))
                            {
                                char const *data = static_cast<char *>(memchr(pch, ':', size));

                                // +CGATT: %d\r\nOK\r\n
                                // │     │
                                // │     └ data
                                // └ pch

                                const uint8_t status = strtoul(data+1, nullptr, 0);

                                if(status)
                                {
                                    // AT+CSQ - signal quality report
                                    command_t *command = command_front(cmd_buffer);
                                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                                    command_push(cmd_buffer);
                                }
                                else
                                {
                                    GSM_DEBUG("Disconnected from GPRS.\r\n", 25);
                                    modem->state = State::online;
                                }
                                command_pop(cmd_buffer);
                            }
                            break;
                        }

                        pch = static_cast<uint8_t *>(memstr(pending->data, "+CSQ:", pending->size));
                        if(pch)
                        {
                            const uint8_t size = pending->size - (pch - pending->data);
                            if(memstr(pch, "OK\r\n", size))
                            {
                                char const *data = static_cast<char *>(memchr(pch, ':', size));

                                // +CSQ: %d,%d\r\nOK\r\n
                                // │   │
                                // │   └ data
                                // └ pch

                                modem->signal = strtoul(data+1, nullptr, 0);
                                if(!modem->signal || modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                command_pop(cmd_buffer);
                            }
                            break;
                        }
                        break;
                    case State::handshaking:
                        /** State::handshaking - handle connect() and transition to State::open on success. */
                        if(memstr(pending->data, "CONNECT OK", pending->size)
                        || memstr(pending->data, "ALREADY CONNECT", pending->size))
                        {
                            GSM_DEBUG("TCP connection established.\r\n", 29);
                            modem->state = State::open;
                            command_pop(cmd_buffer);
                            break;
                        }
                        else if(memstr(pending->data, "CONNECT FAIL", pending->size))
                        {
                            GSM_DEBUG("TCP connection failed.\r\n", 24);
                            command_pop(cmd_buffer);
                            break;
                        }
                        else if(memstr(pending->data, "+CGATT:", pending->size))
                            command_pop(cmd_buffer);
                        else if(memstr(pending->data, "+CSQ:", pending->size))
                            command_pop(cmd_buffer);
                        break;
                    case State::open:
                        /** State::open - socket is established, handle write() and listen for incoming data */
                        if(memstr(pending->data, "CLOSE OK", pending->size))
                        {
                            GSM_DEBUG("TCP socket disconnected.\r\n", 26);
                            modem->state = State::ready;
                            command_clear(cmd_buffer);

                            modem->tx_buffer.tail = modem->tx_buffer.head;
                            modem->tx_buffer.count = 0;
                            break;
                        }

                        switch(modem->socket_state)
                        {
                            case SocketState::idle:
                                if(memstr(pending->data, "STATE: CONNECT OK", pending->size))
                                {
                                    if(modem->tx_buffer.count > 0)
                                    {
                                        // AT+CIPSEND=[size] - indicate that data is about to be sent
                                        command_t *command = command_front(cmd_buffer);
                                        command->size = sprintf((char*)command->data, "AT+CIPSEND=%u\r\n", modem->tx_buffer.count);
                                        command_push(cmd_buffer);

                                        #ifdef DEBUG
                                        char buf[100];
                                        int len = sprintf(buf, "Stage %d bytes\r\n", modem->tx_buffer.count);
                                        GSM_DEBUG(buf, len);
                                        #endif

                                        modem->socket_state = SocketState::stage;
                                    }
                                    else
                                    {
                                        // AT+CIPRXGET=4 - query socket unread bytes
                                        command_t *command = command_front(cmd_buffer);
                                        command->size = sprintf((char*)command->data, "AT+CIPRXGET=4\r\n");
                                        command_push(cmd_buffer);

                                        modem->socket_state = SocketState::query;
                                    }
                                    command_pop(cmd_buffer);
                                    break;
                                }
                                else if(memstr(pending->data, "STATE: TCP CLOSED", pending->size))
                                {
                                    GSM_DEBUG("Server closed TCP socket.\r\n", 27);
                                    modem->socket_state = SocketState::idle;
                                    modem->state = State::ready;
                                    command_clear(cmd_buffer);

                                    modem->tx_buffer.tail = modem->tx_buffer.head;
                                    modem->tx_buffer.count = 0;
                                    break;
                                }
                                break;
                            case SocketState::query:
                                pch = static_cast<uint8_t *>(memstr(pending->data, "+CIPRXGET: 4,", pending->size));
                                if(pch)
                                {
                                    const uint8_t size = pending->size - (pch - pending->data);
                                    if(memstr(pch, "OK\r\n", size))
                                    {
                                        char const *data = static_cast<char *>(memchr(pch, ',', size));

                                        // +CIPRXGET: 4,%d\r\nOK\r\n
                                        // │           │
                                        // │           └ data
                                        // └ pch

                                        const uint16_t count = strtoul(data+1, nullptr, 0);
                                        if(count)
                                        {
                                            // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
                                            command_t *command = command_front(cmd_buffer);
                                            command->size = sprintf((char*)command->data, "AT+CIPRXGET=2,%u\r\n", count);
                                            command_push(cmd_buffer);

                                            #ifdef DEBUG
                                            char buf[100];
                                            int len = sprintf(buf, "Request %d bytes\r\n", count);
                                            GSM_DEBUG(buf, len);
                                            #endif

                                            modem->socket_state = SocketState::receive;
                                        }
                                        else
                                        {
                                            // AT+CSQ - signal quality report
                                            command_t *command = command_front(cmd_buffer);
                                            command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                                            command_push(cmd_buffer);
                                        }
                                        command_pop(cmd_buffer);
                                    }
                                    break;
                                }

                                pch = static_cast<uint8_t *>(memstr(pending->data, "+CSQ:", pending->size));
                                if(pch)
                                {
                                    const uint8_t size = pending->size - (pch - pending->data);
                                    char const *data = static_cast<char *>(memchr(pch, ':', size));

                                    if(memstr(pch, "OK\r\n", size))
                                    {
                                        // +CSQ: %d,%d\r\nOK\r\n
                                        // │   │
                                        // │   └ data
                                        // └ pch

                                        modem->signal = strtoul(data+1, nullptr, 0);
                                        if(!modem->signal || modem->signal == 99)
                                        {
                                            GSM_DEBUG("Modem offline.\r\n", 16);
                                            modem->state = State::offline;
                                            disconnect(ctx);
                                        }
                                        command_pop(cmd_buffer);
                                    }
                                    break;
                                }
                                break;
                            case SocketState::stage:
                                if(memstr(pending->data, "ERROR", pending->size))
                                {
                                    GSM_DEBUG("Staging error\r\n", 15);
                                    command_pop(cmd_buffer);
                                    break;
                                }

                                pch = static_cast<uint8_t *>(memstr(pending->data, "AT+CIPSEND=", pending->size));
                                if(pch)
                                {
                                    const uint8_t size = pending->size - (pch - pending->data);
                                    if(memchr(pch, '>', size))
                                    {
                                        char const *data = static_cast<char *>(memchr(pch, '=', size));

                                        // +CIPSEND=%d\r\n>
                                        // │       │
                                        // │       └ data
                                        // └ pch

                                        uint16_t count = strtoul(data+1, nullptr, 0);
                                        uint8_t send_buffer[BUFFER_SIZE];

                                        for(uint16_t i=0; i < count; i++)
                                        {
                                            const uint16_t tail = (modem->tx_buffer.tail + i) % BUFFER_SIZE;
                                            send_buffer[i] = modem->tx_buffer.data[tail];
                                        }

                                        #ifdef DEBUG
                                        char buf[100];
                                        int len = sprintf(buf, "Transmit %d bytes\r\n", count);
                                        GSM_DEBUG(buf, len);
                                        #endif

                                        ctx->write(send_buffer, count);
                                        cmd_buffer->pending->size = 0;
                                        modem->socket_state = SocketState::transmit;
                                    }
                                    break;
                                }
                                break;
                            case SocketState::transmit:
                                pch = static_cast<uint8_t *>(memstr(pending->data, "DATA ACCEPT:", pending->size));
                                if(pch)
                                {
                                    const uint8_t size = pending->size - (pch - pending->data);
                                    if(memchr(pch, '\n', size))
                                    {
                                        char const *data = static_cast<char *>(memchr(pch, ':', size));

                                        // +DATA ACCEPT:%d\r\n
                                        // │           │
                                        // │           └ data
                                        // └ pch

                                        uint16_t count = strtoul(data+1, nullptr, 0);
                                        modem->tx_buffer.tail = (modem->tx_buffer.tail + count) % BUFFER_SIZE;
                                        modem->tx_buffer.count -= count;

                                        #ifdef DEBUG
                                        char buf[100];
                                        int len = sprintf(buf, "Accept %d bytes\r\n", count);
                                        GSM_DEBUG(buf, len);
                                        #endif

                                        command_pop(cmd_buffer);
                                    }
                                }
                                break;
                            case SocketState::receive:
                                pch = static_cast<uint8_t *>(memstr(pending->data, "+CIPRXGET: 2,", pending->size));
                                if(pch)
                                {
                                    const uint8_t size = pending->size - (pch - pending->data);
                                    if(memstr(pch, "OK\r\n", size))
                                    {
                                        char const *data_count = static_cast<char *>(memchr(pch, ',', size));
                                        uint8_t const *data = static_cast<uint8_t *>(memchr(pch, '\n', size));

                                        // +CIPRXGET: 2,%d,%d,%s\r\n%s\r\nOK\r\n
                                        // │           │           │
                                        // │           │           └ data
                                        // │           └ data_count
                                        // └ pch

                                        data_buffer_t *rx_buffer = &modem->rx_buffer;
                                        const uint16_t count = strtoul(data_count+1, nullptr, 0);

                                        if((size - (data - pch)) >= count)
                                        {
                                            for(uint16_t i=1; i <= count; ++i)
                                            {
                                                rx_buffer->data[rx_buffer->head] = *(data+i);
                                                if(++rx_buffer->head >= BUFFER_SIZE)
                                                    rx_buffer->head = 0U;
                                            }

                                            rx_buffer->count += count;
                                            if(rx_buffer->count >= BUFFER_SIZE)
                                                rx_buffer->count = BUFFER_SIZE-1;

                                            #ifdef DEBUG
                                            char buf[100];
                                            int buflen = sprintf(buf, "Receive %d bytes\r\n", count);
                                            GSM_DEBUG(buf, buflen);
                                            #endif

                                            command_pop(cmd_buffer);
                                        }
                                    }
                                }
                                break;
                        }
                        break;
                }
            }
            else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
            {
                switch(modem->state)
                {
                    case State::authenticating:
                        GSM_DEBUG("Authentication timeout.\r\n", 25);
                        command_clear(cmd_buffer);
                        modem->state = State::online;
                        break;
                    case State::handshaking:
                        GSM_DEBUG("Handshaking timeout.\r\n", 22);
                        command_clear(cmd_buffer);
                        modem->state = State::ready;
                        break;
                    default:
                        GSM_DEBUG("Command timeout\r\n", 17);
                        command_pop(cmd_buffer);
                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        break;
                }
            }
        }
        else if(cmd_buffer->count == 0)
        {
            command_t *command = nullptr;

            switch(modem->state)
            {
                case State::none:
                    // ATI - device identification
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "ATI\r\n");
                    command->timeout_ms = 1000; // 1 seconds
                    command_push(cmd_buffer);
                    break;
                case State::init:
                case State::locked:
                    // AT+CPIN? - unlock status
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CPIN?\r\n");
                    command->timeout_ms = 5000; // 5 seconds
                    command_push(cmd_buffer);
                    break;
                case State::offline:
                    // AT+CREG - network registration status
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CREG?\r\n");
                    command_push(cmd_buffer);
                    break;
                case State::online:
                    // AT+CSQ - signal quality report
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    command_push(cmd_buffer);
                    break;
                case State::authenticating:
                    GSM_DEBUG("Authentication error.\r\n", 23);
                    modem->state = State::online;
                    break;
                case State::ready:
                    // AT+CGATT? - state of GPRS attachment
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CGATT?\r\n");
                    command->timeout_ms = 75000; // 75 seconds
                    command_push(cmd_buffer);
                    break;
                case State::handshaking:
                    GSM_DEBUG("Handshaking error.\r\n", 20);
                    modem->state = State::ready;
                    break;
                case State::open:
                    // AT+CIPSTATUS - TCP connection status
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPSTATUS\r\n");
                    command_push(cmd_buffer);
                    modem->socket_state = SocketState::idle;
                    break;
            }
        }
        else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
        {
            clear(ctx);
            modem->update_timer = elapsed_ms + SEND_PERIOD_MS;
            cmd_buffer->pending = &cmd_buffer->pool[cmd_buffer->tail];

            ctx->write(cmd_buffer->pending->data, cmd_buffer->pending->size);
            cmd_buffer->timer = elapsed_ms + cmd_buffer->pending->timeout_ms;
            cmd_buffer->pending->size = 0;
        }
    }

    State status(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->state;
    }

    bool authenticated(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->state == State::ready
            || modem->state == State::handshaking
            || modem->state == State::open;
    }

    bool connected(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return (modem->state == State::open);
    }

    uint8_t signal(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->signal;
    }

    int unlock(context_t *ctx, const char *pin)
    {
        if(ctx == nullptr || pin == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        // AT+CPIN=[pin] - enter pin
        command_t *command = command_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CPIN=\"%s\"\r\n", pin);
        command_push(&modem->cmd_buffer);

        return 0;
    }

    int authenticate(context_t *ctx, const char *apn, const char *user, const char *pwd)
    {
        if(ctx == nullptr || apn == nullptr || user == nullptr || pwd == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_t *command = nullptr;

        switch(modem->state)
        {
            // Invalid state, return error
            case State::none:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::authenticating:
                return -EALREADY;
            case State::handshaking:
            case State::ready:
            case State::open:
                return -EISCONN;

            // Continue
            case State::online:
                break;
        }

        if(modem->cmd_buffer.count + 7 >= POOL_SIZE)
            return -ENOBUFS;

        GSM_DEBUG("Authenticating...\r\n", 19);
        modem->state = State::authenticating;

        // AT+CIPSHUT - deactivate GPRS PDP context
        command = command_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPSHUT\r\n");
        command->timeout_ms = 65000; // 65 seconds
        command_push(&modem->cmd_buffer);

        // AT+CGATT=0 - detach from GPRS service
        command = command_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT=0\r\n");
        command->timeout_ms = 75000; // 75 seconds
        command_push(&modem->cmd_buffer);

        // AT+CGDCONT=1,[type],[apn] - define GPRS PDP context
        // AT+CGACT=1,1 - activate GPRS PDP context
        command = command_front(&modem->cmd_buffer);
        command->size = snprintf(
            (char*)command->data, BUFFER_SIZE,
            "AT+CGDCONT=1,\"IP\",\"%s\";"
            "+CGACT=1,1\r\n",
            apn);
        command->timeout_ms = 150000; // 150 second
        command_push(&modem->cmd_buffer);

        // AT+SAPBR=3,1,[tag],[value] - configure bearer
        // AT+SAPBR=1,1 - open bearer
        command = command_front(&modem->cmd_buffer);
        command->size = sprintf(
            (char*)command->data,
            "AT+SAPBR=3,1,\"Contype\",\"GPRS\";"
            "+SAPBR=3,1,\"APN\",\"%s\";"
            "+SAPBR=3,1,\"USER\",\"%s\";"
            "+SAPBR=3,1,\"PWD\",\"%s\";"
            "+SAPBR=1,1\r\n",
            apn, user, pwd);
        command->timeout_ms = 850000; // 85 seconds
        command_push(&modem->cmd_buffer);

        // AT+CGATT=1 - attach to GPRS service
        // AT+CIPMUX=0 - single IP mode
        // AT+CIPQSEND=1 - quick send mode
        // AT+CIPRXGET=1 - manual data mode
        // AT+CSTT=[apn],[user],[password] - set apn/user/password for GPRS PDP context
        command = command_front(&modem->cmd_buffer);
        command->size = snprintf(
            (char*)command->data, BUFFER_SIZE,
            "AT+CGATT=1;"
            "+CIPMUX=0;"
            "+CIPQSEND=1;"
            "+CIPRXGET=1;"
            "+CSTT=\"%s\",\"%s\",\"%s\"\r\n",
            apn, user, pwd);
        command->timeout_ms = 75000; // 75 seconds
        command_push(&modem->cmd_buffer);

        // AT+CIICR - bring up wireless connection
        command = command_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIICR\r\n");
        command->timeout_ms = 60000; // 60 seconds
        command_push(&modem->cmd_buffer);

        // AT+CIFSR - get local IP address
        // AT+CDNSCFG=[primary],[secondary] - configure DNS
        command = command_front(&modem->cmd_buffer);
        command->size = sprintf(
            (char*)command->data,
            "AT+CIFSR;"
            "+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\";\r\n");
        command->timeout_ms = 10000; // 10 seconds
        command_push(&modem->cmd_buffer);

        return 0;
    }

    int connect(context_t *ctx, const char *host, uint16_t port)
    {
        if(ctx == nullptr || host == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        switch(modem->state)
        {
            // Invalid state, return error
            case State::none:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::online:
            case State::authenticating:
                return -ENOTCONN;
            case State::handshaking:
                return -EALREADY;
            case State::open:
                return -EADDRINUSE;

            // Continue
            case State::ready:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        GSM_DEBUG("Handshaking...\r\n", 16);
        modem->state = State::handshaking;

        // AT+CIPSTART=[type],[ip],[port] - start TCP/UDP connection to 'ip':'port'
        command_t *command = command_front(&modem->cmd_buffer);
        command->size = snprintf(
            (char*)command->data, BUFFER_SIZE,
            "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
            host, port);
        command->timeout_ms = 75000; // 75 seconds
        command_push(&modem->cmd_buffer);

        return 0;
    }

    int disconnect(context_t *ctx)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        switch(modem->state)
        {
            // Invalid state, return error
            case State::none:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
                return -ENETUNREACH;
            case State::online:
            case State::authenticating:
            case State::handshaking:
            case State::ready:
                return -ENOTSOCK;

            // Continue
            case State::open:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        // AT+CIPCLOSE - close TCP/UDP connection
        command_t *command = command_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPCLOSE\r\n");
        command_push(&modem->cmd_buffer);

        modem->tx_buffer.tail = modem->tx_buffer.head;
        modem->tx_buffer.count = 0;
        return 0;
    }

    int clear(context_t *ctx)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        modem->rx_buffer.count = 0;
        modem->rx_buffer.head = 0;
        modem->rx_buffer.tail = 0;
        return 0;
    }

    int flush(context_t *ctx, uint32_t timeout_ms)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        uint32_t timer = ctx->elapsed_ms() + timeout_ms;

        while(modem->tx_buffer.count > 0)
        {
            process(ctx);
            if(timeout_ms && (int32_t)(ctx->elapsed_ms() - timer) > 0)
                return -ETIME;
        }

        return 0;
    }

    int reset(context_t *ctx)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        // Clear command buffer
        modem->cmd_buffer.pending = nullptr;
        modem->cmd_buffer.count = 0;
        modem->cmd_buffer.head = 0;
        modem->cmd_buffer.tail = 0;
        modem->cmd_buffer.timer = 0;

        // Clear data buffers
        modem->tx_buffer.data[0] = '\0';
        modem->tx_buffer.count = 0;
        modem->tx_buffer.head = 0;
        modem->tx_buffer.tail = 0;

        modem->rx_buffer.data[0] = '\0';
        modem->rx_buffer.count = 0;
        modem->rx_buffer.head = 0;
        modem->rx_buffer.tail = 0;

        // Reset state machine
        modem->update_timer = 0;
        modem->state = State::none;
        modem->socket_state = SocketState::idle;
        modem->signal = 99;
        modem->errors = 0;

        // Clear id
        modem->id[0] = '\0';

        return 0;
    }

    uint16_t available(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->rx_buffer.count;
    }

    uint16_t read(context_t *ctx, uint8_t *data,  uint16_t size)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        data_buffer_t *rx_buffer = &modem->rx_buffer;

        uint16_t count = std::min(rx_buffer->count, size);

        for(uint16_t i=0; i < count; i++)
        {
            data[i] = rx_buffer->data[rx_buffer->tail];
            if(++rx_buffer->tail >= BUFFER_SIZE)
                rx_buffer->tail = 0U;
        }
        rx_buffer->count -= count;

        return count;
    }

    uint16_t write(context_t *ctx, const uint8_t *data, uint16_t size)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        data_buffer_t *tx_buffer = &modem->tx_buffer;

        const uint16_t buffer_free = BUFFER_SIZE-tx_buffer->count;
        uint16_t count = std::min(size, buffer_free);

        if(count > 0)
        {
            for(uint16_t i=0; i < count; i++)
            {
                tx_buffer->data[tx_buffer->head] = data[i]; 
                if(++tx_buffer->head >= BUFFER_SIZE)
                    tx_buffer->head = 0U;
            }
            tx_buffer->count += count;
        }

        return count;
    }

    int authenticate_sync(context_t *ctx, const char *apn, const char *user, const char *pwd, uint32_t timeout_ms)
    {
        if(ctx == nullptr || apn == nullptr)
            return -EINVAL;

        uint32_t timer = ctx->elapsed_ms() + timeout_ms;

        while(!authenticated(ctx))
        {
            authenticate(ctx, apn, user, pwd);
            process(ctx);
            if(timeout_ms && (int32_t)(ctx->elapsed_ms() - timer) > 0)
                return -ETIME;
        }

        return 0;
    }

    int connect_sync(context_t *ctx, const char *host, uint16_t port, uint32_t timeout_ms)
    {
        if(ctx == nullptr || host == nullptr)
            return -EINVAL;
        
        if(!authenticated(ctx))
            return -EPERM;

        uint32_t timer = ctx->elapsed_ms() + timeout_ms;

        while(!connected(ctx))
        {
            connect(ctx, host, port);
            process(ctx);
            if(timeout_ms && (int32_t)(ctx->elapsed_ms() - timer) > 0)
                return -ETIME;
        }

        return 0;
    }

    uint16_t read_sync(context_t *ctx, uint8_t *data,  uint16_t size, uint32_t timeout_ms)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        data_buffer_t *rx_buffer = &modem->rx_buffer;

        uint16_t count = std::min(BUFFER_SIZE, size);
        uint32_t timer = ctx->elapsed_ms() + timeout_ms;

        while(rx_buffer->count < count)
        {
            process(ctx);
            if(timeout_ms && (int32_t)(ctx->elapsed_ms() - timer) >= 0)
            {
                count = rx_buffer->count;
                break;
            }
        }

        for(uint16_t i=0; i < count; i++)
        {
            data[i] = rx_buffer->data[rx_buffer->tail];
            if(++rx_buffer->tail >= BUFFER_SIZE)
                rx_buffer->tail = 0U;
        }
        rx_buffer->count -= count;
        return count;
    }

    uint16_t write_sync(context_t *ctx, const uint8_t *data, uint16_t size, uint32_t timeout_ms)
    {
        uint32_t timer = ctx->elapsed_ms() + timeout_ms;
        uint16_t count = write(ctx, data, size);

        while(count < size)
        {
            process(ctx);
            count += write(ctx, data+count, size-count);
            if(timeout_ms && (int32_t)(ctx->elapsed_ms() - timer) >= 0)
                return count;
        }

        return count;
    }
}
