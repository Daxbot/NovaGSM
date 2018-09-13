/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file NovaGSM.cpp
 * @author Wilkins White
 * @copyright 2018 Nova Dynamics LLC
 */

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
 *     online -> authenticating     [ label = "connect()" ];
 *     authenticating -> online     [ label = "error/\ntimeout" ];
 *     authenticating -> connected  [ label = "OK" ];
 *     connected -> none            [ label = "reset()" ];
 *     connected -> offline         [ label = "" ];
 *     connected -> online          [ label = "disconnect()" ];
 *     connected -> handshaking     [ label = "open()" ];
 *     handshaking -> connected     [ label = "error/\ntimeout" ];
 *     handshaking -> idle          [ label = "OK" ];
 *     idle -> none                 [ label = "reset()" ];
 *     idle -> offline              [ label = "" ];
 *     idle -> connected            [ label = "close()" ];
 *     idle -> busy                 [ label = "" ];
 *     busy -> idle;
 * }
 * @enddot
 */

#include <algorithm>
#include <cstdio>
#include <errno.h>

#include "NovaGSM.h"

namespace GSM
{
    namespace // Anonymous
    {
        constexpr uint32_t BAUDRATE = 115200;       /**< Communication baudrate. */
        constexpr uint32_t DEFAULT_TIMEOUT = 1e5;   /**< Default command timeout (microseconds). */
        constexpr uint16_t BUFFER_SIZE = 1460;      /**< Maximum size of a command data buffer. */
        constexpr uint16_t ID_SIZE = 20;            /**< Maximum size of the modem ID string. */
        constexpr uint8_t POOL_SIZE = 10;           /**< Number of pre-allocated command_t structs in the command_buffer_t pool. */
        constexpr uint8_t MAX_ERRORS = 10;          /**< Maximum communication errors before modem is considered MIA. */

        /** Represents an AT command to be sent. */
        typedef struct {
            uint32_t timeout;               /**< How long to wait for a response (microseconds). */
            uint8_t data[BUFFER_SIZE];      /**< Complete message. */
            uint16_t size;                  /**< Size of message. */
            State handler;                  /**< Required state to handle response. */
        } command_t;

        /** Ring buffer holding queued commands. */
        typedef struct {
            command_t *pending;             /**< Most recent command awaiting response. */
            command_t pool[POOL_SIZE];      /**< Pre-allocated command_t pool. */
            uint8_t count;                  /**< Number of commands in the buffer. */
            uint8_t head;                   /**< Index of last pushed command. */
            uint8_t tail;                   /**< Index of last popped command. */
            uint32_t timer;                 /**< Time the pending command will expire (microseconds). */
        } command_buffer_t;

        /** Private data structure stored in context_t. */
        typedef struct {
            command_buffer_t cmd_buffer;    /**< Queued commands to send. */
            uint8_t rx_data[BUFFER_SIZE];   /**< Ring buffer to store data from the web socket. */
            uint16_t rx_count;              /**< Number of bytes in rx_data. */
            uint16_t rx_head;               /**< Index of last byte added to rx_data. */
            uint16_t rx_tail;               /**< Index of last byte read from rx_data. */
            State state;                    /**< State of the modem. */
            char id[ID_SIZE];               /**< Identification string reported by ATI. */
            uint8_t signal;                 /**< Signal rssi value reported by AT+CSQ. */
            uint8_t errors;                 /**< Timeout error counter.  If it exceeds MAX_ERRORS call reset(). */
        } modem_t;

        /** Gets the next free command_t struct from the buffer.
         * 
         * @param [in] buffer context command buffer.
         */
        inline command_t *buffer_front(command_buffer_t *buffer)
        {
            command_t *packet = &buffer->pool[buffer->head++];
            if(buffer->head >= POOL_SIZE)
                buffer->head = 0;

            packet->size = 0;
            packet->timeout = DEFAULT_TIMEOUT;
            packet->handler = State::none;

            return packet;
        }

        /** Indicates that the HEAD command_t struct is ready to send.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void buffer_push(command_buffer_t *buffer)
        {
            buffer->count += 1;
        }

        /** Frees the TAIL command_t struct.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void buffer_pop(command_buffer_t *buffer)
        {
            buffer->pending = nullptr;
            buffer->count -= 1;
            if(++buffer->tail >= POOL_SIZE)
                buffer->tail = 0;
        }

        /** Clears all pending commands.
         * 
         * @param [in] buffer context command buffer.
         */
        inline void buffer_clear(command_buffer_t *buffer)
        {
            while(buffer->count > 0)
                buffer_pop(buffer);
        }
    }

    void init(void *context)
    {
        context_t *ctx = static_cast<context_t*>(context);
        modem_t *modem = new modem_t;

        ctx->uart_begin(BAUDRATE);

        modem->cmd_buffer.pending = nullptr;
        modem->cmd_buffer.count = 0;
        modem->cmd_buffer.head = 0;
        modem->cmd_buffer.tail = 0;
        modem->cmd_buffer.timer = 0;

        modem->rx_count = 0;
        modem->rx_head = 0;
        modem->rx_tail = 0;

        modem->state = State::none;
        modem->id[0] = '\0';
        modem->signal = 99;

        ctx->priv = static_cast<void*>(modem);
    }

    void deinit(void *context)
    {
        context_t *ctx = static_cast<context_t*>(context);
        delete static_cast<modem_t*>(ctx->priv);
    }

    void process(void *context, uint32_t micros)
    {
        context_t *ctx = static_cast<context_t*>(context);
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_buffer_t *buffer = &modem->cmd_buffer;

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
         * returns to State::offline.  The user must call connect() and open() again
         * to re-establish.
         */

        if(!buffer->pending && buffer->count > 0)
        {
            buffer->pending = &buffer->pool[buffer->tail];


            ctx->uart_clear();
            ctx->uart_write(
                buffer->pending->data,
                buffer->pending->size);

            buffer->timer = micros + buffer->pending->timeout;
            if(buffer->pending->handler != State::none)
                modem->state = buffer->pending->handler;
        }

        switch(modem->state)
        {
            case State::none:
                /** State::none - wait for the modem to respond to an ATI query and transition to State::init. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];
                        ctx->uart_read(c, 1);
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        buffer->pending->data[buffer->pending->size] = '\0';

                        if(sscanf((char*)buffer->pending->data, "ATI\r\nATI\r\n%[^\r]", modem->id) > 0)
                        {
                            command_t *command = nullptr;

                            ctx->debug("Modem is: ", 10);
                            ctx->debug(modem->id, strlen(modem->id));
                            ctx->debug("\r\n", 2);

                            // AT&F0 - reset to factory defaults
                            // ATE0 - disable command echo
                            // AT+CLTS=1 - enable local timestamps
                            // AT+CFUN=1,1 - reset phone module
                            command = buffer_front(buffer);
                            command->size = sprintf((char*)command->data, "AT&F0E0;+CLTS=1;+CFUN=1,1\r\n");
                            command->timeout = 10000000; // 10 seconds
                            command->handler = State::init;
                            buffer_push(buffer);

                            modem->state = State::init;
                        }
                        buffer_pop(buffer);
                    }
                }
                else
                {
                    // ATI - device identification
                    command_t *command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "ATI\r\n");
                    command->timeout = 1000000; // 1 seconds
                    buffer_push(buffer);
                }
                break;
            case State::init:
                /** State::init - query the SIM card and transition to State::offline or State::locked. */
            case State::locked:
                /** State::locked - SIM is locked, call unlock() with the password to transition to State::offline. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "+CPIN: SIM PIN")
                            || strstr((char *)buffer->pending->data, "+CPIN: SIM PUK"))
                            {
                                ctx->debug("SIM locked.\r\n", 13);
                                modem->state = State::locked;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "+CPIN: READY"))
                            {
                                ctx->debug("SIM ready.\r\n", 12);
                                modem->state = State::offline;
                                buffer_pop(buffer);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            buffer_pop(buffer);
                    }
                }
                else
                {
                    // AT+CPIN? - unlock status
                    command_t *command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CPIN?\r\n");
                    command->timeout = 5000000; // 5 seconds
                    buffer_push(buffer);
                }
                break;
            case State::offline:
                /** State::offline - wait for a signal and transition to State::online. */
            case State::online:
                /** State::online - registered on network, wait for connect() to transition to State::authenticating. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "+CSQ:") != nullptr)
                            {
                                char *str = strchr((char *)buffer->pending->data, ':');
                                modem->signal = strtoul(str+2, nullptr, 0);
                                if(modem->signal != 99)
                                {
                                    if(modem->state != State::online)
                                    {
                                        ctx->debug("Modem online.\r\n", 15);
                                        modem->state = State::online;
                                    }
                                }
                                else if(modem->state != State::offline)
                                {
                                    ctx->debug("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "SHUT OK"))
                                buffer_pop(buffer);
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            buffer_pop(buffer);
                    }
                }
                else
                {
                    // AT+CSQ - signal quality report
                    command_t *command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(buffer);
                }
                break;
            case State::authenticating:
                /** State::authenticating - handle connect() and transition to State::connected on success. */
                if(buffer->pending && (int32_t)(micros - buffer->timer) < 0)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "ERROR"))
                            {
                                ctx->debug("Authentication failed.\r\n", 24);
                                ctx->debug((char*)buffer->pending->data, buffer->pending->size);

                                modem->state = State::online;
                                buffer_clear(buffer);
                                disconnect(ctx);
                            }
                            else if(strstr((char *)buffer->pending->data, "SHUT OK"))
                            {
                                ctx->debug("Authentication aborted.\r\n", 25);
                                modem->state = State::online;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "+CIFSR") 
                                 && strstr((char *)buffer->pending->data, "OK"))
                            {
                                ctx->debug("Authentication success.\r\n", 24);
                                modem->state = State::connected;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "+CIICR")
                                 && strstr((char *)buffer->pending->data, "OK"))
                                buffer_pop(buffer); // continue
                            else if(strstr((char *)buffer->pending->data, "+CGATT")
                                 && strstr((char *)buffer->pending->data, "OK"))
                                buffer_pop(buffer); // continue
                            else if(strstr((char *)buffer->pending->data, "+SAPBR")
                                 && strstr((char *)buffer->pending->data, "OK"))
                                buffer_pop(buffer); // continue
                            else if(strstr((char *)buffer->pending->data, "+CGDCONT")
                                 && strstr((char *)buffer->pending->data, "OK"))
                                buffer_pop(buffer); // continue
                        }
                    }
                }
                else
                {
                    ctx->debug("Authentication timeout.\r\n", 25);
                    modem->state = State::online;
                    disconnect(ctx);
                }
                break;
            case State::connected:
                /** State::connected - connected to GPRS, wait for open() to transition to State::handshaking. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "+CSQ:"))
                            {
                                char *str = strchr((char *)buffer->pending->data, ':');
                                modem->signal = strtoul(str+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    ctx->debug("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "+CGATT:"))
                            {
                                char *str = strchr((char *)buffer->pending->data, ':');
                                if(*(str+2) == '0')
                                {
                                    ctx->debug("Disconnected from GPRS.\r\n", 25);
                                    modem->state = State::online;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "SHUT OK"))
                            {
                                ctx->debug("Disconnected from GPRS.\r\n", 25);
                                modem->state = State::online;
                                buffer_pop(buffer);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = nullptr;

                    // AT+CSQ - signal quality report
                    command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(buffer);

                    // AT+CGATT? - state of GPRS attachment
                    command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CGATT?\r\n");
                    command->timeout = 75000000; // 75 seconds
                    buffer_push(buffer);
                }
                break;
            case State::handshaking:
                /** State::handshaking - handle open() and transition to State::idle on success. */
                if(buffer->pending && (int32_t)(micros - buffer->timer) < 0)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "CONNECT OK") 
                            || strstr((char *)buffer->pending->data, "ALREADY CONNECT"))
                            {
                                ctx->debug("Handshaking success.\r\n", 22);
                                modem->state = State::idle;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "CONNECT FAIL"))
                            {
                                ctx->debug("Handshaking failed.\r\n", 21);
                                modem->state = State::connected;
                                buffer_pop(buffer);
                                close(ctx);
                            }
                            else if(strstr((char *)buffer->pending->data, "CLOSE OK"))
                            {
                                ctx->debug("Handshaking aborted.\r\n", 22);
                                modem->state = State::connected;
                                buffer_pop(buffer);
                            }
                        }
                    }
                }
                else 
                {
                    ctx->debug("Handshaking timeout.\r\n", 22);
                    modem->state = State::connected;
                    close(ctx);
                }
                break;
            case State::idle:
                /** State::idle - socket is open, handle write() and wait for socket rx data to transition to State::busy */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];

                        ctx->uart_read(c, 1);
                        if(*c == '\n')
                        {
                            buffer->pending->data[buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)buffer->pending->data, "+CIPRXGET: 4"))
                            {
                                char *str = strchr((char *)buffer->pending->data, ',');
                                uint16_t size = strtoul(str+1, nullptr, 0);

                                if(size)
                                {
                                    // Handle immediately:
                                    // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
                                    buffer->pending->size = snprintf((char*)buffer->pending->data, BUFFER_SIZE, "AT+CIPRXGET=2,%u\r\n", size);

                                    ctx->uart_clear();
                                    ctx->uart_write(
                                        buffer->pending->data,
                                        buffer->pending->size);

                                    buffer->timer = micros + DEFAULT_TIMEOUT;
                                    modem->state = State::busy;
                                }
                            }
                            else if(strstr((char *)buffer->pending->data, "+CSQ:"))
                            {
                                char *str = strchr((char *)buffer->pending->data, ':');
                                modem->signal = strtoul(str+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    ctx->debug("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)buffer->pending->data, "CLOSED")
                                 || strstr((char *)buffer->pending->data, "CLOSE OK"))
                            {
                                ctx->debug("TCP socket closed.\r\n", 20);
                                modem->state = State::connected;
                                buffer_pop(buffer);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = nullptr;

                    // AT+CIPSTATUS - TCP connection status
                    command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPSTATUS\r\n");
                    buffer_push(buffer);

                    // AT+CSQ - signal quality report
                    command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(buffer);

                    // AT+CIPRXGET=4 - query socket unread bytes
                    command = buffer_front(buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPRXGET=4\r\n");
                    buffer_push(buffer);
                }
                break;
            case State::busy:
                /** State::busy - read socket data into the rx ring buffer and transition to State::idle. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];
                        ctx->uart_read(c, 1);
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        buffer->pending->data[buffer->pending->size] = '\0';

                        char *str = strchr((char *)buffer->pending->data, ':');
                        if(str != nullptr)
                        {
                            uint16_t size = strtoul(str+4, nullptr, 0);
                            str = strchr(str, '\n');

                            for(uint16_t i=1; i <= size; ++i)
                            {
                                modem->rx_data[modem->rx_head] = str[i];
                                if(++modem->rx_head >= BUFFER_SIZE)
                                    modem->rx_head = 0U;
                            }

                            modem->rx_count += size;
                            if(modem->rx_count >= BUFFER_SIZE)
                                modem->rx_count = BUFFER_SIZE-1;
                        }

                        modem->state = State::idle;
                        buffer_pop(buffer);
                    }
                }
                else
                {
                    ctx->debug("Read timeout.\r\n", 15);
                    modem->state = State::idle;
                }
                break;
        }
    }

    State status(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->state;
    }

    uint8_t signal(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->signal;
    }

    int reset(context_t *ctx)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        // Clear rx buffer
        clear(ctx);

        // Clear command buffer
        buffer_clear(&modem->cmd_buffer);
        
        // Reset state machine
        modem->state = State::none;
        modem->id[0] = '\0';
        modem->signal = 99;
        return 0;
    }

    int unlock(context_t *ctx, const char *pin)
    {
        if(ctx == nullptr || pin == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        buffer_clear(&modem->cmd_buffer);

        // AT+CPIN=[pin] - enter pin
        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CPIN=\"%s\"\r\n", pin);
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int connect(context_t *ctx, const char *apn, const char *user, const char *pwd)
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
            case State::connected:
            case State::handshaking:
            case State::idle:
            case State::busy:
                return -EISCONN;

            // Continue
            case State::online:
                break;
        }

        buffer_clear(&modem->cmd_buffer);

        // AT+CIPSHUT - deactivate GPRS PDP context
        // AT+CGATT - detach from GPRS service
        // AT+SAPBR=3,1,[tag],[value] - configure bearer settings
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPSHUT;+CGATT=0;+SAPBR=3,1,\"Contype\",\"GPRS\";+SAPBR=3,1,\"APN\",\"%s\"\r\n", apn);
        command->timeout = 65000000; // 65 seconds
        buffer_push(&modem->cmd_buffer);

        if(strlen(user) > 0)
        {
            // AT+SAPBR=3,1,[tag],[value] - configure bearer settings
            command = buffer_front(&modem->cmd_buffer);
            command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+SAPBR=3,1,\"USER\",\"%s\";AT+SAPBR=3,1,\"PWD\",\"%s\"\r\n", user, pwd);
            buffer_push(&modem->cmd_buffer);
        }

        // AT+CGDCONT=1,[type],[apn] - define GPRS PDP context
        // AT+CGACT=1,1 - activate GPRS PDP context
        command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CGDCONT=1,\"IP\",\"%s\";+CGACT=1,1\r\n", apn);
        command->handler = State::authenticating;
        command->timeout = 150000000; // 150 second
        buffer_push(&modem->cmd_buffer);

        // AT+SAPBR=1,1 - open bearer
        command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+SAPBR=1,1\r\n");
        command->handler = State::authenticating;
        command->timeout = 850000000; // 85 seconds
        buffer_push(&modem->cmd_buffer);

        // AT+CGATT=1 - attach to GPRS service
        // AT+CIPMUX=0 - single IP mode
        // AT+CIPQSEND=1 - quick send mode
        // AT+CIPRXGET=1 - manual data mode
        // AT+CSTT=[apn],[user],[password] - set apn/user/password for GPRS PDP context
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT=1;+CIPMUX=0;+CIPQSEND=1;+CIPRXGET=1;+CSTT=\"%s\",\"%s\",\"%s\"\r\n", apn, user, pwd);
        command->timeout = 75000000; // 75 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        // AT+CIICR - bring up wireless connection
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIICR\r\n");
        command->timeout = 60000000; // 60 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        // AT+CIFSR - get local IP address
        // AT+CDNSCFG=[primary],[secondary] - configure DNS
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIFSR;+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\r\n");
        command->timeout = 10000000; // 10 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int disconnect(context_t *ctx)
    {
        if(ctx == nullptr)
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
            case State::online:
                return -ENOTCONN;

            // Socket is open, close it first
            case State::handshaking:
            case State::idle:
            case State::busy:
                close(ctx);
                [[fallthrough]];

            // Continue
            case State::authenticating:
            case State::connected:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        // AT+CIPSHUT - deactivate GPRS PDP context
        // AT+CGATT - detach from GPRS service
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPSHUT;+CGATT=0\r\n");
        command->timeout = 65000000; // 65 seconds
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int open(context_t *ctx, const char *host, uint16_t port)
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
            case State::idle:
            case State::busy:
                return -EADDRINUSE;

            // Continue
            case State::connected:
                break;
        }

        buffer_clear(&modem->cmd_buffer);

        // AT+CIPSTART=[type],[ip],[port] - start TCP/UDP connection to 'ip':'port'
        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", host, port);
        command->timeout = 75000000; // 75 seconds
        command->handler = State::handshaking;
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int close(context_t *ctx)
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
            case State::connected:
                return -ENOTSOCK;

            // Continue
            case State::handshaking:
            case State::idle:
            case State::busy:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        // AT+CIPCLOSE - close TCP/UDP connection
        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPCLOSE\r\n");
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int clear(context_t *ctx)
    {
        if(ctx == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        modem->rx_count = 0;
        modem->rx_head = 0;
        modem->rx_tail = 0;
        return 0;
    }

    uint16_t available(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->rx_count;
    }

    uint16_t read(context_t *ctx, uint8_t *data,  uint16_t size)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        uint16_t count = std::min(modem->rx_count, size);

        for(uint16_t i=0; i < count; i++)
        {
            data[i] = modem->rx_data[modem->rx_tail];
            if(++modem->rx_tail >= BUFFER_SIZE)
                modem->rx_tail = 0U;
        }
        modem->rx_count -= count;

        return count;
    }

    int write(context_t *ctx, uint8_t *data, uint16_t size)
    {
        if(ctx == nullptr || data == nullptr)
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
            case State::connected:
            case State::handshaking:
                return -ENOSTR;

            // Continue
            case State::idle:
            case State::busy:
                break;
        }

        if(modem->cmd_buffer.count+2 > POOL_SIZE)
            return -ENOBUFS;

        uint16_t count = std::min(BUFFER_SIZE, size);

        if(count > 0)
        {
            command_t * command = nullptr;

            // AT+CIPSEND=[size] - indicate that data is about to be sent
            command = buffer_front(&modem->cmd_buffer);
            command->size = sprintf((char*)command->data, "AT+CIPSEND=%u\r\n", size);
            buffer_push(&modem->cmd_buffer);

            command = buffer_front(&modem->cmd_buffer);
            memcpy(command->data, data, count);
            command->size = count;
            buffer_push(&modem->cmd_buffer);
        }

        return count;
    }
}
