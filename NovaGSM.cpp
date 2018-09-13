/** Driver for establishing a TCP connection through a standard GSM/GPRS modem using AT commands.
 *
 * @file NovaGSM.cpp
 * @author Wilkins White
 * @copyright 2018 Nova Dynamics LLC
 */

/**
 * @dot
 * digraph process {
 *     none -> init                 [ label = "ATI" ];
 *     init -> none                 [ label = "no response" ];
 *     init -> locked               [ label = "SIM locked" ];
 *     init -> offline              [ label = "SIM ready" ];
 *     locked -> none               [ label = "no response" ];
 *     locked -> offline            [ label = "unlock()" ];
 *     offline -> none              [ label = "no response" ];
 *     offline -> online            [ label = "signal" ];
 *     online -> none               [ label = "no response" ];
 *     online -> offline            [ label = "no signal" ];
 *     online -> authenticating     [ label = "connect()" ];
 *     authenticating -> online     [ label = "error/\ntimeout" ];
 *     authenticating -> connected  [ label = "OK" ];
 *     connected -> none            [ label = "no response" ];
 *     connected -> online          [ label = "disconnect()" ];
 *     connected -> handshaking     [ label = "open()" ];
 *     handshaking -> connected     [ label = "error/\ntimeout" ];
 *     handshaking -> idle          [ label = "OK" ];
 *     idle -> none                 [ label = "no response" ];
 *     idle -> connected            [ label = "close()" ];
 *     idle -> busy                 [ label = "RX data" ];
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
        constexpr uint32_t DEFAULT_TIMEOUT = 1e5;   /**< Default command timeout if not specified. */
        constexpr uint16_t BUFFER_SIZE = 1460;      /**< Maximum size of a command data buffer. */
        constexpr uint16_t ID_SIZE = 20;            /**< Maximum size of the modem ID string. */
        constexpr uint8_t POOL_SIZE = 25;           /**< Number of pre-allocated command_t structs in the command_buffer_t pool. */
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
            uint8_t errors;                 /**< Timeout error counter.  If it exceeds MAX_ERRORS reset to State::none. */
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
            buffer->pending = NULL;
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
        command_t *command = NULL;

        ctx->uart_begin(BAUDRATE);

        modem->cmd_buffer.pending = NULL;
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
            case State::none:   /** State::none - Wait for the modem to respond to an ATI query and transition to State::init. */
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
                            ctx->debug("Modem is: ", 10);
                            ctx->debug(modem->id, strlen(modem->id));
                            ctx->debug("\r\n", 2);

                            modem->state = State::init;
                            reset(ctx);
                        }
                        buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "ATI\r\n");
                    command->timeout = 1000000; // 1 seconds
                    buffer_push(&modem->cmd_buffer);
                }
                break;
            case State::init:   /** State::init -  Query the SIM card and transition to State::offline or State::locked. */
            case State::locked: /** State::locked - SIM is locked, call unlock() with the password to transition to State::offline. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            char *str = reinterpret_cast<char *>(command->data);

                            if(strstr((char *)command->data, "+CPIN: SIM PIN") != NULL
                            || strstr((char *)command->data, "+CPIN: SIM PUK") != NULL)
                            {
                                ctx->debug("SIM locked.\r\n", 13);
                                modem->state = State::locked;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)command->data, "+CPIN: READY") != NULL)
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
                        {
                            buffer_clear(buffer);
                            modem->state = State::none;
                        }
                        else buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CPIN?\r\n");
                    command->timeout = 5000000; // 5 seconds
                    buffer_push(&modem->cmd_buffer);
                }
                break;
            case State::offline:    /** State::offline - Wait for a signal and transition to State::online. */
            case State::online:     /** State::online - Registered on network, wait for connect() to transition to State::authenticating. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)command->data, "+CSQ:") != NULL)
                            {
                                char *str = strchr((char *)command->data, ':');
                                modem->signal = strtoul(str+2, NULL, 0);
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
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                        {
                            buffer_clear(buffer);
                            modem->state = State::none;
                        }
                        else buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = NULL;

                    command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(&modem->cmd_buffer);
                }
                break;
            case State::authenticating: /** State::authenticating - Handle connect() and transition to State::connected on success. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)command->data, "OK") != NULL)
                            {
                                buffer_pop(&modem->cmd_buffer);
                            }
                            else if(strstr((char *)command->data, "ERROR") != NULL)
                            {
                                ctx->debug("Authentication failed.\r\n", 24);
                                modem->state = State::online;
                                buffer_clear(&modem->cmd_buffer);
                                disconnect(ctx);
                            }
                            else if(strstr((char *)command->data, "+CGATT:") != NULL)
                            {
                                char *str = strchr((char *)command->data, ':');
                                if(*(str+2) == '1')
                                {
                                    ctx->debug("GPRS connected.\r\n", 17);
                                    modem->state = State::connected;
                                }
                                buffer_pop(&modem->cmd_buffer);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                        buffer_pop(buffer);
                }
                else
                {
                    // Authentication failed
                    modem->state = State::online;
                    disconnect(ctx);
                }
                break;
            case State::connected:  /** State::connected - Connected to GPRS, wait for open() to transition to State::handshaking. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)command->data, "+CGATT:") != NULL)
                            {
                                char *str = strchr((char *)command->data, ':');
                                if(*(str+2) == '0')
                                {
                                    ctx->debug("Disconnected from GPRS.\r\n", 25);
                                    modem->state = State::online;
                                }
                                buffer_pop(&modem->cmd_buffer);
                            }
                            else if(strstr((char *)command->data, "+CSQ:") != NULL)
                            {
                                char *str = strchr((char *)command->data, ':');
                                modem->signal = strtoul(str+2, NULL, 0);
                                if(modem->signal == 99)
                                {
                                    ctx->debug("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)command->data, "SHUT OK") != NULL)
                            {
                                ctx->debug("GPRS connection closed.\r\n", 25);
                                modem->state = State::online;
                                buffer_pop(buffer);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        if(++modem->errors >= MAX_ERRORS)
                        {
                            buffer_clear(buffer);
                            modem->state = State::none;
                        }
                        else buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = NULL;

                    command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CGATT?\r\n");
                    command->timeout = 75000000; // 75 seconds
                    buffer_push(&modem->cmd_buffer);

                    command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(&modem->cmd_buffer);
                }
                break;
            case State::handshaking:    /** State::handshaking - Handle open() and transition to State::idle on success. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)command->data, "CONNECT OK") != NULL 
                            || strstr((char *)command->data, "ALREADY CONNECT") != NULL)
                            {
                                ctx->debug("TCP socket open.\r\n", 18);
                                modem->state = State::idle;
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)command->data, "CONNECT FAIL") != NULL)
                            {
                                ctx->debug("Handshaking failed.\r\n", 21);
                                modem->state = State::connected;
                                buffer_pop(buffer);
                                close(ctx);
                            }
                        }
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                        buffer_pop(buffer);
                }
                else
                {
                    // Handshaking failed
                    modem->state = State::connected;
                    close(ctx);
                }
                break;
            case State::idle:   /** State::idle - Socket is open, handle write() and wait for socket rx data to transition to State::busy */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        command_t *command = buffer->pending;
                        uint8_t *c = &command->data[command->size++];

                        ctx->uart_read(c, 1);
                        if(command->size > 2 && *c == '\n' && *(c-1) == '\r')
                        {
                            command->data[command->size] = '\0';
                            modem->errors = 0;

                            if(strstr((char *)command->data, "+CIPRXGET: 4") != NULL)
                            {
                                char *str = strchr((char *)command->data, ',');
                                uint16_t size = strtoul(str+1, NULL, 0);

                                if(size)
                                {
                                    // Insert at front of queue
                                    command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CIPRXGET=2,%u\r\n", size);

                                    ctx->uart_clear();
                                    ctx->uart_write(
                                        command->data,
                                        command->size);

                                    buffer->timer = micros + DEFAULT_TIMEOUT;
                                    modem->state = State::busy;
                                }
                            }
                            else if(strstr((char *)command->data, "+CSQ:") != NULL)
                            {
                                char *str = strchr((char *)command->data, ':');
                                modem->signal = strtoul(str+2, NULL, 0);
                                if(modem->signal == 99)
                                {
                                    ctx->debug("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                buffer_pop(buffer);
                            }
                            else if(strstr((char *)command->data, "CLOSE OK") != NULL)
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
                        {
                            buffer_clear(buffer);
                            modem->state = State::none;
                        }
                        else buffer_pop(buffer);
                    }
                }
                else
                {
                    command_t *command = NULL;
                    
                    command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPRXGET=4\r\n");
                    buffer_push(&modem->cmd_buffer);

                    command = buffer_front(&modem->cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    buffer_push(&modem->cmd_buffer);
                }
                break;
            case State::busy:   /** State::busy - Read socket data into the rx ring buffer and transition to State::idle. */
                if(buffer->pending)
                {
                    if(ctx->uart_available())
                    {
                        uint8_t *c = &buffer->pending->data[buffer->pending->size++];
                        ctx->uart_read(c, 1);
                    }
                    else if((int32_t)(micros - buffer->timer) > 0)
                    {
                        command_t *command = buffer->pending;
                        command->data[command->size] = '\0';

                        char *str = strchr((char *)command->data, ':');
                        char data[BUFFER_SIZE] = {0};
                        uint16_t read = 0;
                        uint16_t unread = 0;

                        if(sscanf(str, ": 2,%u,%u\r\n%[^\r]", &read, &unread, data) == 3)
                        {
                            for(uint16_t i=0; i < read; ++i)
                            {
                                modem->rx_data[modem->rx_head] = data[i];
                                if(++modem->rx_head >= BUFFER_SIZE)
                                    modem->rx_head = 0U;
                            }

                            modem->rx_count += read;
                            if(modem->rx_count >= BUFFER_SIZE)
                                modem->rx_count = BUFFER_SIZE-1;

                            if(unread)
                            {
                                // Insert at front of queue
                                command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CIPRXGET=2,%u\r\n", unread);

                                ctx->uart_clear();
                                ctx->uart_write(
                                    command->data,
                                    command->size);

                                buffer->timer = micros + DEFAULT_TIMEOUT;
                            }
                            else
                            {
                                modem->state = State::idle;
                                buffer_pop(&modem->cmd_buffer);
                            }
                        }
                        else 
                        {
                            modem->state = State::idle;
                            buffer_pop(&modem->cmd_buffer);
                        }
                    }
                }
                else modem->state = State::idle;
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
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_t *command = NULL;

        if(modem->state == State::none)
            return -ENODEV;

        if(modem->cmd_buffer.count+5 >= POOL_SIZE)
            return -EAGAIN;

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CLTS=1\r\n");
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT&W\r\n");
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CFUN=1,1\r\n");
        command->timeout = 10000000; // 10 seconds
        command->handler = State::init;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT&FZ\r\n");
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "ATE0\r\n");
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int unlock(context_t *ctx, const char *pin)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        if(modem->state != State::locked)
            return -EPERM;

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -EAGAIN;

        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CPIN=\"%s\"\r\n", pin);
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int connect(context_t *ctx, const char *apn, const char *user, const char *pwd)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_t *command = NULL;

        if(user == NULL || pwd == NULL)
            return -EINVAL;

        switch(modem->state)
        {
            // Invalid state, return error
            case State::none:
                return -ENODEV;
            case State::init:
            case State::locked:
                return -EPERM;
            case State::offline:
                return -EIO;

            // Already connecting, return 0
            case State::authenticating:
                return 0;

            // TCP socket open, close it and continue
            case State::idle:
                close(ctx);
                [[fallthrough]];
            
            // Continue
            case State::connected:
            case State::online:
                break;
        }

        if(modem->cmd_buffer.count+19 >= POOL_SIZE)
            return -EAGAIN;

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPSHUT\r\n");
        command->timeout = 65000000; // 65 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);
        
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT=0\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", apn);
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        if(strlen(user) > 0)
        {
            command = buffer_front(&modem->cmd_buffer);
            command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+SAPBR=3,1,\"USER\",\"%s\"\r\n", user);
            command->handler = State::authenticating;
            buffer_push(&modem->cmd_buffer);

            if(strlen(user) > 0)
            {
                command = buffer_front(&modem->cmd_buffer);
                command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+SAPBR=3,1,\"PWD\",\"%s\"\r\n", pwd);
                command->handler = State::authenticating;
                buffer_push(&modem->cmd_buffer);
            }
        }

        command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", apn);
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGACT=1,1\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+SAPBR=1,1\r\n");
        command->timeout = 85000000; // 85 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+SAPBR=2,1\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT=1\r\n");
        command->timeout = 75000000; // 75 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPMUX=0\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);
        
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPQSEND=1\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);
        
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPRXGET=1\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", apn, user, pwd);
        command->timeout = 60000000;
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIICR\r\n");
        command->timeout = 60000000;
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIFSR;E0\r\n");
        command->timeout = 10000000;
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\r\n");
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT?\r\n");
        command->timeout = 75000000; // 75 seconds
        command->handler = State::authenticating;
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int disconnect(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        command_t *command = NULL;

        if(modem->state == State::none)
            return -ENODEV;

        if(modem->cmd_buffer.count+3 >= POOL_SIZE)
            return -EAGAIN;

        if(modem->state == State::idle)
            close(ctx);

        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPSHUT\r\n");
        command->timeout = 65000000; // 65 seconds
        buffer_push(&modem->cmd_buffer);
        
        command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CGATT=0\r\n");
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int open(context_t *ctx, const char *host, uint16_t port)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        switch(modem->state)
        {
            // Invalid state, return error
            case State::none:
                return -ENODEV;
            case State::init:
            case State::locked:
            case State::offline:
            case State::online:
                return -EPERM;
            case State::authenticating:
                return -EBUSY;

            // Continue
            case State::idle:
            case State::connected:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -EAGAIN;

        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = snprintf((char*)command->data, BUFFER_SIZE, "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", host, port);
        command->timeout = 75000000; // 75 seconds
        command->handler = State::handshaking;
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int close(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);

        if(modem->state == State::none)
            return -ENODEV;

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -EAGAIN;

        command_t *command = buffer_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPCLOSE\r\n");
        buffer_push(&modem->cmd_buffer);

        return 0;
    }

    int write(context_t *ctx, uint8_t *data, uint16_t size)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        if(modem->state != State::idle)
            return -EBADF;

        if(modem->cmd_buffer.count+2 > POOL_SIZE)
            return -EAGAIN;

        uint16_t count = std::min(BUFFER_SIZE, size);

        if(count > 0)
        {
            command_t * command = NULL;

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

    int read(context_t *ctx, uint8_t *data,  uint16_t size)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        if(modem->state != State::idle)
            return -EBADF;
        
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

    uint16_t available(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return modem->rx_count;
    }
}
