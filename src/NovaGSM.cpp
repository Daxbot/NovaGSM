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
 *     ready -> idle                [ label = "connect()" ];
 *     idle -> none                 [ label = "reset()" ];
 *     idle -> offline              [ label = "lost\nsignal" ];
 *     idle -> ready                [ label = "disconnect()" ];
 *     idle -> busy                 [ label = "" ];
 *     busy -> idle;
 * }
 * @enddot
 */

namespace GSM
{
    namespace // Anonymous
    {
        constexpr uint32_t DEFAULT_TIMEOUT_MS = 200;        /**< Default command timeout milliseconds. */
        constexpr uint32_t UPDATE_PERIOD_MS = 50;           /**< Default modem update rate. */
        constexpr uint16_t BUFFER_SIZE = GSM_BUFFER_SIZE;   /**< Size of the pending data ring buffers. */
        constexpr uint16_t COMMAND_SIZE = 200;              /**< Size of a command data buffer. */
        constexpr uint16_t ID_SIZE = 20;                    /**< Size of the modem ID string buffer. */
        constexpr uint16_t ADDR_SIZE = 20;                  /**< Size of the IP address string buffer. */
        constexpr uint16_t CREDENTIAL_SIZE = 50;            /**< Size of the GPRS credential buffers. */
        constexpr uint8_t POOL_SIZE = 10;                   /**< Number of pre-allocated command_t structs in the command_buffer_t pool. */
        constexpr uint8_t MAX_ERRORS = 10;                  /**< Communication errors before modem is considered not connected. */

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
            uint8_t signal;                     /**< Signal rssi value reported by AT+CSQ. */
            uint8_t errors;                     /**< Timeout error counter.  If it exceeds MAX_ERRORS call reset(). */
            char id[ID_SIZE];                   /**< Identification string reported by ATI. */
            char address[ADDR_SIZE];            /**< IP address assigned by GPRS. */
            char apn[CREDENTIAL_SIZE+1];        /**< GPRS access point name. */
            char user[CREDENTIAL_SIZE+1];       /**< GPRS authentication user name. */
            char pwd[CREDENTIAL_SIZE+1];        /**< GPRS authentication password. */
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

        if(!cmd_buffer->pending && cmd_buffer->count > 0)
        {
            cmd_buffer->pending = &cmd_buffer->pool[cmd_buffer->tail];

            ctx->write(cmd_buffer->pending->data, cmd_buffer->pending->size);
            //GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);
            cmd_buffer->timer = elapsed_ms + cmd_buffer->pending->timeout_ms;
            cmd_buffer->pending->size = 0;
        }

        switch(modem->state)
        {
            case State::none:
                /** State::none - wait for the modem to respond to an ATI query and transition to State::init. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size++];
                        ctx->read(pch, 1);
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        cmd_buffer->pending->data[cmd_buffer->pending->size] = '\0';

                        if(sscanf((char*)cmd_buffer->pending->data, "ATI\r\n%[^\r]", modem->id) > 0)
                        {
                            command_t *command = nullptr;
                            modem->state = State::init;

                            GSM_DEBUG("Modem is: ", 10);
                            GSM_DEBUG(modem->id, strlen(modem->id));
                            GSM_DEBUG("\r\n", 2);

                            // AT&F0 - reset to factory defaults
                            // ATE0 - disable command echo
                            // AT+CLTS=1 - enable local timestamps
                            // AT+CFUN=1,1 - reset phone module
                            command = command_front(cmd_buffer);
                            command->size = sprintf(
                                (char*)command->data,
                                "AT&F0E0;"
                                "+CLTS=1;"
                                "+CFUN=1,1\r\n");
                            command->timeout_ms = 10000; // 10 seconds
                            command_push(cmd_buffer);
                        }
                        command_pop(cmd_buffer);
                    }
                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    // ATI - device identification
                    command_t *command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "ATI\r\n");
                    command->timeout_ms = 1000; // 1 seconds
                    command_push(cmd_buffer);

                    modem->update_timer = elapsed_ms + UPDATE_PERIOD_MS;
                }
                break;
            case State::init:
                /** State::init - query the SIM card and transition to State::offline or State::locked. */
            case State::locked:
                /** State::locked - SIM is locked, call unlock() with the password to transition to State::offline. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];

                        ctx->read(pch, 1);
                        if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "+CPIN: SIM PIN")
                            || strstr(data, "+CPIN: SIM PUK"))
                            {
                                GSM_DEBUG("SIM locked.\r\n", 13);
                                modem->state = State::locked;
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "+CPIN: READY"))
                            {
                                GSM_DEBUG("SIM ready.\r\n", 12);
                                modem->state = State::offline;
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "+CFUN"))
                            {
                                // If it ends with 'OK' continue
                                if(data[cmd_buffer->pending->size - 4] == 'O'
                                && data[cmd_buffer->pending->size - 3] == 'K')
                                    command_pop(cmd_buffer);
                            }
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        GSM_DEBUG("Command timeout: ", 17);
                        GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);

                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            command_pop(cmd_buffer);
                    }
                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    // AT+CPIN? - unlock status
                    command_t *command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CPIN?\r\n");
                    command->timeout_ms = 5000; // 5 seconds
                    command_push(cmd_buffer);

                    modem->update_timer = elapsed_ms + UPDATE_PERIOD_MS;
                }
                break;
            case State::offline:
                /** State::offline - wait for a signal and transition to State::online. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];

                        ctx->read(pch, 1);
                        if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "+CSQ:"))
                            {
                                data = strchr(data, ':');
                                modem->signal = strtoul(data+2, nullptr, 0);
                                if(modem->signal != 99)
                                {
                                    GSM_DEBUG("Modem online.\r\n", 15);
                                    modem->state = State::online;
                                }
                                command_pop(cmd_buffer);
                            }
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        GSM_DEBUG("Command timeout: ", 17);
                        GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);

                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            command_pop(cmd_buffer);
                    }
                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    // AT+CSQ - signal quality report
                    command_t *command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    command_push(cmd_buffer);

                    modem->update_timer = elapsed_ms + UPDATE_PERIOD_MS;
                }
                break;
            case State::online:
                /** State::online - registered on network, wait for authenticate() to transition to State::authenticating. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];

                        ctx->read(pch, 1);
                        if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "+CSQ:"))
                            {
                                data = strchr(data, ':');
                                modem->signal = strtoul(data+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                command_pop(cmd_buffer);
                            }
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        GSM_DEBUG("Command timeout: ", 17);
                        GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);

                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            command_pop(cmd_buffer);
                    }
                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    command_t *command = nullptr;

                    // AT+CSQ - signal quality report
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    command_push(cmd_buffer);

                    modem->update_timer = elapsed_ms + UPDATE_PERIOD_MS;

                    if(modem->apn[0] != '\0')
                    {
                        GSM_DEBUG("Authenticating...\r\n", 19);
                        modem->state = State::authenticating;

                        // AT+CIPSHUT - deactivate GPRS PDP context
                        command = command_front(cmd_buffer);
                        command->size = sprintf((char*)command->data, "AT+CIPSHUT\r\n");
                        command->timeout_ms = 65000; // 65 seconds
                        command_push(&modem->cmd_buffer);

                        // AT+CGATT=0 - detach from GPRS service
                        command = command_front(cmd_buffer);
                        command->size = sprintf((char*)command->data, "AT+CGATT=0\r\n");
                        command->timeout_ms = 65000; // 65 seconds
                        command_push(&modem->cmd_buffer);

                        // AT+CGDCONT=1,[type],[apn] - define GPRS PDP context
                        // AT+CGACT=1,1 - activate GPRS PDP context
                        command = command_front(cmd_buffer);
                        command->size = snprintf(
                            (char*)command->data, BUFFER_SIZE,
                            "AT+CGDCONT=1,\"IP\",\"%s\";"
                            "+CGACT=1,1\r\n",
                            modem->apn);
                        command->timeout_ms = 150000; // 150 second
                        command_push(&modem->cmd_buffer);

                        // AT+SAPBR=3,1,[tag],[value] - configure bearer
                        // AT+SAPBR=1,1 - open bearer
                        command = command_front(cmd_buffer);
                        command->size = sprintf(
                            (char*)command->data,
                            "AT+SAPBR=3,1,\"Contype\",\"GPRS\";"
                            "+SAPBR=3,1,\"APN\",\"%s\";"
                            "+SAPBR=3,1,\"USER\",\"%s\";"
                            "+SAPBR=3,1,\"PWD\",\"%s\";"
                            "+SAPBR=1,1\r\n",
                            modem->apn, modem->user, modem->pwd);
                        command->timeout_ms = 850000; // 85 seconds
                        command_push(cmd_buffer);

                        // AT+CGATT=1 - attach to GPRS service
                        // AT+CIPMUX=0 - single IP mode
                        // AT+CIPQSEND=1 - quick send mode
                        // AT+CIPRXGET=1 - manual data mode
                        // AT+CSTT=[apn],[user],[password] - set apn/user/password for GPRS PDP context
                        command = command_front(cmd_buffer);
                        command->size = snprintf(
                            (char*)command->data, BUFFER_SIZE,
                            "AT+CGATT=1;"
                            "+CIPMUX=0;"
                            "+CIPQSEND=1;"
                            "+CIPRXGET=1;"
                            "+CSTT=\"%s\",\"%s\",\"%s\"\r\n",
                            modem->apn, modem->user, modem->pwd);
                        command->timeout_ms = 75000; // 75 seconds
                        command_push(cmd_buffer);

                        // AT+CIICR - bring up wireless connection
                        command = command_front(cmd_buffer);
                        command->size = sprintf((char*)command->data, "AT+CIICR\r\n");
                        command->timeout_ms = 60000; // 60 seconds
                        command_push(cmd_buffer);

                        // AT+CIFSR - get local IP address
                        // AT+CDNSCFG=[primary],[secondary] - configure DNS
                        command = command_front(cmd_buffer);
                        command->size = sprintf(
                            (char*)command->data,
                            "AT+CIFSR;"
                            "+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\";\r\n");
                        command->timeout_ms = 10000; // 10 seconds
                        command_push(cmd_buffer);
                    }
                }
                break;
            case State::authenticating:
                /** State::authenticating - handle authenticate() and transition to State::ready on success. */
                if(cmd_buffer->pending && (int32_t)(elapsed_ms - cmd_buffer->timer) < 0)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];
                        ctx->read(pch, 1);

                        if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "ERROR"))
                            {
                                GSM_DEBUG("Authentication failed.\r\n", 24);
                                modem->state = State::online;
                                command_clear(cmd_buffer);
                            }
                            else if(strstr(data, "+CSQ:"))
                            {
                                data = strchr(data, ':');
                                modem->signal = strtoul(data+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                    command_clear(cmd_buffer);
                                }
                                else command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "CIFSR"))
                            {
                                // If it ends with 'OK' extract the IP address
                                if(data[cmd_buffer->pending->size - 4] == 'O'
                                && data[cmd_buffer->pending->size - 3] == 'K')
                                {
                                    data = strchr(data, '\n');
                                    sscanf(data+1, "%[^\r]", modem->address);
                                    GSM_DEBUG("Authentication success.\r\n", 25);

                                    GSM_DEBUG("Address is: ", 12);
                                    GSM_DEBUG(modem->address, strlen(modem->address));
                                    GSM_DEBUG("\r\n", 2);

                                    modem->state = State::ready;
                                    command_pop(cmd_buffer);
                                }
                            }
                            else if(strstr(data, "CIPSHUT")
                                 || strstr(data, "CGATT")
                                 || strstr(data, "CGDCONT")
                                 || strstr(data, "CGACT")
                                 || strstr(data, "SAPBR")
                                 || strstr(data, "CIICR"))
                            {
                                // If it ends with 'OK' continue
                                if(data[cmd_buffer->pending->size - 4] == 'O'
                                && data[cmd_buffer->pending->size - 3] == 'K')
                                    command_pop(cmd_buffer);
                            }
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                }
                else
                {
                    GSM_DEBUG("Authentication timeout.\r\n", 25);
                    modem->state = State::online;
                    command_clear(cmd_buffer);
                }
                break;
            case State::ready:
                /** State::ready - ready to GPRS, wait for connect() to transition to State::idle. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];

                        ctx->read(pch, 1);
                        if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "+CSQ:"))
                            {
                                data = strchr(data, ':');
                                modem->signal = strtoul(data+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "+CGATT:"))
                            {
                                data = strchr(data, ':');
                                if(*(data+2) == '0')
                                {
                                    GSM_DEBUG("Disconnected from GPRS.\r\n", 25);
                                    modem->state = State::online;
                                }
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "SHUT OK"))
                            {
                                GSM_DEBUG("Disconnected from GPRS.\r\n", 25);
                                modem->state = State::online;
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "CONNECT OK") 
                                 || strstr(data, "ALREADY CONNECT"))
                            {
                                GSM_DEBUG("TCP connection established.\r\n", 29);
                                modem->state = State::idle;
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "CONNECT FAIL"))
                            {
                                GSM_DEBUG("TCP connection failed.\r\n", 24);
                                command_pop(cmd_buffer);
                            }
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        GSM_DEBUG("Command timeout: ", 17);
                        GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);

                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            command_pop(cmd_buffer);
                    }
                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    command_t *command = nullptr;

                    // AT+CSQ - signal quality report
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    command_push(cmd_buffer);

                    // AT+CGATT? - state of GPRS attachment
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CGATT?\r\n");
                    command->timeout_ms = 75000; // 75 seconds
                    command_push(cmd_buffer);

                    modem->update_timer = elapsed_ms + UPDATE_PERIOD_MS;
                }
                break;
            case State::idle:
                /** State::idle - socket is established, handle write() and wait for socket rx data to transition to State::busy */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size];

                        ctx->read(pch, 1);
                        if(*pch == '>')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            data = strstr(data, "+CIPSEND=");
                            if(data)
                            {
                                uint16_t count = strtoul(data+9, nullptr, 0);
                                if(count)
                                {
                                    data_buffer_t *tx_buffer = &modem->tx_buffer;
                                    uint8_t send_buffer[BUFFER_SIZE];

                                    for(uint16_t i=0; i < count; i++)
                                    {
                                        send_buffer[i] = tx_buffer->data[tx_buffer->tail];
                                        if(++tx_buffer->tail >= BUFFER_SIZE)
                                            tx_buffer->tail = 0U;
                                    }
                                    tx_buffer->count -= count;

                                    ctx->write(send_buffer, count);
                                    cmd_buffer->pending->size = 0;
                                    cmd_buffer->timer = elapsed_ms + DEFAULT_TIMEOUT_MS;
                                }
                            }
                        }
                        else if(*pch == '\n')
                        {
                            char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                            data[++cmd_buffer->pending->size] = '\0';
                            modem->errors = 0;

                            if(strstr(data, "+CIPRXGET: 4"))
                            {
                                data = strchr(data, ',');
                                uint16_t count = strtoul(data+1, nullptr, 0);

                                if(count)
                                {
                                    // Handle immediately:
                                    // AT+CIPRXGET=2,[size] - read 'size' bytes from the socket
                                    cmd_buffer->pending->size = snprintf(
                                        (char *)cmd_buffer->pending->data, BUFFER_SIZE,
                                        "AT+CIPRXGET=2,%u\r\n",
                                        count);

                                    ctx->write(
                                        cmd_buffer->pending->data,
                                        cmd_buffer->pending->size);

                                    cmd_buffer->pending->size = 0;
                                    cmd_buffer->timer = elapsed_ms + DEFAULT_TIMEOUT_MS;
                                    modem->state = State::busy;
                                }
                                else command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "+CIPSEND:"))
                            {
                                data = strchr(data, ':');
                                uint16_t count = strtoul(data+2, nullptr, 0);
                                if(count > modem->tx_buffer.count)
                                    count = modem->tx_buffer.count;

                                if(count)
                                {
                                    // Handle immediately:
                                    // AT+CIPSEND=[size] - indicate that data is about to be sent
                                    cmd_buffer->pending->size = sprintf(
                                        (char*)cmd_buffer->pending->data,
                                        "AT+CIPSEND=%u\r\n",
                                        count);

                                    ctx->write(
                                        cmd_buffer->pending->data,
                                        cmd_buffer->pending->size);
                                }
                                else command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "+CSQ:"))
                            {
                                data = strchr(data, ':');
                                modem->signal = strtoul(data+2, nullptr, 0);
                                if(modem->signal == 99)
                                {
                                    GSM_DEBUG("Modem offline.\r\n", 16);
                                    modem->state = State::offline;
                                }
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "STATE:"))
                            {
                                if(strstr(data, "CONNECT OK") == nullptr)
                                {
                                    GSM_DEBUG("TCP socket disconnected.\r\n", 26);
                                    modem->state = State::ready;
                                }
                                command_pop(cmd_buffer);
                            }
                            else if(strstr(data, "ALREADY CONNECT"))
                                command_pop(cmd_buffer);
                            else if(strstr(data, "DATA ACCEPT"))
                                command_pop(cmd_buffer);
                        }
                        else if(*pch != '\0')
                            cmd_buffer->pending->size += 1;
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        GSM_DEBUG("Command timeout: ", 17);
                        GSM_DEBUG((char*)cmd_buffer->pending->data, cmd_buffer->pending->size);

                        if(++modem->errors >= MAX_ERRORS)
                            reset(ctx);
                        else
                            command_pop(cmd_buffer);
                    }
                }
                else if(modem->tx_buffer.count > 0)
                {
                    // AT+CIPSEND? - query the modem's buffer size 
                    command_t *command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPSEND?\r\n");
                    command_push(cmd_buffer);

                }
                else if((int32_t)(elapsed_ms - modem->update_timer) > 0)
                {
                    command_t *command = nullptr;

                    // AT+CIPSTATUS - TCP connection status
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPSTATUS\r\n");
                    command_push(cmd_buffer);

                    // AT+CSQ - signal quality report
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CSQ\r\n");
                    command_push(cmd_buffer);

                    // AT+CIPRXGET=4 - query socket unread bytes
                    command = command_front(cmd_buffer);
                    command->size = sprintf((char*)command->data, "AT+CIPRXGET=4\r\n");
                    command_push(cmd_buffer);
                }
                
                break;
            case State::busy:
                /** State::busy - read socket data into the rx ring buffer and transition to State::idle. */
                if(cmd_buffer->pending)
                {
                    if(ctx->available())
                    {
                        uint8_t *pch = &cmd_buffer->pending->data[cmd_buffer->pending->size++];
                        ctx->read(pch, 1);
                    }
                    else if((int32_t)(elapsed_ms - cmd_buffer->timer) > 0)
                    {
                        char *data = reinterpret_cast<char *>(cmd_buffer->pending->data);
                        data[cmd_buffer->pending->size] = '\0';

                        data = strchr(data, ':');
                        if(data != nullptr)
                        {
                            data_buffer_t *rx_buffer = &modem->rx_buffer;
                            uint16_t size = strtoul(data+4, nullptr, 0);
                            data = strchr(data, '\n');

                            for(uint16_t i=1; i <= size; ++i)
                            {
                                rx_buffer->data[rx_buffer->head] = data[i];
                                if(++rx_buffer->head >= BUFFER_SIZE)
                                    rx_buffer->head = 0U;
                            }

                            rx_buffer->count += size;
                            if(rx_buffer->count >= BUFFER_SIZE)
                                rx_buffer->count = BUFFER_SIZE-1;
                        }

                        modem->state = State::idle;
                        command_pop(cmd_buffer);
                    }
                }
                else
                {
                    GSM_DEBUG("Read timeout.\r\n", 15);
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

    bool connected(context_t *ctx)
    {
        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        return (modem->state == State::idle || modem->state == State::busy);
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
        snprintf(modem->apn, CREDENTIAL_SIZE, "%s", apn);
        snprintf(modem->user, CREDENTIAL_SIZE, "%s", user);
        snprintf(modem->pwd, CREDENTIAL_SIZE, "%s", pwd);

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
            case State::idle:
            case State::busy:
                return -EADDRINUSE;

            // Continue
            case State::ready:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

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
            case State::ready:
                return -ENOTSOCK;

            // Continue
            case State::idle:
            case State::busy:
                break;
        }

        if(modem->cmd_buffer.count >= POOL_SIZE)
            return -ENOBUFS;

        // AT+CIPCLOSE - close TCP/UDP connection
        command_t *command = command_front(&modem->cmd_buffer);
        command->size = sprintf((char*)command->data, "AT+CIPCLOSE\r\n");
        command_push(&modem->cmd_buffer);

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

        // Clear pending bytes
        while(ctx->available())
            ctx->read(modem->rx_buffer.data, 1);

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
        modem->signal = 99;
        modem->errors = 0;

        // Clear credentials
        modem->id[0] = '\0';
        modem->address[0] = '\0';
        modem->apn[0] = '\0';
        modem->user[0] = '\0';
        modem->pwd[0] = '\0';

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

    int connect_sync(context_t *ctx, const char *host, uint16_t port, uint32_t timeout_ms)
    {
        if(ctx == nullptr || host == nullptr)
            return -EINVAL;

        modem_t *modem = static_cast<modem_t*>(ctx->priv);
        uint32_t timer = ctx->elapsed_ms() + timeout_ms;

        if(modem->apn[0] == '\0')
            return -EPERM;

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
