#include <sys/cdefs.h>
/*-
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2018, Jan Breuer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi_server.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  TCP/IP SCPI Server
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "assert.h"

#include "scpi/scpi.h"
#include "scpi-def.h"
#include "scpi_server.h"

#include "lwip/tcpip.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "lwip/tcp.h"
#include "lwip/inet.h"
#include "lwip/api.h"

#define SCPI_THREAD_PRIO (tskIDLE_PRIORITY + 0) // osPriorityNormal = 0

#define SCPI_MSG_TIMEOUT                0
#define SCPI_MSG_TEST                   1
#define SCPI_MSG_IO_LISTEN              2
#define SCPI_MSG_CONTROL_IO_LISTEN      3
#define SCPI_MSG_IO                     4
#define SCPI_MSG_CONTROL_IO             5
#define SCPI_MSG_SET_ESE_REQ            6
#define SCPI_MSG_SET_ERROR              7

typedef struct {
    struct netconn *io_listen;
    struct netconn *control_io_listen;
    struct netconn *io;
    struct netconn *control_io;
    xQueueHandle evtQueue;
    /* FILE * fio; */
    /* fd_set fds; */
} user_data_t;

struct _queue_event_t {
    uint8_t cmd;
    uint8_t param1;
    int16_t param2;
} __attribute__((__packed__));
typedef struct _queue_event_t queue_event_t;


user_data_t user_data = {
    .io_listen = NULL,
    .io = NULL,
    .control_io_listen = NULL,
    .control_io = NULL,
    .evtQueue = 0,
};

/* SCPI_Write
 * calls netconn_write_partly(), its result is of type err_t. See api_lib.c err_enum_t for possible values.
 * ERR_OK : success
 * ERR_MEM : out of memory
 * ERR_BUF : buffer error
 * ERR_TIMEOUT : timeout [...] */
size_t SCPI_Write(scpi_t * context, const char * data, size_t len) {
    if ((context->user_context != NULL) && (data != NULL)) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->io) {
            // printf("%.*s\r\n", (int)(len), data); // un-comment for debug output
            err_t result = netconn_write(u->io, data, len, NETCONN_COPY);
            if(result != ERR_OK){
                printf("\r\nSCPI_Write ERROR %d\r\n", result);
                return 0;
            }
            return len;
        }
    }
    return 0;
}

scpi_result_t SCPI_Flush(scpi_t * context) {
    if (context->user_context != NULL) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->io) {
            /* flush not implemented */
            return SCPI_RES_OK;
        }
    }
    return SCPI_RES_OK;
}

int SCPI_Error(scpi_t * context, int_fast16_t err) {
    (void) context;
    /* BEEP */
    iprintf("**ERROR: %ld, \"%s\"\r\n", (int32_t) err, SCPI_ErrorTranslate(err));
    if (err != 0) {
        /* New error */
        SCPI_Event_ErrorIndicatorOn(context, err);
    } else {
        /* No more errors in the queue */
        SCPI_Event_ErrorIndicatorOff(context, err);
    }
    return 0;
}

scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    char b[16];

    if (SCPI_CTRL_SRQ == ctrl) {
        iprintf("**SRQ: 0x%X (%d)\r\n", val, val);
    } else {
        iprintf("**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
    }

    if (context->user_context != NULL) {
        user_data_t * u = (user_data_t *) (context->user_context);
        if (u->control_io) {
            snprintf(b, sizeof (b), "SRQ%d\r\n", val);
            return netconn_write(u->control_io, b, strlen(b), NETCONN_COPY) == ERR_OK ? SCPI_RES_OK : SCPI_RES_ERR;
        }
    }
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t * context) {
    (void) context;
    iprintf("**Reset\r\n");
    return SCPI_RES_OK;
}

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context) {
    SCPI_ResultInt(context, SCPI_CONTROL_PORT);
    return SCPI_RES_OK;
}

static void setEseReq(void) {
    SCPI_RegSetBits(&scpi_context, SCPI_REG_ESR, ESR_REQ);
}

static void setError(int16_t err) {
    SCPI_ErrorPush(&scpi_context, err);
}

void SCPI_RequestControl(void) {
    queue_event_t msg;
    msg.cmd = SCPI_MSG_SET_ESE_REQ;

    /* Avoid sending evtQueue message if ESR_REQ is already set
    if((SCPI_RegGet(&scpi_context, SCPI_REG_ESR) & ESR_REQ) == 0) {
        xQueueSend(user_data.evtQueue, &msg, 1000);
    }
     */

    xQueueSend(user_data.evtQueue, &msg, 1000);
}

void SCPI_AddError(int16_t err) {
    queue_event_t msg;
    msg.cmd = SCPI_MSG_SET_ERROR;
    msg.param2 = err;

    xQueueSend(user_data.evtQueue, &msg, 1000);
}

void scpi_netconn_callback(struct netconn * conn, enum netconn_evt evt, u16_t len) {
    queue_event_t msg;
    (void) len;


    if (evt == NETCONN_EVT_RCVPLUS) {
        msg.cmd = SCPI_MSG_TEST;
        if (conn == user_data.io) {
            msg.cmd = SCPI_MSG_IO;
        } else if (conn == user_data.io_listen) {
            msg.cmd = SCPI_MSG_IO_LISTEN;
        } else if (conn == user_data.control_io) {
            msg.cmd = SCPI_MSG_CONTROL_IO;
        } else if (conn == user_data.control_io_listen) {
            msg.cmd = SCPI_MSG_CONTROL_IO_LISTEN;
        }
                xQueueSend(user_data.evtQueue, &msg, 1000);
    }
}

static struct netconn * createServer(int port) {
    struct netconn * conn;
    err_t err;

    conn = netconn_new_with_callback(NETCONN_TCP, scpi_netconn_callback);
    if (conn == NULL) {
        return NULL;
    }

    err = netconn_bind(conn, NULL, port);
    if (err != ERR_OK) {
        netconn_delete(conn);
        return NULL;
    }
    // keep_alive, keep_intvl and keep_cnt re being overwritten by netconn_listen (Why?).
    // Specify TCP_KEEPIDLE_DEFAULT, TCP_KEEPINTVL_DEFAULT and TCP_KEEPCNT_DEFAULT in lwipopts.h instead, as
    // Defaults are also being overwritten when tcp_priv.h is regenerated.
    netconn_listen(conn);

    return conn;
}

static void waitServer(user_data_t * user_data, queue_event_t * evt) {
    /* 5s timeout */
    if (xQueueReceive(user_data->evtQueue, evt, 5000 * portTICK_RATE_MS) != pdPASS) {
        evt->cmd = SCPI_MSG_TIMEOUT;
    }
}

static int processIoListen(user_data_t * user_data) {
    struct netconn *newconn;

    if (netconn_accept(user_data->io_listen, &newconn) == ERR_OK) {
        if (user_data->io) {
            /* Close unwanted connection */
            netconn_close(newconn);
            netconn_delete(newconn);
        } else {
            /* connection established */
            SCPI_Event_DeviceConnected(newconn);
            iprintf("***Connection established %s\r\n", inet_ntoa(newconn->pcb.ip->remote_ip));

            ip_set_option(newconn->pcb.tcp, SOF_KEEPALIVE);
            newconn->pcb.tcp->keep_idle   = SCPI_KEEP_IDLE;  // Override TCP_KEEPIDLE_DEFAULT  for this connection.
            newconn->pcb.tcp->keep_intvl  = SCPI_KEEP_INTVL; // Override TCP_KEEPINTVL_DEFAULT for this connection.
            newconn->pcb.tcp->keep_cnt    = SCPI_KEEP_CNT;   // Override TCP_KEEPCNT_DEFAULT   for this connection.

            user_data->io = newconn;
        }
    }

    return 0;
}

static int processSrqIoListen(user_data_t * user_data) {
    struct netconn *newconn;

    if (netconn_accept(user_data->control_io_listen, &newconn) == ERR_OK) {
        if (user_data->control_io) {
            netconn_close(newconn);
            netconn_delete(newconn);
        } else {
            /* control connection established */
            iprintf("***Control Connection established %s\r\n", inet_ntoa(newconn->pcb.ip->remote_ip));

            ip_set_option(newconn->pcb.tcp, SOF_KEEPALIVE);
            newconn->pcb.tcp->keep_idle   = SCPI_KEEP_IDLE;  // Override TCP_KEEPIDLE_DEFAULT  for this connection.
            newconn->pcb.tcp->keep_intvl  = SCPI_KEEP_INTVL; // Override TCP_KEEPINTVL_DEFAULT for this connection.
            newconn->pcb.tcp->keep_cnt    = SCPI_KEEP_CNT;   // Override TCP_KEEPCNT_DEFAULT   for this connection.

            user_data->control_io = newconn;
        }
    }

    return 0;
}

static void closeIo(user_data_t * user_data) {
    /* connection closed */
    SCPI_Event_DeviceDisconnected(user_data->io);
    netconn_close(user_data->io);
    netconn_delete(user_data->io);
    user_data->io = NULL;
    iprintf("***Connection closed\r\n");
}

static void closeSrqIo(user_data_t * user_data) {
    /* control connection closed */
    netconn_close(user_data->control_io);
    netconn_delete(user_data->control_io);
    user_data->control_io = NULL;
    iprintf("***Control Connection closed\r\n");
}

static int processIo(user_data_t * user_data) {
    struct netbuf *inbuf;
    char* buf;
    u16_t buflen;

    if (netconn_recv(user_data->io, &inbuf) != ERR_OK) {
        goto fail1;
    }
    if (netconn_err(user_data->io) != ERR_OK) {
        goto fail2;
    }

    netbuf_data(inbuf, (void**) &buf, &buflen);

    if (buflen > 0) {
        SCPI_Input(&scpi_context, buf, buflen);
    } else {
        /* goto fail2; */
    }

    netbuf_delete(inbuf);

    return 0;

fail2:
    netbuf_delete(inbuf);
fail1:
    closeIo(user_data);

    return 0;
}

static int processSrqIo(user_data_t * user_data) {
    struct netbuf *inbuf;
    char* buf;
    u16_t buflen;

    if (netconn_recv(user_data->control_io, &inbuf) != ERR_OK) {
        goto fail1;
    }
    if (netconn_err(user_data->control_io) != ERR_OK) {
        goto fail2;
    }

    netbuf_data(inbuf, (void**) &buf, &buflen);

    if (buflen > 0) {
        /* TODO process control */
    } else {
        /* goto fail2; */
    }

    netbuf_delete(inbuf);

    return 0;

fail2:
    netbuf_delete(inbuf);
fail1:
    closeSrqIo(user_data);

    return 0;
}

/*
 *
 */
_Noreturn static void scpi_server_thread(void *arg) {
    queue_event_t evt;

    (void) arg;

    user_data.evtQueue = xQueueCreate(10, sizeof (queue_event_t));
    LWIP_ASSERT("user_data.evtQueue != NULL", user_data.evtQueue != NULL);

    /* user_context will be pointer to socket */
    SCPI_Init(&scpi_context,
            scpi_commands,
            &scpi_interface,
            scpi_units_def,
            SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
            scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
            scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);

    scpi_context.user_context = &user_data;

    user_data.io_listen = createServer(SCPI_DEVICE_PORT);
    LWIP_ASSERT("user_data.io_listen != NULL", user_data.io_listen != NULL);

    user_data.control_io_listen = createServer(SCPI_CONTROL_PORT);
    LWIP_ASSERT("user_data.control_io_listen != NULL", user_data.control_io_listen != NULL);

    while (1) {
        waitServer(&user_data, &evt);

        if (evt.cmd == SCPI_MSG_TIMEOUT) { /* timeout */
            SCPI_Input(&scpi_context, NULL, 0);
        }

        if ((user_data.io_listen != NULL) && (evt.cmd == SCPI_MSG_IO_LISTEN)) {
            processIoListen(&user_data);
        }

        if ((user_data.control_io_listen != NULL) && (evt.cmd == SCPI_MSG_CONTROL_IO_LISTEN)) {
            processSrqIoListen(&user_data);
        }

        if ((user_data.io != NULL) && (evt.cmd == SCPI_MSG_IO)) {
            processIo(&user_data);
        }

        if ((user_data.control_io != NULL) && (evt.cmd == SCPI_MSG_CONTROL_IO)) {
            processSrqIo(&user_data);
        }

        if (evt.cmd == SCPI_MSG_SET_ESE_REQ) {
            setEseReq();
        }

        if (evt.cmd == SCPI_MSG_SET_ERROR) {
            setError(evt.param2);
        }

    }

    vTaskDelete(NULL);
}

void scpi_server_init(void) {
    osThreadId SCPIThreadID = sys_thread_new("SCPI", scpi_server_thread,NULL,
        2 * DEFAULT_THREAD_STACKSIZE, SCPI_THREAD_PRIO);
    /* osThreadCreate() returns 0 when stacksize * sizeof(StackType_t) < xFreeBytesRemaining
     * see FreeRTOSConfig.h: #define configTOTAL_HEAP_SIZE ( ( size_t ) ( 25 * 1024 )
     * or increase TOTAL_HEAP_SIZE in CubeMX > FreeRTOS > Memory management settings. */
    assert(SCPIThreadID);
}

/* Called by processIoListen() for additional reporting. Override on demand. */
void __attribute__((weak)) SCPI_Event_DeviceConnected(struct netconn * conn) {
    /* Remote or Eth LED ON */
}

/* Called by closeIO() for additional reporting. Override on demand. */
void __attribute__((weak)) SCPI_Event_DeviceDisconnected(struct netconn * conn) {
    /* Remote or Eth LED OFF */
}

/* Called by SCPI_Error() for reporting. Override on demand. */
void __attribute__((weak)) SCPI_Event_ErrorIndicatorOn(scpi_t * context, int_fast16_t err) {
    /* New error : Beep, Error LED ON */
}

/* Called by SCPI_Error() for reporting. Override on demand. */
void __attribute__((weak)) SCPI_Event_ErrorIndicatorOff(scpi_t * context, int_fast16_t err) {
    /* No more errors in the queue : Error LED OFF */
}
