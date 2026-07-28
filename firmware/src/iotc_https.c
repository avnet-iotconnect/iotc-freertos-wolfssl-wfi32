/*
 * iotc_https.c
 *
 * Blocking HTTP/HTTPS wrapper for IoTConnect using the project's Harmony
 * NET_PRES + wolfSSL http_client (see http_client.h).
 *
 * Design
 * ------
 * http_client_send_request() performs the whole transaction synchronously and
 * reports progress through its callback.  This module registers that callback
 * to accumulate the response body into a heap buffer that grows as needed,
 * and hands ownership of it to the caller.
 *
 * Only one request may be in flight at a time.  The module is not re-entrant;
 * ensure requests are serialised at the call site.
 *
 * TLS note
 * --------
 * TLS is terminated on the host by wolfSSL through NET_PRES.  The root CA of
 * the server must be present in the net_pres certificate store; there is no
 * per-request certificate material to pass in.  See iotc_https.h.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "definitions.h"

#include "http_client.h"
#include "iotc_https.h"

/* ======================================================================
 * Build-time tunables (override via project defines if needed)
 * ====================================================================== */

/** Size of the receive/work buffer handed to http_client.  It must be able to
 *  hold the complete response header block of any request made. */
#ifndef IOTC_HTTP_RECV_BUF_SIZE
#define IOTC_HTTP_RECV_BUF_SIZE         (2048U)
#endif

/** Initial heap allocation for the growing response buffer. */
#ifndef IOTC_HTTP_RESP_INITIAL_CAP
#define IOTC_HTTP_RESP_INITIAL_CAP      (2048U)
#endif

/** Hard cap on the response buffer to prevent unbounded heap growth. */
#ifndef IOTC_HTTP_RESP_MAX_CAP
#define IOTC_HTTP_RESP_MAX_CAP          (32U * 1024U)
#endif

/** Overall request timeout in milliseconds. */
#ifndef IOTC_HTTP_REQUEST_TIMEOUT_MS
#define IOTC_HTTP_REQUEST_TIMEOUT_MS    (30000U)
#endif

#define IOTC_HTTPS_PRNT(fmt, ...)  SYS_CONSOLE_PRINT("[IOTC_HTTP] " fmt, ##__VA_ARGS__)
#define IOTC_HTTPS_DBG(level, fmt, ...) \
    SYS_DEBUG_PRINT(level, "[IOTC_HTTP] " fmt, ##__VA_ARGS__)

/* ======================================================================
 * Module-private state
 * ====================================================================== */

/** Completion state of the in-flight request. */
typedef enum {
    HTTP_REQ_IDLE     = 0,
    HTTP_REQ_WAITING,
    HTTP_REQ_DONE_OK,
    HTTP_REQ_DONE_ERR,
} http_req_state_t;

static struct http_client_module  s_http_mod;
static bool                       s_initialized = false;
static http_req_state_t           s_req_state   = HTTP_REQ_IDLE;

/* ---- Response accumulation buffer ---- */
static char  *s_resp_buf = NULL;   /* heap-allocated, grows as needed   */
static size_t s_resp_len = 0U;     /* bytes written (excluding NUL)     */
static size_t s_resp_cap = 0U;     /* current allocated capacity        */
static bool   s_resp_ovf = false;  /* true when cap/OOM limit is hit    */

/** Receive/work buffer for the http_client layer. */
static char s_recv_buf[IOTC_HTTP_RECV_BUF_SIZE];

/* ======================================================================
 * Internal helpers
 * ====================================================================== */

/**
 * Append @p len bytes from @p data into the heap-based response buffer.
 * Doubles capacity as needed up to IOTC_HTTP_RESP_MAX_CAP.
 * Sets s_resp_ovf on allocation failure or cap exceeded.
 */
static void resp_append(const void *data, size_t len)
{
    if (data == NULL || len == 0U || s_resp_ovf) {
        return;
    }

    size_t needed = s_resp_len + len + 1U; /* +1 for NUL terminator */

    if (needed > s_resp_cap) {
        size_t new_cap = (s_resp_cap == 0U)
                         ? IOTC_HTTP_RESP_INITIAL_CAP
                         : s_resp_cap;
        while (new_cap < needed) {
            new_cap *= 2U;
        }
        if (new_cap > IOTC_HTTP_RESP_MAX_CAP) {
            new_cap = IOTC_HTTP_RESP_MAX_CAP;
        }
        if (new_cap < needed) {
            IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "response exceeds %lu bytes\r\n",
                           (unsigned long)IOTC_HTTP_RESP_MAX_CAP);
            s_resp_ovf = true;
            return;
        }

        char *nb = (char *)malloc(new_cap);
        if (nb == NULL) {
            IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "out of memory for the response\r\n");
            s_resp_ovf = true;
            return;
        }
        if (s_resp_buf != NULL) {
            memcpy(nb, s_resp_buf, s_resp_len);
            free(s_resp_buf);
        }
        s_resp_buf = nb;
        s_resp_cap = new_cap;
    }

    memcpy(s_resp_buf + s_resp_len, data, len);
    s_resp_len            += len;
    s_resp_buf[s_resp_len] = '\0';
}

/** Free and reset all response state. */
static void resp_reset(void)
{
    if (s_resp_buf != NULL) {
        free(s_resp_buf);
        s_resp_buf = NULL;
    }
    s_resp_len = 0U;
    s_resp_cap = 0U;
    s_resp_ovf = false;
}

/** Map iotc_http_opcode_t to enum http_method. */
static enum http_method opcode_to_method(iotc_http_opcode_t op)
{
    switch (op) {
    case IOTC_HTTP_OPCODE_POST:   return HTTP_METHOD_POST;
    case IOTC_HTTP_OPCODE_PUT:    return HTTP_METHOD_PUT;
    case IOTC_HTTP_OPCODE_DELETE: return HTTP_METHOD_DELETE;
    case IOTC_HTTP_OPCODE_GET:
    default:                      return HTTP_METHOD_GET;
    }
}

/* ======================================================================
 * http_client callback
 * ====================================================================== */

static void http_cb(struct http_client_module *mod, int type,
                    union http_client_data *data)
{
    (void)mod;

    switch (type) {
    /* ------------------------------------------------------------------ */
    case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
        if (data->sock_connected.result == 0) {
            IOTC_HTTPS_DBG(SYS_ERROR_INFO, "socket connected\r\n");
        } else {
            IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "connect failed (%d)\r\n",
                           data->sock_connected.result);
            s_req_state = HTTP_REQ_DONE_ERR;
        }
        break;

    /* ------------------------------------------------------------------ */
    case HTTP_CLIENT_CALLBACK_REQUESTED:
        IOTC_HTTPS_DBG(SYS_ERROR_INFO, "request sent\r\n");
        break;

    /* ------------------------------------------------------------------ */
    case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
    {
        uint16_t code = data->recv_response.response_code;

        IOTC_HTTPS_DBG(SYS_ERROR_INFO, "HTTP %u, content_length=%lu, chunked=%u\r\n",
                       (unsigned)code,
                       (unsigned long)data->recv_response.content_length,
                       (unsigned)data->recv_response.is_chunked);

        if (code != 200U) {
            s_req_state = HTTP_REQ_DONE_ERR;
            break;
        }

        /* Capture the body if it arrived inline (fits in the recv buffer). */
        if (data->recv_response.content != NULL &&
            data->recv_response.content_length > 0U) {
            resp_append(data->recv_response.content,
                        (size_t)data->recv_response.content_length);
        }

        /* When the body is not delivered piecewise, it is complete now. */
        if (data->recv_response.is_chunked == 0U) {
            s_req_state = HTTP_REQ_DONE_OK;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
    {
        if (s_req_state == HTTP_REQ_DONE_ERR) {
            break;   /* a non-200 status was already reported */
        }

        resp_append(data->recv_chunked_data.data,
                    (size_t)data->recv_chunked_data.length);

        if (data->recv_chunked_data.is_complete) {
            s_req_state = HTTP_REQ_DONE_OK;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case HTTP_CLIENT_CALLBACK_DISCONNECTED:
        IOTC_HTTPS_DBG(SYS_ERROR_INFO, "disconnected, reason=%d\r\n",
                       data->disconnected.reason);

        /* Only an error if the response never completed; a clean disconnect
         * after DONE_OK is the normal end of a "Connection: close" exchange. */
        if (s_req_state == HTTP_REQ_WAITING) {
            s_req_state = HTTP_REQ_DONE_ERR;
        }
        break;

    default:
        break;
    }
}

/* ======================================================================
 * Public API
 * ====================================================================== */

int iotc_http_client_init(void)
{
    struct http_client_config cfg;
    int rc;

    if (s_initialized) {
        return 0;
    }

    http_client_get_config_defaults(&cfg);
    cfg.recv_buffer      = s_recv_buf;
    cfg.recv_buffer_size = IOTC_HTTP_RECV_BUF_SIZE;
    cfg.timeout          = IOTC_HTTP_REQUEST_TIMEOUT_MS;

    rc = http_client_init(&s_http_mod, &cfg);
    if (rc != 0) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "http_client_init error %d\r\n", rc);
        return rc;
    }

    rc = http_client_register_callback(&s_http_mod, http_cb);
    if (rc != 0) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "register_callback error %d\r\n", rc);
        (void)http_client_deinit(&s_http_mod);
        return rc;
    }

    s_initialized = true;
    IOTC_HTTPS_PRNT("initialized\r\n");
    return 0;
}

/* ---------------------------------------------------------------------- */

void iotc_http_client_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    (void)http_client_deinit(&s_http_mod);
    resp_reset();
    s_req_state   = HTTP_REQ_IDLE;
    s_initialized = false;
}

/* ---------------------------------------------------------------------- */

void http_client_response_free(char *resp)
{
    if (resp != NULL) {
        free(resp);
    }
}

/* ---------------------------------------------------------------------- */

int http_client_request(
    const char        *url,
    iotc_http_opcode_t op_code,
    const void        *body,
    size_t             body_len,
    char             **resp_out,
    size_t            *resp_len_out
)
{
    http_req_state_t final_state;
    bool             needs_entity;
    int              rc;

    /* Output initialisation. */
    if (resp_out     != NULL) { *resp_out     = NULL; }
    if (resp_len_out != NULL) { *resp_len_out = 0U; }

    if (url == NULL) {
        return -EINVAL;
    }
    if (!s_initialized) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR,
                       "not initialised -- call iotc_http_client_init()\r\n");
        return -ENODEV;
    }
    if (s_req_state != HTTP_REQ_IDLE) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "request already in progress\r\n");
        return -EBUSY;
    }

    /* Validate the body for methods that require one. */
    needs_entity = (op_code == IOTC_HTTP_OPCODE_POST ||
                    op_code == IOTC_HTTP_OPCODE_PUT);
    if (needs_entity && (body == NULL || body_len == 0U)) {
        return -EINVAL;
    }

    /* Set up the response capture.  When the caller passes NULL for resp_out,
     * set the overflow flag so resp_append() discards all data. */
    resp_reset();
    s_resp_ovf = (resp_out == NULL);

    s_req_state = HTTP_REQ_WAITING;

    /* http_client_send_request() blocks until the transaction is finished and
     * drives http_cb() along the way, so the completion state is final when
     * it returns. */
    rc = http_client_send_request(&s_http_mod, url, opcode_to_method(op_code),
                                  body, (uint32_t)body_len,
                                  NULL /* ext_header */);

    final_state = s_req_state;
    s_req_state = HTTP_REQ_IDLE;

    if (rc != 0) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "request failed (%d)\r\n", rc);
        resp_reset();
        return rc;
    }

    if (final_state != HTTP_REQ_DONE_OK) {
        IOTC_HTTPS_DBG(SYS_ERROR_ERROR, "request did not complete\r\n");
        resp_reset();
        return -EIO;
    }

    /* ------------------------------------------------------------------ */
    /* Hand off the captured response to the caller.                       */
    /* ------------------------------------------------------------------ */
    if (resp_out != NULL) {
        if (s_resp_ovf) {
            /* Memory exhausted, or the cap was hit: the body is incomplete. */
            resp_reset();
            return -ENOMEM;
        }
        *resp_out = s_resp_buf;
        if (resp_len_out != NULL) {
            *resp_len_out = s_resp_len;
        }
        /* Detach: ownership transfers to the caller. */
        s_resp_buf = NULL;
        s_resp_len = 0U;
        s_resp_cap = 0U;
    } else {
        resp_reset();
    }

    return 0;
}
