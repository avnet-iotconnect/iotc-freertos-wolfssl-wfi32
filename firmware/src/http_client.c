/*
 * http_client.c
 *
 * Blocking HTTP/1.1 client on top of the Harmony NET_PRES socket API
 * (TCP/IP stack + wolfSSL) for the WFI32 (PIC32MZW1).
 *
 * See http_client.h for the API and for the differences against the ASF
 * WINC1500 http_client service this module replaces.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "definitions.h"
#include "FreeRTOS.h"
#include "task.h"

#include "http_client.h"

/* ======================================================================
 * Build-time tunables
 * ====================================================================== */

/** Poll interval of the blocking loops, in milliseconds. */
#ifndef HTTP_CLIENT_POLL_MS
#define HTTP_CLIENT_POLL_MS             (5U)
#endif

/** DNS resolution timeout, in milliseconds. */
#ifndef HTTP_CLIENT_DNS_TIMEOUT_MS
#define HTTP_CLIENT_DNS_TIMEOUT_MS      (15000U)
#endif

/** TCP connect timeout, in milliseconds. */
#ifndef HTTP_CLIENT_CONNECT_TIMEOUT_MS
#define HTTP_CLIENT_CONNECT_TIMEOUT_MS  (15000U)
#endif

/** TLS handshake timeout, in milliseconds. */
#ifndef HTTP_CLIENT_TLS_TIMEOUT_MS
#define HTTP_CLIENT_TLS_TIMEOUT_MS      (20000U)
#endif

#define HTTP_CLIENT_PRNT(fmt, ...)  SYS_CONSOLE_PRINT("[HTTP] " fmt, ##__VA_ARGS__)
#define HTTP_CLIENT_DBG(level, fmt, ...) \
    SYS_DEBUG_PRINT(level, "[HTTP] " fmt, ##__VA_ARGS__)

#if HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT
/* Defined in app_aws.c.  net_pres_enc_glue.c reads this global when it opens
 * the wolfSSL session and uses it as the SNI host name, so it has to name the
 * host this request is going to while the handshake runs. */
extern char g_Cloud_Endpoint[100];
#endif

/* ======================================================================
 * Small helpers
 * ====================================================================== */

static uint32_t ms_now(void)
{
    return (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
}

/** true when \p deadline (an ms_now() value) has passed. */
static bool ms_expired(uint32_t deadline)
{
    return (int32_t)(ms_now() - deadline) >= 0;
}

static void http_client_dispatch(struct http_client_module *const module,
                                 int type, union http_client_data *data)
{
    if (module->cb != NULL) {
        module->cb(module, type, data);
    }
}

/** Case-insensitive strncmp.  Provided locally: strncasecmp lives in
 *  <strings.h>, which is not part of every XC32 newlib configuration. */
static int hc_strncasecmp(const char *a, const char *b, size_t n)
{
    size_t i;

    for (i = 0U; i < n; i++) {
        int ca = tolower((unsigned char)a[i]);
        int cb = tolower((unsigned char)b[i]);

        if (ca != cb) {
            return ca - cb;
        }
        if (ca == 0) {
            break;
        }
    }
    return 0;
}

/** Case-insensitive search of \p needle in the NUL-terminated \p haystack. */
static const char *stristr(const char *haystack, const char *needle)
{
    size_t nlen = strlen(needle);

    if (nlen == 0U) {
        return haystack;
    }

    for (; *haystack != '\0'; haystack++) {
        if (hc_strncasecmp(haystack, needle, nlen) == 0) {
            return haystack;
        }
    }
    return NULL;
}

/**
 * Find the value of header \p name inside the NUL-terminated header block
 * \p headers.  Returns a pointer to the first character of the value, or NULL.
 */
static const char *header_value(const char *headers, const char *name)
{
    const char *p = headers;
    size_t      nlen = strlen(name);

    /* Header lines start right after a CRLF; skip the status line. */
    while ((p = strstr(p, "\r\n")) != NULL) {
        p += 2;
        if (hc_strncasecmp(p, name, nlen) == 0 && p[nlen] == ':') {
            p += nlen + 1U;
            while (*p == ' ' || *p == '\t') {
                p++;
            }
            return p;
        }
    }
    return NULL;
}

/* ======================================================================
 * URL parsing
 * ====================================================================== */

/**
 * Split \p url into scheme / host / port / path.
 *
 * \return 0 on success, -EINVAL on a malformed or oversized URL.
 */
static int url_parse(const char *url, uint8_t *tls, char *host, size_t host_size,
                     uint16_t *port, char *path, size_t path_size)
{
    const char *p = url;
    const char *host_start;
    const char *host_end;
    const char *colon;
    size_t      host_len;

    if (hc_strncasecmp(p, "https://", 8) == 0) {
        *tls  = 1U;
        *port = 443U;
        p    += 8;
    } else if (hc_strncasecmp(p, "http://", 7) == 0) {
        *tls  = 0U;
        *port = 80U;
        p    += 7;
    } else {
        return -EINVAL;
    }

    host_start = p;
    host_end   = host_start;
    while (*host_end != '\0' && *host_end != '/' && *host_end != '?') {
        host_end++;
    }
    if (host_end == host_start) {
        return -EINVAL;
    }

    /* Optional ":port" -- ignore a colon that belongs to an IPv6 literal. */
    colon = memchr(host_start, ':', (size_t)(host_end - host_start));
    if (colon != NULL && memchr(host_start, ']', (size_t)(host_end - host_start)) == NULL) {
        long val = strtol(colon + 1, NULL, 10);
        if (val <= 0 || val > 65535) {
            return -EINVAL;
        }
        *port    = (uint16_t)val;
        host_len = (size_t)(colon - host_start);
    } else {
        host_len = (size_t)(host_end - host_start);
    }

    if (host_len == 0U || host_len >= host_size) {
        return -EINVAL;
    }
    memcpy(host, host_start, host_len);
    host[host_len] = '\0';

    if (*host_end == '\0') {
        if (path_size < 2U) {
            return -EINVAL;
        }
        path[0] = '/';
        path[1] = '\0';
    } else {
        if (strlen(host_end) >= path_size) {
            return -EINVAL;
        }
        strcpy(path, host_end);
    }

    return 0;
}

static const char *method_name(enum http_method method)
{
    switch (method) {
    case HTTP_METHOD_POST:    return "POST";
    case HTTP_METHOD_DELETE:  return "DELETE";
    case HTTP_METHOD_PUT:     return "PUT";
    case HTTP_METHOD_OPTIONS: return "OPTIONS";
    case HTTP_METHOD_HEAD:    return "HEAD";
    case HTTP_METHOD_GET:
    default:                  return "GET";
    }
}

/* ======================================================================
 * Socket layer
 * ====================================================================== */

/**
 * Resolve \p host into \p addr, waiting for the DNS client if needed.
 *
 * \return 0 on success, negative errno code on failure.
 */
static int host_resolve(const char *host, IP_MULTI_ADDRESS *addr, uint32_t deadline)
{
    TCPIP_DNS_RESULT result;

    if (TCPIP_Helper_StringToIPAddress(host, &addr->v4Add)) {
        /* Already a dotted-quad literal. */
        return 0;
    }

    result = TCPIP_DNS_Resolve(host, TCPIP_DNS_TYPE_A);
    if (result < 0) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "DNS resolve of %s failed (%d)\r\n",
                        host, (int)result);
        return -ENOENT;
    }

    for (;;) {
        result = TCPIP_DNS_IsResolved(host, addr, IP_ADDRESS_TYPE_IPV4);

        if (result == TCPIP_DNS_RES_OK) {
            HTTP_CLIENT_DBG(SYS_ERROR_INFO, "%s -> %d.%d.%d.%d\r\n", host,
                            addr->v4Add.v[0], addr->v4Add.v[1],
                            addr->v4Add.v[2], addr->v4Add.v[3]);
            return 0;
        }
        if (result != TCPIP_DNS_RES_PENDING) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "DNS lookup of %s failed (%d)\r\n",
                            host, (int)result);
            return -ENOENT;
        }
        if (ms_expired(deadline)) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "DNS lookup of %s timed out\r\n", host);
            return -ETIMEDOUT;
        }
        vTaskDelay(50U / portTICK_PERIOD_MS);
    }
}

/**
 * Open a TCP (and, when module->tls is set, TLS) session to module->host.
 *
 * \return 0 on success, negative errno code on failure.
 */
static int sock_open(struct http_client_module *const module)
{
    IP_MULTI_ADDRESS      addr;
    NET_PRES_SKT_ERROR_T  error = NET_PRES_SKT_OK;
    uint32_t              deadline;
    int                   rc;
#if HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT
    char                  saved_endpoint[sizeof(g_Cloud_Endpoint)];
#endif

    memset(&addr, 0, sizeof(addr));

    deadline = ms_now() + HTTP_CLIENT_DNS_TIMEOUT_MS;
    rc = host_resolve(module->host, &addr, deadline);
    if (rc != 0) {
        return rc;
    }

    module->socket = NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT,
                                         IP_ADDRESS_TYPE_IPV4,
                                         (NET_PRES_SKT_PORT_T)module->port,
                                         (NET_PRES_ADDRESS *)&addr, &error);
    if (module->socket == NET_PRES_INVALID_SOCKET) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "socket open failed (%d)\r\n", (int)error);
        module->socket = NET_PRES_INVALID_SOCKET;
        return -ENOSPC;
    }
    module->connected = 1U;

    /* Clear the stale reset flag the same way the AWS network port does. */
    (void)NET_PRES_SocketWasReset(module->socket);

    deadline = ms_now() + HTTP_CLIENT_CONNECT_TIMEOUT_MS;
    while (!NET_PRES_SocketIsConnected(module->socket)) {
        if (ms_expired(deadline)) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "connect to %s:%u timed out\r\n",
                            module->host, (unsigned)module->port);
            return -ETIMEDOUT;
        }
        vTaskDelay(HTTP_CLIENT_POLL_MS / portTICK_PERIOD_MS);
    }

    if (module->tls == 0U) {
        return 0;
    }

#if HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT
    /* Point the SNI source at this request's host for the handshake, then put
     * back whatever the application had there (the MQTT broker endpoint). */
    memcpy(saved_endpoint, g_Cloud_Endpoint, sizeof(saved_endpoint));
    snprintf(g_Cloud_Endpoint, sizeof(g_Cloud_Endpoint), "%s", module->host);
#endif

    if (!NET_PRES_SocketEncryptSocket(module->socket)) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "TLS session create failed\r\n");
        rc = -EIO;
        goto tls_done;
    }

    deadline = ms_now() + HTTP_CLIENT_TLS_TIMEOUT_MS;
    while (NET_PRES_SocketIsNegotiatingEncryption(module->socket)) {
        if (ms_expired(deadline)) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "TLS handshake timed out\r\n");
            rc = -ETIMEDOUT;
            goto tls_done;
        }
        vTaskDelay(HTTP_CLIENT_POLL_MS / portTICK_PERIOD_MS);
    }

    if (!NET_PRES_SocketIsSecure(module->socket)) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "TLS negotiation failed\r\n");
        rc = -EIO;
        goto tls_done;
    }

    rc = 0;

tls_done:
#if HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT
    memcpy(g_Cloud_Endpoint, saved_endpoint, sizeof(saved_endpoint));
#endif
    return rc;
}

/**
 * Send \p len bytes, blocking until they are all handed to the stack.
 *
 * \return 0 on success, negative errno code on failure.
 */
static int sock_write_all(struct http_client_module *const module,
                          const void *data, uint32_t len, uint32_t deadline)
{
    const uint8_t *p    = (const uint8_t *)data;
    uint32_t       left = len;

    while (left > 0U) {
        uint16_t want = (left > 0xFFFFU) ? 0xFFFFU : (uint16_t)left;
        uint16_t room;

        if (NET_PRES_SocketWasReset(module->socket) ||
            !NET_PRES_SocketIsConnected(module->socket)) {
            return -ECONNRESET;
        }

        room = NET_PRES_SocketWriteIsReady(module->socket, want, 1U);
        if (room > 0U) {
            uint16_t sent;

            if (room < want) {
                want = room;
            }
            sent = NET_PRES_SocketWrite(module->socket, p, want);
            if (sent > 0U) {
                p    += sent;
                left -= sent;
                continue;
            }
        }

        if (ms_expired(deadline)) {
            return -ETIMEDOUT;
        }
        vTaskDelay(HTTP_CLIENT_POLL_MS / portTICK_PERIOD_MS);
    }

    (void)NET_PRES_SocketFlush(module->socket);
    return 0;
}

/**
 * Append received bytes to \p buf.
 *
 * \return  >0  number of bytes appended.
 * \return   0  the peer closed the connection and no data is left.
 * \return  <0  negative errno code.
 */
static int sock_read_some(struct http_client_module *const module, char *buf,
                          uint32_t cap, uint32_t len, uint32_t deadline)
{
    for (;;) {
        uint16_t ready = NET_PRES_SocketReadIsReady(module->socket);

        if (ready > 0U) {
            uint32_t space = cap - len;
            uint16_t want;
            uint16_t got;

            if (space == 0U) {
                return -EOVERFLOW;
            }
            want = (space > 0xFFFFU) ? 0xFFFFU : (uint16_t)space;
            if (ready < want) {
                want = ready;
            }
            got = NET_PRES_SocketRead(module->socket, buf + len, want);
            if (got > 0U) {
                return (int)got;
            }
        }

        if (NET_PRES_SocketWasReset(module->socket)) {
            return -ECONNRESET;
        }
        if (!NET_PRES_SocketIsConnected(module->socket)) {
            /* Peer closed; report EOF only once the receive queue is drained. */
            if (NET_PRES_SocketReadIsReady(module->socket) == 0U) {
                return 0;
            }
            continue;
        }
        if (ms_expired(deadline)) {
            return -ETIMEDOUT;
        }
        vTaskDelay(HTTP_CLIENT_POLL_MS / portTICK_PERIOD_MS);
    }
}

/* ======================================================================
 * Response handling
 * ====================================================================== */

/** Deliver one piece of the body to the callback. */
static void deliver_body(struct http_client_module *const module,
                         char *data, uint32_t length, bool complete)
{
    union http_client_data cbd;

    module->resp.read_length += length;

    memset(&cbd, 0, sizeof(cbd));
    cbd.recv_chunked_data.data        = data;
    cbd.recv_chunked_data.length      = length;
    cbd.recv_chunked_data.is_complete = complete ? 1 : 0;
    http_client_dispatch(module, HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA, &cbd);
}

/**
 * Read a body of known length that was not fully buffered yet.
 *
 * \p buf/\p len hold the part of the body already received.
 * \p remaining is what is still outstanding (0 when framed by connection
 * close, in which case the read runs until EOF).
 */
static int body_read_plain(struct http_client_module *const module, char *buf,
                           uint32_t cap, uint32_t len, uint32_t remaining,
                           bool until_close, uint32_t deadline)
{
    if (len > 0U) {
        bool done = (!until_close && len >= remaining);

        if (!until_close && len > remaining) {
            len = remaining;
        }
        remaining -= len;
        deliver_body(module, buf, len, done);
        if (done) {
            return 0;
        }
    }

    for (;;) {
        int got = sock_read_some(module, buf, cap, 0U, deadline);

        if (got < 0) {
            return got;
        }
        if (got == 0) {
            /* EOF. */
            if (until_close) {
                deliver_body(module, buf, 0U, true);
                return 0;
            }
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "connection closed with %lu body "
                            "bytes outstanding\r\n", (unsigned long)remaining);
            return -ECONNRESET;
        }

        if (!until_close) {
            uint32_t take = ((uint32_t)got > remaining) ? remaining : (uint32_t)got;

            remaining -= take;
            deliver_body(module, buf, take, (remaining == 0U));
            if (remaining == 0U) {
                return 0;
            }
        } else {
            deliver_body(module, buf, (uint32_t)got, false);
        }
    }
}

/**
 * Read and decode a "Transfer-Encoding: chunked" body.
 *
 * \p buf/\p len hold the part of the body already received; the buffer is
 * reused as the working window and is compacted as chunks are consumed.
 */
static int body_read_chunked(struct http_client_module *const module, char *buf,
                             uint32_t cap, uint32_t len, uint32_t deadline)
{
    uint32_t chunk_left  = 0U;   /* bytes left in the current chunk  */
    bool     want_header = true; /* next thing to parse is a size line */

    for (;;) {
        if (want_header) {
            char    *eol;
            uint32_t line_len;

            /* A chunk size line is "<hex>[;ext]\r\n". */
            buf[len] = '\0';   /* cap is one byte larger than the usable size */
            eol = strstr(buf, "\r\n");
            if (eol == NULL) {
                int got;

                if (len + 1U >= cap) {
                    return -EOVERFLOW;
                }
                got = sock_read_some(module, buf, cap - 1U, len, deadline);
                if (got < 0) {
                    return got;
                }
                if (got == 0) {
                    return -ECONNRESET;
                }
                len += (uint32_t)got;
                continue;
            }

            chunk_left = (uint32_t)strtoul(buf, NULL, 16);
            line_len   = (uint32_t)(eol - buf) + 2U;
            memmove(buf, buf + line_len, len - line_len);
            len -= line_len;

            if (chunk_left == 0U) {
                /* Last chunk; the trailer is not needed by any caller. */
                deliver_body(module, buf, 0U, true);
                return 0;
            }
            want_header = false;
            continue;
        }

        if (len > 0U) {
            uint32_t take = (len > chunk_left) ? chunk_left : len;

            deliver_body(module, buf, take, false);
            chunk_left -= take;
            memmove(buf, buf + take, len - take);
            len -= take;

            if (chunk_left == 0U) {
                /* Consume the CRLF that terminates the chunk data. */
                while (len < 2U) {
                    int got = sock_read_some(module, buf, cap - 1U, len, deadline);

                    if (got < 0) {
                        return got;
                    }
                    if (got == 0) {
                        return -ECONNRESET;
                    }
                    len += (uint32_t)got;
                }
                memmove(buf, buf + 2, len - 2U);
                len -= 2U;
                want_header = true;
            }
            continue;
        }

        {
            int got = sock_read_some(module, buf, cap - 1U, len, deadline);

            if (got < 0) {
                return got;
            }
            if (got == 0) {
                return -ECONNRESET;
            }
            len += (uint32_t)got;
        }
    }
}

/**
 * Receive the status line and headers, then the body.
 *
 * \return 0 on success, negative errno code on failure.
 */
static int response_receive(struct http_client_module *const module, uint32_t deadline)
{
    char    *buf = module->config.recv_buffer;
    uint32_t cap = module->config.recv_buffer_size;
    uint32_t len = 0U;
    char    *hdr_end;
    uint32_t hdr_len;
    uint32_t body_len;
    char     saved;
    const char *value;
    bool     until_close = false;
    union http_client_data cbd;
    int      rc;

    /* ---- status line + headers ---- */
    for (;;) {
        int got;

        buf[len] = '\0';
        hdr_end  = strstr(buf, "\r\n\r\n");
        if (hdr_end != NULL) {
            break;
        }

        if (len + 1U >= cap) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "response headers exceed the "
                            "receive buffer (%lu bytes)\r\n", (unsigned long)cap);
            return -EOVERFLOW;
        }

        got = sock_read_some(module, buf, cap - 1U, len, deadline);
        if (got < 0) {
            return got;
        }
        if (got == 0) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "connection closed before the "
                            "response headers were complete\r\n");
            return -ECONNRESET;
        }
        len += (uint32_t)got;
    }

    hdr_len  = (uint32_t)(hdr_end - buf) + 4U;
    body_len = len - hdr_len;

    /* Parse over a temporarily NUL-terminated header block. */
    saved         = buf[hdr_len - 2U];
    buf[hdr_len - 2U] = '\0';

    if (hc_strncasecmp(buf, "HTTP/1.", 7) != 0) {
        buf[hdr_len - 2U] = saved;
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "malformed status line\r\n");
        return -EBADMSG;
    }
    module->resp.response_code = (uint16_t)strtoul(buf + 9, NULL, 10);

    value = header_value(buf, "Content-Length");
    module->resp.content_length = (value != NULL)
                                  ? (uint32_t)strtoul(value, NULL, 10) : 0U;

    value = header_value(buf, "Transfer-Encoding");
    module->resp.is_chunked = (value != NULL && stristr(value, "chunked") != NULL)
                              ? 1U : 0U;

    if (module->resp.is_chunked == 0U && header_value(buf, "Content-Length") == NULL) {
        /* No framing information: the body ends when the server closes. */
        until_close = true;
    }

    buf[hdr_len - 2U] = saved;

    HTTP_CLIENT_DBG(SYS_ERROR_INFO, "HTTP %u, content_length=%lu, chunked=%u\r\n",
                    (unsigned)module->resp.response_code,
                    (unsigned long)module->resp.content_length,
                    (unsigned)module->resp.is_chunked);

    /* ---- response callback ---- */
    memset(&cbd, 0, sizeof(cbd));
    cbd.recv_response.response_code  = module->resp.response_code;
    cbd.recv_response.content_length = module->resp.content_length;

    if (module->req.method == HTTP_METHOD_HEAD) {
        cbd.recv_response.is_chunked = 0U;
        cbd.recv_response.content    = NULL;
        http_client_dispatch(module, HTTP_CLIENT_CALLBACK_RECV_RESPONSE, &cbd);
        return 0;
    }

    if (module->resp.is_chunked == 0U && !until_close &&
        body_len >= module->resp.content_length) {
        /* Whole body already in the buffer: hand it over in one go. */
        cbd.recv_response.is_chunked = 0U;
        cbd.recv_response.content    = buf + hdr_len;
        module->resp.read_length     = module->resp.content_length;
        http_client_dispatch(module, HTTP_CLIENT_CALLBACK_RECV_RESPONSE, &cbd);
        return 0;
    }

    cbd.recv_response.is_chunked = 1U;
    cbd.recv_response.content    = NULL;
    http_client_dispatch(module, HTTP_CLIENT_CALLBACK_RECV_RESPONSE, &cbd);

    /* ---- body ---- */
    memmove(buf, buf + hdr_len, body_len);

    if (module->resp.is_chunked != 0U) {
        rc = body_read_chunked(module, buf, cap, body_len, deadline);
    } else {
        rc = body_read_plain(module, buf, cap, body_len,
                             module->resp.content_length, until_close, deadline);
    }

    return rc;
}

/* ======================================================================
 * Public API
 * ====================================================================== */

void http_client_get_config_defaults(struct http_client_config *const config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->port             = 80U;
    config->tls              = 0U;
    config->timeout          = 30000U;
    config->recv_buffer      = NULL;
    config->recv_buffer_size = 0U;
    config->send_buffer_size = 1024U;
    config->user_agent       = HTTP_CLIENT_USER_AGENT;
}

int http_client_init(struct http_client_module *const module,
                     struct http_client_config *config)
{
    if (module == NULL || config == NULL) {
        return -EINVAL;
    }
    if (config->recv_buffer == NULL || config->recv_buffer_size < 256U) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "a receive buffer of at least 256 "
                        "bytes is required\r\n");
        return -EINVAL;
    }

    memset(module, 0, sizeof(*module));
    module->socket = NET_PRES_INVALID_SOCKET;
    module->config = *config;

    if (module->config.send_buffer_size < 256U) {
        module->config.send_buffer_size = 1024U;
    }
    if (module->config.timeout == 0U) {
        module->config.timeout = 30000U;
    }
    if (module->config.user_agent == NULL) {
        module->config.user_agent = HTTP_CLIENT_USER_AGENT;
    }

    return 0;
}

int http_client_deinit(struct http_client_module *const module)
{
    if (module == NULL) {
        return -EINVAL;
    }

    (void)http_client_close(module);
    module->cb = NULL;
    return 0;
}

int http_client_register_callback(struct http_client_module *const module,
                                  http_client_callback_t callback)
{
    if (module == NULL || callback == NULL) {
        return -EINVAL;
    }

    module->cb = callback;
    return 0;
}

int http_client_unregister_callback(struct http_client_module *const module)
{
    if (module == NULL) {
        return -EINVAL;
    }

    module->cb = NULL;
    return 0;
}

int http_client_close(struct http_client_module *const module)
{
    if (module == NULL) {
        return -EINVAL;
    }

    if (module->socket != NET_PRES_INVALID_SOCKET) {
        NET_PRES_SocketClose(module->socket);
        module->socket = NET_PRES_INVALID_SOCKET;
    }
    module->connected = 0U;
    return 0;
}

int http_client_send_request(struct http_client_module *const module,
                             const char *url, enum http_method method,
                             const void *body, uint32_t body_len,
                             const char *ext_header)
{
    union http_client_data cbd;
    char    *req_buf = NULL;
    uint32_t deadline;
    int      written;
    int      rc;

    if (module == NULL || url == NULL) {
        return -EINVAL;
    }
    if (module->config.recv_buffer == NULL) {
        return -ENODEV;
    }
    if ((method == HTTP_METHOD_POST || method == HTTP_METHOD_PUT) &&
        (body == NULL || body_len == 0U)) {
        return -EINVAL;
    }

    /* A previous session may still be open (keep-alive is not used here). */
    (void)http_client_close(module);

    memset(&module->req,  0, sizeof(module->req));
    memset(&module->resp, 0, sizeof(module->resp));
    module->req.method         = method;
    module->req.body           = body;
    module->req.content_length = (body != NULL) ? body_len : 0U;

    rc = url_parse(url, &module->tls, module->host, sizeof(module->host),
                   &module->port, module->req.uri, sizeof(module->req.uri));
    if (rc != 0) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "cannot parse URL\r\n");
        return rc;
    }
    module->config.tls  = module->tls;
    module->config.port = module->port;

    deadline = ms_now() + module->config.timeout;

    /* ---- connect ---- */
    rc = sock_open(module);

    memset(&cbd, 0, sizeof(cbd));
    cbd.sock_connected.result = rc;
    http_client_dispatch(module, HTTP_CLIENT_CALLBACK_SOCK_CONNECTED, &cbd);

    if (rc != 0) {
        goto disconnect;
    }

    /* ---- build and send the request ---- */
    req_buf = (char *)malloc(module->config.send_buffer_size);
    if (req_buf == NULL) {
        rc = -ENOMEM;
        goto disconnect;
    }

    written = snprintf(req_buf, module->config.send_buffer_size,
                       "%s %s " HTTP_PROTO_NAME "\r\n"
                       "Host: %s\r\n"
                       "User-Agent: %s\r\n"
                       "Accept: */*\r\n"
                       "Connection: close\r\n",
                       method_name(method), module->req.uri, module->host,
                       module->config.user_agent);
    if (written < 0 || (uint32_t)written >= module->config.send_buffer_size) {
        rc = -EOVERFLOW;
        goto disconnect;
    }

    if (module->req.content_length > 0U) {
        int n = snprintf(req_buf + written,
                         module->config.send_buffer_size - (uint32_t)written,
                         "Content-Length: %lu\r\n",
                         (unsigned long)module->req.content_length);
        if (n < 0 || (uint32_t)(written + n) >= module->config.send_buffer_size) {
            rc = -EOVERFLOW;
            goto disconnect;
        }
        written += n;
    }

    if (ext_header != NULL) {
        int n = snprintf(req_buf + written,
                         module->config.send_buffer_size - (uint32_t)written,
                         "%s", ext_header);
        if (n < 0 || (uint32_t)(written + n) >= module->config.send_buffer_size) {
            rc = -EOVERFLOW;
            goto disconnect;
        }
        written += n;
    }

    if ((uint32_t)written + 2U >= module->config.send_buffer_size) {
        rc = -EOVERFLOW;
        goto disconnect;
    }
    memcpy(req_buf + written, "\r\n", 2);
    written += 2;

    rc = sock_write_all(module, req_buf, (uint32_t)written, deadline);
    if (rc != 0) {
        HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "sending the request failed (%d)\r\n", rc);
        goto disconnect;
    }

    if (module->req.content_length > 0U) {
        rc = sock_write_all(module, module->req.body,
                            module->req.content_length, deadline);
        if (rc != 0) {
            HTTP_CLIENT_DBG(SYS_ERROR_ERROR, "sending the body failed (%d)\r\n", rc);
            goto disconnect;
        }
        module->req.sent_length = module->req.content_length;
    }

    memset(&cbd, 0, sizeof(cbd));
    cbd.requested.sent_length = (int)module->req.sent_length;
    http_client_dispatch(module, HTTP_CLIENT_CALLBACK_REQUESTED, &cbd);

    /* ---- receive ---- */
    rc = response_receive(module, deadline);

disconnect:
    if (req_buf != NULL) {
        free(req_buf);
    }
    (void)http_client_close(module);

    memset(&cbd, 0, sizeof(cbd));
    cbd.disconnected.reason = rc;
    http_client_dispatch(module, HTTP_CLIENT_CALLBACK_DISCONNECTED, &cbd);

    return rc;
}
