/*
 * http_client.h
 *
 * Minimal blocking HTTP/1.1 client for the WFI32 (PIC32MZW1) Harmony
 * TCP/IP + wolfSSL stack.
 *
 * This is the WFI32 counterpart of the ASF WINC1500 "http_client" service
 * used by the PIC32CM LS60 demo.  The public API deliberately mirrors that
 * one (config / module / callback structures, http_client_send_request(),
 * http_client_close(), ...) so that the layers above it -- iotc_https.c and
 * iotconnect.c -- stay nearly identical between the two ports.
 *
 * Differences from the ASF version
 * --------------------------------
 * - The ASF version is event driven and needs socket / DNS events routed to
 *   it from the application.  Here the Harmony NET_PRES socket API is used
 *   from a FreeRTOS task, so http_client_send_request() simply BLOCKS (with
 *   vTaskDelay() based polling) until the whole transaction is done.  There
 *   is therefore no http_client_socket_event_handler() / _resolve_handler().
 * - The sw_timer module is not used; timeouts are tracked with the FreeRTOS
 *   tick counter.
 * - Request bodies are passed as a plain buffer instead of the ASF
 *   "http_entity" callback interface (http_entity.h is not needed).
 *
 * Callback sequence for one successful request
 * --------------------------------------------
 *   HTTP_CLIENT_CALLBACK_SOCK_CONNECTED
 *   HTTP_CLIENT_CALLBACK_REQUESTED
 *   HTTP_CLIENT_CALLBACK_RECV_RESPONSE          (status code + headers)
 *   HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA * N  (only if is_chunked was set)
 *   HTTP_CLIENT_CALLBACK_DISCONNECTED
 *
 * If the complete body was already buffered when the headers were parsed,
 * RECV_RESPONSE carries it in .content and .is_chunked is 0 -- no
 * RECV_CHUNKED_DATA callbacks follow.  Otherwise .is_chunked is 1, .content
 * is NULL, and the body is delivered piecewise through RECV_CHUNKED_DATA
 * until one of them has .is_complete set.  This is true both for HTTP
 * "Transfer-Encoding: chunked" responses and for large Content-Length
 * responses; the chunk framing itself is decoded here.
 *
 * TLS notes
 * ---------
 * TLS is provided by NET_PRES / wolfSSL.  Two things must be in place:
 *   1. The root CA that signs the server certificate must be present in the
 *      net_pres certificate store (net_pres_cert_store.c).
 *   2. SNI: the Harmony encryption glue (net_pres_enc_glue.c) reads the
 *      server name from the application global g_Cloud_Endpoint[] when the
 *      TLS session is opened.  This module therefore temporarily points
 *      g_Cloud_Endpoint at the host being requested and restores the
 *      previous value when the request finishes.  Set
 *      HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT to 0 if that glue is ever changed
 *      to take the host from somewhere else.
 */

#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "configuration.h"
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol version string sent in the request line. */
#define HTTP_PROTO_NAME                 "HTTP/1.1"

/** Maximum length of the request URI (path + query). */
#ifndef HTTP_MAX_URI_LENGTH
#define HTTP_MAX_URI_LENGTH             (512)
#endif

/** Maximum length of the host name of a request. */
#ifndef HTTP_MAX_HOST_LENGTH
#define HTTP_MAX_HOST_LENGTH            (128)
#endif

/** Default User-Agent header value. */
#ifndef HTTP_CLIENT_USER_AGENT
#define HTTP_CLIENT_USER_AGENT          "MCHP-WFI32/1.0"
#endif

/** Set to 0 if net_pres_enc_glue.c no longer takes SNI from g_Cloud_Endpoint. */
#ifndef HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT
#define HTTP_CLIENT_SNI_VIA_CLOUD_ENDPOINT  (1)
#endif

/**
 * \brief HTTP method.
 */
enum http_method {
    HTTP_METHOD_GET = 1,
    HTTP_METHOD_POST,
    HTTP_METHOD_DELETE,
    HTTP_METHOD_PUT,
    HTTP_METHOD_OPTIONS,
    HTTP_METHOD_HEAD,
};

/**
 * \brief HTTP client callback type.
 */
enum http_client_callback_type {
    /** The TCP (and TLS, if enabled) session to the server is up. */
    HTTP_CLIENT_CALLBACK_SOCK_CONNECTED,
    /** The request line, headers and body have been sent. */
    HTTP_CLIENT_CALLBACK_REQUESTED,
    /** The response status line and headers have been received. */
    HTTP_CLIENT_CALLBACK_RECV_RESPONSE,
    /** A piece of the response body. */
    HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA,
    /** The session was closed. */
    HTTP_CLIENT_CALLBACK_DISCONNECTED,
};

/** Data of the HTTP_CLIENT_CALLBACK_SOCK_CONNECTED callback. */
struct http_client_data_sock_connected {
    /** 0 on success, negative errno code on failure. */
    int result;
};

/** Data of the HTTP_CLIENT_CALLBACK_REQUESTED callback. */
struct http_client_data_requested {
    /** Number of body bytes sent. */
    int sent_length;
};

/** Data of the HTTP_CLIENT_CALLBACK_RECV_RESPONSE callback. */
struct http_client_data_recv_response {
    /** HTTP status code, e.g. 200. */
    uint16_t response_code;
    /** 0: .content holds the complete body.  1: body follows through
     *  HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA callbacks. */
    uint8_t  is_chunked;
    /** Body length.  0 when unknown (chunked / connection-close framing). */
    uint32_t content_length;
    /** Body buffer, valid only for the duration of the callback.  NULL when
     *  .is_chunked is set. */
    char    *content;
};

/** Data of the HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA callback. */
struct http_client_data_recv_chunked_data {
    /** Length of this piece of the body.  May be 0 on the final callback. */
    uint32_t length;
    /** Body bytes, valid only for the duration of the callback. */
    char    *data;
    /** Non-zero on the last piece of the body. */
    char     is_complete;
};

/** Data of the HTTP_CLIENT_CALLBACK_DISCONNECTED callback. */
struct http_client_data_disconnected {
    /** 0 for a normal close, negative errno code otherwise. */
    int reason;
};

/** Callback payload. */
union http_client_data {
    struct http_client_data_sock_connected     sock_connected;
    struct http_client_data_requested          requested;
    struct http_client_data_recv_response      recv_response;
    struct http_client_data_recv_chunked_data  recv_chunked_data;
    struct http_client_data_disconnected       disconnected;
};

struct http_client_module;

/**
 * \brief HTTP client callback.
 *
 * \param[in] module_inst  Module instance.
 * \param[in] type         One of enum http_client_callback_type.
 * \param[in] data         Event payload.
 */
typedef void (*http_client_callback_t)(struct http_client_module *module_inst,
                                       int type, union http_client_data *data);

/**
 * \brief HTTP client configuration.
 *
 * Initialise with http_client_get_config_defaults() before modifying.
 */
struct http_client_config {
    /** Server TCP port.  Overridden by the port in the URL, if present.
     *  Default 80 (or 443 when .tls is set by the URL scheme). */
    uint16_t port;
    /** Non-zero to use TLS.  Set automatically from an "https://" URL. */
    uint8_t  tls;
    /** Overall per-request timeout in milliseconds.  Default 30000. */
    uint32_t timeout;
    /** Receive/work buffer supplied by the caller.  Required. */
    char    *recv_buffer;
    /** Size of .recv_buffer.  Must hold the largest response header block. */
    uint32_t recv_buffer_size;
    /** Size of the heap buffer used to build the request.  Default 1024. */
    uint32_t send_buffer_size;
    /** User-Agent header value.  Must stay valid while the module is used. */
    const char *user_agent;
};

/** State of the request being processed. */
struct http_client_req {
    /** URI (path + query) of the request. */
    char             uri[HTTP_MAX_URI_LENGTH];
    /** Method of the request. */
    enum http_method method;
    /** Request body, or NULL. */
    const void      *body;
    /** Length of the request body. */
    uint32_t         content_length;
    /** Number of body bytes actually sent. */
    uint32_t         sent_length;
};

/** State of the response being processed. */
struct http_client_resp {
    /** HTTP status code. */
    uint16_t response_code;
    /** Value of the Content-Length header, 0 when absent. */
    uint32_t content_length;
    /** Number of body bytes delivered to the callback. */
    uint32_t read_length;
    /** Non-zero when the response used Transfer-Encoding: chunked. */
    uint8_t  is_chunked;
};

/** HTTP client instance. */
struct http_client_module {
    /** NET_PRES socket of the session, or -1 when closed. */
    NET_PRES_SKT_HANDLE_T socket;
    /** Host name of the current session. */
    char      host[HTTP_MAX_HOST_LENGTH];
    /** Port of the current session. */
    uint16_t  port;
    /** Non-zero when the current session is TLS. */
    uint8_t   tls;
    /** Non-zero while a socket is open. */
    uint8_t   connected;
    /** Registered callback, or NULL. */
    http_client_callback_t     cb;
    /** Configuration passed to http_client_init(). */
    struct http_client_config  config;
    /** Current request. */
    struct http_client_req     req;
    /** Current response. */
    struct http_client_resp    resp;
};

/**
 * \brief Fill \p config with the default values.
 */
void http_client_get_config_defaults(struct http_client_config *const config);

/**
 * \brief Initialise an HTTP client instance.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_init(struct http_client_module *const module,
                     struct http_client_config *config);

/**
 * \brief Release an HTTP client instance and close any open session.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_deinit(struct http_client_module *const module);

/**
 * \brief Register the event callback.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_register_callback(struct http_client_module *const module,
                                  http_client_callback_t callback);

/**
 * \brief Unregister the event callback.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_unregister_callback(struct http_client_module *const module);

/**
 * \brief Perform one complete HTTP transaction.  Blocks until done.
 *
 * \param[in] module      Module instance.
 * \param[in] url         Absolute URL, "http://host[:port]/path" or
 *                        "https://host[:port]/path".
 * \param[in] method      Request method.
 * \param[in] body        Request body for POST/PUT, or NULL.
 * \param[in] body_len    Length of \p body, 0 when \p body is NULL.
 * \param[in] ext_header  Extra headers, each terminated with "\r\n", or NULL.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_send_request(struct http_client_module *const module,
                             const char *url, enum http_method method,
                             const void *body, uint32_t body_len,
                             const char *ext_header);

/**
 * \brief Close the session if one is open.  Safe to call at any time.
 *
 * \return 0 on success, negative errno code on failure.
 */
int http_client_close(struct http_client_module *const module);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_CLIENT_H */
