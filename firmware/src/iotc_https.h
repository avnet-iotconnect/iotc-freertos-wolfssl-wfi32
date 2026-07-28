/*
 * iotc_https.h
 *
 * Blocking HTTP/HTTPS wrapper for IoTConnect on the WFI32 (PIC32MZW1).
 *
 * This is the WFI32 counterpart of the PIC32CM LS60 / WINC1500 module of the
 * same name.  The API is identical, so iotconnect.c is portable between the
 * two; the transport underneath is the Harmony NET_PRES + wolfSSL based
 * http_client of this project (see http_client.h) instead of the ASF WINC1500
 * service.
 *
 * Because the Harmony socket API is used from a FreeRTOS task, the request is
 * genuinely blocking: there is no event pump to drive and none of the WINC
 * socket / DNS event routing helpers of the original port exist here.
 *
 * Integration
 * -----------
 * 1. Call iotc_http_client_init() once, after the Wi-Fi link is up and an IP
 *    address has been obtained.
 * 2. Call http_client_request() from a task context (never from an ISR or a
 *    Harmony callback) -- it blocks for the duration of the transaction.
 * 3. Free every response with http_client_response_free().
 *
 * TLS requirements
 * ----------------
 * - The root CA that signs the IoTConnect discovery / identity endpoints must
 *   be present in the net_pres certificate store (net_pres_cert_store.c).
 * - SNI is taken from the application global g_Cloud_Endpoint[]; http_client.c
 *   sets and restores it around each request.  See http_client.h.
 */

#ifndef IOTC_HTTPS_H
#define IOTC_HTTPS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------- */
/* HTTP method / opcode type                                               */
/* ---------------------------------------------------------------------- */

/**
 * HTTP operation codes used with http_client_request().
 * Maps directly onto enum http_method from http_client.h.
 */
typedef enum {
    IOTC_HTTP_OPCODE_GET    = 0,
    IOTC_HTTP_OPCODE_POST   = 1,
    IOTC_HTTP_OPCODE_PUT    = 2,
    IOTC_HTTP_OPCODE_DELETE = 3,
} iotc_http_opcode_t;

/* ---------------------------------------------------------------------- */
/* Initialization                                                          */
/* ---------------------------------------------------------------------- */

/**
 * Initialize the HTTP client module.
 *
 * Must be called once before any call to http_client_request().  Owns the
 * receive buffer and the http_client instance; the caller provides nothing.
 * Calling it more than once is harmless.
 *
 * @return  0 on success, negative errno code on failure.
 */
int iotc_http_client_init(void);

/**
 * Release the resources taken by iotc_http_client_init() and close any
 * connection that is still open.
 */
void iotc_http_client_deinit(void);

/* ---------------------------------------------------------------------- */
/* Request / response API                                                  */
/* ---------------------------------------------------------------------- */

/**
 * Perform a single blocking HTTP or HTTPS request.
 *
 * The call blocks until the response body is received, an error occurs, or
 * the timeout expires.  Only one request may be in flight at a time; the
 * module is not re-entrant, so serialise calls at the call site.
 *
 * @param url          Full URL beginning with "http://" or "https://".
 * @param op_code      IOTC_HTTP_OPCODE_GET, POST, PUT, or DELETE.
 * @param body         Request body for POST/PUT requests.  NULL for GET/DELETE.
 * @param body_len     Length of @p body in bytes.  0 for GET/DELETE.
 * @param resp_out     On success, receives a heap-allocated NUL-terminated
 *                     string containing the response body.  The caller must
 *                     free it with http_client_response_free().
 *                     Pass NULL to discard the response body.
 * @param resp_len_out On success, receives the response body length (not
 *                     including the NUL terminator).  May be NULL.
 *
 * @return  0 on success, negative errno code on failure.
 */
int http_client_request(
    const char        *url,
    iotc_http_opcode_t op_code,
    const void        *body,
    size_t             body_len,
    char             **resp_out,
    size_t            *resp_len_out
);

/**
 * Free a response buffer previously returned by http_client_request().
 *
 * @param resp  Buffer to free.  NULL is safe to pass.
 */
void http_client_response_free(char *resp);

#ifdef __cplusplus
}
#endif

#endif /* IOTC_HTTPS_H */
