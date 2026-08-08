/*
 * iotconnect.h
 *
 * IoTConnect SDK integration for the WFI32 (PIC32MZW1) FreeRTOS / wolfSSL
 * AWS IoT Core demo.
 *
 * This is the WFI32 port of the PIC32CM LS60 / WINC1500 module of the same
 * name.  The flow (iotcl init -> HTTPS discovery + identity -> MQTT connect ->
 * telemetry) is unchanged; the transports differ:
 *
 * Transport layer
 * ---------------
 * - Discovery / Identity : HTTPS GET through iotc_https (Harmony NET_PRES +
 *                          wolfSSL, see iotc_https.h).
 * - Telemetry / C2D      : MQTT over TLS through the AWS IoT Device SDK
 *                          (IotMqtt) and this project's wolfSSL network
 *                          interface, IOT_NETWORK_INTERFACE_WOLFSSL.
 *
 * Prerequisites
 * -------------
 * - Wi-Fi is connected, an IP address has been obtained, and the clock has
 *   been set from NTP (TLS certificate validity checks depend on it).
 * - IotSdk_Init() and IotMqtt_Init() have already been called; APP_AWS_Tasks()
 *   does this in its APP_AWS_CLOUD_SDK_INIT state.
 * - iotc_http_client_init() has been called.
 *
 * All functions here BLOCK and must be called from a task context.
 */

#ifndef IOTCONNECT_H
#define IOTCONNECT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "iotcl.h"
#include "iot_mqtt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------- */
/* Configuration types                                                     */
/* ---------------------------------------------------------------------- */

typedef enum {
    IOTC_CT_AWS   = 0,
    IOTC_CT_AZURE = 1,
} IotConnectConnectionType;

/**
 * Callback invoked for every inbound MQTT message on the C2D subscription.
 * Matches the signature of IotMqttCallbackInfo_t::function.
 */
typedef void (*IotConnectMqttCallback_t)(void *pContext,
                                         IotMqttCallbackParam_t *pPublish);

/**
 * Callback that fills a telemetry message with the values to publish.
 * Called by iotc_publish_telemetry() on a message created with
 * iotcl_telemetry_create(); add values with iotcl_telemetry_set_*().  Do not
 * send or destroy the message -- iotc_publish_telemetry() does both.
 */
typedef void (*IotConnectTelemetryCallback_t)(IotclMessageHandle msg);

/** Pass as telemetry_count to publish until the connection drops. */
#define IOTC_TELEMETRY_UNLIMITED    (0xFFFFFFFFU)

/**
 * Parameters passed to iotconnect_sdk().
 *
 * event_callback  Called for every inbound MQTT message on the C2D
 *                 subscription topic.  May be NULL if C2D is not needed, in
 *                 which case no subscription is made.  A typical
 *                 implementation just forwards the payload to
 *                 iotcl_mqtt_receive_c2d().
 *
 * cmd_cb            Optional iotcl command callback (see iotcl_c2d.h).
 * telemetry_cb      Fills each telemetry message.  NULL -> a minimal default
 *                   payload is sent instead.
 * telemetry_count   Number of telemetry messages to publish before returning.
 *                   0 -> use the compiled-in default (IOTC_SDK_PUBLISH_COUNT).
 *                   IOTC_TELEMETRY_UNLIMITED -> until the connection drops.
 * telemetry_delay_ms  Delay between successive publishes in ms.
 *                     0 -> use the compiled-in default (IOTC_SDK_PUBLISH_DELAY_MS).
 */
typedef struct {
    const char                   *env;
    const char                   *cpid;
    const char                   *duid;
    IotConnectConnectionType      connection_type;
    IotConnectMqttCallback_t      event_callback;
    IotclCommandCallback          cmd_cb;
    IotConnectTelemetryCallback_t telemetry_cb;
    uint32_t                      telemetry_count;
    uint32_t                      telemetry_delay_ms;
} IotConnectClientConfig;

/* ---------------------------------------------------------------------- */
/* High-level entry point                                                  */
/* ---------------------------------------------------------------------- */

/**
 * Run the full IoTConnect flow:
 *   1. Initialise the iotcl library.
 *   2. Discovery + Identity over HTTPS.
 *   3. MQTT connect to the resolved broker.
 *   4. Publish @p c->telemetry_count telemetry messages.
 *
 * @param c  Configuration; env, cpid, duid and connection_type are required.
 * @return   0 on success, negative errno on failure.
 */
int iotconnect_sdk(IotConnectClientConfig *c);

/* ---------------------------------------------------------------------- */
/* Discrete step functions (usable independently)                         */
/* ---------------------------------------------------------------------- */

/**
 * Run IoTConnect DRA Discovery and Identity over HTTPS and configure the
 * iotcl library MQTT settings (broker host, topics, client_id).
 *
 * Requires iotcl_init() and iotc_http_client_init() to have been called first.
 *
 * @return 0 on success, non-zero on failure.
 */
int run_http_identity(const char *duid, const char *cpid, const char *env);

/**
 * Connect to the IoTConnect MQTT broker using the credentials obtained by
 * run_http_identity().
 *
 * @param event_callback  Callback for C2D messages.  NULL to skip the
 *                        subscription.
 * @return 0 on success, negative errno on failure.
 */
int iotc_mqtt_connect(IotConnectMqttCallback_t event_callback);

/**
 * Disconnect from the broker and release the MQTT connection.
 */
void iotc_mqtt_disconnect(void);

/**
 * @return true while the MQTT connection is up.
 */
bool iotc_mqtt_is_connected(void);

/**
 * Build a telemetry payload with iotcl and publish it to the reporting topic.
 *
 * The payload is filled by the callback given to iotc_set_telemetry_callback()
 * or in IotConnectClientConfig::telemetry_cb.
 *
 * @return 0 on success, negative errno on failure.
 */
int iotc_publish_telemetry(void);

/**
 * Set the callback that fills each telemetry message.  iotconnect_sdk() does
 * this from IotConnectClientConfig::telemetry_cb; call it directly when
 * driving the discrete step functions yourself.
 *
 * @param cb  Callback, or NULL to fall back to the default payload.
 */
void iotc_set_telemetry_callback(IotConnectTelemetryCallback_t cb);

/**
 * Cut short the wait between two publishes of the iotconnect_sdk() telemetry
 * loop, so that the next message is sent immediately.  Use it when something
 * the device reports has just changed (a button press, an LED command).
 *
 * Safe to call from any task; does nothing when the loop is not running.
 * Not safe to call from an ISR.
 */
void iotc_request_publish(void);

/**
 * The MQTT connection used by this module, so the application can publish or
 * subscribe on it directly.  NULL when not connected.
 */
IotMqttConnection_t iotc_mqtt_get_connection(void);

#ifdef __cplusplus
}
#endif

#endif /* IOTCONNECT_H */
