/*
 * iotconnect.c
 *
 * IoTConnect SDK integration for the WFI32 (PIC32MZW1) FreeRTOS / wolfSSL
 * AWS IoT Core demo.
 *
 * This file is the Harmony / AWS IoT Device SDK port of the PIC32CM LS60
 * WINC1500 / Paho implementation of the same name.
 *
 * Architecture
 * ------------
 *  HTTP (discovery/identity)
 *      iotc_https, which uses the NET_PRES + wolfSSL http_client of this
 *      project -- see iotc_https.h.
 *
 *  MQTT (telemetry / C2D)
 *      The AWS IoT Device SDK MQTT library (IotMqtt) over
 *      IOT_NETWORK_INTERFACE_WOLFSSL, exactly like app_aws.c.  Device
 *      authentication is X.509 mTLS with the key/certificate in the ECC608
 *      (the net_pres wolfSSL glue installs the ATECC callbacks), so no MQTT
 *      username or password is used.
 *
 *  Blocking
 *      Everything here runs in a FreeRTOS task context and blocks; there is
 *      no event pump to service as there was on the WINC1500 port.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "definitions.h"
#include "FreeRTOS.h"
#include "task.h"

/* AWS IoT Device SDK */
#include "iot_platform_types_pic32mzw1.h"
#include "iot_mqtt.h"
#include "iot_network_wolfssl.h"

/* iotc-c-lib */
#include "iotcl.h"
#include "iotcl_telemetry.h"
#include "iotcl_dra_discovery.h"
#include "iotcl_dra_identity.h"

/* Project */
#include "iotc_config.h"
#include "iotc_https.h"
#include "iotconnect.h"

/* ======================================================================
 * Build-time tunables
 * ====================================================================== */

/** MQTT broker port.  AWS IoT Core uses 8883 (mTLS). */
#ifndef IOTC_MQTT_BROKER_PORT
#define IOTC_MQTT_BROKER_PORT           (8883U)
#endif

/** Keep-alive interval sent in the MQTT CONNECT packet (seconds). */
#ifndef IOTC_MQTT_KEEP_ALIVE_S
#define IOTC_MQTT_KEEP_ALIVE_S          (60U)
#endif

/** MQTT CONNACK / SUBACK / PUBACK timeout (ms). */
#ifndef IOTC_MQTT_TIMEOUT_MS
#define IOTC_MQTT_TIMEOUT_MS            (5000U)
#endif

/** Default number of telemetry publishes when the config value is 0. */
#ifndef IOTC_SDK_PUBLISH_COUNT
#define IOTC_SDK_PUBLISH_COUNT          (100U)
#endif

/** Default delay between publishes (ms) when the config value is 0. */
#ifndef IOTC_SDK_PUBLISH_DELAY_MS
#define IOTC_SDK_PUBLISH_DELAY_MS       (2000U)
#endif

/** Discovery hosts. */
#ifndef IOTC_DISCOVERY_SERVER
#define IOTC_DISCOVERY_SERVER           IOTCL_DRA_DEFAULT_DISCOVERY_HOST_AWS
#endif
#ifndef IOTC_DISCOVERY_SERVER_PROD
#define IOTC_DISCOVERY_SERVER_PROD      "discoveryconsole.iotconnect.io"
#endif

#define IOTC_PRNT(fmt, ...)  SYS_CONSOLE_PRINT("[IOTC] " fmt, ##__VA_ARGS__)
#define IOTC_DBG(level, fmt, ...) \
    SYS_DEBUG_PRINT(level, "[IOTC] " fmt, ##__VA_ARGS__)

/* The net_pres wolfSSL glue takes the TLS SNI host name from this global, so
 * it has to hold the broker host name before IotMqtt_Connect() opens the
 * session.  Defined in app_aws.c. */
extern char g_Cloud_Endpoint[100];

/* ======================================================================
 * Module-private state
 * ====================================================================== */

static IotMqttConnection_t s_mqtt_connection = IOT_MQTT_CONNECTION_INITIALIZER;
static bool                s_mqtt_connected  = false;

/** Fills each telemetry message; NULL -> the default payload below. */
static IotConnectTelemetryCallback_t s_telemetry_cb = NULL;

/** Task running the iotconnect_sdk() telemetry loop, so that
 *  iotc_request_publish() can wake it early.  NULL when the loop is idle. */
static TaskHandle_t s_telemetry_task = NULL;

/* ======================================================================
 * Internal helpers
 * ====================================================================== */

/** MQTT send callback registered with iotcl.
 *  Called by iotcl_mqtt_send_telemetry() and the ack helpers to publish a
 *  JSON string on the given topic. */
static void iotc_mqtt_send_cb(const char *topic, const char *json_str)
{
    IotMqttPublishInfo_t publishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    IotMqttError_t       status;

    if (!s_mqtt_connected) {
        IOTC_DBG(SYS_ERROR_ERROR, "mqtt_send_cb: not connected\r\n");
        return;
    }
    if (topic == NULL || json_str == NULL) {
        return;
    }

    publishInfo.qos             = IOT_MQTT_QOS_1;
    publishInfo.pTopicName      = topic;
    publishInfo.topicNameLength = (uint16_t)strlen(topic);
    publishInfo.pPayload        = json_str;
    publishInfo.payloadLength   = strlen(json_str);
    publishInfo.retryMs         = PUBLISH_RETRY_MS;
    publishInfo.retryLimit      = PUBLISH_RETRY_LIMIT;

    status = IotMqtt_PublishSync(s_mqtt_connection, &publishInfo, 0,
                                 IOTC_MQTT_TIMEOUT_MS);
    if (status == IOT_MQTT_SUCCESS) {
        IOTC_PRNT("published -> %s : %s\r\n", topic, json_str);
    } else {
        IOTC_DBG(SYS_ERROR_ERROR, "publish to %s failed (%s)\r\n",
                 topic, IotMqtt_strerror(status));
    }
}

/** MQTT disconnect notification from the SDK. */
static void iotc_mqtt_disconnect_cb(void *param1,
                                    IotMqttCallbackParam_t *const pOperation)
{
    (void)param1;
    (void)pOperation;

    IOTC_DBG(SYS_ERROR_ERROR, "MQTT connection terminated\r\n");
    s_mqtt_connected = false;
}

/** Initialise the iotcl library with the given device parameters. */
static int iotc_init_with_config(const char *duid, const char *cpid,
                                 IotclCommandCallback cmd_cb)
{
    IotclClientConfig cfg;
    int rc;

    if (duid == NULL || cpid == NULL) {
        return IOTCL_ERR_MISSING_VALUE;
    }

    iotcl_init_client_config(&cfg);
    cfg.device.duid          = duid;
    cfg.device.cpid          = cpid;
    /* The host and the topics come from the identity response. */
    cfg.device.instance_type = IOTCL_DCT_CUSTOM;
    cfg.mqtt_send_cb         = iotc_mqtt_send_cb;
    cfg.events.cmd_cb        = cmd_cb;

    rc = iotcl_init(&cfg);
    if (rc != 0) {
        IOTC_DBG(SYS_ERROR_ERROR, "iotcl_init failed: %d\r\n", rc);
    }
    return rc;
}

/** Release all iotcl and MQTT resources. */
static void iotc_sdk_deinit(void)
{
    s_telemetry_task = NULL;
    iotc_mqtt_disconnect();
    iotcl_deinit();
}

/* ======================================================================
 * Public API
 * ====================================================================== */

/* ---------------------------------------------------------------------- */
int run_http_identity(const char *duid, const char *cpid, const char *env)
{
    IotclDraUrlContext discovery_url = { 0 };
    IotclDraUrlContext identity_url  = { 0 };
    char   *response     = NULL;
    size_t  response_len = 0U;
    int     status       = 0;

    if (duid == NULL || cpid == NULL || env == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "run_http_identity: NULL argument\r\n");
        return -EINVAL;
    }

    /* ---- Discovery ---- */
    /* iotcl_dra_discovery_init_url_with_host() takes a non-const host. */
    if (strcmp(env, "prod") == 0) {
        status = iotcl_dra_discovery_init_url_with_host(&discovery_url,
                                                        (char *)IOTC_DISCOVERY_SERVER_PROD,
                                                        cpid, env);
    } else {
        status = iotcl_dra_discovery_init_url_with_host(&discovery_url,
                                                        (char *)IOTC_DISCOVERY_SERVER,
                                                        cpid, env);
    }

    if (status != 0) {
        IOTC_DBG(SYS_ERROR_ERROR, "discovery URL build failed\r\n");
        goto cleanup;
    }

    IOTC_PRNT("discovery URL: %s\r\n", iotcl_dra_url_get_url(&discovery_url));

    status = http_client_request(iotcl_dra_url_get_url(&discovery_url),
                                 IOTC_HTTP_OPCODE_GET,
                                 NULL, 0U,
                                 &response, &response_len);
    if (status != 0 || response == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "discovery HTTP request failed (%d)\r\n", status);
        if (status == 0) { status = -EIO; }
        goto cleanup;
    }

    /* Cap the printed preview; the console print buffer is not sized for a
     * full multi-kilobyte response body. */
    IOTC_PRNT("discovery response (%u bytes): %.300s%s\r\n",
              (unsigned)response_len, response,
              (response_len > 300U) ? "..." : "");

    status = iotcl_dra_discovery_parse(&identity_url, 0, response);
    if (status != 0) {
        IOTC_DBG(SYS_ERROR_ERROR, "discovery parse failed\r\n");
        goto cleanup;
    }

    http_client_response_free(response);
    response     = NULL;
    response_len = 0U;

    /* ---- Identity ---- */
    status = iotcl_dra_identity_build_url(&identity_url, duid);
    if (status != 0) {
        goto cleanup;
    }

    IOTC_PRNT("identity URL: %s\r\n", iotcl_dra_url_get_url(&identity_url));

    status = http_client_request(iotcl_dra_url_get_url(&identity_url),
                                 IOTC_HTTP_OPCODE_GET,
                                 NULL, 0U,
                                 &response, &response_len);
    if (status != 0 || response == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "identity HTTP request failed (%d)\r\n", status);
        if (status == 0) { status = -EIO; }
        goto cleanup;
    }

    IOTC_PRNT("identity response (%u bytes): %.300s%s\r\n",
              (unsigned)response_len, response,
              (response_len > 300U) ? "..." : "");

    status = iotcl_dra_identity_configure_library_mqtt(response);
    if (status != 0) {
        IOTC_DBG(SYS_ERROR_ERROR, "identity parse failed\r\n");
        goto cleanup;
    }

    {
        IotclMqttConfig *mc = iotcl_mqtt_get_config();

        if (mc != NULL) {
            IOTC_PRNT("broker   : %s\r\n", (mc->host      != NULL) ? mc->host      : "(null)");
            IOTC_PRNT("pub_rpt  : %s\r\n", (mc->pub_rpt   != NULL) ? mc->pub_rpt   : "(null)");
            IOTC_PRNT("sub_c2d  : %s\r\n", (mc->sub_c2d   != NULL) ? mc->sub_c2d   : "(null)");
            IOTC_PRNT("client_id: %s\r\n", (mc->client_id != NULL) ? mc->client_id : "(null)");

            /* AWS IoT Core uses X.509 mTLS -- no MQTT username needed. */
            if (mc->username != NULL) {
                iotcl_free(mc->username);
                mc->username = NULL;
            }
        }
    }

cleanup:
    if (response != NULL) {
        http_client_response_free(response);
    }
    iotcl_dra_url_deinit(&discovery_url);
    iotcl_dra_url_deinit(&identity_url);
    return status;
}

/* ---------------------------------------------------------------------- */
int iotc_mqtt_connect(IotConnectMqttCallback_t event_callback)
{
    IotclMqttConfig       *mc = iotcl_mqtt_get_config();
    IotMqttNetworkInfo_t   networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
    IotMqttConnectInfo_t   connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;
    struct IotNetworkServerInfo serverInfo = { 0 };
    IotMqttError_t         status;

    if (mc == NULL || mc->host == NULL || mc->client_id == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "iotcl MQTT config not available -- run "
                 "run_http_identity() first\r\n");
        return -EINVAL;
    }

    IOTC_PRNT("connecting to broker %s:%u (clientId=%s)\r\n",
              mc->host, (unsigned)IOTC_MQTT_BROKER_PORT, mc->client_id);

    /* Clean up any previous connection. */
    iotc_mqtt_disconnect();

    /* net_pres_enc_glue.c reads the SNI host name from this global when the
     * TLS session is opened, so it has to name the broker. */
    snprintf(g_Cloud_Endpoint, sizeof(g_Cloud_Endpoint), "%s", mc->host);

    serverInfo.pHostName = mc->host;
    serverInfo.port      = IOTC_MQTT_BROKER_PORT;

    networkInfo.createNetworkConnection        = true;
    networkInfo.u.setup.pNetworkServerInfo     = &serverInfo;
    networkInfo.u.setup.pNetworkCredentialInfo = NULL;
    networkInfo.pNetworkInterface              = IOT_NETWORK_INTERFACE_WOLFSSL;
    networkInfo.disconnectCallback.function        = iotc_mqtt_disconnect_cb;
    networkInfo.disconnectCallback.pCallbackContext = NULL;

    connectInfo.awsIotMqttMode        = true;
    connectInfo.cleanSession          = true;
    connectInfo.keepAliveSeconds      = IOTC_MQTT_KEEP_ALIVE_S;
    connectInfo.pClientIdentifier     = mc->client_id;
    connectInfo.clientIdentifierLength = (uint16_t)strlen(mc->client_id);
    /* X.509 mTLS: no username or password. */
    connectInfo.pUserName             = NULL;
    connectInfo.userNameLength        = 0;
    connectInfo.pPassword             = NULL;
    connectInfo.passwordLength        = 0;

    status = IotMqtt_Connect(&networkInfo, &connectInfo, IOTC_MQTT_TIMEOUT_MS,
                             &s_mqtt_connection);
    if (status != IOT_MQTT_SUCCESS) {
        IOTC_DBG(SYS_ERROR_ERROR, "MQTT connect failed (%s)\r\n",
                 IotMqtt_strerror(status));
        s_mqtt_connection = IOT_MQTT_CONNECTION_INITIALIZER;
        return -ECONNREFUSED;
    }

    s_mqtt_connected = true;
    IOTC_PRNT("MQTT connected\r\n");

    /* ---- Subscribe to the C2D topic ---- */
    if (event_callback != NULL && mc->sub_c2d != NULL) {
        IotMqttSubscription_t subscription = IOT_MQTT_SUBSCRIPTION_INITIALIZER;

        subscription.qos                      = IOT_MQTT_QOS_1;
        subscription.pTopicFilter             = mc->sub_c2d;
        subscription.topicFilterLength        = (uint16_t)strlen(mc->sub_c2d);
        subscription.callback.function        = event_callback;
        subscription.callback.pCallbackContext = NULL;

        status = IotMqtt_SubscribeSync(s_mqtt_connection, &subscription, 1, 0,
                                       IOTC_MQTT_TIMEOUT_MS);
        if (status != IOT_MQTT_SUCCESS) {
            IOTC_DBG(SYS_ERROR_ERROR, "C2D subscribe failed (%s), continuing\r\n",
                     IotMqtt_strerror(status));
        } else {
            IOTC_PRNT("subscribed to %s\r\n", mc->sub_c2d);
        }
    }

    return 0;
}

/* ---------------------------------------------------------------------- */
void iotc_mqtt_disconnect(void)
{
    if (s_mqtt_connection != IOT_MQTT_CONNECTION_INITIALIZER) {
        IotMqtt_Disconnect(s_mqtt_connection, s_mqtt_connected ? 0 : IOT_MQTT_FLAG_CLEANUP_ONLY);
        s_mqtt_connection = IOT_MQTT_CONNECTION_INITIALIZER;
    }
    s_mqtt_connected = false;
}

/* ---------------------------------------------------------------------- */
bool iotc_mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

/* ---------------------------------------------------------------------- */
IotMqttConnection_t iotc_mqtt_get_connection(void)
{
    return s_mqtt_connection;
}

/* ---------------------------------------------------------------------- */
void iotc_set_telemetry_callback(IotConnectTelemetryCallback_t cb)
{
    s_telemetry_cb = cb;
}

/* ---------------------------------------------------------------------- */
void iotc_request_publish(void)
{
    TaskHandle_t task = s_telemetry_task;

    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

/* ---------------------------------------------------------------------- */
int iotc_publish_telemetry(void)
{
    IotclMessageHandle msg;
    int rc;

    if (!s_mqtt_connected) {
        IOTC_DBG(SYS_ERROR_ERROR, "publish_telemetry: not connected\r\n");
        return -ENOTCONN;
    }

    msg = iotcl_telemetry_create();
    if (msg == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "iotcl_telemetry_create failed\r\n");
        return -ENOMEM;
    }

    if (s_telemetry_cb != NULL) {
        s_telemetry_cb(msg);
    } else {
        /* No callback configured: send a minimal payload so the device still
         * shows up as reporting.  See app_iotconnect.c for the sensor set. */
        iotcl_telemetry_set_string(msg, "version", "1.0.0");
        iotcl_telemetry_set_number(msg, "random", (double)(rand() % 100));
    }

    /* iotcl_mqtt_send_telemetry() invokes iotc_mqtt_send_cb() above, which
     * publishes on the reporting topic. */
    rc = iotcl_mqtt_send_telemetry(msg, false);
    iotcl_telemetry_destroy(msg);

    return (rc == 0) ? 0 : -EIO;
}

/* ---------------------------------------------------------------------- */
int iotconnect_sdk(IotConnectClientConfig *c)
{
    uint32_t count;
    uint32_t delay_ms;
    uint32_t i;
    int      status;

    if (c == NULL || c->env == NULL || c->cpid == NULL || c->duid == NULL) {
        IOTC_DBG(SYS_ERROR_ERROR, "invalid configuration -- env/cpid/duid "
                 "are required\r\n");
        return IOTCL_ERR_MISSING_VALUE;
    }

    if (c->connection_type != IOTC_CT_AWS && c->connection_type != IOTC_CT_AZURE) {
        IOTC_DBG(SYS_ERROR_ERROR, "invalid connection_type\r\n");
        return IOTCL_ERR_MISSING_VALUE;
    }

    iotc_sdk_deinit();
    iotc_set_telemetry_callback(c->telemetry_cb);

    /* ---- 1. Init the iotcl library ---- */
    status = iotc_init_with_config(c->duid, c->cpid, c->cmd_cb);
    if (status != IOTCL_SUCCESS) {
        iotc_sdk_deinit();
        return status;
    }

    /* ---- 2. Discovery + Identity over HTTPS ---- */
    status = run_http_identity(c->duid, c->cpid, c->env);
    if (status != IOTCL_SUCCESS) {
        IOTC_DBG(SYS_ERROR_ERROR, "HTTP identity failed (%d)\r\n", status);
        iotc_sdk_deinit();
        return status;
    }

    /* ---- 3. MQTT connect ---- */
    status = iotc_mqtt_connect(c->event_callback);
    if (status != 0) {
        IOTC_DBG(SYS_ERROR_ERROR, "MQTT connect failed (%d)\r\n", status);
        iotc_sdk_deinit();
        return status;
    }

    IOTC_PRNT("starting telemetry loop\r\n");

    /* ---- 4. Telemetry publish loop ---- */
    s_telemetry_task = xTaskGetCurrentTaskHandle();
    count    = (c->telemetry_count    != 0U) ? c->telemetry_count    : IOTC_SDK_PUBLISH_COUNT;
    delay_ms = (c->telemetry_delay_ms != 0U) ? c->telemetry_delay_ms : IOTC_SDK_PUBLISH_DELAY_MS;

    for (i = 0U; (count == IOTC_TELEMETRY_UNLIMITED) || (i < count); i++) {
        if (!s_mqtt_connected) {
            IOTC_DBG(SYS_ERROR_ERROR, "connection lost after %lu publishes\r\n",
                     (unsigned long)i);
            iotc_sdk_deinit();
            return -ECONNRESET;
        }

        if (iotc_publish_telemetry() != 0) {
            IOTC_DBG(SYS_ERROR_ERROR, "publish failed at index %lu\r\n",
                     (unsigned long)i);
            iotc_sdk_deinit();
            return -EIO;
        }

        /* The MQTT library runs its own task, so keep-alive and inbound
         * messages are serviced while this task sleeps.  The wait ends early
         * when iotc_request_publish() is called. */
        (void)ulTaskNotifyTake(pdTRUE, delay_ms / portTICK_PERIOD_MS);
    }

    IOTC_PRNT("telemetry loop complete\r\n");
    iotc_sdk_deinit();
    return IOTCL_SUCCESS;
}
