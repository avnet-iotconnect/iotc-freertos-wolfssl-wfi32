/*******************************************************************************
  IoTConnect application task

  File Name:
    app_iotconnect.h

  Summary:
    FreeRTOS task that runs the full IoTConnect flow: HTTPS discovery,
    HTTPS identity, MQTT connect and periodic sensor telemetry.

  Description:
    This task is an alternative to APP_AWS_Tasks() in app_aws.c.  Instead of
    taking the broker endpoint and the client ID from the "cloud config" file
    on the USB MSD volume, it obtains them at run time from the IoTConnect
    Device REST API (discovery + identity) and then publishes the same
    telemetry set that APP_AWS_Tasks() publishes.

    Only ONE of the two may own an MQTT connection: they share the broker
    endpoint global (g_Cloud_Endpoint, which the wolfSSL glue uses for SNI)
    and would register two MQTT clients with the same client ID.  In this
    project APP_AWS_Tasks() has been replaced by APP_IOTC_Tasks() in tasks.c,
    so APP_IOTC_Tasks() also performs the one-time IotSdk_Init() and
    IotMqtt_Init() that APP_AWS_Tasks() used to do.

    app_aws.c is still compiled: it owns g_Cloud_Endpoint / g_Aws_ClientID and
    appAwsData, which app_usb_msd.c and app_ps.c use.  Only its task is gone.

  Integration:
    - APP_IOTC_Initialize() is called from APP_Initialize() in app.c.
    - APP_IOTC_Tasks() is the task body; tasks.c runs it in place of
      APP_AWS_Tasks().  It blocks and never returns.
*******************************************************************************/

#ifndef _APP_IOTCONNECT_H
#define _APP_IOTCONNECT_H

#include <stdint.h>
#include <stdbool.h>

#include "configuration.h"
#include "iotcl.h"

#ifdef __cplusplus
extern "C" {
#endif

// *****************************************************************************

#define APP_IOTC_DBG(level, fmt, ...) SYS_DEBUG_PRINT(level, "[APP_IOTC] " fmt, ##__VA_ARGS__)
#define APP_IOTC_PRNT(fmt, ...)       SYS_CONSOLE_PRINT("[APP_IOTC] " fmt, ##__VA_ARGS__)

/* ---- Account configuration ---------------------------------------------
 * CPID and environment come from iotc_config.h (IOTC_AWS_CPID / IOTC_AWS_ENV).
 * The environment string must match the one shown in the IoTConnect portal;
 * it is usually lower case ("poc").
 * ---------------------------------------------------------------------- */
#ifndef APP_IOTC_ENV
#define APP_IOTC_ENV                IOTC_AWS_ENV
#endif

#ifndef APP_IOTC_CPID
#define APP_IOTC_CPID               IOTC_AWS_CPID
#endif

/* Define APP_IOTC_DUID to pin the device unique ID at build time.  When it is
 * not defined, the ID is taken from the ClientID in the USB MSD cloud config
 * file, and failing that from the ECC608 serial number. */
/* #define APP_IOTC_DUID            "my-device-id" */

/** Seconds between publishes. */
#ifndef APP_IOTC_PUBLISH_PERIOD_MS
#define APP_IOTC_PUBLISH_PERIOD_MS  (PUBLISH_FREQUENCY_MS)
#endif

/** Delay before retrying the whole flow after a failure, in ms. */
#ifndef APP_IOTC_RETRY_DELAY_MS
#define APP_IOTC_RETRY_DELAY_MS     (10000U)
#endif

/** Stack depth (in words) and priority used by the xTaskCreate() call for
 *  this task in tasks.c. */
#ifndef APP_IOTC_TASK_STACK_WORDS
#define APP_IOTC_TASK_STACK_WORDS   (2048U)
#endif
#ifndef APP_IOTC_TASK_PRIORITY
#define APP_IOTC_TASK_PRIORITY      (1U)
#endif

/** Set to 1 only if APP_AWS_Tasks() is still created in tasks.c; the task
 *  then parks it instead of letting it open a second MQTT connection. */
#ifndef APP_IOTC_PARK_APP_AWS
#define APP_IOTC_PARK_APP_AWS       (0)
#endif

// *****************************************************************************

/**
 * Initialise the module state.  Call once before the scheduler starts.
 */
void APP_IOTC_Initialize(void);

/**
 * Task body: initialises the AWS SDK, waits for the network, then runs
 * iotconnect_sdk() in a retry loop.  Blocks and never returns; tasks.c runs
 * it as the "APP_IOTC_Tasks" thread.
 */
void APP_IOTC_Tasks(void);

/**
 * @return true while the IoTConnect MQTT connection is up.
 */
bool APP_IOTC_IsConnected(void);

/**
 * Request an out-of-band telemetry publish, e.g. after a button press or a
 * command that changed the LED state.  The next publish happens immediately
 * instead of waiting for the period to elapse.
 */
void APP_IOTC_RequestPublish(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_IOTCONNECT_H */
