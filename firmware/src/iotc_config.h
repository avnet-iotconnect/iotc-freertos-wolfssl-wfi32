/* This file contains configuration settings for the IoTConnect AWS demo */

#ifndef IOT_CONFIG_H_
#define IOT_CONFIG_H_

#define IOTC_AWS_ENV                "POC"
#define IOTC_AWS_CPID               "97FF86E8728645E9B89F7B07977E4B15"
#define IOTC_AWS_BROKER             "a3etk4e19usyja-ats.iot.us-east-1.amazonaws.com"
#define IOTC_AWS_DISCOVERY_URL      "awsdiscovery.iotconnect.io"

#define IOTC_AWS_DSICOVERY_PATH \
    "GET /api/v2.1/dsdk/cpId/" IOTC_AWS_CPID "/env/" IOTC_AWS_ENV " HTTP/1.1\r\n" \
    "Host: " IOTC_AWS_DISCOVERY_URL "\r\n" \
    "Connection: close\r\n" \
    "\r\n"

#define IOTC_DSICOVERY_PATH     IOTC_AWS_DSICOVERY_PATH
#define IOTC_DSICOVERY_URL      IOTC_AWS_DISCOVERY_URL


#define APP_USE_X509_CERT   
#define APP_AWS_TOPIC_NAME_MAX_LEN            128
#define APP_AWS_MAX_MSG_LLENGTH               256

/* App settings */
#define IOT_DEMO_SECURED_CONNECTION    ( true )
#define IOT_DEMO_SERVER                "a3etk4e19usyja-ats.iot.us-east-1.amazonaws.com"
#define IOT_DEMO_PORT                  ( 8883 )
#define CLIENT_IDENTIFIER_MAX_LENGTH             ( 256 )
#define KEEP_ALIVE_SECONDS                       ( 60 )
#define MQTT_TIMEOUT_MS                          ( 5000 )
#define IOT_MQTT_RESPONSE_WAIT_MS                ( 5000 )
#define AWS_IOT_MQTT_ENABLE_METRICS              ( 0 ) //(disabled to avoid setting/sending username in MQTT connect)
#define PUBLISH_TOPIC_COUNT                       ( 1 )
#define SUBSCRIBE_TOPIC_COUNT                    ( 1 )
#define PUBLISH_RETRY_LIMIT                      ( 10 )
#define PUBLISH_RETRY_MS                         ( 2000 )
#define PUBLISH_FREQUENCY_MS                     ( 2000 )

/* Enable asserts in the libraries. */
#define IOT_CONTAINERS_ENABLE_ASSERTS           ( 0 )
#define IOT_MQTT_ENABLE_ASSERTS                 ( 1 )
#define IOT_TASKPOOL_ENABLE_ASSERTS             ( 1 )
#define AWS_IOT_SHADOW_ENABLE_ASSERTS           ( 1 )
#define AWS_IOT_DEFENDER_ENABLE_ASSERTS         ( 1 )
#define AWS_IOT_JOBS_ENABLE_ASSERTS             ( 1 )

/* Library logging configuration. IOT_LOG_LEVEL_GLOBAL provides a global log
 * level for all libraries; the library-specific settings override the global
 * setting. If both the library-specific and global settings are undefined,
 * no logs will be printed. */
#define IOT_LOG_LEVEL_GLOBAL                    IOT_LOG_ERROR
#define IOT_LOG_LEVEL_DEMO                      IOT_LOG_DEBUG
#define IOT_LOG_LEVEL_PLATFORM                  IOT_LOG_ERROR
#define IOT_LOG_LEVEL_NETWORK                   IOT_LOG_ERROR
#define IOT_LOG_LEVEL_TASKPOOL                  IOT_LOG_ERROR
#define IOT_LOG_LEVEL_MQTT                      IOT_LOG_ERROR
#define AWS_IOT_LOG_LEVEL_SHADOW                IOT_LOG_ERROR
#define AWS_IOT_LOG_LEVEL_DEFENDER              IOT_LOG_ERROR
#define AWS_IOT_LOG_LEVEL_JOBS                  IOT_LOG_ERROR

/* Default assert and memory allocation functions. */
#include <assert.h>
#include <stdlib.h>
#include "system_definitions.h"
#include "tcpip/tcpip.h"

#define Iot_DefaultAssert    assert
#define Iot_DefaultMalloc    pvPortMalloc
#define Iot_DefaultFree      vPortFree
#define IotNetwork_Malloc    Iot_DefaultMalloc
#define IotNetwork_Free      Iot_DefaultFree

/* The build system will choose the appropriate system types file for the platform
 * layer based on the host operating system. */
#include "iot_platform_types_pic32mzw1.h"

#endif /* ifndef IOT_CONFIG_H_ */
