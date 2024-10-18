/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_aws.c

  Summary:
    This file contains the source code for the MPLAB Harmony application AWS part.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

#include "iotc_config.h"
#include "app_common.h"
#include "app_aws.h"
#include "cJSON.h"
#include "iot_network_wolfssl.h"

#if 1
#include "iotcl_log.h"
#include "iotcl_util.h"
#endif

// *****************************************************************************

char g_Cloud_Endpoint[100];
char g_Aws_ClientID[CLIENT_IDENTIFIER_MAX_LENGTH];

static IotclClientConfig config;
static IotclMqttConfig *IoTC_mqtt;
extern APP_DEVICE_DATA app_deviceData;

// *****************************************************************************

/* Publish to cloud every 'PUBLISH_FREQUENCY_MS' milliseconds */
static void pubTimerCallback(uintptr_t context) 
{
    appAwsData.publishToCloud = true;
}

/* MQTT disconnect callback */
static void mqttDisconnectCallback( void * param1, IotMqttCallbackParam_t * const pOperation )
{
    APP_AWS_DBG(SYS_ERROR_ERROR, "MQTT connection terminated \r\n");
    MQTT_DISCONNECTED;
}

/* Called by the MQTT library when an operation completes. */
static void operationCompleteCallback( void * param1, IotMqttCallbackParam_t * const pOperation )
{
    appAwsData.pendingMessages--;
    if( pOperation->u.operation.result == IOT_MQTT_SUCCESS )
    {
        APP_AWS_DBG(SYS_ERROR_INFO, "MQTT %s successfully sent \r\n", IotMqtt_OperationType( pOperation->u.operation.type ));
    }
    else
    {
        APP_AWS_DBG(SYS_ERROR_ERROR, "MQTT %s could not be sent. Error %s \r\n",
                     IotMqtt_OperationType( pOperation->u.operation.type ),
                     IotMqtt_strerror( pOperation->u.operation.result ) );
    }
}


static void MqttCallback( void * param1, IotMqttCallbackParam_t * const pPublish )
{
    const char * pPayload = pPublish->u.message.info.pPayload;
    APP_AWS_PRNT("MQTT RECEIVED: %s \r\n",pPayload);
    iotcl_mqtt_receive_c2d(pPayload); 
}

// *****************************************************************************
static void IoTC_Publish(const char *topic, const char *json_str) 
{    
    
    IotMqttError_t publishStatus = IOT_MQTT_STATUS_PENDING;
    IotMqttPublishInfo_t publishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    IotMqttCallbackInfo_t publishComplete = IOT_MQTT_CALLBACK_INFO_INITIALIZER;
    
    char pPublishPayload[ APP_AWS_MAX_MSG_LLENGTH ];   
    char pubTopic[APP_AWS_TOPIC_NAME_MAX_LEN];
    const char * pPublishTopics[ PUBLISH_TOPIC_COUNT ] =
    {
        pubTopic,
    };
    
    publishComplete.function = operationCompleteCallback;
    publishComplete.pCallbackContext = NULL;

    /* Set the common members of the publish info. */
    publishInfo.qos = IOT_MQTT_QOS_1;
    publishInfo.topicNameLength = strlen(IoTC_mqtt->pub_rpt);
    publishInfo.pTopicName = IoTC_mqtt->pub_rpt;
    publishInfo.pPayload = json_str;
    publishInfo.payloadLength = strlen(json_str);
    publishInfo.retryMs = PUBLISH_RETRY_MS;
    publishInfo.retryLimit = PUBLISH_RETRY_LIMIT;
    
    /* PUBLISH a message. This is an asynchronous function that notifies of
     * completion through a callback. */
    publishStatus = IotMqtt_PublishAsync( appAwsData.mqttConnection,
                                          &publishInfo,
                                          ( 0x80000000 ),
                                          &publishComplete,
                                          NULL );
    
    if( publishStatus != IOT_MQTT_STATUS_PENDING )
    {
        APP_AWS_PRNT("MQTT PUBLISH returned error %s \r\n", IotMqtt_strerror( publishStatus ) );
    }
    appAwsData.pendingMessages++;
} 

// *****************************************************************************
static void IoTC_CMD(IotclC2dEventData data) {
    const char *ack_id = iotcl_c2d_get_ack_id(data);
    const char *command = iotcl_c2d_get_command(data);
    
    if (!command) 
        return;
    
    if (0 == strcmp(command, "led-green on")) {
        APP_AWS_PRNT("LED GREEN: ON\r\n");
        app_deviceData.LED_Green = true;        
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "led-green off")) {
        APP_AWS_PRNT("LED GREEN: OFF\r\n");
        app_deviceData.LED_Green = false;
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "led-blue on")) {
        APP_AWS_PRNT("LED BLUE: ON\r\n");
        app_deviceData.LED_Blue = true;
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "led-blue off")) {
        APP_AWS_PRNT("LED BLUE: OFF\r\n");
        app_deviceData.LED_Blue = false;
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "led-red on")) {
        APP_AWS_PRNT("LED RED: ON\r\n");
        app_deviceData.LED_Red = true;
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "led-red off")) {
        APP_AWS_PRNT("LED RED: OFF\r\n");
        app_deviceData.LED_Red = false;
        appAwsData.publishToCloud = true;
    } 
    else if (0 == strcmp(command, "led-red off")) {
        APP_AWS_PRNT("LED RED: OFF\r\n");
        app_deviceData.LED_Red = false;
        appAwsData.publishToCloud  = true;
    } 
    else if (0 == strcmp(command, "reset-counters")) {
        APP_AWS_PRNT("RESET COUNTERS: OFF\r\n");
        appAwsData.pendingMessages = 0;
        appAwsData.publishToCloud  = true;
    } 
    else {
        APP_AWS_PRNT("Unknown COMMAND\r\n");
    } 

}
// *****************************************************************************

void APP_AWS_Initialize( void )
{
    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_SDK_INIT;
    appAwsData.mqttConnection = IOT_MQTT_CONNECTION_INITIALIZER;
    memset(g_Aws_ClientID, 0, CLIENT_IDENTIFIER_MAX_LENGTH);
    memset(g_Cloud_Endpoint, 0, 100);
    MQTT_DISCONNECTED;
    appAwsData.pubTimerHandle = SYS_TIME_HANDLE_INVALID;
    appAwsData.publishToCloud = false;
    appAwsData.pendingMessages = 0;
    
}
static void set_up_qualification_mode(void) {
    IotclMqttConfig *mc = iotcl_mqtt_get_config();
    if (!mc) {
        IOTCL_ERROR(IOTCL_ERR_CONFIG_ERROR, "set_up_qualification_mode called, but not initialized?");
    	return;
    }
    const char* qualification_topic = "qualification";
    if (0 != strcmp(qualification_topic, mc->pub_rpt)) {
        IOTCL_INFO("AWS Qualification is set up");
    	// we need to override
    	iotcl_free(mc->pub_rpt);
    	iotcl_free(mc->pub_ack);
    	iotcl_free(mc->sub_c2d);
    	mc->pub_rpt = iotcl_strdup(qualification_topic);
    	mc->pub_ack = iotcl_strdup(qualification_topic);
    	mc->sub_c2d = iotcl_strdup(qualification_topic);
    }
}

// *****************************************************************************

void APP_AWS_Tasks ( void )
{
    switch ( appAwsData.awsCloudTaskState )
    {
        /* AWS cloud task initial state. */
        case APP_AWS_CLOUD_SDK_INIT:
        {   		
			IotMqttError_t mqttInitStatus = IOT_MQTT_SUCCESS;
			bool sdkInitialized = false;

            /* Call the SDK initialization function. */
            sdkInitialized = IotSdk_Init();

            if( sdkInitialized == false ){
                APP_AWS_DBG(SYS_ERROR_ERROR, "Error occurred while SDK init \r\n" );
                appAwsData.awsCloudTaskState = APP_AWS_CLOUD_ERROR;
            }

			mqttInitStatus = IotMqtt_Init();
           
		    if( mqttInitStatus != IOT_MQTT_SUCCESS ){
		        /* Failed to initialize MQTT library. */
		        APP_AWS_DBG(SYS_ERROR_ERROR, "Error occurred while MQTT init \r\n" );
                appAwsData.awsCloudTaskState = APP_AWS_CLOUD_ERROR;
		    }
                       
            appAwsData.awsCloudTaskState = APP_AWS_CLOUD_PENDING;
            break;
        }
        
        /* AWS cloud task pending for WLAN task trigger */
        case APP_AWS_CLOUD_PENDING:
        {
            if(WIFI_IS_CONNECTED && IP_ADDR_IS_OBTAINED && NTP_IS_DONE){
                appAwsData.awsCloudTaskState = APP_AWS_CLOUD_MQTT_CONNECT;
            }
            break;
        }  

        /* MQTT connect */
        case APP_AWS_CLOUD_MQTT_CONNECT:
        {
            if(WIFI_IS_CONNECTED && IP_ADDR_IS_OBTAINED && NTP_IS_DONE)
            {
                IotMqttError_t connectStatus = IOT_MQTT_STATUS_PENDING;
                int status = 0;
                struct IotNetworkServerInfo serverInfo = {0};
                IotMqttNetworkInfo_t networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
                IotMqttConnectInfo_t connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;
                char pClientIdentifierBuffer[ CLIENT_IDENTIFIER_MAX_LENGTH ] = { 0 };

                if (g_Cloud_Endpoint[0] != NULL)
                    serverInfo.pHostName = g_Cloud_Endpoint;
                else
                {
                    APP_AWS_DBG(SYS_ERROR_ERROR, "endpoint null\r\n");
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_ERROR;
                    break;
                }
                
                serverInfo.port = IOT_DEMO_PORT;

                /* Set the members of the network info not set by the initializer */
                networkInfo.createNetworkConnection = true;
                networkInfo.u.setup.pNetworkServerInfo = &serverInfo;
                networkInfo.u.setup.pNetworkCredentialInfo = NULL;
                networkInfo.pNetworkInterface = IOT_NETWORK_INTERFACE_WOLFSSL;

                /* Set MQTT disconnect callback*/
                networkInfo.disconnectCallback.function = mqttDisconnectCallback;
                networkInfo.disconnectCallback.pCallbackContext = NULL;

                /* Set the members of the connection info not set by the initializer. */
                connectInfo.awsIotMqttMode = true;
                connectInfo.cleanSession = true;
                connectInfo.keepAliveSeconds = KEEP_ALIVE_SECONDS;

                /* AWS mqtt doesn't use username or password */
                connectInfo.pUserName = NULL;
                connectInfo.userNameLength = 0;
                connectInfo.pPassword = NULL;
                connectInfo.passwordLength = 0;

                if (g_Aws_ClientID[0] != NULL)
                {
                    status = sprintf(pClientIdentifierBuffer, "%s", g_Aws_ClientID);
                    if( status < 0 )
                        APP_AWS_DBG(SYS_ERROR_ERROR, "Failed to generate unique client identifier for demo \r\n" );
                    else
                    {
                        /* Set the client identifier buffer and length. */
                        connectInfo.pClientIdentifier = pClientIdentifierBuffer;
                        connectInfo.clientIdentifierLength = ( uint16_t ) status;
                    }
                }
                else{
                    APP_AWS_DBG(SYS_ERROR_ERROR, "client id null\r\n");
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_ERROR;
                    break;
                }
                iotcl_deinit();
                iotcl_init_client_config(&config);
                config.device.instance_type = IOTCL_DCT_AWS_DEDICATED;
                config.device.duid = pClientIdentifierBuffer;
                config.mqtt_send_cb = IoTC_Publish;
                config.events.cmd_cb = IoTC_CMD;                
                iotcl_init_and_print_config(&config);
#if 1
                set_up_qualification_mode();
#endif                
                IoTC_mqtt = iotcl_mqtt_get_config();
                

                /* Establish the MQTT connection. */
                APP_AWS_PRNT("MQTT connect ...  \r\n");
                
                connectStatus = IotMqtt_Connect( &networkInfo,
                                                 &connectInfo,
                                                 MQTT_TIMEOUT_MS,
                                                 &appAwsData.mqttConnection );

                /* Back-off for 1 second before retry MQTT connection */
                if( connectStatus != IOT_MQTT_SUCCESS ){
                    APP_AWS_PRNT("MQTT CONNECT returned error %s \r\n", IotMqtt_strerror( connectStatus ) );
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else{
                    APP_AWS_PRNT("MQTT connected \r\n");
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_MQTT_SUBSCRIBE_TO_TOPIC;
                    MQTT_CONNECTED;
                }
            }
            else{
                /* Either Wi-Fi disconnected or IP address not assigned */
                appAwsData.awsCloudTaskState = APP_AWS_CLOUD_PENDING;
            }
			
            break;
        }

        /* Subscribe */
        case APP_AWS_CLOUD_MQTT_SUBSCRIBE_TO_TOPIC:
        {
			uint8_t i = 0;

		    IotMqttError_t subscriptionStatus = IOT_MQTT_STATUS_PENDING;
		    IotMqttSubscription_t pSubscriptions[ SUBSCRIBE_TOPIC_COUNT ] = { IOT_MQTT_SUBSCRIPTION_INITIALIZER };
            char subTopic[APP_AWS_TOPIC_NAME_MAX_LEN];
            char * pSubscribeTopics[ SUBSCRIBE_TOPIC_COUNT ] =
            {
                subTopic,
            };
                        
		    /* Set the members of the subscription list */
		    for( i = 0; i < SUBSCRIBE_TOPIC_COUNT; i++ ){
		        pSubscriptions[i].qos = IOT_MQTT_QOS_1;
		        pSubscriptions[i].pTopicFilter =  IoTC_mqtt->sub_c2d;
		        pSubscriptions[i].topicFilterLength = strlen(IoTC_mqtt->sub_c2d);
		        pSubscriptions[i].callback.pCallbackContext = NULL;
		        pSubscriptions[i].callback.function = MqttCallback;
		    }
            
            if (MQTT_IS_CONNECTED)
            {
                subscriptionStatus = IotMqtt_SubscribeSync( appAwsData.mqttConnection,
                                                        pSubscriptions,
                                                        SUBSCRIBE_TOPIC_COUNT,
                                                        0,
                                                        MQTT_TIMEOUT_MS );
                if ( subscriptionStatus == IOT_MQTT_SUCCESS){
                    APP_AWS_PRNT("MQTT subscriptions accepted \r\n" );
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_MQTT_PUBLISH_TO_TOPIC;
                }
            }
            else
                /* MQTT disconnected */
                appAwsData.awsCloudTaskState = APP_AWS_CLOUD_PENDING;

            break;
        }
        
        /* Publish */
        case APP_AWS_CLOUD_MQTT_PUBLISH_TO_TOPIC:
        {
            if(appAwsData.pubTimerHandle == SYS_TIME_HANDLE_INVALID)
            {
                appAwsData.pubTimerHandle = SYS_TIME_CallbackRegisterMS(pubTimerCallback, (uintptr_t) 0, PUBLISH_FREQUENCY_MS, SYS_TIME_PERIODIC);
                if (appAwsData.pubTimerHandle == SYS_TIME_HANDLE_INVALID) {
                    APP_AWS_DBG(SYS_ERROR_ERROR, "Failed creating a timer for periodic publish \r\n");
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_ERROR;
                    break;
                }
            }
            
            if (MQTT_IS_CONNECTED)
            {
                if(appAwsData.publishToCloud == true)
                {
                    int status = 0;

                    /* Publish messages. */
                    IotclMessageHandle msg = iotcl_telemetry_create();
#if 1
                    iotcl_telemetry_set_string(msg, "qualification", "true");
#else
                    iotcl_telemetry_set_number(msg, "WFI32IoT_button1", app_deviceData.switch1Status);
                    iotcl_telemetry_set_number(msg, "WFI32IoT_button1_count", app_deviceData.switch1Cnt);
                    iotcl_telemetry_set_number(msg, "WFI32IoT_button2", app_deviceData.switch2Status);
                    iotcl_telemetry_set_number(msg, "WFI32IoT_button2_count", app_deviceData.switch2Cnt);
                    iotcl_telemetry_set_number(msg, "Onboard_Light_Lux", APP_readLight());
                    iotcl_telemetry_set_number(msg, "Onboard_Temp_DegC", APP_readTemp());
                    iotcl_telemetry_set_string(msg, "LED_Blue", app_deviceData.LED_Blue ? "On" : "Off");
                    iotcl_telemetry_set_string(msg, "LED_Green", app_deviceData.LED_Green ? "On" : "Off");
                    iotcl_telemetry_set_string(msg, "LED_Red",  app_deviceData.LED_Red ? "On" : "Off");
#endif
                    iotcl_mqtt_send_telemetry(msg, false);
                    iotcl_telemetry_destroy(msg); 
                    appAwsData.publishToCloud = false;
                }
            }
            else
                /* MQTT disconnected; try to re-connect only if ALL older messages are done */
                /* in context of a callback is received whether success or failure */
                if(appAwsData.pendingMessages == 0)
                    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_PENDING;
            break;
        }
        
        /* Idle */
        case APP_AWS_CLOUD_IDLE:
        {
            break;
        }
        
        /* Error */
        case APP_AWS_CLOUD_ERROR:
        {
            SYS_CONSOLE_MESSAGE("APP_AWS_CLOUD_ERROR\r\n");
            appAwsData.awsCloudTaskState = APP_AWS_CLOUD_IDLE;
            break;
        }        

        default:
        {
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
