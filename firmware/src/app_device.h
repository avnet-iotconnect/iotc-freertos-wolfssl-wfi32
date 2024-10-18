/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_device.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_DEVICE_Initialize" and "APP_DEVICE_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_DEVICE_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_DEVICE_H
#define _APP_DEVICE_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"   

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    APP_DEVICE_STATE_INIT=0,
    APP_DEVICE_STATE_MONITOR_SWITCH1,
    APP_DEVICE_STATE_MONITOR_SWITCH2,
    APP_DEVICE_STATE_SET_LED,                   
    APP_DEVICE_STATE_SENSORS_CHECK,
    APP_DEVICE_STATE_SENSORS_TURN_ON_MCP9808,
    APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_MCP9808,
    APP_DEVICE_STATE_SENSORS_TURN_ON_OPT3001,
    APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_OPT3001,
    APP_DEVICE_STATE_SENSORS_SHUTDOWN_MCP9808,
    APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_MCP9808,
    APP_DEVICE_STATE_SENSORS_SHUTDOWN_OPT3001,
    APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_OPT3001,
    APP_DEVICE_STATE_SENSORS_READ_TEMP,
    APP_DEVICE_STATE_SENSORS_WAIT_READ_TEMP,
    APP_DEVICE_STATE_SENSORS_READ_LIGHT,
    APP_DEVICE_STATE_SENSORS_WAIT_READ_LIGHT,
    APP_DEVICE_STATE_SENSORS_READ_MCP9808_DEV_ID,
    APP_DEVICE_STATE_SENSORS_WAIT_MCP9808_DEV_ID,
    APP_DEVICE_STATE_SENSORS_READ_OPT3001_DEV_ID,
    APP_DEVICE_STATE_SENSORS_WAIT_OPT3001_DEV_ID,
    APP_DEVICE_STATE_IDLE,
    APP_DEVICE_STATE_ERROR,              

} APP_DEVICE_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

/* Control structure periodic operations */
typedef struct
{
    uint16_t counter;
    uint16_t reload;
    bool periodic;
} APP_CTRL_TIMER_S; 

/* I2C tranfser status */
typedef enum
{
    I2C_TRANSFER_STATUS_IN_PROGRESS,
    I2C_TRANSFER_STATUS_SUCCESS,
    I2C_TRANSFER_STATUS_ERROR,
    I2C_TRANSFER_STATUS_IDLE,
} APP_CTRL_I2C_TRANSFER_STATUS;    

/* I2C */
typedef struct
{
    DRV_HANDLE i2cHandle;
    DRV_I2C_TRANSFER_HANDLE transferHandle;
    APP_CTRL_I2C_TRANSFER_STATUS transferStatus;
    uint8_t txBuffer[4];
    uint16_t rxBuffer;
} APP_CTRL_I2C;

/* MCP9808 structure */
typedef struct
{
    bool IsShutdown;
    int16_t temperature;
    uint16_t deviceID;
} APP_CTRL_MCP9808;

/* OPT3001 structure */
typedef struct
{
    bool IsShutdown;
    uint32_t light;
    uint16_t deviceID;
} APP_CTRL_OPT3001; 

/* RTCC data */
typedef struct {
    bool rtccAlarm;
    struct tm sysTime;
}APP_CTRL_RTCC; 
/******************************************/
typedef struct
{
    /* The application's current state */
    APP_DEVICE_STATES state;
    bool LED_Red;
    bool LED_Green;
    bool LED_Blue;
    bool switch1Status;      // SWITCH1_STATE_PRESSED or SWITCH1_STATE_RELEASED
    uint16_t switch1Cnt;     // button press counter    
    bool switch2Status;      // SWITCH1_STATE_PRESSED or SWITCH1_STATE_RELEASED
    uint16_t switch2Cnt;     // button press counter  
     
    SYS_TIME_HANDLE timeHandle;
    APP_CTRL_TIMER_S sensorsReadCtrl;
    bool readSensors;
    bool shutdownSensors;
    bool turnOnSensors;
    APP_CTRL_I2C i2c;
    APP_CTRL_MCP9808 mcp9808;
    APP_CTRL_OPT3001 opt3001;
    APP_CTRL_RTCC rtccData;    
    
} APP_DEVICE_DATA;

extern APP_DEVICE_DATA app_deviceData; 
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_sensorsOn(void);
void APP_sensorsOff(void);
int16_t APP_readTemp(void);
uint32_t APP_readLight(void);

/*******************************************************************************
  Function:
    void APP_DEVICE_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_DEVICE_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_DEVICE_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_DEVICE_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_DEVICE_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_DEVICE_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_DEVICE_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_DEVICE_H */

/*******************************************************************************
 End of File
 */

