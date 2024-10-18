/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_device.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_device.h"
#include "definitions.h" 
#include <math.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define SWITCH1_STATE_PRESSED   0U
#define SWITCH1_STATE_RELEASED  1U
#define SWITCH2_STATE_PRESSED   0U
#define SWITCH2_STATE_RELEASED  1U  

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_DEVICE_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DEVICE_DATA app_deviceData;

// *****************************************************************************
/* Main timer resolution */
#define TIMER_RESOLUTION_MS         50

/* Sensors */
#define SENSORS_READ_FREQ_MS        2000

/* MCP9808 registers */
#define MCP9808_I2C_ADDRESS         0x18 
#define MCP9808_REG_CONFIG          0x01
#define MCP9808_REG_TAMBIENT		0x05
#define MCP9808_REG_MANUF_ID		0x06
#define MCP9808_REG_DEVICE_ID		0x07
#define MCP9808_REG_RESOLUTION		0x08

/* MCP9808 other settings */
#define MCP9808_CONFIG_DEFAULT		0x00
#define MCP9808_CONFIG_SHUTDOWN		0x0100
#define MCP9808_RES_DEFAULT         62500
#define MCP9808_MANUF_ID            0x54
#define MCP9808_DEVICE_ID           0x0400
#define MCP9808_DEVICE_ID_MASK		0xff00

/* OPT3001 registers */
#define OPT3001_I2C_ADDRESS             0x44
#define OPT3001_REG_RESULT              0x00
#define OPT3001_REG_CONFIG              0x01
#define OPT3001_REG_LOW_LIMIT           0x02
#define OPT3001_REG_HIGH_LIMIT          0x03
#define OPT3001_REG_MANUFACTURER_ID     0x7E
#define OPT3001_REG_DEVICE_ID           0x7F

/* MCP9808 other settings */
#define OPT3001_CONFIG_SHUTDOWN             0x00
#define OPT3001_CONFIG_CONT_CONVERSION		0xCE10        //continuous convesrion
#define OPT3001_MANUF_ID                    0x5449
#define OPT3001_DEVICE_ID                   0x3001

// *****************************************************************************

/* I2C */
static bool i2cReadReg(uint8_t, uint16_t, uint8_t);
static bool i2cWriteReg(uint8_t, uint16_t, uint16_t);
static void i2cReadRegComp(uint8_t, uint8_t);
static void i2cWriteRegComp(uint8_t, uint8_t);

// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Main timer handler */
static void timeCallback(uintptr_t param) {
    uint8_t led;
    
    /* Sensors */
    if(app_deviceData.sensorsReadCtrl.reload > 0 && 
            ++app_deviceData.sensorsReadCtrl.counter == app_deviceData.sensorsReadCtrl.reload){
        app_deviceData.readSensors = true;     
        if(app_deviceData.sensorsReadCtrl.periodic)
            app_deviceData.sensorsReadCtrl.counter = 0;
        else
            app_deviceData.sensorsReadCtrl.reload = 0;
    }
}  

/* I2C transfer callback */
static void i2cTransferCallback(DRV_I2C_TRANSFER_EVENT event, 
        DRV_I2C_TRANSFER_HANDLE transferHandle, 
        uintptr_t context){
    switch(event)
    {
        case DRV_I2C_TRANSFER_EVENT_COMPLETE:
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_SUCCESS;
            break;
        case DRV_I2C_TRANSFER_EVENT_ERROR:
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_ERROR;
            break;
        default:
            break;
    }
}

/* RTCC callback*/
void rtcc_callback(uintptr_t context) {
    app_deviceData.rtccData.rtccAlarm = true;
}  
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* I2C read */
static bool i2cReadReg(uint8_t addr, uint16_t reg, uint8_t size){
    bool ret = false;
    app_deviceData.i2c.transferHandle == DRV_I2C_TRANSFER_HANDLE_INVALID;
    app_deviceData.i2c.txBuffer[0] = (uint8_t)reg;
    
    DRV_I2C_WriteReadTransferAdd(app_deviceData.i2c.i2cHandle, 
            addr, 
            (void*)app_deviceData.i2c.txBuffer, 1, 
            (void*)&app_deviceData.i2c.rxBuffer, size, 
            &app_deviceData.i2c.transferHandle);
    if(app_deviceData.i2c.transferHandle == DRV_I2C_TRANSFER_HANDLE_INVALID)
    {
        //SYS_CONSOLE_PRINT( "I2C read reg %x error \r\n", reg);
        ret = false;
    }
    else
        ret = true;
    return ret;
}

/* I2C read complete */
static void i2cReadRegComp(uint8_t addr, uint8_t reg){
    app_deviceData.i2c.rxBuffer = (app_deviceData.i2c.rxBuffer << 8) | (app_deviceData.i2c.rxBuffer >> 8);
    // SYS_CONSOLE_PRINT( "I2C read complete - periph addr %x val %x\r\n", addr, app_deviceData.i2c.rxBuffer);
    switch(addr)
    {   
        /* MCP9808 */
        case MCP9808_I2C_ADDRESS:
            if (reg == MCP9808_REG_TAMBIENT){
                uint8_t upperByte = (uint8_t)(app_deviceData.i2c.rxBuffer >> 8);
                uint8_t lowerByte = ((uint8_t)(app_deviceData.i2c.rxBuffer & 0x00FF));
                upperByte = upperByte & 0x1F;
                if ((upperByte & 0x10) == 0x10)
                {         // Ta < 0 degC
                    upperByte = upperByte & 0x0F;       // Clear sign bit
                    app_deviceData.mcp9808.temperature = ((upperByte * 16) + lowerByte/16);          
                }
                else
                {
                    app_deviceData.mcp9808.temperature = 256 - ((upperByte * 16) + lowerByte/16);
                }
                SYS_CONSOLE_PRINT("MCP9808 Temperature %d (C)\r\n", app_deviceData.mcp9808.temperature);                
            }
            else if (reg == MCP9808_REG_DEVICE_ID){
                app_deviceData.mcp9808.deviceID = app_deviceData.i2c.rxBuffer;
                SYS_CONSOLE_PRINT( "MCP9808 Device ID %x\r\n", app_deviceData.mcp9808.deviceID);                
            }
            break;

        /* OPT3001 */
        case OPT3001_I2C_ADDRESS:
            if (reg == OPT3001_REG_RESULT){
                uint16_t m = app_deviceData.i2c.rxBuffer & 0x0FFF;
                uint16_t e = (app_deviceData.i2c.rxBuffer & 0xF000) >> 12;
                app_deviceData.opt3001.light = (m*pow(2,e))/100;
                SYS_CONSOLE_PRINT( "OPT3001 Light %d (lux)\r\n", app_deviceData.opt3001.light); 
            }
            else if (reg == OPT3001_REG_DEVICE_ID){
                app_deviceData.opt3001.deviceID = app_deviceData.i2c.rxBuffer;
                SYS_CONSOLE_PRINT("OPT3001 Device ID %x\r\n", app_deviceData.opt3001.deviceID);                
            }
            break;

        default:
            break;
    }
}  

/* I2C write */
static bool i2cWriteReg(uint8_t addr, uint16_t reg, uint16_t val){
    bool ret = false;
    app_deviceData.i2c.transferHandle == DRV_I2C_TRANSFER_HANDLE_INVALID;
    app_deviceData.i2c.txBuffer[0] = (uint8_t)reg;
    app_deviceData.i2c.txBuffer[1] = (uint8_t)(val >> 8);
    app_deviceData.i2c.txBuffer[2] = (uint8_t)(val & 0x00FF);
    
    DRV_I2C_WriteTransferAdd(app_deviceData.i2c.i2cHandle, 
            addr, 
            (void*)app_deviceData.i2c.txBuffer, 3, 
            &app_deviceData.i2c.transferHandle);
    
    if(app_deviceData.i2c.transferHandle == DRV_I2C_TRANSFER_HANDLE_INVALID)
    {
        SYS_CONSOLE_PRINT( "I2C write reg %x error \r\n", reg);
        ret = false;
    }
    else
        ret = true;
    
    return ret;
}

/* I2C write complete */
static void i2cWriteRegComp(uint8_t addr, uint8_t reg){
     SYS_CONSOLE_PRINT("I2C write complete - periph addr %x\r\n", addr);
} 

/* Sensors sub-module init */
static void sensorsInit(){
    /* Issue I2C read operation to get sensors readings*/
    app_deviceData.readSensors = false;
      
    /* Default is sensors on Vs sensors shutdown */
    app_deviceData.turnOnSensors = true;
    app_deviceData.shutdownSensors = false;
    
    /*sensors periodic behavior configuration*/
    app_deviceData.sensorsReadCtrl.counter = 0;
    app_deviceData.sensorsReadCtrl.reload = SENSORS_READ_FREQ_MS/TIMER_RESOLUTION_MS;
    app_deviceData.sensorsReadCtrl.periodic = true;
    
    /*sensors structures*/
    memset(&app_deviceData.mcp9808, 0, sizeof(app_deviceData.mcp9808));
    memset(&app_deviceData.opt3001, 0, sizeof(app_deviceData.opt3001));
    app_deviceData.mcp9808.IsShutdown = true;
    app_deviceData.opt3001.IsShutdown = true;
    
    /*I2C structure*/
    memset(&app_deviceData.i2c, 0, sizeof(app_deviceData.i2c));
    app_deviceData.i2c.i2cHandle = DRV_I2C_TRANSFER_HANDLE_INVALID;
    app_deviceData.i2c.transferHandle = DRV_I2C_TRANSFER_HANDLE_INVALID;
}

/* Setup RTCC */
static void setup_rtcc(void) {
    struct tm sys_time;
    struct tm alarm_time;
    RTCC_ALARM_MASK mask;

    // Time setting 31-12-2019 23:59:58 Monday
    sys_time.tm_hour = 0;
    sys_time.tm_min = 0;
    sys_time.tm_sec = 0;

    sys_time.tm_year = 0;
    sys_time.tm_mon = 1;
    sys_time.tm_mday = 1;
    sys_time.tm_wday = 0;

    // Alarm setting 01-01-2020 00:00:05 Tuesday
    alarm_time.tm_hour = 00;
    alarm_time.tm_min = 00;
    alarm_time.tm_sec = 01;

    alarm_time.tm_year = 0;
    alarm_time.tm_mon = 1;
    alarm_time.tm_mday = 1;
    alarm_time.tm_wday = 0;

    RTCC_CallbackRegister(rtcc_callback, (uintptr_t) NULL);

    if (RTCC_TimeSet(&sys_time) == false) {
        SYS_CONSOLE_PRINT("Error setting time\r\n");
        return;
    }
    
    mask = RTCC_ALARM_MASK_SECOND;
    if (RTCC_AlarmSet(&alarm_time, mask) == false) {
        SYS_CONSOLE_PRINT("Error setting alarm\r\n");
    }
    
    SYS_CONSOLE_PRINT("OK setting time\r\n");
} 

/* Sensors Turn on & enable periodic I2C reading */
void APP_sensorsOn(void)
{
    app_deviceData.turnOnSensors = true;
    app_deviceData.sensorsReadCtrl.counter = 0;
    app_deviceData.sensorsReadCtrl.reload = SENSORS_READ_FREQ_MS/TIMER_RESOLUTION_MS;
    app_deviceData.sensorsReadCtrl.periodic = true;
}

/* Sensors Shutdown (to save power) & disable periodic I2C reading */
void APP_sensorsOff(void)
{
    app_deviceData.shutdownSensors = true;
    app_deviceData.sensorsReadCtrl.counter = 0;
    app_deviceData.sensorsReadCtrl.reload = 0;
    app_deviceData.sensorsReadCtrl.periodic = false;
}

/* Read MCP9808 Temperature */
int16_t APP_readTemp(void)
{
    return app_deviceData.mcp9808.temperature;
}

/* Read OPT3001 Light */
uint32_t APP_readLight(void)
{
    return app_deviceData.opt3001.light;
}   
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_DEVICE_Initialize ( void )

  Remarks:
    See prototype in app_device.h.
 */

void APP_DEVICE_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_deviceData.state = APP_DEVICE_STATE_INIT;
    app_deviceData.timeHandle == SYS_TIME_HANDLE_INVALID;
    
    memset((void*)&app_deviceData.rtccData, 0 , sizeof(app_deviceData.rtccData)); 
    
    app_deviceData.LED_Red = false;
    app_deviceData.LED_Green =false;
    app_deviceData.LED_Blue = false;
    
    app_deviceData.switch1Cnt = 0;
    app_deviceData.switch1Status = false;    
    app_deviceData.switch2Cnt = 0;
    app_deviceData.switch2Status = false;     
    
    /* Start a periodic timer to handle periodic events*/
    app_deviceData.timeHandle = SYS_TIME_CallbackRegisterMS(timeCallback, 
                                                            (uintptr_t) 0, 
                                                            TIMER_RESOLUTION_MS, 
                                                            SYS_TIME_PERIODIC);
    if (app_deviceData.timeHandle == SYS_TIME_HANDLE_INVALID) {
        SYS_CONSOLE_PRINT("Failed creating a periodic timer \r\n");
        return;
    }         
}
/******************************************************************************
  Function:
    void APP_DEVICE_Tasks ( void )

  Remarks:
    See prototype in app_device.h.
 */

void APP_DEVICE_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_deviceData.state )
    {
        /* Application's initial state. */
        case APP_DEVICE_STATE_INIT:
        {
            /* Open I2C driver client */
            app_deviceData.i2c.i2cHandle = DRV_I2C_Open( DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE );
            if (app_deviceData.i2c.i2cHandle == DRV_HANDLE_INVALID)
            {
                SYS_CONSOLE_PRINT("Failed to open I2C driver for sensors reading\r\n");
                app_deviceData.state = APP_DEVICE_STATE_ERROR;
            }
            else{
                DRV_I2C_TransferEventHandlerSet(app_deviceData.i2c.i2cHandle, i2cTransferCallback, 0);
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            
            /* Setup RTCC */
            setup_rtcc();   
            app_deviceData.state = APP_DEVICE_STATE_IDLE;
            break;

        }
        
        case APP_DEVICE_STATE_MONITOR_SWITCH1:
        {
            static uint8_t switch1PressedCnt = 0;
            if (SWITCH1_Get() == SWITCH1_STATE_PRESSED)
            {
                switch1PressedCnt++ ;
                if (switch1PressedCnt == 2)
                {
                    app_deviceData.switch1Status = true;
                    app_deviceData.switch1Cnt++;
                    SYS_CONSOLE_PRINT("[app_device] SW1 pressed (%d)\r\n", app_deviceData.switch1Cnt);
                }
                else if (switch1PressedCnt == 100)
                {
                    app_deviceData.switch1Cnt = 0;
                    SYS_CONSOLE_PRINT("[app_device] SW1 pressed (%d)\r\n", app_deviceData.switch1Cnt);
                
                }
            }
            else
            {
                switch1PressedCnt = 0;
                app_deviceData.switch1Status = false;
            }
 
            app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH2;
            break;
        }
        
        case APP_DEVICE_STATE_MONITOR_SWITCH2:
        {
            static uint8_t switch2PressedCnt = 0;
            if (SWITCH2_Get() == SWITCH2_STATE_PRESSED)
            {
                switch2PressedCnt++ ;
                if (switch2PressedCnt == 2)
                {
                    app_deviceData.switch2Status = true;
                    app_deviceData.switch2Cnt++;
                    SYS_CONSOLE_PRINT("[app_device] SW2 pressed (%d)\r\n", app_deviceData.switch2Cnt);
                
                }
                else if (switch2PressedCnt == 100)
                {
                    app_deviceData.switch2Cnt = 0;
                    SYS_CONSOLE_PRINT("[app_device] SW2 pressed (%d)\r\n", app_deviceData.switch2Cnt);
                }
            }
            else
            {
                switch2PressedCnt = 0;
                app_deviceData.switch2Status = false;
            }
 
            app_deviceData.state = APP_DEVICE_STATE_SET_LED;
            break;
        }
        
        case APP_DEVICE_STATE_SET_LED:
        {
            if (app_deviceData.LED_Red) {LED_RED_On();} else {LED_RED_Off();}
            if (app_deviceData.LED_Green) {LED_GREEN_On();} else {LED_GREEN_Off();}
            if (app_deviceData.LED_Blue) {LED_BLUE_On();} else {LED_BLUE_Off();          }
            app_deviceData.state = APP_DEVICE_STATE_SENSORS_CHECK;
            break;
        }        
        
        case APP_DEVICE_STATE_SENSORS_CHECK:
        {
            /* Read RTCC */
            if (app_deviceData.rtccData.rtccAlarm) {
                app_deviceData.rtccData.rtccAlarm = false;
                RTCC_TimeGet(&app_deviceData.rtccData.sysTime);
            }
            
            /* User request to turn on sensors */
            if(app_deviceData.turnOnSensors){
                app_deviceData.turnOnSensors = false;
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_TURN_ON_MCP9808;
            }
            /* User request to shutdown sensors (to save power) */
            else if(app_deviceData.shutdownSensors){
                app_deviceData.shutdownSensors = false;
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_SHUTDOWN_MCP9808;
            }
            /* read MCP9808 device ID*/
            else if(app_deviceData.mcp9808.deviceID == 0 &&
                    app_deviceData.mcp9808.IsShutdown == false){
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_MCP9808_DEV_ID;
            }
            /* read OPT3001 device ID*/
            else if(app_deviceData.opt3001.deviceID == 0 &&
                    app_deviceData.opt3001.IsShutdown == false){
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_OPT3001_DEV_ID;
            }
            else if(app_deviceData.readSensors && 
                    app_deviceData.mcp9808.IsShutdown == false && 
                    app_deviceData.opt3001.IsShutdown == false){
                app_deviceData.readSensors = false;
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            }
            break;
        } 

        /* MCP9808 turn on */
        case APP_DEVICE_STATE_SENSORS_TURN_ON_MCP9808:
        {
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            if(i2cWriteReg(MCP9808_I2C_ADDRESS, MCP9808_REG_CONFIG, MCP9808_CONFIG_DEFAULT))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_MCP9808;
            else
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_TURN_ON_OPT3001;
            break;
        }
        
        /* MCP9808 wait for turn on */
        case APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_MCP9808:
        {
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                app_deviceData.mcp9808.IsShutdown = false;
                i2cWriteRegComp(MCP9808_I2C_ADDRESS, MCP9808_REG_CONFIG);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_TURN_ON_OPT3001;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C write MCP9808_REG_CONFIG error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_TURN_ON_OPT3001;
            }
            break;
        }
        
        /* OPT3001 turn on */
        case APP_DEVICE_STATE_SENSORS_TURN_ON_OPT3001:
        {
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            app_deviceData.turnOnSensors = false;
            if(i2cWriteReg(OPT3001_I2C_ADDRESS, OPT3001_REG_CONFIG, OPT3001_CONFIG_CONT_CONVERSION))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_OPT3001;
            else
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            break;
        }
        
        /* OPT3001 wait for turn on */
        case APP_DEVICE_STATE_SENSORS_WAIT_TURN_ON_OPT3001:
        {
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                app_deviceData.opt3001.IsShutdown = false;
                i2cWriteRegComp(OPT3001_I2C_ADDRESS, OPT3001_REG_CONFIG);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C write OPT3001_REG_CONFIG error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            }
            break;
        }
        
        /* MCP9808 shutdown (to save power)*/
        case APP_DEVICE_STATE_SENSORS_SHUTDOWN_MCP9808:
        {
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            if(i2cWriteReg(MCP9808_I2C_ADDRESS, MCP9808_REG_CONFIG, MCP9808_CONFIG_SHUTDOWN))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_MCP9808;
            else
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_SHUTDOWN_OPT3001;
            break;
        }
        
        /* MCP9808 wait for shutdown */
        case APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_MCP9808:
        {
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                app_deviceData.mcp9808.IsShutdown = true;
                i2cWriteRegComp(MCP9808_I2C_ADDRESS, MCP9808_REG_CONFIG);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_SHUTDOWN_OPT3001;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C write MCP9808_REG_CONFIG error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_SHUTDOWN_OPT3001;
            }
            break;
        }
        
        /* OPT3001 shutdown (to save power)*/
        case APP_DEVICE_STATE_SENSORS_SHUTDOWN_OPT3001:
        {
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            app_deviceData.turnOnSensors = false;
            if(i2cWriteReg(OPT3001_I2C_ADDRESS, OPT3001_REG_CONFIG, OPT3001_CONFIG_SHUTDOWN))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_OPT3001;
            else
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            break;
        }
        
        /* OPT3001 wait for shutdown */
        case APP_DEVICE_STATE_SENSORS_WAIT_SHUTDOWN_OPT3001:
        {
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                app_deviceData.opt3001.IsShutdown = true;
                i2cWriteRegComp(OPT3001_I2C_ADDRESS, OPT3001_REG_CONFIG);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C write OPT3001_REG_CONFIG error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_TEMP;
            }
            break;
        }
        
        /* MCP9808 read ambient temperature */
        case APP_DEVICE_STATE_SENSORS_READ_TEMP:
        {
            /* Schedule MCP9808 temperature reading */
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            if(i2cReadReg(MCP9808_I2C_ADDRESS, MCP9808_REG_TAMBIENT, 2))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_READ_TEMP;
            else
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_LIGHT;
            break;
        }
        
        /* MCP9808 wait for read ambient temperature */
        case APP_DEVICE_STATE_SENSORS_WAIT_READ_TEMP:
        {
            /* MCP9808 Temperature reading operation done */
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                i2cReadRegComp(MCP9808_I2C_ADDRESS, MCP9808_REG_TAMBIENT);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_LIGHT;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C read temperature error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_READ_LIGHT;
            }
            break;
        }
        
        /* OPT3001 read ambient light */
        case APP_DEVICE_STATE_SENSORS_READ_LIGHT:
        {
            /* Schedule OPT3001 light reading */
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
            if(i2cReadReg(OPT3001_I2C_ADDRESS, OPT3001_REG_RESULT, 2))
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_READ_LIGHT;
            else
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            break;
        }
        
        /* OPT3001 wait for read ambient light */
        case APP_DEVICE_STATE_SENSORS_WAIT_READ_LIGHT:
        {
            /* OPT3001 Light reading operation done */
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                i2cReadRegComp(OPT3001_I2C_ADDRESS, OPT3001_REG_RESULT);
                app_deviceData.state = APP_DEVICE_STATE_SENSORS_CHECK; // APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C read light error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            break;
        }
        
        /* MCP9808 read device ID */
        case APP_DEVICE_STATE_SENSORS_READ_MCP9808_DEV_ID:
        {
            /* Schedule MCP9808 device ID reading */
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
                if(i2cReadReg(MCP9808_I2C_ADDRESS, MCP9808_REG_DEVICE_ID, 2))
                    app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_MCP9808_DEV_ID;
                else
                    app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            break;
        }
        
        /* MCP9808 wait read device ID */
        case APP_DEVICE_STATE_SENSORS_WAIT_MCP9808_DEV_ID:
        {
            /* MCP9808 device ID reading operation done */
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                i2cReadRegComp(MCP9808_I2C_ADDRESS, MCP9808_REG_DEVICE_ID);
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C read MCP9808 device ID error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            break;
        }
        
        /* OPT3001 read device ID */
        case APP_DEVICE_STATE_SENSORS_READ_OPT3001_DEV_ID:
        {
            /* Schedule OPT3001 device ID reading */
            app_deviceData.i2c.transferStatus = I2C_TRANSFER_STATUS_IN_PROGRESS;
                if(i2cReadReg(OPT3001_I2C_ADDRESS, OPT3001_REG_DEVICE_ID, 2))
                    app_deviceData.state = APP_DEVICE_STATE_SENSORS_WAIT_OPT3001_DEV_ID;
                else
                    app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            break;
        }
        
        /* OPT3001 wait read device ID */
        case APP_DEVICE_STATE_SENSORS_WAIT_OPT3001_DEV_ID:
        {
            /* OPT3001 device ID reading operation done */
            if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_SUCCESS){
                i2cReadRegComp(OPT3001_I2C_ADDRESS, OPT3001_REG_DEVICE_ID);
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            else if(app_deviceData.i2c.transferStatus == I2C_TRANSFER_STATUS_ERROR){
                SYS_CONSOLE_PRINT( "I2C read OPT3001 device ID error \r\n");
                app_deviceData.state = APP_DEVICE_STATE_MONITOR_SWITCH1;
            }
            break;
        }
                       
       /* Idle */
        case APP_DEVICE_STATE_IDLE:
        {
            break;
        }
        
        /* Error */
        case APP_DEVICE_STATE_ERROR:
        {
             /* TODO: Handle error in application's state machine. */
            break;
        }         
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }

}


/*******************************************************************************
 End of File
 */
