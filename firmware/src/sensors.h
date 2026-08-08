#ifndef _SENSORS_H
#define _SENSORS_H

// *****************************************************************************
// *****************************************************************************
// Section: Definitions
// *****************************************************************************
// *****************************************************************************
#define WFI32IOT_SENSORS
#define CLICK_ALTITUDE4
#define CLICK_ALTITUDE2
#define CLICK_PHT
#define CLICK_TEMPHUM14
#define CLICK_ULTRALOWPRESS
#define CLICK_T6713
#define CLICK_AIRQUALITY7
#define CLICK_T9602
#define CLICK_VAVPRESS
#define SEND_LED_PROPERTIES_WITH_TELEMETRY
//#define PNP_CERTIFICATION_TESTING

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#ifdef CLICK_ALTITUDE4
    #include "./clicks/altitude4.h"
#endif

#ifdef CLICK_ALTITUDE2
    #include "./clicks/altitude2.h"
#endif
#ifdef CLICK_PHT
    #include "./clicks/pht.h"
#endif
#ifdef CLICK_TEMPHUM14
    #include "./clicks/temphum14.h"
#endif
#ifdef CLICK_ULTRALOWPRESS
    #include "./clicks/ultralowpress.h"
#endif
#ifdef CLICK_T6713
    #include "./clicks/T6713.h"
#endif
#ifdef CLICK_AIRQUALITY7
    #include "./clicks/airquality7.h"
#endif 
#ifdef CLICK_T9602
    #include "./clicks/T9602.h"
#endif 
#ifdef CLICK_VAVPRESS
    #include "./clicks/vavpress.h"
#endif

void check_click_sensors(void);
void read_click_sensors(void);
void add_sensor_data_to_telemetry(IotclMessageHandle);
#endif //_SENSORS_H