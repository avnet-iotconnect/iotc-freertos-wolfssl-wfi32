//
// Copyright: Avnet 2024
// Created by Shu Liu <shu.liu@avnet.com> on 10/17/24.
//
#include <stdio.h>
#include "definitions.h"
#include "sensors.h"

//Struct to hold boolean connection values of supported click boards
typedef struct {  
  bool air7;
  bool alt2;
  bool alt4;
  bool pht;
  bool t6713;
  bool t9602;
  bool temphum14;
  bool ulp;
  bool vav;     
} click_board_connections;

//Declare connection struct and initialize all values to false
static click_board_connections click_board_detection = {0};
static alt2_data_struct alt2_data;
static air7_data_struct air7_data;
static alt4_data_struct alt4_data;
static pht_data_struct pht_data;
static t6713_data_struct t6713_data;
static t9602_data_struct t9602_data;
static temphum14_data_struct temphum14_data;
static ulp_data_struct ulp_data;
static vav_data_struct vav_data;



    
void check_click_sensors (void) {
     
    SYS_CONSOLE_PRINT("Detecting attached Click Board sensors...\r\n");

    /* mikroBUS RST (J402 pin 2) is RC12 and INT (pin 15) is RA13 - see the
     * WFI32-IoT user guide Table 3-1. GPIO_Initialize leaves both as outputs
     * driven low, so the click is held in reset and its interrupt line is
     * being fought by the MCU. Pulse reset properly and free the INT line.
     * The pin states are printed so this is verifiable rather than assumed. */
    SYS_CONSOLE_PRINT("mikroBUS RST(RC12) at entry = %d\r\n", GPIO_PinRead(RST_PIN) ? 1 : 0);

    /* Note: do NOT reconfigure RA13 here. It is mikroBUS INT, but this board
     * also relies on it being driven as an output - making it an input breaks
     * the Wi-Fi association. */

    GPIO_PinClear(RST_PIN);                 /* assert reset  */
    vTaskDelay(20);
    GPIO_PinSet(RST_PIN);                   /* release reset */
    vTaskDelay(250);                        /* let the click boot */

    SYS_CONSOLE_PRINT("mikroBUS RST(RC12) after release = %d\r\n", GPIO_PinRead(RST_PIN) ? 1 : 0);

    /* Bus scan first: the click probes below only report a device they can
     * already talk to, so without this we cannot tell an undetected click from
     * one sitting on a bus the driver does not own. */
    {
        uint8_t addr;

        /* Per the WFI32-IoT user guide (DS50003262) Table 3-1, mikroBUS SDA/SCL
         * are SDA1/RPA5 and SCL1/RPA4 - the same pins as the on-board MCP9808
         * (0x18) and OPT3001 (0x44). So a click must appear on I2C1, and those
         * two sensors are the built-in proof the scan works. */
        SYS_CONSOLE_PRINT("I2C1 scan (mikroBUS + on-board sensors):\r\n");
        for (addr = 0x08; addr < 0x78; addr++) {
            if (APP_SENSORS_probe(addr)) {
                SYS_CONSOLE_PRINT("  I2C1 ack 0x%02X\r\n", addr);
            }
        }

        static const uint8_t clickAddrs[] = {
            0x15,   /* T6713            */
            0x27,   /* Altitude 4       */
            0x28,   /* T9602            */
            0x40,   /* TempHum14 / PHT  */
            0x41,   /* TempHum14 (VCC)  */
            0x5C,   /* VAV Press        */
            0x6C,   /* Ultra Low Press  */
            0x70,   /* Air Quality 7    */
            0x76,   /* Altitude 2 / PHT */
            0x77,   /* Altitude 2 alt   */
        };
        uint8_t i;

        /* Probe each click address five times; a real device answers every
         * time. 0x18 and 0x44 are probed too as a built-in positive control -
         * if those are not 5/5 the scan itself is not to be trusted. */
        SYS_CONSOLE_PRINT("I2C1 hits out of 5:\r\n");
        for (i = 0; i < sizeof(clickAddrs); i++) {
            uint8_t n, hits = 0;
            for (n = 0; n < 5; n++) {
                vTaskDelay(5);
                if (APP_SENSORS_probe(clickAddrs[i])) { hits++; }
            }
            SYS_CONSOLE_PRINT("  click 0x%02X  %d/5\r\n", clickAddrs[i], hits);
        }
        {
            uint8_t n, h18 = 0, h44 = 0;
            for (n = 0; n < 5; n++) {
                vTaskDelay(5);
                if (APP_SENSORS_probe(0x18)) { h18++; }
                vTaskDelay(5);
                if (APP_SENSORS_probe(0x44)) { h44++; }
            }
            SYS_CONSOLE_PRINT("  control MCP9808 0x18 %d/5, OPT3001 0x44 %d/5\r\n", h18, h44);
        }
        SYS_CONSOLE_PRINT("scan done\r\n");
    }

    //AQ7
    //air7_data_struct air7_data;
    AIRQUALITY7_readData(&air7_data);
    //If calibration values is non-zero, it is connected
    if (0 != air7_data.resistor_val) {
        SYS_CONSOLE_PRINT("Air Quality 7 Click Detected!\r\n");
        click_board_detection.air7 = true;
    }
    
    //ALT2
    //alt2_data_struct alt2_data;
    ALTITUDE2_cal_vals(&alt2_data);
    //If a calibration values is non-zero, it is connected
    if (0 != alt2_data.cal_1) {
        SYS_CONSOLE_PRINT("Altitude 2 Click Detected!\r\n");
        click_board_detection.alt2 = true;
    }
    
    //ALT4
    //alt4_data_struct alt4_data;
    ALTITUDE4_readData(&alt4_data);
    //If a pressure value is not 260 or a temp value is not -40, it is connected
    //(+260 and -40 correspond to zero-readings on ALT4)
    if (260 != alt4_data.pressure || -40 != alt4_data.temperature) {
        SYS_CONSOLE_PRINT("Altitude 4 Click Detected!\r\n");
        click_board_detection.alt4 = true;
    }
    
    //PHT
    //pht_data_struct pht_data;
    PHT_cal_vals(&pht_data);
    //If a calibration values is non-zero, it is connected
    if (0 != pht_data.cal_1) {
        SYS_CONSOLE_PRINT("PHT Click Detected!\r\n");
        click_board_detection.pht = true;
    }
    
    //T6713
    //t6713_data_struct t6713_data;
    T6713_readData(&t6713_data);
    //If a CO2 measurement is non-zero, it is connected
    if (0 != t6713_data.co2) {
        SYS_CONSOLE_PRINT("T6713 CO2 Sensor Detected!\r\n");
        T6713_calibrate();
        click_board_detection.t6713 = true;
    }
    
    //T9602
    //t9602_data_struct t9602_data;
    //If humidity or temperature measurements are non-zero, it is connected
    //(-40 corresponds to a zero reading for temperature)
    T9602_readData(&t9602_data);
    if (0 != t9602_data.humidity || -40 != t9602_data.temperature) {
        SYS_CONSOLE_PRINT("T9602 TempHum Sensor Detected!\r\n");
        click_board_detection.t9602 = true;
    }
    
    //TempHum14
    uint32_t temphum14_serial_number = 0;
    //If the serial number is non-zero, it is connected
    temphum14_serial_number = TEMPHUM14_init(0x40);
    SYS_CONSOLE_PRINT("TempHum14 probe @0x40 -> serial 0x%06X\r\n", (unsigned int)temphum14_serial_number);
    if (0 != temphum14_serial_number) {
        SYS_CONSOLE_PRINT("TempHum14 Click Detected!\r\n");
        click_board_detection.temphum14 = true;
    }
    
    //ULP
    uint32_t ulp_serial_number = 0;
    //If serial number is non-zero, it is connected
    ulp_serial_number = ULTRALOWPRESS_init();
    if (0 != ulp_serial_number) {
        SYS_CONSOLE_PRINT("Ultra Low Press Click Detected!\r\n");
        click_board_detection.ulp = true;
    }
    
    //VAVpress
    uint16_t vav_cal_ID = 0;
    //If calibration ID is non-zero, it is connected
    vav_cal_ID = VAVPRESS_init();
    if (0 != vav_cal_ID) {
        SYS_CONSOLE_PRINT("VAVPress Click Detected!\r\n");
        click_board_detection.vav = true;
    }
}

void read_click_sensors (void) {
    
        //Air Quality 7 Click Reading
    if (click_board_detection.air7 == true) {
        AIRQUALITY7_readData(&air7_data);
    }
    
        //Altitude 2 Click Reading
    if (click_board_detection.alt2 == true) {
        //alt2_data_struct alt2_data;
        //Read the calibration values form the sensor
        ALTITUDE2_cal_vals(&alt2_data);
        //Take measurements from sensor using the calibration values as multipliers
        ALTITUDE2_readData(&alt2_data);
        //Report data pulled from readData function output
        //iotcl_telemetry_set_number(msg, "ALT2_Temp_DegC", alt2_data.temperature);
        //iotcl_telemetry_set_number(msg, "ALT2_Pressure_mBar", alt2_data.pressure);
        //iotcl_telemetry_set_number(msg, "ALT2_Altitude_m", alt2_data.altitude);
    } 
    
    
    //Altitude 4 Click Reading
    if (click_board_detection.alt4 == true) {
        //alt4_data_struct alt4_data;
        ALTITUDE4_readData(&alt4_data);
        //Report data pulled from readData function output
        //iotcl_telemetry_set_number(msg, "ALT4_Temp_DegC", alt4_data.temperature);
        //iotcl_telemetry_set_number(msg, "ALT4_Pressure_mBar", alt4_data.pressure);   
        //iotcl_telemetry_set_number(msg, "ALT4_Altitude_m", alt4_data.altitude);
    }
    
    
    //PHT Click Reading
    if (click_board_detection.pht == true) {
        //pht_data_struct pht_data;
        //Read the calibration values form the sensor
        PHT_cal_vals(&pht_data);
        //Take measurements from sensor using the calibration values as multipliers
        PHT_readData(&pht_data);
        //Report data pulled from readData function output
        //iotcl_telemetry_set_number(msg, "PHT_Temp_DegC", pht_data.temperature);
        //iotcl_telemetry_set_number(msg, "PHT_Pressure_mBar", pht_data.pressure);  
        //iotcl_telemetry_set_number(msg, "PHT_Humidity_Percent", pht_data.humidity);
    } 
    
    
    //T6713-6H Proto Click Reading
    if (click_board_detection.t6713 == true) {
        //t6713_data_struct t6713_data;
        T6713_readData(&t6713_data);
        //If the sensor has been warming up for at least 10 minutes
        /*
        if (t6713_data.co2 != 0) {
            //Report status and data from readData function output
            iotcl_telemetry_set_string(msg, "T6713_Status", "READY");
            iotcl_telemetry_set_number(msg, "T6713_CO2_ppm", t6713_data.co2);
        } else {
            //Report status and zeros for data
            iotcl_telemetry_set_string(msg, "T6713_Status", "WARMING_UP");
            iotcl_telemetry_set_number(msg, "T6713_CO2_ppm", 0);
        }
        */
    }

    
    //T9602 Terminal 2 Click Reading
    if (click_board_detection.t9602 == true) {
        //t9602_data_struct t9602_data;
        T9602_readData(&t9602_data);
        //If the sensor has been warming up for at least 2 minutes
        /*
        if (t9602_data.humidity != 0 && t9602_data.temperature != 0) {
            //Report status and data from readData function output
            iotcl_telemetry_set_string(msg, "T9602_Status", "READY");
            iotcl_telemetry_set_number(msg, "T9602_Humidity_Percent", t9602_data.humidity);
            iotcl_telemetry_set_number(msg, "T9602_Temp_DegC", t9602_data.temperature);
        } else {
            //Report status and zeros for data
            iotcl_telemetry_set_string(msg, "T9602_Status", "WARMING_UP");
            iotcl_telemetry_set_number(msg, "T9602_Humidity_Percent", 0);
            iotcl_telemetry_set_number(msg, "T9602_Temp_DegC", 0);
        }
        */
    }
    
    
    //TempHum14 Click Reading
    if (click_board_detection.temphum14 == true) {
        //temphum14_data_struct temphum14_data;
        TEMPHUM14_setConversion(TEMPHUM14_I2C_SLAVE_ADDR_GND, TEMPHUM14_CONVERSION_HUM_OSR_0_020, TEMPHUM14_CONVERSION_TEMP_0_040 );
        TEMPHUM14_getTemperatureHumidity (TEMPHUM14_I2C_SLAVE_ADDR_GND, &temphum14_data);

        //Report data from readData function output
        //iotcl_telemetry_set_number(msg, "TH14_Humidity_Percent", temphum14_data.humidity);
        //iotcl_telemetry_set_number(msg, "TH14_Temp_DegC", temphum14_data.temperature);
    }
    
    
    //ULP Click Reading
    if (click_board_detection.ulp == true) {
        //ulp_data_struct ulp_data;
        if (ULTRALOWPRESS_isReady()) 
        {
            ULTRALOWPRESS_getData(&ulp_data);
            //Report data from getData function output
            //iotcl_telemetry_set_number(msg, "ULP_Temp_DegC", ulp_data.temperature);
            //iotcl_telemetry_set_number(msg, "ULP_Pressure_Pa", ulp_data.pressure);
        }
    }
    
    
    //VAVPress Click Reading
    if (click_board_detection.vav == true) {
        //vav_data_struct vav_data;
        VAVPRESS_getSensorReadings(&vav_data);
        //Report data from getSensorReadings function output
        //iotcl_telemetry_set_number(msg, "VAV_Pressure_Pa", vav_data.pressure);
        //iotcl_telemetry_set_number(msg, "VAV_Temp_DegC", vav_data.temperature);
    }
}

void add_sensor_data_to_telemetry(IotclMessageHandle msg) {
    
        //Air Quality 7 Click Reading
    if (click_board_detection.air7 == true) {
        
        if (air7_data.tvoc != 0 && air7_data.co2 != 0) {
            //Report status and data from readData function output
            iotcl_telemetry_set_string(msg, "Amp_AIR7_Status", "READY");
            iotcl_telemetry_set_number(msg, "Amp_AIR7_tVOC_ppb", air7_data.tvoc);   
            iotcl_telemetry_set_number(msg, "Amp_AIR7_CO2_ppm", air7_data.co2);
        } else {
            //Report status and zeros for data
            iotcl_telemetry_set_string(msg, "Amp_AIR7_Status", "WARMING_UP");
            iotcl_telemetry_set_number(msg, "Amp_AIR7_tVOC_ppb", 0);   
            iotcl_telemetry_set_number(msg, "Amp_AIR7_CO2_ppm", 0);
        }
    }
        //Altitude 2 Click Reading
    if (click_board_detection.alt2 == true) {
        //alt2_data_struct alt2_data;
        //Read the calibration values form the sensor
        //ALTITUDE2_cal_vals(&alt2_data);
        //Take measurements from sensor using the calibration values as multipliers
        //ALTITUDE2_readData(&alt2_data);
        //Report data pulled from readData function output
        iotcl_telemetry_set_number(msg, "TE_ALT2_Temp_DegC", alt2_data.temperature);
        iotcl_telemetry_set_number(msg, "TE_ALT2_Pressure_mBar", alt2_data.pressure);
        iotcl_telemetry_set_number(msg, "TE_ALT2_Altitude_m", alt2_data.altitude);
    } 
    
    
    //Altitude 4 Click Reading
    if (click_board_detection.alt4 == true) {
        /* No local alt4_data here: declaring one shadowed the file-scope struct
         * that read_click_sensors() fills, so this reported uninitialised stack
         * instead of the real measurement. */
        //ALTITUDE4_readData(&alt4_data);
        //Report data pulled from readData function output
        iotcl_telemetry_set_number(msg, "TE_ALT4_Temp_DegC", alt4_data.temperature);
        iotcl_telemetry_set_number(msg, "TE_ALT4_Pressure_mBar", alt4_data.pressure);   
        iotcl_telemetry_set_number(msg, "TE_ALT4_Altitude_m", alt4_data.altitude);
    }
    
    
    //PHT Click Reading
    if (click_board_detection.pht == true) {
        //pht_data_struct pht_data;
        //Read the calibration values form the sensor
        //PHT_cal_vals(&pht_data);
        //Take measurements from sensor using the calibration values as multipliers
        //PHT_readData(&pht_data);
        //Report data pulled from readData function output
        iotcl_telemetry_set_number(msg, "TE_PHT_Temp_DegC", pht_data.temperature);
        iotcl_telemetry_set_number(msg, "TE_PHT_Pressure_mBar", pht_data.pressure);  
        iotcl_telemetry_set_number(msg, "TE_PHT_Humidity_Percent", pht_data.humidity);
    } 
    
    
    //T6713-6H Proto Click Reading
    if (click_board_detection.t6713 == true) {
        //t6713_data_struct t6713_data;
        //T6713_readData(&t6713_data);
        //If the sensor has been warming up for at least 10 minutes
        if (t6713_data.co2 != 0) {
            //Report status and data from readData function output
            iotcl_telemetry_set_string(msg, "TEL_T6713_Status", "READY");
            iotcl_telemetry_set_number(msg, "TEL_T6713_CO2_ppm", t6713_data.co2);
        } else {
            //Report status and zeros for data
            iotcl_telemetry_set_string(msg, "TEL_T6713_Status", "WARMING_UP");
            iotcl_telemetry_set_number(msg, "TEL_T6713_CO2_ppm", 0);
        }
    }

    
    //T9602 Terminal 2 Click Reading
    if (click_board_detection.t9602 == true) {
        //t9602_data_struct t9602_data;
        //T9602_readData(&t9602_data);
        //If the sensor has been warming up for at least 2 minutes
        if (t9602_data.humidity != 0 && t9602_data.temperature != 0) {
            //Report status and data from readData function output
            iotcl_telemetry_set_string(msg, "TEL_T9602_Status", "READY");
            iotcl_telemetry_set_number(msg, "TEL_T9602_Humidity_Percent", t9602_data.humidity);
            iotcl_telemetry_set_number(msg, "TEL_T9602_Temp_DegC", t9602_data.temperature);
        } else {
            //Report status and zeros for data
            iotcl_telemetry_set_string(msg, "TEL_T9602_Status", "WARMING_UP");
            iotcl_telemetry_set_number(msg, "TEL_T9602_Humidity_Percent", 0);
            iotcl_telemetry_set_number(msg, "TEL_T9602_Temp_DegC", 0);
        }
    }
    
    
    //TempHum14 Click Reading
    if (click_board_detection.temphum14 == true) {
        //temphum14_data_struct temphum14_data;
        //TEMPHUM14_setConversion(TEMPHUM14_I2C_SLAVE_ADDR_GND, TEMPHUM14_CONVERSION_HUM_OSR_0_020, TEMPHUM14_CONVERSION_TEMP_0_040 );
        //TEMPHUM14_getTemperatureHumidity (TEMPHUM14_I2C_SLAVE_ADDR_GND, &temphum14_data);

        //Report data from readData function output
        iotcl_telemetry_set_number(msg, "TE_TH14_Humidity_Percent", temphum14_data.humidity);
        iotcl_telemetry_set_number(msg, "TE_TH14_Temp_DegC", temphum14_data.temperature);
    }
    
    
    //ULP Click Reading
    if (click_board_detection.ulp == true) {
        //ulp_data_struct ulp_data;
        //if (ULTRALOWPRESS_isReady()) 
        //{
            //ULTRALOWPRESS_getData(&ulp_data);
            //Report data from getData function output
            iotcl_telemetry_set_number(msg, "TE_ULP_Temp_DegC", ulp_data.temperature);
            iotcl_telemetry_set_number(msg, "TE_ULP_Pressure_Pa", ulp_data.pressure);
        //}
    }
    
    
    //VAVPress Click Reading
    if (click_board_detection.vav == true) {
        //vav_data_struct vav_data;
        //VAVPRESS_getSensorReadings(&vav_data);
        //Report data from getSensorReadings function output
        iotcl_telemetry_set_number(msg, "TE_VAV_Pressure_Pa", vav_data.pressure);
        iotcl_telemetry_set_number(msg, "TE_VAV_Temp_DegC", vav_data.temperature);
    }
}