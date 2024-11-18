//
// Copyright: Avnet 2024
// Created by Shu Liu <shu.liu@avnet.com> on 10/17/24.
//
#include <stdio.h>
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
     
    printf("Detecting attached Click Board sensors...\r\n");
    
    //AQ7
    //air7_data_struct air7_data;
    AIRQUALITY7_readData(&air7_data);
    //If calibration values is non-zero, it is connected
    if (0 != air7_data.resistor_val) {
        printf("Air Quality 7 Click Detected!\r\n");
        click_board_detection.air7 = true;
    }
    
    //ALT2
    //alt2_data_struct alt2_data;
    ALTITUDE2_cal_vals(&alt2_data);
    //If a calibration values is non-zero, it is connected
    if (0 != alt2_data.cal_1) {
        printf("Altitude 2 Click Detected!\r\n");
        click_board_detection.alt2 = true;
    }
    
    //ALT4
    //alt4_data_struct alt4_data;
    ALTITUDE4_readData(&alt4_data);
    //If a pressure value is not 260 or a temp value is not -40, it is connected
    //(+260 and -40 correspond to zero-readings on ALT4)
    if (260 != alt4_data.pressure || -40 != alt4_data.temperature) {
        printf("Altitude 4 Click Detected!\r\n");
        click_board_detection.alt4 = true;
    }
    
    //PHT
    //pht_data_struct pht_data;
    PHT_cal_vals(&pht_data);
    //If a calibration values is non-zero, it is connected
    if (0 != pht_data.cal_1) {
        printf("PHT Click Detected!\r\n");
        click_board_detection.pht = true;
    }
    
    //T6713
    //t6713_data_struct t6713_data;
    T6713_readData(&t6713_data);
    //If a CO2 measurement is non-zero, it is connected
    if (0 != t6713_data.co2) {
        printf("T6713 CO2 Sensor Detected!\r\n");
        T6713_calibrate();
        click_board_detection.t6713 = true;
    }
    
    //T9602
    //t9602_data_struct t9602_data;
    //If humidity or temperature measurements are non-zero, it is connected
    //(-40 corresponds to a zero reading for temperature)
    T9602_readData(&t9602_data);
    if (0 != t9602_data.humidity || -40 != t9602_data.temperature) {
        printf("T9602 TempHum Sensor Detected!\r\n");
        click_board_detection.t9602 = true;
    }
    
    //TempHum14
    uint32_t temphum14_serial_number = 0;
    //If the serial number is non-zero, it is connected
    temphum14_serial_number = TEMPHUM14_init(0x40);
    if (0 != temphum14_serial_number) {
        printf("TempHum14 Click Detected!\r\n");
        click_board_detection.temphum14 = true;
    }
    
    //ULP
    uint32_t ulp_serial_number = 0;
    //If serial number is non-zero, it is connected
    ulp_serial_number = ULTRALOWPRESS_init();
    if (0 != ulp_serial_number) {
        printf("Ultra Low Press Click Detected!\r\n");
        click_board_detection.ulp = true;
    }
    
    //VAVpress
    uint16_t vav_cal_ID = 0;
    //If calibration ID is non-zero, it is connected
    vav_cal_ID = VAVPRESS_init();
    if (0 != vav_cal_ID) {
        printf("VAVPress Click Detected!\r\n");
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
        alt4_data_struct alt4_data;
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