#include "T6713.h"

extern APP_DEVICE_DATA app_deviceData;

//This function reads the CO2 data from
//the sensor and updates the data array parameter
void T6713_readData(t6713_data_struct *t6713_data)
{
    //Clear the i2c buffer
    for (int index = 0; index < 10; index++) {
        app_deviceData.i2c.rxBuffBytes[index] = 0;
    }
    
    //Send command to initiate measurement
    uint8_t send_data[5] = {T6713_CMD_GET_PPM, 0x13, 0x8B, 0x00, 0x01};
    APP_SENSORS_write(T6713_DEV_ADDR , send_data, 5);
    
    //delay thread for 100ms so sensor can update data
    vTaskDelay( 100 / portTICK_PERIOD_MS );
    
    //Send command to read the sensor values
    APP_SENSORS_justRead(T6713_DEV_ADDR, 4);
    
    //Extracting CO2 data
    uint16_t calculated_co2 = 0;
    calculated_co2 = (app_deviceData.i2c.rxBuffBytes[2] * 256) + app_deviceData.i2c.rxBuffBytes[3];
    t6713_data->co2 = (int)calculated_co2;
    
}

//This function is used to calibrate the sensor if the readings are wrong.
//YOU DO NOT NEED TO CHANGE THE SENT VALUE, it automatically knows what to do.
void T6713_calibrate()
{
    app_deviceData.i2c.rxBuffBytes[0] = 0;
    
    uint8_t send_data[5] = {0x05, 0x03, 0xEC, 0xFF, 0x00};
    
    APP_SENSORS_write(T6713_DEV_ADDR , send_data, 5);
    
}

