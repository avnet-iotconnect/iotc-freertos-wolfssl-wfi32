#include "T9602.h"

extern APP_DEVICE_DATA app_deviceData;

void T9602_readData(t9602_data_struct *t9602_data)
{
    //Clear the i2c buffer
    for (int index = 0; index < 10; index++) {
        app_deviceData.i2c.rxBuffBytes[index] = 0;
    }
    
    //Send command to read the sensor values
    APP_SENSORS_justRead(0x28, 4);
    
    //delay thread for 100ms so sensor can update data
    vTaskDelay( 100 / portTICK_PERIOD_MS );
    
    //Extracting Relative Humidity Data
    uint16_t raw_humidity = 0x0000;
    raw_humidity += app_deviceData.i2c.rxBuffBytes[0];
    raw_humidity &= 63;
    raw_humidity *= 256;
    raw_humidity += app_deviceData.i2c.rxBuffBytes[1];
    float corrected_humidity = (float)raw_humidity;
    corrected_humidity /= 16384;
    corrected_humidity *= 100;
    t9602_data->humidity = corrected_humidity;
    
    //Extracting Temperature Data
    uint16_t raw_temperature = 0x0000;
    raw_temperature += app_deviceData.i2c.rxBuffBytes[2];
    raw_temperature *= 64;
    raw_temperature += (app_deviceData.i2c.rxBuffBytes[3] >> 2);
    float corrected_temperature = (float)raw_temperature;
    corrected_temperature /= 16384;
    corrected_temperature *= 165;
    corrected_temperature -= 40;
    t9602_data->temperature = corrected_temperature;
}
