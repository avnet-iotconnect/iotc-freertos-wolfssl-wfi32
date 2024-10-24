#include "pht.h"

extern APP_DEVICE_DATA app_deviceData;

// --------------------------------------------------------- PUBLIC FUNCTIONS 
//This function simply resets the sensor and reloads the calibration values
void PHT_reset()
{
    APP_SENSORS_writeByte(PHT_I2C_SLAVE_ADDR_P_AND_T, 0x1E);
}

//This function reads the calibration values from the PROM
//and updates the array parameter with those values
void PHT_cal_vals(pht_data_struct *pht_data)
{
    //Clear the i2c buffer
    for (int index = 0; index < 10; index++) {
        app_deviceData.i2c.rxBuffBytes[index] = 0;
    }
    
    //Read the pressure sensitivity prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xA2, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c1 = app_deviceData.i2c.rxBuffBytes[0];
    c1 = c1 << 8;
    c1 = c1 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_1 = c1;
    
    //Read the pressure offset prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xA4, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c2 = app_deviceData.i2c.rxBuffBytes[0];
    c2 = c2 << 8;
    c2 = c2 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_2 = c2;
    
    //Read the temperature coefficient of pressure sensitivity prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xA6, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c3 = app_deviceData.i2c.rxBuffBytes[0];
    c3 = c3 << 8;
    c3 = c3 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_3 = c3;
    
    //Read the temperature coefficient of pressure offset prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xA8, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c4 = app_deviceData.i2c.rxBuffBytes[0];
    c4 = c4 << 8;
    c4 = c4 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_4 = c4;
    
    //Read the reference temperature prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xAA, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c5 = app_deviceData.i2c.rxBuffBytes[0];
    c5 = c5 << 8;
    c5 = c5 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_5 = c5;
    
    //Read the temperature coefficient prom value
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0xAC, 2);
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    uint16_t c6 = app_deviceData.i2c.rxBuffBytes[0];
    c6 = c6 << 8;
    c6 = c6 | app_deviceData.i2c.rxBuffBytes[1];
    pht_data->cal_6 = c6;
}

//This function reads the temperature and pressure
//data from the sensor and uses the calibration values
//to scale the data and update the data array parameter
void PHT_readData(pht_data_struct *pht_data)
{
    //Clear the i2c buffer
    for (int index = 0; index < 10; index++) {
        app_deviceData.i2c.rxBuffBytes[index] = 0;
    }
    
    //Send measure pressure command
    APP_SENSORS_writeByte(PHT_I2C_SLAVE_ADDR_P_AND_T, 0x46);
    
    //delay thread for 10ms so registers can update
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
    //Read the measurement registers and puts the data into rx.BuffBytes
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0x00, 3);
    
    //delay thread for 10ms so registers can update
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
    //Extract bytes and bit-shift them to create 2-byte pressure reading
    uint32_t raw_pressure = app_deviceData.i2c.rxBuffBytes[0];
    raw_pressure = raw_pressure << 8;
    raw_pressure = raw_pressure | app_deviceData.i2c.rxBuffBytes[1];
    raw_pressure = raw_pressure << 8;
    raw_pressure = raw_pressure | app_deviceData.i2c.rxBuffBytes[2];
    
    
    //Send measure pressure command
    APP_SENSORS_writeByte(PHT_I2C_SLAVE_ADDR_P_AND_T, 0x56);
    
    //delay thread for 10ms so registers can update
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
    //Read the measurement registers and puts the data into rx.BuffBytes
    APP_SENSORS_writeReadBytes(PHT_I2C_SLAVE_ADDR_P_AND_T, 0x00, 3);
    
    //delay thread for 10ms so registers can update
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    
    //Extract bytes and bit-shift them to create 2-byte pressure reading
    uint32_t raw_temperature = app_deviceData.i2c.rxBuffBytes[0];
    raw_temperature = raw_temperature << 8;
    raw_temperature = raw_temperature | app_deviceData.i2c.rxBuffBytes[1];
    raw_temperature = raw_temperature << 8;
    raw_temperature = raw_temperature | app_deviceData.i2c.rxBuffBytes[2];
    
    //Calculate relative values from readings and prom cal values
    float dT = raw_temperature - (pht_data->cal_5 * 256.0);
    float corrected_temperature = 2000.0 + (dT * pht_data->cal_6 / 8388608.0);
    float offset = (pht_data->cal_2 * 131072.0 ) + ((pht_data->cal_4 * dT) / 64.0);
    float sensitivity = (pht_data->cal_1 * 65536.0) + ((pht_data->cal_3 * dT) / 128.0);
    float corrected_pressure = (((raw_pressure * sensitivity) / 2097152.0) - offset) / 32768.0;
    
    //Update the passed-in variables with temperature and pressure values
    pht_data->pressure = corrected_pressure / 100.0;
    pht_data->temperature = corrected_temperature / 100.0;
    
    
    //Relative humidity measurement and calculations
   
    APP_SENSORS_writeByte(PHT_I2C_SLAVE_ADDR_RH, PHT_RH_MEASURE_RH_HOLD);
    
    //delay thread for 20ms so registers can update
    vTaskDelay( 20 / portTICK_PERIOD_MS );
    
    APP_SENSORS_justRead(PHT_I2C_SLAVE_ADDR_RH, 2);
        
    uint32_t raw_humidity = app_deviceData.i2c.rxBuffBytes[0];
    raw_humidity = raw_humidity << 8;
    raw_humidity = raw_humidity | app_deviceData.i2c.rxBuffBytes[1];
    float corrected_humidity = (float)raw_humidity;
    corrected_humidity *= 12500.0;
    corrected_humidity /= 65536.0;
    corrected_humidity -= 600.0;
    corrected_humidity /= 100.0;

    pht_data->humidity = corrected_humidity;
}
