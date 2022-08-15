/* 
    GY91.h - streamlined library for reading all sensor values from a GY-91 board for MEMS INS purposes
    Created by Husayn Kartodirdjo
*/

#ifndef GY91
#define GY91

#include <Arduino.h>
#include <Wire.h>

#define MPU9250_ADDRESS_AD0_LOW  0x68
#define MPU9250_ADDRESS_AD0_HIGH 0x69

#define ACC_FULL_SCALE_2_G       0x00
#define ACC_FULL_SCALE_4_G       0x08
#define ACC_FULL_SCALE_8_G       0x10
#define ACC_FULL_SCALE_16_G      0x18

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define MAG_MODE_POWERDOWN        0x0
#define MAG_MODE_SINGLE           0x1
#define MAG_MODE_CONTINUOUS_8HZ   0x2
#define MAG_MODE_EXTERNAL         0x4
#define MAG_MODE_CONTINUOUS_100HZ 0x6
#define MAG_MODE_SELFTEST         0x8
#define MAG_MODE_FUSEROM          0xF

#define MPU9250_BUFF_LEN_ACCEL 6
#define MPU9250_BUFF_LEN_GYRO  6
#define MPU9250_BUFF_LEN_MAG   7

#define BMP280_ADDRESS (0x77)       /**< The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT (0x76)   /**< Alternative I2C address for the sensor. */
#define BMP280_CHIPID (0x58)        /**< Default chip ID. */

enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};
/** Oversampling rate for the sensor. */
enum sensor_sampling {   
    SAMPLING_NONE = 0x00, /** No over-sampling. */
    SAMPLING_X1 = 0x01, /** 1x over-sampling. */
    SAMPLING_X2 = 0x02, /** 2x over-sampling. */
    SAMPLING_X4 = 0x03, /** 4x over-sampling. */
    SAMPLING_X8 = 0x04, /** 8x over-sampling. */
    SAMPLING_X16 = 0x05 /** 16x over-sampling. */
};
/** Operating mode for the sensor. */
enum sensor_mode {
    MODE_SLEEP = 0x00, /** Sleep mode. */
    MODE_FORCED = 0x01, /** Forced mode. */
    MODE_NORMAL = 0x03, /** Normal mode. */
    MODE_SOFT_RESET_CODE = 0xB6 /** Software reset. */
};
/** Filtering level for sensor data. */
enum sensor_filter {
    FILTER_OFF = 0x00, /** No filtering. */
    FILTER_X2 = 0x01, /** 2x filtering. */
    FILTER_X4 = 0x02, /** 4x filtering. */
    FILTER_X8 = 0x03, /** 8x filtering. */
    FILTER_X16 = 0x04 /** 16x filtering. */
};
/** Standby duration in ms */
enum standby_duration {
    STANDBY_MS_1 = 0x00, /** 1 ms standby. */
    STANDBY_MS_63 = 0x01, /** 62.5 ms standby. */
    STANDBY_MS_125 = 0x02, /** 125 ms standby. */
    STANDBY_MS_250 = 0x03, /** 250 ms standby. */
    STANDBY_MS_500 = 0x04, /** 500 ms standby. */
    STANDBY_MS_1000 = 0x05, /** 1000 ms standby. */
    STANDBY_MS_2000 = 0x06, /** 2000 ms standby. */
    STANDBY_MS_4000 = 0x07 /** 4000 ms standby. */
};

class GY91
{
private:
    TwoWire* myWire;
    float accelRange;
    float gyroRange;
    uint8_t magXAdjust, magYAdjust, magZAdjust;
    void beginWireIfNull();
    float accelGet(uint8_t highIndex, uint8_t lowIndex);
    float gyroGet(uint8_t highIndex, uint8_t lowIndex);
    int16_t magGet(uint8_t highIndex, uint8_t lowIndex);
    void magEnableSlaveMode();
    void magReadAdjustValues();
    void magWakeup();
    uint8_t i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    uint8_t i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
public:
    GY91(uint8_t address = MPU9250_ADDRESS_AD0_LOW);
    const uint8_t address;
    int16_t magXOffset, magYOffset, magZOffset;
    uint8_t accelBuff[MPU9250_BUFF_LEN_ACCEL];
    uint8_t gyroBuff[MPU9250_BUFF_LEN_GYRO];
    uint8_t magBuff[MPU9250_BUFF_LEN_MAG];
    
    void setWire(TwoWire *wire);
    uint8_t readId(uint8_t *id);

    void beginAccel(uint8_t mode = ACC_FULL_SCALE_16_G);
    uint8_t accelUpdate();
    float accelX();
    float accelY();
    float accelZ();

    void beginGyro(uint8_t mode = GYRO_FULL_SCALE_2000_DPS);
    uint8_t gyroUpdate();
    float gyroX();
    float gyroY();
    float gyroZ();

    void beginMag(uint8_t mode = MAG_MODE_CONTINUOUS_8HZ);
    void magSetMode(uint8_t mode);
    uint8_t magUpdate();
    float magX();
    float magY();
    float magZ();

    double getTemperature
};

GY91::GY91(/* args */)
{
}

GY91::~GY91()
{
}


































#endif