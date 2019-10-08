/*
MPU6050.cpp - Class file for the MPU6050 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: 1.0.3
(c) 2014-2015 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */




#include <math.h>
#include <stdbool.h>
#include "MPU6050.h"
#include "Timers.h"
#include <stdio.h>
#include "Definitions.h"
#include "system_config/default/framework/driver/i2c/drv_i2c_static_buffer_model.h"
float timeStep = 0.005;
unsigned long prevtime;
unsigned long time;
float yaw = 0;
float roll = 0;
float pitch = 0;
timers_t sec, ms10, ledTimeout;
timers_t printTimer;
timers_t IMU_UpdateTimer;
timers_t timeOut;
bool isWithinFloat(double sample, double lowBound, double highBound);
void isTimedOut();

void isTimedOut() {
    if (timerDone(&timeOut)) {
        while (!timerDone(&ledTimeout));
        //LATE ^= 0xF0;
    }

}

bool beginMPU(MPU_6050_t *mpu, mpu6050_dps_t scale, mpu6050_range_t range, MPU6050_ADDRESS mpua) //use chosen scale enum, chosen range enum, and MPU6050_ADDRESS for parameters
{

    setTimerInterval(&ledTimeout, 100);
    setTimerInterval(&printTimer, 100);
    setTimerInterval(&IMU_UpdateTimer, 10);
    setTimerInterval(&timeOut, 500);
    // Set Address
    mpu->Address = mpua;

    //Wire.begin(); //intialize I2C

    // Reset calibrate values
    mpu->dg.XAxis = 0;
    mpu->dg.YAxis = 0;
    mpu->dg.ZAxis = 0;
    mpu->useCalibrate = false;

    // Reset threshold values
    mpu->tg.XAxis = 0;
    mpu->tg.YAxis = 0;
    mpu->tg.ZAxis = 0;
    mpu->actualThreshold = 0;
    // Check MPU6050 Who Am I Register
    //if (readRegister8(MPU6050_REG_WHO_AM_I) != mpuAddress)
    //    {
    //    	return false;
    //    }

 
    // Set Clock Source
    setClockSource(mpu->Address, MPU6050_CLOCK_PLL_XGYRO);
    // Set Scale & Range
    setScale(mpu, scale);

    setRange(mpu, range);

    //SET THE SAMPLE RATE
    writeRegister8(mpu->Address, MPU6050_SAMPLE_RATE_DIVIDER, 9); //IMU_UpdateTimer.timerInterval -1);

    //Config DLPF
    writeRegister8(mpu->Address, MPU6050_REG_CONFIG, 1);

    // Disable Sleep Mode
    setSleepEnabled(mpu->Address, false);

    readRawGyro(&MPU_1);
    readRawAccel(&MPU_1);
    return true;
}

double lowG_x = 0, highG_x = -5000;
double lowG_y = 0, highG_y = -5000;
double lowG_z = 0, highG_z = -5000;
double offsetG_X;
double offsetG_Y;
double offsetG_Z;
unsigned long lastMillis;
double xAngle;
double yAngle;
double zAngle;
float gyroXDPS;
double gyroYDPS;
double gyroZDPS;
#define SCALING_GYRO 0.0151//67//0.01197//297 //0.0151 //.015267

void updateYAxis(void) {
    static bool firstTime = true;
    //First time through
    if (firstTime) {
        firstTime = false;
        lastMillis = millis();
    }

    // if 10ms have elapsed since last we updated the angle
    if (timerDone(&IMU_UpdateTimer)) {
        // Just making sure we didn't just calculated the angle
        if (lastMillis != millis()) {
            // Read data form IMU
            readRawGyro(&MPU_1);
            // Scalling new value based of the offset found during initialization
            gyroXDPS = (double)((MPU_1.rg.XAxis) - offsetG_X);
            // if there is noise within dead-band set the velocity vector to zero
            if (isWithinFloat(gyroXDPS, lowG_x * 2.2, highG_x * 2.2)){
                gyroXDPS = 0.00;
            }
            // Fixing a strange occurance where clockwise direction results in a greater magnitude
            // velocity vector the counter-clockwise with the same speed of rotation
            if(gyroXDPS < 0)gyroXDPS = gyroXDPS*1.226;
            // Accumulating the angle with the new velocity * scaler * TimeElapsed
            xAngle += (gyroXDPS * SCALING_GYRO * (((double)millis() - lastMillis) / 1000.0));
            lastMillis = millis();
        }
    }

}

double getX_Angle() {
    return xAngle;
}

bool isWithinInt(int sample, int lowBound, int highBound) {
    return (sample > lowBound && sample < highBound);

}

bool isWithinFloat(double sample, double lowBound, double highBound) {
    return (sample > lowBound && sample < highBound);

}
#define NUMBER_SAMPLES_CALIBRATE    150
double sumG_X = 0, sumG_Y = 0, sumG_Z = 0;

void zeroIMUAxisGyro(void) {

    int i = 0;

    //Wait for power on bootup
    for (i = 0; i < NUMBER_SAMPLES_CALIBRATE; i++) {
        while (!timerDone(&IMU_UpdateTimer));
        //blinkDecimal(100);  


    }

    //Calibrate using a number of samples defined above
    for (i = 0; i < NUMBER_SAMPLES_CALIBRATE; i++) {


        //Wait for 10ms
        while (!timerDone(&IMU_UpdateTimer));

        //Read the values
        readRawGyro(&MPU_1);

        //Sum all values up for an average
        sumG_X += MPU_1.rg.XAxis;
        sumG_Y += MPU_1.rg.YAxis;
        sumG_Z += MPU_1.rg.ZAxis;

        //Keep track of the highest values
        if (MPU_1.rg.XAxis > highG_x) highG_x = MPU_1.rg.XAxis;
        if (MPU_1.rg.YAxis > highG_y) highG_y = MPU_1.rg.YAxis;
        if (MPU_1.rg.ZAxis > highG_z) highG_z = MPU_1.rg.ZAxis;

        //Keep track of the lowest values
        if (MPU_1.rg.XAxis < lowG_x) lowG_x = MPU_1.rg.YAxis;
        if (MPU_1.rg.YAxis < lowG_y) lowG_y = MPU_1.rg.XAxis;
        if (MPU_1.rg.ZAxis < lowG_z) lowG_z = MPU_1.rg.ZAxis;


    }

    //CALIBRATE X
    offsetG_X = (sumG_X / NUMBER_SAMPLES_CALIBRATE);
    highG_x -= offsetG_X;
    lowG_x -= offsetG_X;
    //printf("TotalX:  %f\n",sumX);
    //    printf("OffsetX: %f\n",offsetG_X);
    //    printf("HighX: %f\n",highG_x);
    //    printf("LowX: %f\r\n",lowG_x);

    //CALIBRATE Y
    offsetG_Y = (sumG_Y / NUMBER_SAMPLES_CALIBRATE);
    highG_y -= offsetG_Y;
    lowG_y -= offsetG_Y;
    //printf("TotalY:  %f\n",sumY);
    //    printf("OffsetY: %f\n",offsetG_Y);
    //    printf("HighY: %f\n",highG_y);
    //    printf("LowY: %f\r\n",lowG_y);

    //CALIBRATE Z
    offsetG_Z = (sumG_Z / NUMBER_SAMPLES_CALIBRATE);
    highG_z -= offsetG_Z;
    lowG_z -= offsetG_Z;
    //printf("TotalZ:  %f\n",sumZ);
    //    printf("OffsetZ: %f\n",offsetG_Z);
    //    printf("HighZ: %f\n",highG_z);
    //    printf("LowZ: %f\r\n",lowG_z);


}

void resetMPUAngles() {
    yaw = 0;
    pitch = 0;
}

void setScale(MPU_6050_t *mpu, mpu6050_dps_t scale) {
    uint8_t value;

    switch (scale) {
        case MPU6050_SCALE_250DPS:
            mpu->dpsPerDigit = .007633f; //.007874;//.007633f;   // (1/313)
            break;
        case MPU6050_SCALE_500DPS:
            mpu->dpsPerDigit = .015267f;
            break;
        case MPU6050_SCALE_1000DPS:
            mpu->dpsPerDigit = .030487f;
            break;
        case MPU6050_SCALE_2000DPS:
            mpu->dpsPerDigit = .060975f;
            break;
        default:
            break;
    }

    value = readRegister8(mpu->Address, MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);//| (1 << 7);
    writeRegister8(mpu->Address, MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t getScale(MPU6050_ADDRESS address) {
    uint8_t value;
    value = readRegister8(address, MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t) value;
}

void setRange(MPU_6050_t *mpu, mpu6050_range_t range) {
    uint8_t value;

    switch (range) {
        case MPU6050_RANGE_2G:
            mpu->rangePerDigit = .000061f; // 1/16384
            break;
        case MPU6050_RANGE_4G:
            mpu->rangePerDigit = .000122f;
            break;
        case MPU6050_RANGE_8G:
            mpu->rangePerDigit = .000244f;
            break;
        case MPU6050_RANGE_16G:
            mpu->rangePerDigit = .0004882f;
            break;
        default:
            break;
    }

    value = readRegister8(mpu->Address, MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(mpu->Address, MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t getRange(MPU6050_ADDRESS address) {
    uint8_t value;
    value = readRegister8(address, MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t) value;
}

//void setDHPFMode(mpu6050_dhpf_t dhpf)
//{
//    uint8_t value;
//    value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
//    value &= 0b11111000;
//    value |= dhpf;
//    writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
//}
//
//void setDLPFMode(mpu6050_dlpf_t dlpf)
//{
//    uint8_t value;
//    value = readRegister8(MPU6050_REG_CONFIG);
//    value &= 0b11111000;
//    value |= dlpf;
//    writeRegister8(MPU6050_REG_CONFIG, value);
//}

void setClockSource(MPU6050_ADDRESS address, mpu6050_clockSource_t source) {
    uint8_t value;
    value = readRegister8(address, MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(address, MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t getClockSource(MPU6050_ADDRESS address) {
    uint8_t value;
    value = readRegister8(address, MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t) value;
}

bool getSleepEnabled(MPU6050_ADDRESS address) {
    return readRegisterBit(address, MPU6050_REG_PWR_MGMT_1, 6);
}

void setSleepEnabled(MPU6050_ADDRESS address, bool state) {
    writeRegisterBit(address, MPU6050_REG_PWR_MGMT_1, 6, state);
}

//bool getIntZeroMotionEnabled(void)
//{
//    return readRegisterBit(MPU6050_REG_INT_ENABLE, 5);
//}
//
//void setIntZeroMotionEnabled(bool state)
//{
//    writeRegisterBit(MPU6050_REG_INT_ENABLE, 5, state);
//}
//
//bool getIntMotionEnabled(void)
//{
//    return readRegisterBit(MPU6050_REG_INT_ENABLE, 6);
//}
//
//void setIntMotionEnabled(bool state)
//{
//    writeRegisterBit(MPU6050_REG_INT_ENABLE, 6, state);
//}
//
//bool getIntFreeFallEnabled(void)
//{
//    return readRegisterBit(MPU6050_REG_INT_ENABLE, 7);
//}
//
//void setIntFreeFallEnabled(bool state)
//{
//    writeRegisterBit(MPU6050_REG_INT_ENABLE, 7, state);
//}
//
//uint8_t getMotionDetectionThreshold(void)
//{
//    return readRegister8(MPU6050_REG_MOT_THRESHOLD);
//}
//
//void setMotionDetectionThreshold(uint8_t threshold)
//{
//    writeRegister8(MPU6050_REG_MOT_THRESHOLD, threshold);
//}
//
//uint8_t getMotionDetectionDuration(void)
//{
//    return readRegister8(MPU6050_REG_MOT_DURATION);
//}
//
//void setMotionDetectionDuration(uint8_t duration)
//{
//    writeRegister8(MPU6050_REG_MOT_DURATION, duration);
//}
//
//uint8_t getZeroMotionDetectionThreshold(void)
//{
//    return readRegister8(MPU6050_REG_ZMOT_THRESHOLD);
//}
//
//void setZeroMotionDetectionThreshold(uint8_t threshold)
//{
//    writeRegister8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
//}
//
//uint8_t getZeroMotionDetectionDuration(void)
//{
//    return readRegister8(MPU6050_REG_ZMOT_DURATION);
//}
//
//void setZeroMotionDetectionDuration(uint8_t duration)
//{
//    writeRegister8(MPU6050_REG_ZMOT_DURATION, duration);
//}
//
//uint8_t getFreeFallDetectionThreshold(void)
//{
//    return readRegister8(MPU6050_REG_FF_THRESHOLD);
//}
//
//void setFreeFallDetectionThreshold(uint8_t threshold)
//{
//    writeRegister8(MPU6050_REG_FF_THRESHOLD, threshold);
//}
//
//uint8_t getFreeFallDetectionDuration(void)
//{
//    return readRegister8(MPU6050_REG_FF_DURATION);
//}
//
//void setFreeFallDetectionDuration(uint8_t duration)
//{
//    writeRegister8(MPU6050_REG_FF_DURATION, duration);
//}
//bool getI2CMasterModeEnabled(void)
//{
//    return readRegisterBit(MPU6050_REG_USER_CTRL, 5);
//}
//
//void setI2CMasterModeEnabled(bool state)
//{
//    writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
//}
//
//void setI2CBypassEnabled(bool state)
//{
//    return writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
//}
//
//bool getI2CBypassEnabled(void)
//{
//    return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
//}
//

//void setAccelPowerOnDelay(mpu6050_onDelay_t delay)
//{
//    uint8_t value;
//    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
//    value &= 0b11001111;
//    value |= (delay << 4);
//    writeRegister8(MPU6050_REG_MOT_DETECT_CTRL, value);
//}
//
//mpu6050_onDelay_t getAccelPowerOnDelay(void)
//{
//    uint8_t value;
//    value = readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
//    value &= 0b00110000;
//    return (mpu6050_onDelay_t)(value >> 4);
//}

//uint8_t getIntStatus(void)
//{
//    return readRegister8(MPU6050_REG_INT_STATUS);
//}

//Activites readActivites(void)
//{
//    uint8_t data = readRegister8(MPU6050_REG_INT_STATUS);
//
//    a.isOverflow = ((data >> 4) & 1);
//    a.isFreeFall = ((data >> 7) & 1);
//    a.isInactivity = ((data >> 5) & 1);
//    a.isActivity = ((data >> 6) & 1);
//    a.isDataReady = ((data >> 0) & 1);
//
//    data = readRegister8(MPU6050_REG_MOT_DETECT_STATUS);
//
//    a.isNegActivityOnX = ((data >> 7) & 1);
//    a.isPosActivityOnX = ((data >> 6) & 1);
//
//    a.isNegActivityOnY = ((data >> 5) & 1);
//    a.isPosActivityOnY = ((data >> 4) & 1);
//
//    a.isNegActivityOnZ = ((data >> 3) & 1);
//    a.isPosActivityOnZ = ((data >> 2) & 1);
//
//    return a;
//}

Vector readRawAccel(MPU_6050_t *mpu) //TODO: possibly change to a burst read
{
    //should only need to transmit the address of the first registers
    unsigned char bytesTX = {MPU6050_REG_ACCEL_XOUT_H};
    unsigned char bytesRX[6];


    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(mpu->Address, &bytesTX, 1, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR));

    handle = DRV_I2C0_Receive(mpu->Address, &bytesRX, 6, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR));


    mpu->ra.XAxis = (float)(bytesRX[0] << 8 | bytesRX[1]);
    mpu->ra.YAxis = bytesRX[2] << 8 | bytesRX[3];
    mpu->ra.ZAxis = bytesRX[4] << 8 | bytesRX[5];

    return mpu->ra;
}

Vector readNormalizeAccel(MPU_6050_t *mpu) {
    readRawAccel(mpu);

    mpu->na.XAxis = mpu->ra.XAxis * mpu->rangePerDigit * 9.80665f;
    mpu->na.YAxis = mpu->ra.YAxis * mpu->rangePerDigit * 9.80665f;
    mpu->na.ZAxis = mpu->ra.ZAxis * mpu->rangePerDigit * 9.80665f;

    return mpu->na;
}

Vector readScaledAccel(MPU_6050_t *mpu) {
    readRawAccel(mpu);

    mpu->na.XAxis = mpu->ra.XAxis * mpu->rangePerDigit;
    mpu->na.YAxis = mpu->ra.YAxis * mpu->rangePerDigit;
    mpu->na.ZAxis = mpu->ra.ZAxis * mpu->rangePerDigit;

    return mpu->na;
}

char bytesRX[6];

Vector readRawGyro(MPU_6050_t *mpu) //TODO: possibly change to a burst read
{
    resetTimer(&timeOut);
    char bytesTX = {MPU6050_REG_GYRO_XOUT_H};

    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(mpu->Address, &bytesTX, 1, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();
    }
    resetTimer(&timeOut);
    handle = DRV_I2C0_Receive(mpu->Address, &bytesRX, 6, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();
    }


    mpu->rg.XAxis = bytesRX[0] << 8 | bytesRX[1];
    mpu->rg.YAxis = (float)(bytesRX[2] << 8 | bytesRX[3]);
    mpu->rg.ZAxis = bytesRX[4] << 8 | bytesRX[5];

    return mpu->rg;
}

//unsigned long timePrev = 0,time = 0;
//char i = 1;
//double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
//Vector readAngleGyroAccel()
//{
//    
//    time = millis();
//    float timeStep = (millis() - timePrev);
//    timeStep /= 1000; // time-step in s
//    timePrev = time;
//    //readScaledAccel();
//    
//    Vector gs = readNormalizeGyro();
//
//    
//    // calculate accelerometer angles
//    arx = (180/3.141592) * atan(ra.XAxis / sqrt(ra.YAxis*(ra.YAxis) + ra.ZAxis*(ra.ZAxis))); 
//    ary = (180/3.141592) * atan(ra.YAxis / sqrt(ra.XAxis*(ra.XAxis)+ ra.ZAxis*(ra.ZAxis)));
//    arz = (180/3.141592) * atan(sqrt(ra.YAxis*(ra.YAxis) + ra.XAxis*(ra.XAxis)) / ra.ZAxis);
//
//    // set initial values equal to accel values
//    if (i == 1) {
//      grx = arx;
//      gry = ary;
//      grz = arz;
//    }
//    // integrate to find the gyro angle
//    else{
//      grx = grx + (timeStep * gs.XAxis);
//      gry = gry + (timeStep * gs.YAxis);
//      grz = grz + (timeStep * gs.ZAxis);
//    }   // apply filter
//    rx = (0.1 * arx) + (0.9 * grx);
//    ry = (0.1 * ary) + (0.9 * gry);
//    rz = (0.1 * arz) + (0.9 * grz); 
//     
//    i = 0;
//    return (Vector){rx,ry,grz};
//}

Vector readNormalizeGyro(MPU_6050_t *mpu) {
    readRawGyro(mpu);

    if (mpu->useCalibrate) {
        mpu->ng.XAxis = (mpu->rg.XAxis - mpu->dg.XAxis) * mpu->dpsPerDigit;
        mpu->ng.YAxis = (mpu->rg.YAxis - mpu->dg.YAxis) * mpu->dpsPerDigit;
        mpu->ng.ZAxis = (mpu->rg.ZAxis - mpu->dg.ZAxis) * mpu->dpsPerDigit;
    } else {
        mpu->ng.XAxis = mpu->rg.XAxis * mpu->dpsPerDigit;
        mpu->ng.YAxis = mpu->rg.YAxis * mpu->dpsPerDigit;
        mpu->ng.ZAxis = mpu->rg.ZAxis * mpu->dpsPerDigit;
    }

    if (mpu->actualThreshold > 0) {
        if (abs(mpu->ng.XAxis) < mpu->tg.XAxis) mpu->ng.XAxis = 0;
        if (abs(mpu->ng.YAxis) < mpu->tg.YAxis) mpu->ng.YAxis = 0;
        if (abs(mpu->ng.ZAxis) < mpu->tg.ZAxis) mpu->ng.ZAxis = 0;
    }

    return mpu->ng;
}

float readTemperature(MPU6050_ADDRESS address) {
    int16_t T;
    T = readRegister16(address, MPU6050_REG_TEMP_OUT_H);
    return (float) T / 340 + 36.53;
}

int16_t getGyroOffsetX(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_GYRO_XOFFS_H);
}

int16_t getGyroOffsetY(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_GYRO_YOFFS_H);
}

int16_t getGyroOffsetZ(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_GYRO_ZOFFS_H);
}

void setGyroOffsetX(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_GYRO_XOFFS_H, offset);
}

void setGyroOffsetY(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_GYRO_YOFFS_H, offset);
}

void setGyroOffsetZ(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t getAccelOffsetX(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t getAccelOffsetY(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t getAccelOffsetZ(MPU6050_ADDRESS address) {
    return readRegister16(address, MPU6050_REG_ACCEL_ZOFFS_H);
}

void setAccelOffsetX(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void setAccelOffsetY(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void setAccelOffsetZ(MPU6050_ADDRESS address, int16_t offset) {
    writeRegister16(address, MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm

void calibrateGyro(MPU_6050_t *mpu, uint8_t samples) {
    // Set calibrate
    mpu->useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    uint8_t i;
    for (i = 0; i < samples; ++i) {
        readRawGyro(mpu);
        sumX += mpu->rg.XAxis;
        sumY += mpu->rg.YAxis;
        sumZ += mpu->rg.ZAxis;

        sigmaX += mpu->rg.XAxis * mpu->rg.XAxis;
        sigmaY += mpu->rg.YAxis * mpu->rg.YAxis;
        sigmaZ += mpu->rg.ZAxis * mpu->rg.ZAxis;


    }

    // Calculate delta vectors
    mpu->dg.XAxis = sumX / samples;
    mpu->dg.YAxis = sumY / samples;
    mpu->dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    mpu->th.XAxis = sqrt((sigmaX / samples) - (mpu->dg.XAxis * mpu->dg.XAxis));
    mpu->th.YAxis = sqrt((sigmaY / samples) - (mpu->dg.YAxis * mpu->dg.YAxis));
    mpu->th.ZAxis = sqrt((sigmaZ / samples) - (mpu->dg.ZAxis * mpu->dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (mpu->actualThreshold > 0) //TODO: edit this snippet so that this function and setThreshold are not dependent on each other.
    {
        setThreshold(mpu, mpu->actualThreshold);
    }
}

// Get current threshold value

float getThreshold(MPU_6050_t *mpu) {
    return mpu->actualThreshold;
}

// Set threshold value

void setThreshold(MPU_6050_t *mpu, float multiple) {
    if (multiple > 0) {
        // If not calibrated, need calibrate
        if (!mpu->useCalibrate) {
            calibrateGyro(mpu, 50);
        }

        // Calculate threshold vectors
        mpu->tg.XAxis = mpu->th.XAxis * multiple;
        mpu->tg.YAxis = mpu->th.YAxis * multiple;
        mpu->tg.ZAxis = mpu->th.ZAxis * multiple;
    } else {
        // No threshold
        mpu->tg.XAxis = 0;
        mpu->tg.YAxis = 0;
        mpu->tg.ZAxis = 0;
    }

    // Remember old threshold value
    mpu->actualThreshold = multiple;
}

// Fast read 8-bit from register  //TODO: see if we even need this function for anything. It's just a read function that doesn't wait for the buffer.
/*uint8_t fastRegister8(uint8_t reg)
{
    uint8_t value;

    //Wire.beginTransmission(mpuAddress); 
    #if ARDUINO >= 100
    Wire.write(reg);
    #else
    //Wire.send(reg); 
    #endif
    //Wire.endTransmission(); 

    //Wire.beginTransmission(mpuAddress);
    //Wire.requestFrom(mpuAddress, 1);
    #if ARDUINO >= 100
    value = Wire.read();
    #else
    //value = Wire.receive(); 
    #endif
    //Wire.endTransmission(); 

    return value;
}*/

// Read 8-bit from register

int8_t readRegister8(MPU6050_ADDRESS address, uint8_t reg) {
    resetTimer(&timeOut);
    char value;
    int8_t addr = reg;
    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(address, &addr, 1, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();

    }

    handle = DRV_I2C0_Receive(address, &value, 1, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();

    }

    return value;
}

// Write 8-bit to register

void writeRegister8(MPU6050_ADDRESS address, uint8_t reg, uint8_t value) {
    resetTimer(&timeOut);
    char byte[] = {reg, value};
    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(address, byte, 2, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();
    }

}

int16_t readRegister16(MPU6050_ADDRESS address, uint8_t reg) {
    resetTimer(&timeOut);
    int16_t value;
    char addr = reg;
    char byte[2];
    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(address, &addr, 1, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();

    }
    resetTimer(&timeOut);
    handle = DRV_I2C0_Receive(address, &byte, 2, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR)) {
        isTimedOut();
    }

    //DRV_I2C0_TransmitThenReceive (mpuAddress, &reg,1,byte,2,NULL); 

    value = byte[0] << 8 | byte[1];

    return value;
}

void writeRegister16(MPU6050_ADDRESS address, uint8_t reg, int16_t value) //TODO: Verify that the correct value is being sent. May need to swap send order.
{
    resetTimer(&timeOut);
    //TODO: make this an array of two unsigned chars, sendI2C with a how_much of 2
    char byte[3] = {reg, value >> 8, value};
    DRV_I2C_BUFFER_HANDLE handle = DRV_I2C0_Transmit(address, byte, 3, NULL);
    while (!(DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_COMPLETE || DRV_I2C0_TransferStatusGet(handle) == DRV_I2C_BUFFER_EVENT_ERROR));
    {
        isTimedOut();

    }
}

// Read register bit

bool readRegisterBit(MPU6050_ADDRESS address, uint8_t reg, uint8_t pos) {
    int8_t value;
    value = readRegister8(address, reg);

    return ((value >> pos) & 1);
}

// Write register bit

void writeRegisterBit(MPU6050_ADDRESS address, uint8_t reg, uint8_t pos, bool state) {
    int8_t value;
    value = readRegister8(address, reg);

    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }

    writeRegister8(address, reg, value);
}

