#include <Arduino.h>
#include "MPU6050_Kalman.h"

#define RAD_TO_DEG 57.2957795131


uint32_t timer;
const double Accel_Z_corrector = 14418.0;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
/*
Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
*/
void MPU6050_Kalman::init() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero to wake up the sensor
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x19);  // SMPLRT_DIV register
    Wire.write(0x07);  // set to 7 for a data rate of 1kHz
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x00);  // set to 0 for a range of ±2g
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x00);  // set to 0 for a range of ±250 °/s
    Wire.endTransmission(true);
}

void MPU6050_Kalman::MPU6050_Read_Accel(MPU6050_t& DataStruct) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6, true);  // read 6 bytes
    DataStruct.Accel_X_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Accel_Y_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Accel_Z_RAW = Wire.read() << 8 | Wire.read();

    DataStruct.Ax = DataStruct.Accel_X_RAW / 16384.0;
    DataStruct.Ay = DataStruct.Accel_Y_RAW / 16384.0;
    DataStruct.Az = DataStruct.Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Kalman::MPU6050_Read_Gyro(MPU6050_t& DataStruct) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // GYRO_XOUT_H register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6, true);  // read 6 bytes
    DataStruct.Gyro_X_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Gyro_Y_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Gyro_Z_RAW = Wire.read() << 8 | Wire.read();

    DataStruct.Gx = DataStruct.Gyro_X_RAW / 131.0;
    DataStruct.Gy = DataStruct.Gyro_Y_RAW / 131.0;
    DataStruct.Gz = DataStruct.Gyro_Z_RAW / 131.0;
}

void MPU6050_Kalman::MPU6050_Read_Temp(MPU6050_t& DataStruct) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x41);  // TEMP_OUT_H register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 2, true);  // read 2 bytes
    int16_t temp = Wire.read() << 8 | Wire.read();
    DataStruct.Temperature = (float)(temp / 340.0 + 36.53);
}

void MPU6050_Kalman::MPU6050_Read_All(MPU6050_t& DataStruct) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 14, true);  // read 14 bytes
    DataStruct.Accel_X_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Accel_Y_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Accel_Z_RAW = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    DataStruct.Gyro_X_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Gyro_Y_RAW = Wire.read() << 8 | Wire.read();
    DataStruct.Gyro_Z_RAW = Wire.read() << 8 | Wire.read();

    DataStruct.Ax = DataStruct.Accel_X_RAW / 16384.0;
    DataStruct.Ay = DataStruct.Accel_Y_RAW / 16384.0;
    DataStruct.Az = DataStruct.Accel_Z_RAW / 14418.0;
    DataStruct.Temperature = (float)(temp / 340.0 + 36.53);

    DataStruct.Gx = DataStruct.Gyro_X_RAW / 131.0;
    DataStruct.Gy = DataStruct.Gyro_Y_RAW / 131.0;
    DataStruct.Gz = DataStruct.Gyro_Z_RAW / 131.0;

    double dt = (double)(millis() - timer) / 1000;
    timer = millis();

    double roll;
    double roll_sqrt = sqrt(DataStruct.Accel_X_RAW * DataStruct.Accel_X_RAW + DataStruct.Accel_Z_RAW * DataStruct.Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct.Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct.Accel_X_RAW, DataStruct.Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct.KalmanAngleY > 90) || (pitch > 90 && DataStruct.KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct.KalmanAngleY = pitch;
    }
    else {
        KalmanY.angle = getAngle(pitch, DataStruct.Gy, dt, &KalmanY);
    }

    DataStruct.KalmanAngleX = getAngle(roll, DataStruct.Gx, dt, &KalmanX);
    DataStruct.KalmanAngleY = getAngle(pitch, DataStruct.Gy, dt, &KalmanY);
}

double MPU6050_Kalman:: getAngle(double newAngle, double newRate, double dt, Kalman_t* kalman) {
    kalman->angle += dt * (newRate - kalman->bias);

    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    double y = newAngle - kalman->angle;
    double S = kalman->P[0][0] + kalman->R_measure;

    double K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    double P00_temp = kalman->P[0][0];
    double P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}