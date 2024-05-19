#ifndef MPU6050_KALMAN_H
#define MPU6050_KALMAN_H

#include <Wire.h>
#include <math.h>
#include <time.h>

#define MPU6050_ADDR 0x68
//const double Accel_Z_corrector = 14418.0;

struct MPU6050_t {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
   // double KalmanAngleZ;
};

struct Kalman_t {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
};



class MPU6050_Kalman {
public:
    void init();
    void MPU6050_Read_Accel(MPU6050_t& DataStruct);
    void MPU6050_Read_Gyro(MPU6050_t& DataStruct);
    void MPU6050_Read_Temp(MPU6050_t& DataStruct);
    void MPU6050_Read_All(MPU6050_t& DataStruct);
    double getAngle(double newAngle, double newRate, double dt, Kalman_t* kalman);

private:
    // ... (same as in your original code)
};

#endif // MPU6050_KALMAN_H