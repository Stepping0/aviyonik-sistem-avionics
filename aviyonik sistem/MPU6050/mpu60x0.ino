#include <Wire.h>
#include <MPU6050_Kalman.h>

MPU6050_Kalman mpu;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    //MPU6050_Init();
    mpu.init();

    Serial.println("GyroX  |  GyroY  |  GyroZ  |  KalmanX  |  KalmanY  ");
}

void loop() {
    MPU6050_t DataStruct;
    mpu.MPU6050_Read_All(DataStruct);


    /*Serial.print("AccX: "); Serial.print(DataStruct.Ax);
    Serial.print(" | AccY: "); Serial.print(DataStruct.Ay);
    Serial.print(" | AccZ: "); Serial.print(DataStruct.Az);

    Serial.print(" | Temp: "); Serial.print(DataStruct.Temperature);*/

    Serial.print(DataStruct.Gx);Serial.print("  |  ");
    Serial.print(DataStruct.Gy);Serial.print("  |  ");
    Serial.print(DataStruct.Gz);Serial.print("  |  ");

    Serial.print(DataStruct.KalmanAngleX);Serial.print("    |    ");
    Serial.println(DataStruct.KalmanAngleY);
    
    delay(1000);
}
