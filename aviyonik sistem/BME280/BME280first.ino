#include <Wire.h>
#include <Adafruit_BME280.h> // BMP180 sensor library
#include <Adafruit_Sensor.h> 
#include "SimpleKalmanFilter.h"

// Define the BMP180 sensor
Adafruit_BME280 bmp;

// Define Kalman filter parameters
float measurementError = 1; // You may need to adjust these values based on your application
float estimateError = 1;
float processNoise = 0.05;

// Create a SimpleKalmanFilter object
SimpleKalmanFilter kalmanFilter(measurementError, estimateError, processNoise);

void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find BME280 sensor!");
    while (1);
  }
}

void loop() {
  // Read BMe280 sensor data
  float pressure = bmp.readPressure();

  // Update the Kalman filter with the sensor data
  float filteredPressure = kalmanFilter.updateEstimate(pressure);

  // Output the results
  Serial.print("Raw Pressure: ");
  Serial.print(pressure);
  Serial.print(" | Filtered Pressure: ");
  Serial.println(filteredPressure);

  delay(1000); // Adjust the delay based on your application requirements
}
