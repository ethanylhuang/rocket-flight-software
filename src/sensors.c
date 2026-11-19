#include "sensors.h"
#include <math.h>
#include <stdlib.h>

// Mocking functions to inject fake data
// In a real implementation, these would read from I2C/SPI registers.

void updateIMU(IMUData *imuData) {
    // Default to zero (gravity is handled by subtracting it in estimator)
    imuData->accel.x = 0.0;
    imuData->accel.y = 0.0;
    imuData->accel.z = 9.81; // Earth gravity (m/s^2)
    
    imuData->gyro.roll = 0.0;
    imuData->gyro.pitch = 0.0;
    imuData->gyro.yaw = 0.0;
    
    // Using a dummy timestamp. In real code, use hardware timer.
    imuData->timestamp = 0; 
}

void updateBarometer(BarometerData *baroData) {
    baroData->altitude = 0.0;
    baroData->pressure = 101325.0; // 1 atm
    baroData->timestamp = 0;
}

void updateGPS(GPSData *gpsData) {
    gpsData->latitude = 0.0;
    gpsData->longitude = 0.0;
    gpsData->altitude = 0.0;
    gpsData->velocity.x = 0.0;
    gpsData->velocity.y = 0.0;
    gpsData->velocity.z = 0.0;
    gpsData->timestamp = 0;
}
