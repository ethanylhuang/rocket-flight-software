#ifndef SENSORS_H
#define SENSORS_H

#include "types.h"
#include <stdint.h>

/**
 * @brief IMU sensor data structure.
 */
typedef struct {
    Vector3d accel;    // Linear acceleration (m/s^2)
    AngularRates gyro; // Angular velocity (rad/s)
    uint64_t timestamp; // Timestamp in microseconds
} IMUData;

/**
 * @brief Barometer sensor data structure.
 */
typedef struct {
    double altitude;   // Pressure altitude (m)
    double pressure;   // Ambient pressure (Pa)
    uint64_t timestamp; // Timestamp in microseconds
} BarometerData;

/**
 * @brief GPS sensor data structure.
 */
typedef struct {
    double latitude;   // Degrees
    double longitude;  // Degrees
    double altitude;   // Meters
    Vector3d velocity; // Velocity (m/s) - local NED or similar
    uint64_t timestamp; // Timestamp in microseconds
} GPSData;

/**
 * @brief Container for all sensor readings.
 */
typedef struct {
    IMUData imu;
    BarometerData baro;
    GPSData gps;
} SensorData;

/**
 * @brief Interface for sensors.
 * In a real system, these would interact with hardware (SPI/I2C).
 * For simulation, we will provide mock implementations.
 */
void updateIMU(IMUData *imuData);
void updateBarometer(BarometerData *baroData);
void updateGPS(GPSData *gpsData);

#endif // SENSORS_H
