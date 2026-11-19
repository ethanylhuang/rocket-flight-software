#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"

/**
 * @brief Kalman Filter state for vertical position and velocity.
 */
typedef struct {
    double state[2]; // [altitude, verticalVelocity]
    double covariance[2][2];
    double processNoise[2][2];
    double measurementNoise[1][1];
} VerticalKalmanFilter;

/**
 * @brief Flight state representing the estimated output.
 */
typedef struct {
    Vector3d position;       // x, y, z (m)
    Vector3d velocity;       // vx, vy, vz (m/s)
    EulerAngles attitude;    // roll, pitch, yaw (rad)
    AngularRates attitudeRate; // rollRate, pitchRate, yawRate (rad/s)
    double machNumber;       // Mach number
    double dynamicPressure;  // Dynamic pressure (Pa)
} FlightState;

/**
 * @brief State estimator class/module.
 */
typedef struct {
    VerticalKalmanFilter verticalFilter;
    FlightState currentState;
    uint64_t lastTimestamp;
    double airSpeed;     // For Mach number calculation
    double airDensity;   // For Mach and dynamic pressure
} StateEstimator;

// Forward declaration for the helper if needed, or just implement it.
double StateEstimator_getFilterProcessNoiseForVelocity(StateEstimator *estimator, double dt);

/**
 * @brief Initializes the state estimator.
 */
void initEstimator(StateEstimator *estimator);

/**
 * @brief Updates the state estimator with new sensor data.
 * @param estimator Pointer to the estimator instance.
 * @param sensorData Current sensor readings.
 * @param dt Time step since last update (seconds).
 */
void updateEstimator(StateEstimator *estimator, const SensorData *sensorData, double dt);

#endif // ESTIMATOR_H
