#include "estimator.h"
#include "utils.h"
#include <math.h>
#include <string.h>

#define SPEED_OF_SOUND 340.3  // m/s at sea level
#define AIR_DENSITY_SEA_LEVEL 1.225 // kg/m^3

void initEstimator(StateEstimator *estimator) {
    memset(estimator, 0, sizeof(StateEstimator));

    // Initialize Vertical Kalman Filter
    // State: [altitude, verticalVelocity]
    estimator->verticalFilter.state[0] = 0.0; // altitude
    estimator->verticalFilter.state[1] = 0.0; // verticalVelocity

    // Covariance matrix (initial uncertainty)
    estimator->verticalFilter.covariance[0][0] = 1.0;
    estimator->verticalFilter.covariance[0][1] = 0.0;
    estimator->verticalFilter.covariance[1][0] = 0.0;
    estimator->verticalFilter.covariance[1][1] = 1.0;

    // Process noise (how much we expect the model to diverge)
    estimator->verticalFilter.processNoise[0][0] = 0.01; // altitude uncertainty
    estimator->verticalFilter.processNoise[1][1] = 0.01; // velocity uncertainty
    
    // Measurement noise (how much we trust the barometer)
    estimator->verticalFilter.measurementNoise[0][0] = 0.5; // Baro is noisy

    estimator->currentState.position.z = 0.0;
    estimator->currentState.velocity.z = 0.0;
    
    estimator->airSpeed = 0.0;
    estimator->airDensity = AIR_DENSITY_SEA_LEVEL;

    logInfo("State Estimator Initialized.");
}

void updateEstimator(StateEstimator *estimator, const SensorData *sensorData, double dt) {
    if (dt <= 0.0) return;

    // 1. Vertical Kalman Filter (Altitude & Vertical Velocity)
    // Prediction step
    double accelZ = sensorData->imu.accel.z - 9.81;

    // Predict state: x_pred = F * x
    // F = [[1, dt], [0, 1]]
    double x_pred_alt = estimator->verticalFilter.state[0] + (estimator->verticalFilter.state[1] * dt) + (0.5 * accelZ * dt * dt);
    double x_pred_vel = estimator->verticalFilter.state[1] + (accelZ * dt);

    // Predict covariance
    double p00 = estimator->verticalFilter.covariance[0][0];
    double p01 = estimator->verticalFilter.covariance[0][1];
    double p10 = estimator->verticalFilter.covariance[1][0];
    double p11 = estimator->verticalFilter.covariance[1][1];

    double fp00 = p00 + dt * p10;
    double fp01 = p01 + dt * p11;
    double fp10 = p10;
    double fp11 = p11;

    double p_pred00 = fp00 + dt * fp01;
    double p_pred01 = fp01;
    double p_pred10 = fp10 + dt * fp11;
    double p_pred11 = fp11;

    // Add process noise
    p_pred00 += estimator->verticalFilter.processNoise[0][0];
    p_pred11 += estimator->verticalFilter.processNoise[1][1];

    // Update Step (Measurement: Barometer)
    // H = [1, 0]
    // S = H*P*H' + R = p_pred00 + R
    // K = P*H' * S^-1 = [p_pred00, p_pred10]' / S
    double measurement_z = sensorData->baro.altitude;
    double S = p_pred00 + estimator->verticalFilter.measurementNoise[0][0];
    double K0 = p_pred00 / S;
    double K1 = p_pred10 / S;

    double innovation = measurement_z - x_pred_alt;

    estimator->verticalFilter.state[0] = x_pred_alt + K0 * innovation;
    estimator->verticalFilter.state[1] = x_pred_vel + K1 * innovation;

    // P = (I - K*H) * P_pred
    // I-KH = [[1-K0, 0], [-K1, 1]]
    // (I-KH)*P = [[(1-K0)*p_pred00, (1-K0)*p_pred01], [-K1*p_pred00 + p_pred10, -K1*p_pred01 + p_pred11]]
    estimator->verticalFilter.covariance[0][0] = (1.0 - K0) * p_pred00;
    estimator->verticalFilter.covariance[0][1] = (1.0 - K0) * p_pred01;
    estimator->verticalFilter.covariance[1][0] = -K1 * p_pred00 + p_pred10;
    estimator->verticalFilter.covariance[1][1] = -K1 * p_pred01 + p_pred11;

    // Update currentState
    estimator->currentState.position.z = estimator->verticalFilter.state[0];
    estimator->currentState.velocity.z = estimator->verticalFilter.state[1];

    // 2. Rotational position and velocity (Integration)
    estimator->currentState.attitude.roll += sensorData->imu.gyro.roll * dt;
    estimator->currentState.attitude.pitch += sensorData->imu.gyro.pitch * dt;
    estimator->currentState.attitude.yaw += sensorData->imu.gyro.yaw * dt;

    estimator->currentState.attitudeRate.roll = sensorData->imu.gyro.roll;
    estimator->currentState.attitudeRate.pitch = sensorData->imu.gyro.pitch;
    estimator->currentState.attitudeRate.yaw = sensorData->imu.gyro.yaw;

    // 3. Mach and Dynamic Pressure
    // Using GPS velocity for airspeed (crude approximation)
    double vx_gps = sensorData->gps.velocity.x;
    double vy_gps = sensorData->gps.velocity.y;
    double vz_gps = sensorData->gps.velocity.z;
    double airSpeed = sqrt(vx_gps * vx_gps + vy_gps * vy_gps + vz_gps * vz_gps);
    
    estimator->airSpeed = airSpeed;
    estimator->currentState.machNumber = airSpeed / SPEED_OF_SOUND;
    estimator->airDensity = AIR_DENSITY_SEA_LEVEL * exp(-estimator->currentState.position.z / 8500.0);
    estimator->currentState.dynamicPressure = 0.5 * estimator->airDensity * airSpeed * airSpeed;
}