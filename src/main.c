#include "sensors.h"
#include "estimator.h"
#include "utils.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

int main() {
    logInfo("Starting Rocket Flight Software Simulation...");

    StateEstimator estimator;
    initEstimator(&estimator);

    SensorData sensors = {0};
    
    // Initial values
    sensors.imu.accel = (Vector3d){0.0, 0.0, 9.81}; // Resting on pad
    sensors.imu.gyro = (AngularRates){0.0, 0.0, 0.0};
    sensors.baro.altitude = 0.0;
    sensors.baro.pressure = 101325.0;
    sensors.gps.velocity = (Vector3d){0.0, 0.0, 0.0};

    double dt = 0.1; // 10 Hz simulation
    double trueAltitude = 0.0;
    double trueVelocityZ = 0.0;
    
    // Simulation Loop (100 steps)
    for (int i = 0; i < 100; ++i) {
        // Inject fake data:
        // Let's say at step 20, the rocket starts boosting upward
        double trueAccelZ = 0.0;
        if (i > 20 && i < 60) {
            // Boosting: Accelerating upwards at 20 m/s^2 (total accel = 29.81)
            trueAccelZ = 20.0;
            sensors.imu.accel.z = trueAccelZ + 9.81;
            // Let's make the baro more realistic: it should follow the integral of accel
            // But we want to see if Kalman handles it.
            // For a simple test, let's just set baro to something plausible.
        } else if (i >= 60) {
            // Coasting/Descending
            trueAccelZ = -9.81; // cuz gravity
            sensors.imu.accel.z = trueAccelZ + 9.81; // No more thrust
        }

        // Integrate a consistent truth altitude for the barometer
        trueAltitude += trueVelocityZ * dt + 0.5 * trueAccelZ * dt * dt;
        trueVelocityZ += trueAccelZ * dt;
        sensors.baro.altitude = trueAltitude;
        sensors.gps.velocity.z = trueVelocityZ;

        // Also add some noise to sensors
        sensors.baro.altitude += ((double)rand() / RAND_MAX - 0.5) * 0.1;

        // Update estimator
        updateEstimator(&estimator, &sensors, dt);

        // Print state
        printf("Step %d: Alt: %.2f m, Vel_z: %.2f m/s, Mach: %.2f, DynPress: %.2f Pa\n",
               i, estimator.currentState.position.z, estimator.currentState.velocity.z,
               estimator.currentState.machNumber, estimator.currentState.dynamicPressure);
    }

    logInfo("Simulation Finished.");
    return 0;
}