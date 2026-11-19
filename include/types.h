#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

/**
 * @brief Represents a 3D vector (e.g., x, y, z).
 */
typedef struct {
    double x;
    double y;
    double z;
} Vector3d;

/**
 * @brief Represents rotational angles in radians.
 */
typedef struct {
    double roll;
    double pitch;
    double yaw;
} EulerAngles;

/**
 * @brief Represents angular rates (rad/s).
 */
typedef struct {
    double roll;
    double pitch;
    double yaw;
} AngularRates;

#endif // TYPES_H
