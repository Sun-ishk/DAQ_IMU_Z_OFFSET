#ifndef OFFSET_H
#define OFFSET_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float x;
    float y;
    float z;
} OffsetVector3_t;

/* Initialize the mounting offset.
 * height_m = vertical distance from COG to IMU in metres.
 */
void Offset_Init(float height_m);

/* Reset previous gyroscope readings. */
void Offset_Reset(void);

/* Calculate corrected acceleration at the vehicle COG.
 *
 * ax, ay, az : Accelerometer values in g
 * gx, gy, gz : Gyroscope values in degrees/second
 * dt         : Sampling interval in seconds
 *
 * Returns corrected acceleration in g.
 */
OffsetVector3_t Offset_Correct(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    float dt
);

#endif
