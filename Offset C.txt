#include "offset.h"
#include <math.h>

/* Conversion constants
Calculations has to be done using SI units*/
#define DEG_TO_RAD  (0.017453292519943295f)  //Angular Velocity
#define RAD_TO_DEG  (57.29577951308232f)  //
#define GRAVITY     (9.80665f) 

//Vertical mounting offset
static float offset_height = 0.0f;
/* Negligible horizontal offset*/

/* Previous angular velocity */
static float prev_wx = 0.0f;
static float prev_wy = 0.0f;
static float prev_wz = 0.0f;

/* First sample flag */
static bool first_sample = true;
/*static necessary or will return true everytime*/

/* Initialize mounting offset */
void Offset_Init(float height_m)
{
    offset_height = height_m;

    Offset_Reset();
}


/* Reset previous gyroscope values */
void Offset_Reset(void)
{
    prev_wx = 0.0f;
    prev_wy = 0.0f;
    prev_wz = 0.0f;

    first_sample = true;
}
/*First sample true skips 1st value as there is no previous value for it*/

/* Calculate corrected acceleration */
OffsetVector3_t Offset_Correct(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    float dt
)
/*3 is for no. of axis*/
{
    OffsetVector3_t corrected;

    /* Convert acceleration from g to m/s^2  for SI unit calculations*/
    float ax_ms2 = ax * GRAVITY;
    float ay_ms2 = ay * GRAVITY;
    float az_ms2 = az * GRAVITY;

    /* Convert angular velocity from degrees/s to radians/s for SI unit calculations */
    float wx = gx * DEG_TO_RAD;
    float wy = gy * DEG_TO_RAD;
    float wz = gz * DEG_TO_RAD;

    /* Angular acceleration */
    float alpha_x = 0.0f;
    float alpha_y = 0.0f;
    float alpha_z = 0.0f;

    if (!first_sample && dt > 0.0f)
    {
        alpha_x = (wx - prev_wx) / dt;
        alpha_y = (wy - prev_wy) / dt;
        alpha_z = (wz - prev_wz) / dt;
    }

    /* Store current angular velocity */
    prev_wx = wx;
    prev_wy = wy;
    prev_wz = wz;

    first_sample = false;

    /* Tangential and centripetal acceleration corrections*/
    float delta_ax =
        -offset_height * (alpha_y + wx * wz);

    float delta_ay =
         offset_height * (alpha_x - wy * wz);

    float delta_az =
         offset_height * (wx * wx + wy * wy);

    /* Remove mounting-position acceleration */
    float cog_ax = ax_ms2 - delta_ax;
    float cog_ay = ay_ms2 - delta_ay;
    float cog_az = az_ms2 - delta_az;

    /* Convert corrected acceleration back to g */
    corrected.x = cog_ax / GRAVITY;
    corrected.y = cog_ay / GRAVITY;
    corrected.z = cog_az / GRAVITY;

    return corrected;
}
