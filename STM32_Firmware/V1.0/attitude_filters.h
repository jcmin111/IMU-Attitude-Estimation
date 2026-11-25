#ifndef ATTITUDE_FILTERS_H
#define ATTITUDE_FILTERS_H
#include <stdint.h>
typedef struct { float roll_deg, pitch_deg, yaw_deg; float q0,q1,q2,q3; } AttitudeOutput;
typedef struct {
 float roll, pitch, yaw;
 float q0,q1,q2,q3;
 float integral_x,integral_y,integral_z;
 float covariance[16];
 float mag_ref[3];
 float mag_norm_reference;
 uint8_t mag_ref_valid;
 uint8_t initialized;
} AttitudeFilterState;
/* Q and R are EKF tuning parameters; the Q/R ratio sets trust in model vs measurement.
   Q small relative to R => trust gyro prediction more; R small => trust accelerometer more.
   These are normalized-gravity measurement variances, not calibrated physical sensor variances. */
void EKF_SetNoise(float process_q, float measurement_r);
void AttitudeFilters_Reset(AttitudeFilterState *s);
void Complementary_Update(AttitudeFilterState *s, float ax,float ay,float az, float gx,float gy,float gz, float mx,float my,float mz, float dt, AttitudeOutput *out);
void Mahony_Update(AttitudeFilterState *s, float ax,float ay,float az, float gx,float gy,float gz, float mx,float my,float mz, float dt, AttitudeOutput *out);
void AQUA_Update(AttitudeFilterState *s, float ax,float ay,float az, float gx,float gy,float gz, float mx,float my,float mz, float dt, AttitudeOutput *out);
void EKF_Update(AttitudeFilterState *s, float ax,float ay,float az, float gx,float gy,float gz, float mx,float my,float mz, float dt, AttitudeOutput *out);
#endif
