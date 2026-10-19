/* Standalone scalar Kalman filter for motor speed feedback. */
#ifndef ROBOT_ALG_KALMAN_H
#define ROBOT_ALG_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    KALMAN1D_OK = 0,
    KALMAN1D_FIRST_SAMPLE = 1,
    KALMAN1D_INVALID_ARGUMENT = -1,
    KALMAN1D_NOT_INITIALIZED = -2,
    KALMAN1D_INVALID_STATE = -3,
    KALMAN1D_NUMERIC_ERROR = -4
} Kalman1D_Status;

/* One instance per independent signal. Fields are read-only to callers.
 * Init may be called on an uninitialized object; check its return value.
 * Call Init successfully before Update/Reset/SetNoise. INITIALIZER only
 * allows these calls to safely report NOT_INITIALIZED before Init.
 * A single instance must not be updated concurrently by main/ISR contexts.
 */
typedef struct
{
    float estimate;
    float covariance;
    float gain;
    float process_noise_per_second;
    float measurement_noise;
    float initial_covariance;
    unsigned char initialized;
    unsigned char has_estimate;
} Kalman1D;

#define KALMAN1D_INITIALIZER {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0}

/* q: process variance per second, >= 0, units (speed unit)^2 / s.
 * r: measurement variance, >= FLT_MIN, units (speed unit)^2.
 * p0: covariance AFTER seeding from the first sample, >= FLT_MIN.
 * For a first sample with no additional information, use p0 = r.
 * Init/Reset wait for a new sample; they do not seed speed to zero.
 */
Kalman1D_Status Kalman1D_Init(Kalman1D *filter, float q, float r, float p0);
Kalman1D_Status Kalman1D_Reset(Kalman1D *filter);

/* Change noise parameters without discarding the current estimate.
 * To change p0, call Init again. q = 0 is a constant-state model and
 * can become very slow to follow speed changes; normally use q > 0.
 */
Kalman1D_Status Kalman1D_SetNoise(Kalman1D *filter, float q, float r);

/* Call exactly once per NEW feedback sample, not once per main loop.
 * dt_seconds: elapsed sample time in seconds, finite and > 0.
 * After skipped/invalid samples, supply time since the last accepted sample.
 * filtered_speed must be non-NULL and must not alias any filter field.
 * On a negative return, neither filter nor *filtered_speed is changed.
 * A first valid sample returns FIRST_SAMPLE and passes through unchanged.
 * Later samples use x'=x, P'=P+q*dt, H=1 and a Joseph covariance update.
 * This function smooths measurements; it does not detect/remove finite spikes.
 */
Kalman1D_Status Kalman1D_Update(Kalman1D *filter, float measured_speed,
                              float dt_seconds, float *filtered_speed);

#ifdef __cplusplus
}
#endif

#endif
