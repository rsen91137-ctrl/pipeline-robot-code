#include "alg_kalman.h"

#include <float.h>
#include <stddef.h>

/* Ordered comparisons reject NaN and both infinities without relying on
 * C99 isfinite(), which differs between embedded C library versions.
 * Compile with normal IEEE semantics (do not enable fast-math).
 */
static int kalman1d_finite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static int kalman1d_valid_noise(float q, float r)
{
    return kalman1d_finite(q) && q >= 0.0f &&
           kalman1d_finite(r) && r >= FLT_MIN;
}

static Kalman1D_Status kalman1d_check(const Kalman1D *filter)
{
    if (filter == NULL)
        return KALMAN1D_INVALID_ARGUMENT;
    if (filter->initialized != 1)
        return KALMAN1D_NOT_INITIALIZED;
    if (filter->has_estimate > 1 ||
        !kalman1d_valid_noise(filter->process_noise_per_second,
                              filter->measurement_noise) ||
        !kalman1d_finite(filter->initial_covariance) ||
        filter->initial_covariance < FLT_MIN ||
        !kalman1d_finite(filter->estimate) ||
        !kalman1d_finite(filter->covariance) ||
        filter->covariance < FLT_MIN ||
        !kalman1d_finite(filter->gain) ||
        filter->gain < 0.0f || filter->gain > 1.0f)
        return KALMAN1D_INVALID_STATE;
    return KALMAN1D_OK;
}

Kalman1D_Status Kalman1D_Init(Kalman1D *filter, float q, float r, float p0)
{
    Kalman1D next;
    if (filter == NULL || !kalman1d_valid_noise(q, r) ||
        !kalman1d_finite(p0) || p0 < FLT_MIN)
        return KALMAN1D_INVALID_ARGUMENT;

    next.estimate = 0.0f;
    next.covariance = p0;
    next.gain = 0.0f;
    next.process_noise_per_second = q;
    next.measurement_noise = r;
    next.initial_covariance = p0;
    next.initialized = 1;
    next.has_estimate = 0;
    *filter = next;
    return KALMAN1D_OK;
}

Kalman1D_Status Kalman1D_Reset(Kalman1D *filter)
{
    Kalman1D_Status status = kalman1d_check(filter);
    if (status != KALMAN1D_OK)
        return status;
    filter->estimate = 0.0f;
    filter->covariance = filter->initial_covariance;
    filter->gain = 0.0f;
    filter->has_estimate = 0;
    return KALMAN1D_OK;
}

Kalman1D_Status Kalman1D_SetNoise(Kalman1D *filter, float q, float r)
{
    Kalman1D_Status status = kalman1d_check(filter);
    if (status != KALMAN1D_OK)
        return status;
    if (!kalman1d_valid_noise(q, r))
        return KALMAN1D_INVALID_ARGUMENT;
    filter->process_noise_per_second = q;
    filter->measurement_noise = r;
    return KALMAN1D_OK;
}

Kalman1D_Status Kalman1D_Update(Kalman1D *filter, float measured_speed,
                              float dt_seconds, float *filtered_speed)
{
    Kalman1D_Status status;
    double prior_covariance;
    double denominator;
    double gain;
    double complement;
    double estimate;
    double covariance;

    if (filtered_speed == NULL || !kalman1d_finite(measured_speed) ||
        !kalman1d_finite(dt_seconds) || dt_seconds <= 0.0f)
        return KALMAN1D_INVALID_ARGUMENT;
    status = kalman1d_check(filter);
    if (status != KALMAN1D_OK)
        return status;

    if (filter->has_estimate == 0)
    {
        filter->estimate = measured_speed;
        filter->covariance = filter->initial_covariance;
        filter->gain = 1.0f;
        filter->has_estimate = 1;
        *filtered_speed = measured_speed;
        return KALMAN1D_FIRST_SAMPLE;
    }

    /* Local double intermediates avoid overflow in P+R and z-x for valid
     * float inputs. Persistent state and the public interface remain float.
     * On STM32F427 double arithmetic is software-assisted; measure its cost
     * before using this in a very high-frequency interrupt.
     */
    prior_covariance = (double)filter->covariance +
        (double)filter->process_noise_per_second * (double)dt_seconds;
    if (!(prior_covariance >= (double)FLT_MIN &&
          prior_covariance <= (double)FLT_MAX))
        return KALMAN1D_NUMERIC_ERROR;

    denominator = prior_covariance + (double)filter->measurement_noise;
    gain = prior_covariance / denominator;
    complement = 1.0 - gain;
    estimate = complement * (double)filter->estimate +
               gain * (double)measured_speed;
    covariance = complement * complement * prior_covariance +
                 gain * gain * (double)filter->measurement_noise;

    /* Keep covariance positive even at float's lower normal limit. This
     * floor is a numerical safeguard, not a tuning parameter or spike gate.
     */
    if (covariance >= 0.0 && covariance < (double)FLT_MIN)
        covariance = (double)FLT_MIN;
    if (!(estimate >= -(double)FLT_MAX && estimate <= (double)FLT_MAX) ||
        !(covariance >= (double)FLT_MIN && covariance <= (double)FLT_MAX) ||
        !(gain >= 0.0 && gain <= 1.0))
        return KALMAN1D_NUMERIC_ERROR;

    /* Commit only after every check has passed. */
    filter->estimate = (float)estimate;
    filter->covariance = (float)covariance;
    filter->gain = (float)gain;
    *filtered_speed = filter->estimate;
    return KALMAN1D_OK;
}
