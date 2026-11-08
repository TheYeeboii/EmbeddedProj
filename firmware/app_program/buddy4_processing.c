/**
 * @file    buddy4_processing.c
 * @brief   Buddy 4: portable IMU processing (tilt, humps, motion events).
 *
 * Tilt:   median of three, then a time-aware low-pass filter.
 * Humps:  flat -> climbing -> top -> descending -> flat, with debounce.
 * Height: sum of wheel travel x sin(pitch) during the hump.
 * Motion: impact, hump, turning, stationary, accelerating, decelerating
 *         or cruising, from acceleration, compass and wheel odometry.
 * All thresholds below are first estimates; tune them on the track.
 */

#include <math.h>
#include <stddef.h>

#include "buddy4_processing.h"

#define BUDDY4_DEG10 (572.957795f) /* 180 / pi * 10 */

/* Tilt filter. */
#define BUDDY4_FILTER_TAU_MS  (100.0f)
#define BUDDY4_GRAVITY_MIN_SQ (640000.0f)  /* (0.8 g)^2, mg^2 */
#define BUDDY4_GRAVITY_MAX_SQ (1440000.0f) /* (1.2 g)^2       */
/* A few rejected samples in a row (a bump, vibration) keep the filter and
 * the hump; more than this (about the 250 ms staleness limit) reset them. */
#define BUDDY4_MAX_REJECTED_IN_ROW (4u)

/* Hump detection on the total tilt from the flat attitude, tenths of a
 * degree.  First values from one track hump (one wheel up gave about 6
 * degrees: -3 pitch, +5.4 roll); tune them from logs. */
#define BUDDY4_HUMP_ENTER_X10 (50)
#define BUDDY4_HUMP_LEVEL_X10 (25)
#define BUDDY4_HUMP_FALL_X10  (20) /* this far below the peak: coming down */
#define BUDDY4_HUMP_RISE_X10  (2)  /* per reading (4 deg/s): still climbing */
#define BUDDY4_HUMP_CONFIRM   (3u)
/* Level readings that end a hump (400 ms), so a two-wheel crossing that is
 * level on the crest for a moment still counts once. */
#define BUDDY4_HUMP_SETTLE (8u)
/* While level, the flat attitude follows the floor and the sensor mounting
 * over about 2 s, so no zero offsets are needed to detect a hump. */
#define BUDDY4_FLAT_ALPHA (0.025f)
/* One-wheel hump height: the raised wheel lifts its side by about
 * track width x sin(roll change), with no odometry or zero offsets.
 * shortcut: copies Buddy 2's default track width; read it from
 * buddy2_get_calibration() if Buddy 2 measures a different value. */
#define BUDDY4_TRACK_WIDTH_MM    (150.0f)
#define BUDDY4_HUMP_MIN_ROLL_X10 (10) /* below 1 deg: both wheels climbed */
#define BUDDY4_HUMP_TIMEOUT_MS   (15000u)
#define BUDDY4_HUMP_MAX_STEP_MM  (100.0f)
#define BUDDY4_HUMP_COUNT_MAX    (7u)

/* Motion classification. */
#define BUDDY4_IMPACT_LOW_SQ    (250000.0f)  /* (0.5 g)^2 */
#define BUDDY4_IMPACT_HIGH_SQ   (2250000.0f) /* (1.5 g)^2 */
#define BUDDY4_IMPACT_HOLD_MS   (500u)
#define BUDDY4_IMPACT_COUNT_MAX (255u)
#define BUDDY4_RATE_WINDOW_MS   (200u) /* compass turn rate */
/* One encoder tick (10.2 mm at 20 ticks/rev) is 25 mm/s over 400 ms, so a
 * speed change must exceed two ticks to count as speeding up or slowing. */
#define BUDDY4_SPEED_WINDOW_MS   (400u)
#define BUDDY4_SPEED_CHANGE_MM_S (60.0f) /* per window */
#define BUDDY4_STOPPED_MM_S      (5.0f)
#define BUDDY4_MAX_SPEED_MM_S    (1000.0f) /* odometry reset guard */
#define BUDDY4_TURNING_DPS_X10   (250)     /* 25 deg/s */

#define BUDDY4_PACKET_VERSION (2u)

/* Hump heights are reported as uint16_t millimetres. */
#define BUDDY4_HEIGHT_MAX_MM (65535.0f)

static float32_t buddy4_median (float32_t first, float32_t second,
                                float32_t third);
static int16_t   buddy4_wrap_x10 (int32_t change_x10);
static void      buddy4_abort_hump (buddy4_processor_t * p_proc);
static bool buddy4_process_accel (buddy4_processor_t * p_proc, uint32_t now_ms,
                                  uint32_t dt_ms, bool b_accel_ok,
                                  const int16_t accel[3]);
static void buddy4_process_mag (buddy4_processor_t * p_proc, uint32_t now_ms,
                                bool b_mag_ok, const int16_t mag[3]);
static void buddy4_process_speed (buddy4_processor_t * p_proc, uint32_t now_ms,
                                  float32_t distance_mm, float32_t pivot_mm,
                                  bool b_odometry_fresh);
static void buddy4_process_hump (buddy4_processor_t * p_proc, uint32_t now_ms,
                                 float32_t travelled_mm, bool b_path_valid);
static void buddy4_hump_flat (buddy4_processor_t * p_proc, uint32_t now_ms,
                              float32_t tilt, float32_t rise_mm,
                              bool b_path_valid, bool b_turning);
static void buddy4_hump_finish (buddy4_processor_t * p_proc);
static uint8_t buddy4_classify (const buddy4_processor_t * p_proc,
                                uint32_t                   now_ms);
static void buddy4_put_bits (uint8_t * p_out, uint32_t * p_pos, uint32_t value,
                             uint32_t width);
static int32_t buddy4_saturate (int32_t value, int32_t low, int32_t high,
                                bool * p_ok);

static float32_t buddy4_median (float32_t first, float32_t second,
                                float32_t third)
{
    float32_t low  = first;
    float32_t high = second;

    if (low > high)
    {
        low  = second;
        high = first;
    }
    if (high > third)
    {
        high = third;
    }
    return (low > high) ? low : high;
}

static int16_t buddy4_wrap_x10 (int32_t change_x10)
{
    if (change_x10 > 1800)
    {
        change_x10 -= 3600;
    }
    else if (change_x10 < -1800)
    {
        change_x10 += 3600;
    }
    else
    {
        /* already the shortest way round */
    }
    return (int16_t) change_x10;
}

static void buddy4_abort_hump (buddy4_processor_t * p_proc)
{
    p_proc->state.hump_state   = BUDDY4_FLAT;
    p_proc->enter_count        = 0u;
    p_proc->exit_count         = 0u;
    p_proc->elevation          = 0.0f;
    p_proc->peak_elevation     = 0.0f;
    p_proc->peak_roll          = 0.0f;
    p_proc->event_height_valid = false;
}

void buddy4_processor_init (buddy4_processor_t *         p_proc,
                            const buddy4_calibration_t * p_cal)
{
    *p_proc                          = (buddy4_processor_t) {0};
    p_proc->calibration.mag_scale[0] = 1.0f;
    p_proc->calibration.mag_scale[1] = 1.0f;
    p_proc->calibration.mag_scale[2] = 1.0f;
    if (NULL != p_cal)
    {
        p_proc->calibration = *p_cal;
    }
    p_proc->state.calibrated = p_proc->calibration.calibrated ? 1u : 0u;
    p_proc->state.imu_fault  = 1u;
}

/* Accelerometer: tilt filter and impact detection.  Returns true when a new
 * filtered pitch was produced by this sample. */
static bool buddy4_process_accel (buddy4_processor_t * p_proc, uint32_t now_ms,
                                  uint32_t dt_ms, bool b_accel_ok,
                                  const int16_t accel[3])
{
    buddy4_imu_state_t * p_state = &p_proc->state;
    float32_t            accel_x =
        (float32_t) accel[0] - p_proc->calibration.accel_bias_mg[0];
    float32_t accel_y =
        (float32_t) accel[1] - p_proc->calibration.accel_bias_mg[1];
    float32_t accel_z =
        (float32_t) accel[2] - p_proc->calibration.accel_bias_mg[2];
    float32_t norm_sq =
        (accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z);
    bool b_new = false;

    if (b_accel_ok && isfinite(norm_sq) &&
        ((norm_sq < BUDDY4_IMPACT_LOW_SQ) || (norm_sq > BUDDY4_IMPACT_HIGH_SQ)))
    {
        /* One collision gives several wild samples: count at most one
         * impact per 500 ms. */
        if ((0u == p_state->impact_count) ||
            ((now_ms - p_proc->impact_ms) >= BUDDY4_IMPACT_HOLD_MS))
        {
            if (p_state->impact_count < BUDDY4_IMPACT_COUNT_MAX)
            {
                p_state->impact_count++;
            }
            p_proc->impact_ms = now_ms;
        }
    }

    /* +/-20% of g is only a plausibility check, not proof of static gravity. */
    if (b_accel_ok && isfinite(norm_sq) && (norm_sq >= BUDDY4_GRAVITY_MIN_SQ) &&
        (norm_sq <= BUDDY4_GRAVITY_MAX_SQ))
    {
        float32_t pitch = (atan2f(-accel_x, sqrtf((accel_y * accel_y) +
                                                  (accel_z * accel_z))) *
                           BUDDY4_DEG10) -
                          (float32_t) p_proc->calibration.pitch_zero_x10;
        float32_t roll  = (atan2f(accel_y, accel_z) * BUDDY4_DEG10) -
                          (float32_t) p_proc->calibration.roll_zero_x10;

        p_proc->rejected_in_row                      = 0u;
        p_proc->pitch_history[p_proc->history_index] = pitch;
        p_proc->roll_history[p_proc->history_index]  = roll;
        p_proc->history_index = (uint8_t) ((p_proc->history_index + 1u) % 3u);
        if (p_proc->history_count < 3u)
        {
            p_proc->history_count++;
        }
        if (3u == p_proc->history_count)
        {
            float32_t median_pitch = buddy4_median(p_proc->pitch_history[0],
                                                   p_proc->pitch_history[1],
                                                   p_proc->pitch_history[2]);
            float32_t median_roll =
                buddy4_median(p_proc->roll_history[0], p_proc->roll_history[1],
                              p_proc->roll_history[2]);
            float32_t alpha =
                (float32_t) dt_ms / (BUDDY4_FILTER_TAU_MS + (float32_t) dt_ms);

            if (!p_proc->filter_ready)
            {
                p_proc->filtered_pitch = median_pitch;
                p_proc->filtered_roll  = median_roll;
                p_proc->filter_ready   = true;
            }
            else
            {
                p_proc->filtered_pitch +=
                    alpha * (median_pitch - p_proc->filtered_pitch);
                p_proc->filtered_roll +=
                    alpha * (median_roll - p_proc->filtered_roll);
            }
            p_state->pitch_deg_x10 = (int16_t) p_proc->filtered_pitch;
            p_state->roll_deg_x10  = (int16_t) p_proc->filtered_roll;
            if (p_state->pitch_deg_x10 > p_state->max_pitch_deg_x10)
            {
                p_state->max_pitch_deg_x10 = p_state->pitch_deg_x10;
            }
            p_state->accel_timestamp_ms = now_ms;
            b_new                       = true;
        }
    }
    else
    {
        if (p_proc->rejected_in_row < UINT8_MAX)
        {
            p_proc->rejected_in_row++;
        }
        if (p_proc->rejected_in_row > BUDDY4_MAX_REJECTED_IN_ROW)
        {
            p_proc->history_count = 0u;
            p_proc->filter_ready  = false;
        }
    }
    p_state->accel_ok = p_proc->filter_ready ? 1u : 0u;
    return b_new;
}

/* Magnetometer: heading and compass turn rate. */
static void buddy4_process_mag (buddy4_processor_t * p_proc, uint32_t now_ms,
                                bool b_mag_ok, const int16_t mag[3])
{
    buddy4_imu_state_t * p_state = &p_proc->state;
    uint32_t             index;

    /* Reject overflow sentinels, zero horizontal field and non-finite results.
     */
    for (index = 0u; index < 3u; index++)
    {
        if ((mag[index] <= -4096) || (mag[index] > 4095))
        {
            b_mag_ok = false;
        }
    }
    if (b_mag_ok)
    {
        float32_t mag_x =
            ((float32_t) mag[0] - p_proc->calibration.mag_bias[0]) *
            p_proc->calibration.mag_scale[0];
        float32_t mag_y =
            ((float32_t) mag[1] - p_proc->calibration.mag_bias[1]) *
            p_proc->calibration.mag_scale[1];
        float32_t heading = (atan2f(mag_y, mag_x) * BUDDY4_DEG10) +
                            (float32_t) p_proc->calibration.heading_offset_x10;

        b_mag_ok = isfinite(heading) && isfinite(mag_x) && isfinite(mag_y) &&
                   (((mag_x * mag_x) + (mag_y * mag_y)) >= 1.0f);
        if (b_mag_ok)
        {
            heading = fmodf(heading + 36000.0f, 3600.0f);
            if (heading < 0.0f)
            {
                heading += 3600.0f;
            }
            p_state->heading_deg_x10  = (int16_t) heading;
            p_state->mag_timestamp_ms = now_ms;
        }
    }
    p_state->mag_ok = b_mag_ok ? 1u : 0u;

    if (!b_mag_ok)
    {
        p_proc->rate_ready         = false;
        p_state->turn_rate_dps_x10 = 0;
    }
    else if (!p_proc->rate_ready)
    {
        p_state->turn_rate_dps_x10 = 0; /* no rate until a window closes */
        p_proc->rate_ready         = true;
        p_proc->rate_ms            = now_ms;
        p_proc->rate_heading_x10   = p_state->heading_deg_x10;
    }
    else if ((now_ms - p_proc->rate_ms) >= BUDDY4_RATE_WINDOW_MS)
    {
        /* The 15 Hz compass repeats samples at a 20 Hz poll, so the rate is
         * taken over a 200 ms window instead of sample to sample. */
        int32_t change_x10 =
            buddy4_wrap_x10((int32_t) p_state->heading_deg_x10 -
                            (int32_t) p_proc->rate_heading_x10);

        p_state->turn_rate_dps_x10 =
            (int16_t) ((change_x10 * 1000) /
                       (int32_t) (now_ms - p_proc->rate_ms));
        p_proc->rate_ms          = now_ms;
        p_proc->rate_heading_x10 = p_state->heading_deg_x10;
    }
    else
    {
        /* keep the last rate until the window closes */
    }
}

/* Wheel odometry: forward and pivot speed per window, and the speed trend. */
static void buddy4_process_speed (buddy4_processor_t * p_proc, uint32_t now_ms,
                                  float32_t distance_mm, float32_t pivot_mm,
                                  bool b_odometry_fresh)
{
    if (!b_odometry_fresh || !isfinite(distance_mm) || !isfinite(pivot_mm))
    {
        p_proc->window_open = false;
        p_proc->speed_known = false;
        p_proc->speed_trend = 0;
    }
    else if (!p_proc->window_open)
    {
        p_proc->window_open     = true;
        p_proc->window_ms       = now_ms;
        p_proc->window_distance = distance_mm;
        p_proc->window_pivot    = pivot_mm;
    }
    else if ((now_ms - p_proc->window_ms) >= BUDDY4_SPEED_WINDOW_MS)
    {
        float32_t seconds = (float32_t) (now_ms - p_proc->window_ms) / 1000.0f;
        float32_t speed   = (distance_mm - p_proc->window_distance) / seconds;
        float32_t pivot_speed = (pivot_mm - p_proc->window_pivot) / seconds;

        if ((fabsf(speed) > BUDDY4_MAX_SPEED_MM_S) ||
            (fabsf(pivot_speed) > BUDDY4_MAX_SPEED_MM_S))
        {
            /* Faster than the rover can drive: Buddy 2 reset its odometry. */
            p_proc->speed_known = false;
            p_proc->speed_trend = 0;
        }
        else
        {
            float32_t change = fabsf(speed) - fabsf(p_proc->window_speed_mm_s);

            p_proc->speed_trend = 0;
            if (p_proc->speed_known && (change > BUDDY4_SPEED_CHANGE_MM_S))
            {
                p_proc->speed_trend = 1;
            }
            else if (p_proc->speed_known &&
                     (change < -BUDDY4_SPEED_CHANGE_MM_S))
            {
                p_proc->speed_trend = -1;
            }
            else
            {
                /* steady, or the first window */
            }
            p_proc->window_speed_mm_s = speed;
            p_proc->pivot_speed_mm_s  = pivot_speed;
            p_proc->speed_known       = true;
        }
        p_proc->window_ms       = now_ms;
        p_proc->window_distance = distance_mm;
        p_proc->window_pivot    = pivot_mm;
    }
    else
    {
        /* window still open */
    }
}

/* Hump state machine and height; only called with a new filtered pitch.
 * The track hump is under one wheel: the body then leans sideways and a
 * little nose-down, the lean holds on top and goes back to level after it.
 * So detection uses the total tilt (pitch and roll) from the flat attitude,
 * not the sign of the pitch. */
static void buddy4_process_hump (buddy4_processor_t * p_proc, uint32_t now_ms,
                                 float32_t travelled_mm, bool b_path_valid)
{
    buddy4_imu_state_t * p_state = &p_proc->state;
    float32_t            pitch   = p_proc->filtered_pitch;
    float32_t            rise_mm =
        travelled_mm * sinf((pitch + p_proc->last_pitch) * 0.5f / BUDDY4_DEG10);
    float32_t tilt     = 0.0f;
    bool      b_rising = false;
    bool b_turning = (p_state->turn_rate_dps_x10 >= BUDDY4_TURNING_DPS_X10) ||
                     (p_state->turn_rate_dps_x10 <= -BUDDY4_TURNING_DPS_X10);

    if (!p_proc->flat_ready)
    {
        p_proc->flat_pitch = pitch;
        p_proc->flat_roll  = p_proc->filtered_roll;
        p_proc->last_tilt  = 0.0f;
        p_proc->flat_ready = true;
    }
    tilt     = hypotf(pitch - p_proc->flat_pitch,
                      p_proc->filtered_roll - p_proc->flat_roll);
    b_rising = (tilt > (p_proc->last_tilt + (float32_t) BUDDY4_HUMP_RISE_X10));
    p_proc->last_tilt = tilt;

    if (BUDDY4_FLAT == p_state->hump_state)
    {
        buddy4_hump_flat(p_proc, now_ms, tilt, rise_mm, b_path_valid,
                         b_turning);
    }
    else
    {
        p_proc->event_height_valid = p_proc->event_height_valid && b_path_valid;
        if (b_path_valid)
        {
            p_proc->elevation += rise_mm;
        }
        if (p_proc->elevation > p_proc->peak_elevation)
        {
            p_proc->peak_elevation = p_proc->elevation;
        }
        if (tilt > (float32_t) p_state->hump_peak_deg_x10)
        {
            /* Tilt is a small angle in tenths, so it fits int16_t. */
            p_state->hump_peak_deg_x10 = (int16_t) tilt;
        }
        if (fabsf(p_proc->filtered_roll - p_proc->flat_roll) >
            p_proc->peak_roll)
        {
            p_proc->peak_roll =
                fabsf(p_proc->filtered_roll - p_proc->flat_roll);
        }

        if ((now_ms - p_proc->started_ms) > BUDDY4_HUMP_TIMEOUT_MS)
        {
            buddy4_abort_hump(p_proc);
        }
        else if (tilt > (float32_t) BUDDY4_HUMP_LEVEL_X10)
        {
            /* Still leaning: show where on the hump the wheel is. */
            p_proc->exit_count = 0u;
            if (tilt <
                (float32_t) (p_state->hump_peak_deg_x10 - BUDDY4_HUMP_FALL_X10))
            {
                p_state->hump_state = BUDDY4_DESCENDING;
            }
            else if (b_rising)
            {
                p_state->hump_state = BUDDY4_CLIMBING;
                p_proc->enter_count = 0u;
            }
            else if (p_proc->enter_count < BUDDY4_HUMP_CONFIRM)
            {
                p_proc->enter_count++;
            }
            else
            {
                p_state->hump_state = BUDDY4_TOP;
            }
        }
        else
        {
            p_state->hump_state = BUDDY4_DESCENDING;
            p_proc->exit_count++;
            if (p_proc->exit_count >= BUDDY4_HUMP_SETTLE)
            {
                buddy4_hump_finish(p_proc);
            }
        }
    }
}

/* Flat ground: track the level attitude and confirm the start of a hump. */
static void buddy4_hump_flat (buddy4_processor_t * p_proc, uint32_t now_ms,
                              float32_t tilt, float32_t rise_mm,
                              bool b_path_valid, bool b_turning)
{
    buddy4_imu_state_t * p_state = &p_proc->state;

    if (tilt <= (float32_t) BUDDY4_HUMP_LEVEL_X10)
    {
        p_proc->flat_pitch +=
            BUDDY4_FLAT_ALPHA * (p_proc->filtered_pitch - p_proc->flat_pitch);
        p_proc->flat_roll +=
            BUDDY4_FLAT_ALPHA * (p_proc->filtered_roll - p_proc->flat_roll);
    }
    /* A pivot turn leans the body outwards (sideways force), so a lean that
     * starts during a turn is not a hump. */
    if ((tilt >= (float32_t) BUDDY4_HUMP_ENTER_X10) && !b_turning)
    {
        /* Keep the climb made during the debounce window. */
        if (0u == p_proc->enter_count)
        {
            p_proc->elevation          = 0.0f;
            p_proc->peak_elevation     = 0.0f;
            p_proc->event_height_valid = b_path_valid;
        }
        p_proc->event_height_valid = p_proc->event_height_valid && b_path_valid;
        if (b_path_valid)
        {
            p_proc->elevation += rise_mm;
        }
        p_proc->peak_elevation = p_proc->elevation;
        p_proc->enter_count++;
        if (p_proc->enter_count >= BUDDY4_HUMP_CONFIRM)
        {
            p_state->hump_state = BUDDY4_CLIMBING;
            /* Tilt is a small angle in tenths, so it fits int16_t. */
            p_state->hump_peak_deg_x10 = (int16_t) tilt;
            p_state->height_valid      = 0u;
            p_proc->started_ms         = now_ms;
            p_proc->enter_count        = 0u; /* now: readings without a rise */
            p_proc->exit_count         = 0u; /* now: level readings          */
        }
    }
    else
    {
        p_proc->enter_count = 0u;
    }
}

/* Back on level ground: count the hump and record its height. */
static void buddy4_hump_finish (buddy4_processor_t * p_proc)
{
    buddy4_imu_state_t * p_state = &p_proc->state;
    bool      b_height_ok = p_proc->event_height_valid &&
                            (p_proc->peak_elevation >= 0.0f) &&
                            (p_proc->peak_elevation <= BUDDY4_HEIGHT_MAX_MM);
    float32_t height_mm   = p_proc->peak_elevation;

    /* No two-wheel estimate (needs calibration and odometry): use the
     * one-wheel lean, which the track hump produces. */
    if (!b_height_ok &&
        (p_proc->peak_roll >= (float32_t) BUDDY4_HUMP_MIN_ROLL_X10))
    {
        height_mm =
            BUDDY4_TRACK_WIDTH_MM * sinf(p_proc->peak_roll / BUDDY4_DEG10);
        b_height_ok = true;
    }

    p_state->hump_sequence++;
    if (p_state->hump_count < BUDDY4_HUMP_COUNT_MAX)
    {
        p_state->hump_count++;
    }
    if (p_state->hump_peak_deg_x10 > p_state->highest_peak_deg_x10)
    {
        p_state->highest_peak_deg_x10 = p_state->hump_peak_deg_x10;
    }
    p_state->height_valid = b_height_ok ? 1u : 0u;
    /* The height was checked against BUDDY4_HEIGHT_MAX_MM (or comes from a
     * sine of the track width), so rounding fits uint16_t. */
    p_state->hump_height_mm = b_height_ok ? (uint16_t) (height_mm + 0.5f) : 0u;
    if (b_height_ok && (p_state->hump_height_mm > p_state->highest_hump_mm))
    {
        p_state->highest_hump_mm = p_state->hump_height_mm;
    }
    buddy4_abort_hump(p_proc);
}

static uint8_t buddy4_classify (const buddy4_processor_t * p_proc,
                                uint32_t                   now_ms)
{
    const buddy4_imu_state_t * p_state = &p_proc->state;
    int32_t                    rate    = p_state->turn_rate_dps_x10;
    uint8_t                    motion  = BUDDY4_MOTION_CRUISING;

    if ((0u != p_state->impact_count) &&
        ((now_ms - p_proc->impact_ms) < BUDDY4_IMPACT_HOLD_MS))
    {
        motion = BUDDY4_MOTION_IMPACT;
    }
    else if (BUDDY4_FLAT != p_state->hump_state)
    {
        motion = BUDDY4_MOTION_HUMP;
    }
    else if (p_proc->rate_ready && ((rate >= BUDDY4_TURNING_DPS_X10) ||
                                    (rate <= -BUDDY4_TURNING_DPS_X10)))
    {
        motion = BUDDY4_MOTION_TURNING;
    }
    else if (!p_proc->speed_known)
    {
        motion = BUDDY4_MOTION_UNKNOWN;
    }
    else if ((fabsf(p_proc->window_speed_mm_s) < BUDDY4_STOPPED_MM_S) &&
             (fabsf(p_proc->pivot_speed_mm_s) < BUDDY4_STOPPED_MM_S))
    {
        motion = BUDDY4_MOTION_STATIONARY;
    }
    else if (p_proc->speed_trend > 0)
    {
        motion = BUDDY4_MOTION_ACCELERATING;
    }
    else if (p_proc->speed_trend < 0)
    {
        motion = BUDDY4_MOTION_DECELERATING;
    }
    else
    {
        motion = BUDDY4_MOTION_CRUISING;
    }

    return motion;
}

void buddy4_process (buddy4_processor_t * p_proc, uint32_t now_ms,
                     bool b_mag_ok, const int16_t mag[3], bool b_accel_ok,
                     const int16_t accel[3], float32_t distance_mm,
                     float32_t pivot_mm, bool b_odometry_fresh,
                     bool b_forward_valid)
{
    buddy4_imu_state_t * p_state = &p_proc->state;
    uint32_t dt_ms = p_proc->started ? (now_ms - p_proc->last_ms) : 0u;
    bool     b_gap =
        p_proc->started && ((dt_ms > BUDDY4_STALE_MS) || (0u == dt_ms));
    /* Travel since the last good pitch, and during this sample only. */
    float32_t travelled = distance_mm - p_proc->last_distance;
    float32_t step      = distance_mm - p_proc->sample_distance;
    bool b_path_valid   = b_forward_valid && p_proc->calibration.calibrated &&
                          p_proc->calibration.distance_calibrated &&
                          isfinite(step) && (step >= 0.0f) &&
                          (step <= BUDDY4_HUMP_MAX_STEP_MM);
    bool b_new_pitch    = false;

    p_state->sample_interval_ms =
        (dt_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t) dt_ms;
    if (b_gap)
    {
        buddy4_abort_hump(p_proc);
        p_proc->history_count = 0u;
        p_proc->filter_ready  = false;
        p_proc->window_open   = false;
        p_proc->speed_known   = false;
        p_proc->rate_ready    = false;
    }

    b_new_pitch =
        buddy4_process_accel(p_proc, now_ms, dt_ms, b_accel_ok, accel);
    buddy4_process_mag(p_proc, now_ms, b_mag_ok, mag);
    buddy4_process_speed(p_proc, now_ms, distance_mm, pivot_mm,
                         b_odometry_fresh);
    p_state->imu_fault =
        ((0u == p_state->mag_ok) || (0u == p_state->accel_ok)) ? 1u : 0u;

    if (!p_proc->filter_ready || b_gap)
    {
        buddy4_abort_hump(p_proc);
        p_proc->flat_ready    = false; /* attitude unknown: start again */
        p_proc->last_distance = distance_mm;
    }
    else if (b_new_pitch)
    {
        buddy4_process_hump(p_proc, now_ms, travelled, b_path_valid);
        p_proc->last_distance = distance_mm;
    }
    else
    {
        /* Isolated rejected sample: keep the hump and the travel since the
         * last good pitch, but remember if this step spoils the height. */
        p_proc->event_height_valid = p_proc->event_height_valid && b_path_valid;
    }

    p_state->motion_class   = buddy4_classify(p_proc, now_ms) & 0x07u;
    p_proc->sample_distance = distance_mm;
    p_proc->last_ms         = now_ms;
    p_proc->last_pitch      = p_proc->filtered_pitch;
    p_proc->started         = true;
}

static void buddy4_put_bits (uint8_t * p_out, uint32_t * p_pos, uint32_t value,
                             uint32_t width)
{
    uint32_t bit = 0u;

    for (bit = 0u; bit < width; bit++)
    {
        if (0u != (value & (1u << bit)))
        {
            p_out[*p_pos / 8u] |= (uint8_t) (1u << (*p_pos % 8u));
        }
        (*p_pos)++;
    }
}

static int32_t buddy4_saturate (int32_t value, int32_t low, int32_t high,
                                bool * p_ok)
{
    int32_t result = value;

    if (value < low)
    {
        *p_ok  = false;
        result = low;
    }
    else if (value > high)
    {
        *p_ok  = false;
        result = high;
    }
    else
    {
        result = value;
    }

    return result;
}

bool buddy4_pack (const buddy4_imu_state_t * p_state,
                  uint8_t                    out[BUDDY4_PACKET_SIZE])
{
    uint32_t index  = 0u;
    uint32_t pos    = 0u;
    bool     b_fits = true;

    for (index = 0u; index < BUDDY4_PACKET_SIZE; index++)
    {
        out[index] = 0u;
    }
    buddy4_put_bits(out, &pos, BUDDY4_PACKET_VERSION, 4u);
    buddy4_put_bits(out, &pos, p_state->mag_ok, 1u);
    buddy4_put_bits(out, &pos, p_state->accel_ok, 1u);
    buddy4_put_bits(out, &pos, p_state->height_valid, 1u);
    buddy4_put_bits(out, &pos, p_state->calibrated, 1u);
    buddy4_put_bits(
        out, &pos,
        (uint32_t) buddy4_saturate(p_state->heading_deg_x10, 0, 3599, &b_fits),
        12u);
    buddy4_put_bits(
        out, &pos,
        (uint32_t) buddy4_saturate(p_state->pitch_deg_x10, -900, 900, &b_fits) &
            0x7FFu,
        11u);
    buddy4_put_bits(out, &pos,
                    (uint32_t) buddy4_saturate(p_state->roll_deg_x10, -1800,
                                               1800, &b_fits) &
                        0xFFFu,
                    12u);
    buddy4_put_bits(out, &pos,
                    (uint32_t) buddy4_saturate(p_state->hump_peak_deg_x10, -900,
                                               900, &b_fits) &
                        0x7FFu,
                    11u);
    buddy4_put_bits(out, &pos, p_state->hump_state, 2u);
    buddy4_put_bits(out, &pos, p_state->hump_count, 3u);
    buddy4_put_bits(
        out, &pos,
        (uint32_t) buddy4_saturate(p_state->hump_height_mm, 0, 255, &b_fits),
        8u);
    buddy4_put_bits(out, &pos, b_fits ? 0u : 1u, 1u);
    buddy4_put_bits(out, &pos, p_state->imu_fault, 1u);
    buddy4_put_bits(out, &pos, p_state->motion_class, 3u);
    return b_fits;
}

/*** end of file ***/

