#include "buddy4_processing.h"
#include <math.h>

#define DEG10 572.957795f
static float median(float a, float b, float c)
{
    if (a > b)
    {
        float t = a;
        a = b;
        b = t;
    }
    if (b > c)
        b = c;
    return a > b ? a : b;
}
static void abort_hump(buddy4_processor_t *p)
{
    p->state.hump_state = BUDDY4_FLAT;
    p->enter_count = p->exit_count = 0;
    p->elevation = p->peak_elevation = 0;
    p->event_height_valid = false;
}
void buddy4_processor_init(buddy4_processor_t *p, const buddy4_calibration_t *cal)
{
    *p = (buddy4_processor_t){0};
    p->calibration.mag_scale[0] = p->calibration.mag_scale[1] = p->calibration.mag_scale[2] = 1;
    if (cal)
        p->calibration = *cal;
    p->state.calibrated = p->calibration.calibrated;
    p->state.imu_fault = true;
}
void buddy4_process(buddy4_processor_t *p, uint32_t now, bool mok, const int16_t m[3], bool aok,
                    const int16_t a[3], float distance, bool forward)
{
    buddy4_imu_state_t *s = &p->state;
    uint32_t dt = p->started ? now - p->last_ms : 0;
    float ds = distance - p->last_distance;
    bool gap = p->started && (dt > BUDDY4_STALE_MS || dt == 0);
    bool path = forward && p->calibration.calibrated && p->calibration.distance_calibrated &&
                isfinite(ds) && ds >= 0 && ds <= 100;
    float ax = a[0] - p->calibration.accel_bias_mg[0];
    float ay = a[1] - p->calibration.accel_bias_mg[1];
    float az = a[2] - p->calibration.accel_bias_mg[2];
    float norm = ax * ax + ay * ay + az * az;
    unsigned i;
    s->sample_interval_ms = dt;
    s->timestamp_ms = now;
    for (i = 0; i < 3; i++)
    {
        s->mag_raw[i] = m[i];
        s->accel_mg[i] = a[i];
    }
    if (gap)
    {
        abort_hump(p);
        p->history_count = 0;
        p->filter_ready = false;
    }
    /* +/-20% of g is only a plausibility check, not proof of static gravity. */
    aok = aok && isfinite(norm) && norm >= 640000 && norm <= 1440000;
    if (aok)
    {
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * DEG10 - p->calibration.pitch_zero_x10;
        float roll = atan2f(ay, az) * DEG10 - p->calibration.roll_zero_x10;
        p->pitch_history[p->history_index] = pitch;
        p->roll_history[p->history_index] = roll;
        p->history_index = (uint8_t)((p->history_index + 1) % 3);
        if (p->history_count < 3)
            p->history_count++;
        if (p->history_count == 3)
        {
            float mp = median(p->pitch_history[0], p->pitch_history[1], p->pitch_history[2]);
            float mr = median(p->roll_history[0], p->roll_history[1], p->roll_history[2]);
            float alpha = (float)dt / (100.0f + (float)dt);
            if (!p->filter_ready)
            {
                p->filtered_pitch = mp;
                p->filtered_roll = mr;
                p->filter_ready = true;
            }
            else
            {
                p->filtered_pitch += alpha * (mp - p->filtered_pitch);
                p->filtered_roll += alpha * (mr - p->filtered_roll);
            }
            s->pitch_deg_x10 = (int16_t)p->filtered_pitch;
            s->roll_deg_x10 = (int16_t)p->filtered_roll;
            if (s->pitch_deg_x10 > s->max_pitch_deg_x10)
                s->max_pitch_deg_x10 = s->pitch_deg_x10;
            s->accel_timestamp_ms = now;
        }
        else
            aok = false;
    }
    else
    {
        s->rejected_samples++;
        p->history_count = 0;
        p->filter_ready = false;
    }
    s->accel_ok = aok && p->filter_ready;
    /* Preserve original level heading convention. Reject overflow sentinels,
     * zero horizontal field, and non-finite calibration results. */
    for (i = 0; i < 3; i++)
        if (m[i] <= -4096 || m[i] > 4095)
            mok = false;
    if (mok)
    {
        float mx = (m[0] - p->calibration.mag_bias[0]) * p->calibration.mag_scale[0];
        float my = (m[1] - p->calibration.mag_bias[1]) * p->calibration.mag_scale[1];
        float h = atan2f(my, mx) * DEG10 + p->calibration.heading_offset_x10;
        mok = isfinite(h) && isfinite(mx) && isfinite(my) && mx * mx + my * my >= 1;
        if (mok)
        {
            h = fmodf(h + 36000, 3600);
            if (h < 0)
                h += 3600;
            s->heading_deg_x10 = (int16_t)h;
            s->mag_timestamp_ms = now;
        }
    }
    s->mag_ok = mok;
    s->imu_fault = !mok || !s->accel_ok;
    if (!s->accel_ok || gap)
        abort_hump(p);
    else
    {
        float pitch = p->filtered_pitch;
        /* Retain ascent distance during the debounce window. */
        if (s->hump_state == BUDDY4_FLAT)
        {
            if (pitch >= 60)
            {
                if (p->enter_count == 0)
                {
                    p->elevation = p->peak_elevation = 0;
                    p->event_height_valid = path;
                }
                p->event_height_valid = p->event_height_valid && path;
                if (path)
                    p->elevation += ds * sinf((pitch + p->last_pitch) * 0.5f / DEG10);
                p->peak_elevation = p->elevation;
                if (++p->enter_count >= 3)
                {
                    s->hump_state = BUDDY4_CLIMBING;
                    s->hump_peak_deg_x10 = (int16_t)pitch;
                    s->height_valid = false;
                    p->started_ms = now;
                    p->exit_count = 0;
                }
            }
            else
                p->enter_count = 0;
        }
        else
        {
            p->event_height_valid = p->event_height_valid && path;
            if (path)
                p->elevation += ds * sinf((pitch + p->last_pitch) * 0.5f / DEG10);
            if (p->elevation > p->peak_elevation)
                p->peak_elevation = p->elevation;
            if (pitch > s->hump_peak_deg_x10)
                s->hump_peak_deg_x10 = (int16_t)pitch;
            if (now - p->started_ms > 15000)
                abort_hump(p);
            else if (s->hump_state == BUDDY4_CLIMBING && pitch < 30)
            {
                s->hump_state = BUDDY4_TOP;
                p->exit_count = 0;
            }
            else if (s->hump_state == BUDDY4_TOP)
            {
                if (pitch <= -60)
                {
                    if (++p->exit_count >= 3)
                    {
                        s->hump_state = BUDDY4_DESCENDING;
                        p->exit_count = 0;
                    }
                }
                else
                    p->exit_count = 0;
            }
            else if (s->hump_state == BUDDY4_DESCENDING)
            {
                if (fabsf(pitch) <= 30)
                {
                    if (++p->exit_count >= 3)
                    {
                        s->hump_sequence++;
                        if (s->hump_count < 7)
                            s->hump_count++;
                        s->height_valid = p->event_height_valid && p->peak_elevation >= 0 &&
                                          p->peak_elevation <= 65535;
                        s->hump_height_mm =
                            s->height_valid ? (uint16_t)(p->peak_elevation + 0.5f) : 0;
                        if (s->height_valid && s->hump_height_mm > s->highest_hump_mm)
                            s->highest_hump_mm = s->hump_height_mm;
                        abort_hump(p);
                    }
                }
                else
                    p->exit_count = 0;
            }
        }
    }
    p->last_ms = now;
    p->last_distance = distance;
    p->last_pitch = p->filtered_pitch;
    p->started = true;
}
static void bits(uint8_t *out, unsigned *pos, uint32_t value, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++, (*pos)++)
        if (value & (1u << i))
            out[*pos / 8] |= (uint8_t)(1u << (*pos % 8));
}
static int clamp(int v, int lo, int hi, bool *ok)
{
    if (v < lo)
    {
        *ok = false;
        return lo;
    }
    if (v > hi)
    {
        *ok = false;
        return hi;
    }
    return v;
}
bool buddy4_pack(const buddy4_imu_state_t *s, uint8_t out[BUDDY4_PACKET_SIZE])
{
    unsigned i, pos = 0;
    bool ok = true;
    for (i = 0; i < BUDDY4_PACKET_SIZE; i++)
        out[i] = 0;
    bits(out, &pos, 1, 4); /* format version */
    bits(out, &pos, s->mag_ok, 1);
    bits(out, &pos, s->accel_ok, 1);
    bits(out, &pos, s->height_valid, 1);
    bits(out, &pos, s->calibrated, 1);
    bits(out, &pos, (uint32_t)clamp(s->heading_deg_x10, 0, 3599, &ok), 12);
    bits(out, &pos, (uint32_t)clamp(s->pitch_deg_x10, -900, 900, &ok) & 2047u, 11);
    bits(out, &pos, (uint32_t)clamp(s->roll_deg_x10, -1800, 1800, &ok) & 4095u, 12);
    bits(out, &pos, (uint32_t)clamp(s->hump_peak_deg_x10, -900, 900, &ok) & 2047u, 11);
    bits(out, &pos, (uint32_t)clamp(s->hump_state, 0, 3, &ok), 2);
    bits(out, &pos, (uint32_t)clamp(s->hump_count, 0, 7, &ok), 3);
    bits(out, &pos, (uint32_t)clamp(s->hump_height_mm, 0, 255, &ok), 8);
    bits(out, &pos, !ok, 1);
    bits(out, &pos, s->imu_fault, 1);
    return ok;
}
