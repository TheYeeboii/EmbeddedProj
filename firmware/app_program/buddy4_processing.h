#ifndef BUDDY4_PROCESSING_H
#define BUDDY4_PROCESSING_H
#include <stdbool.h>
#include <stdint.h>

#define BUDDY4_PERIOD_MS 50u
#define BUDDY4_STALE_MS 250u
#define BUDDY4_PACKET_SIZE 9u
enum
{
    BUDDY4_FLAT,
    BUDDY4_CLIMBING,
    BUDDY4_TOP,
    BUDDY4_DESCENDING
};

typedef struct
{
    bool mag_ok, accel_ok;
    int16_t mag_raw[3], accel_mg[3];
    int16_t heading_deg_x10, pitch_deg_x10, roll_deg_x10, max_pitch_deg_x10;
    uint32_t timestamp_ms, mag_timestamp_ms, accel_timestamp_ms;
    uint32_t sample_interval_ms, rejected_samples, hump_sequence;
    uint8_t hump_state, hump_count;
    int16_t hump_peak_deg_x10;
    uint16_t hump_height_mm, highest_hump_mm;
    bool height_valid, imu_fault, calibrated;
} buddy4_imu_state_t;

typedef struct
{
    /* Sensor axes must be mounted X forward (negative X nose-up), Y left,
     * Z up as in the original driver. Offsets are measured, never auto-zeroed
     * on boot because boot may occur on a slope. */
    float accel_bias_mg[3], mag_bias[3], mag_scale[3];
    int16_t pitch_zero_x10, roll_zero_x10, heading_offset_x10;
    bool calibrated, distance_calibrated;
} buddy4_calibration_t;

typedef struct
{
    buddy4_imu_state_t state;
    buddy4_calibration_t calibration;
    float pitch_history[3], roll_history[3];
    float filtered_pitch, filtered_roll, elevation, peak_elevation;
    float last_distance, last_pitch;
    uint32_t last_ms, started_ms;
    uint8_t history_count, history_index, enter_count, exit_count;
    bool started, filter_ready, event_height_valid;
} buddy4_processor_t;

void buddy4_processor_init(buddy4_processor_t *p, const buddy4_calibration_t *cal);
/* distance_mm: cumulative average wheel path; forward_valid must exclude
 * reverse/pivot motion and require calibrated, fresh odometry. */
void buddy4_process(buddy4_processor_t *p, uint32_t now_ms, bool mag_ok, const int16_t mag[3],
                    bool accel_ok, const int16_t accel[3], float distance_mm, bool forward_valid);
/* Explicit little-bit-first wire format; false = saturated/out-of-range.
 * Does not transmit C structs or silently truncate. See docs/BUDDY4_IMU.md. */
bool buddy4_pack(const buddy4_imu_state_t *s, uint8_t out[BUDDY4_PACKET_SIZE]);
#endif
