/**
 * @file    buddy4_processing.h
 * @brief   Buddy 4: portable IMU processing (tilt, humps, motion events).
 *
 * No kernel or SDK headers, so this module can be compiled and tested on a
 * PC.  buddy4_imu.c feeds it one sample set per IMU task period.
 */

#ifndef BUDDY4_PROCESSING_H
#define BUDDY4_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>

#define BUDDY4_PERIOD_MS        (50u)
#define BUDDY4_STALE_MS         (250u)
#define BUDDY4_PACKET_SIZE      (9u)

/* Terrain (hump) states: 2 bits. */
#define BUDDY4_FLAT             (0u)
#define BUDDY4_CLIMBING         (1u)
#define BUDDY4_TOP              (2u)
#define BUDDY4_DESCENDING       (3u)

/* Motion classes: 3 bits.  When several apply, the highest number wins. */
#define BUDDY4_MOTION_UNKNOWN       (0u)   /* no fresh odometry            */
#define BUDDY4_MOTION_STATIONARY    (1u)
#define BUDDY4_MOTION_CRUISING      (2u)   /* moving at a steady speed     */
#define BUDDY4_MOTION_ACCELERATING  (3u)
#define BUDDY4_MOTION_DECELERATING  (4u)
#define BUDDY4_MOTION_TURNING       (5u)
#define BUDDY4_MOTION_HUMP          (6u)   /* see hump_state for up/down   */
#define BUDDY4_MOTION_IMPACT        (7u)

/*
 * Snapshot shared with other tasks (copied whole under DI/EI).  Change
 * Requirement 1: flags, states and counters use only the bits their range
 * needs; angles stay int16_t because they are signed and used in maths.
 */
typedef struct
{
    int16_t  heading_deg_x10;       /* 0..3599                           */
    int16_t  pitch_deg_x10;         /* -900..900, nose up positive       */
    int16_t  roll_deg_x10;          /* -1800..1800                       */
    int16_t  max_pitch_deg_x10;     /* largest pitch since boot          */
    int16_t  hump_peak_deg_x10;     /* peak pitch of the last hump       */
    int16_t  highest_peak_deg_x10;  /* largest peak of completed humps   */
    int16_t  turn_rate_dps_x10;     /* compass yaw rate, + = heading up  */
    uint16_t hump_height_mm;
    uint16_t highest_hump_mm;
    uint16_t sample_interval_ms;
    uint16_t hump_sequence;         /* +1 per completed hump (wraps)     */
    uint32_t mag_timestamp_ms;
    uint32_t accel_timestamp_ms;
    uint32_t mag_ok        : 1;
    uint32_t accel_ok      : 1;
    uint32_t height_valid  : 1;
    uint32_t imu_fault     : 1;
    uint32_t calibrated    : 1;
    uint32_t hump_state    : 2;     /* BUDDY4_FLAT..BUDDY4_DESCENDING    */
    uint32_t hump_count    : 3;     /* completed humps, saturates at 7   */
    uint32_t motion_class  : 3;     /* BUDDY4_MOTION_*                   */
    uint32_t impact_count  : 8;     /* sudden impacts, saturates at 255  */
} buddy4_imu_state_t;

typedef struct
{
    /* Rover axes, after buddy4_imu.c rotates the mounted sensor: X along
     * the rover (negative X nose-up), Y across it, Z up.  Offsets are measured, never
     * auto-zeroed on boot because boot may occur on a slope. */
    float   accel_bias_mg[3];
    float   mag_bias[3];
    float   mag_scale[3];
    int16_t pitch_zero_x10;
    int16_t roll_zero_x10;
    int16_t heading_offset_x10;
    bool    calibrated;
    bool    distance_calibrated;
} buddy4_calibration_t;

/* Private working state of the IMU owner task. */
typedef struct
{
    buddy4_imu_state_t   state;
    buddy4_calibration_t calibration;
    float    pitch_history[3];
    float    roll_history[3];
    float    filtered_pitch;
    float    filtered_roll;
    float    elevation;
    float    peak_elevation;
    float    last_distance;         /* odometry at the last good pitch    */
    float    sample_distance;       /* odometry at the previous sample    */
    float    last_pitch;
    float    window_distance;       /* odometry at the speed window start */
    float    window_pivot;          /* pivot travel at the window start   */
    float    window_speed_mm_s;     /* forward speed, last full window    */
    float    pivot_speed_mm_s;      /* pivot speed, last full window      */
    uint32_t last_ms;
    uint32_t started_ms;
    uint32_t window_ms;             /* speed window start                 */
    uint32_t rate_ms;               /* turn-rate window start             */
    uint32_t impact_ms;             /* last impact                        */
    int16_t  rate_heading_x10;      /* heading at the turn-rate window    */
    int8_t   speed_trend;           /* -1 slowing, 0 steady, +1 speeding  */
    uint8_t  history_count;
    uint8_t  history_index;
    uint8_t  enter_count;
    uint8_t  exit_count;
    uint8_t  rejected_in_row;
    bool     started;
    bool     filter_ready;
    bool     event_height_valid;
    bool     window_open;
    bool     speed_known;
    bool     rate_ready;
} buddy4_processor_t;

void buddy4_processor_init(buddy4_processor_t *p_proc, const buddy4_calibration_t *p_cal);

/**
 * @brief Process one sample set.
 * @param distance_mm     Cumulative average wheel path, (left + right) / 2.
 * @param pivot_mm        Cumulative turning path, (right - left) / 2.
 * @param odometry_fresh  Buddy 2 updated its distance recently.
 * @param forward_valid   Both wheels driving forward (needed for height).
 */
void buddy4_process(buddy4_processor_t *p_proc, uint32_t now_ms,
                    bool mag_ok, const int16_t mag[3],
                    bool accel_ok, const int16_t accel[3],
                    float distance_mm, float pivot_mm, bool odometry_fresh, bool forward_valid);

/* Explicit least-significant-bit-first wire format; false = a field was
 * saturated.  See docs/BUDDY4_IMU.md for the bit layout. */
bool buddy4_pack(const buddy4_imu_state_t *p_state, uint8_t out[BUDDY4_PACKET_SIZE]);

#endif /* BUDDY4_PROCESSING_H */
