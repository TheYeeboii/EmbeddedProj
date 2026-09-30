#ifndef BUDDY2_MOTION_H
#define BUDDY2_MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float wheel_diameter_mm;
    float track_width_mm;
    float left_ticks_per_wheel_rev;
    float right_ticks_per_wheel_rev;
    uint16_t pwm_limit;
    uint16_t pwm_deadband;
} buddy2_calibration_t;

typedef struct
{
    int32_t left_ticks;
    int32_t right_ticks;
    float left_speed_mm_s;
    float right_speed_mm_s;
    float left_distance_mm;
    float right_distance_mm;
    uint32_t timestamp_ms;
} buddy2_motion_state_t;

void buddy2_motion_init(void);
void buddy2_set_motor_pwm(int16_t left_pwm, int16_t right_pwm);
void buddy2_stop(void);
void buddy2_get_motion_state(buddy2_motion_state_t *state);
void buddy2_reset_encoders(void);
void buddy2_update(uint32_t now_ms);

bool buddy2_move_forward(float distance_mm, int16_t max_pwm);
bool buddy2_move_backward(float distance_mm, int16_t max_pwm);
bool buddy2_turn_left(float angle_deg, int16_t max_pwm);
bool buddy2_turn_right(float angle_deg, int16_t max_pwm);
bool buddy2_motion_active(void);

void buddy2_set_calibration(const buddy2_calibration_t *calibration);
void buddy2_get_calibration(buddy2_calibration_t *calibration);

bool moveForward(float distance_mm, int16_t max_pwm);
bool moveBackward(float distance_mm, int16_t max_pwm);
bool turnLeft(float angle_deg, int16_t max_pwm);
bool turnRight(float angle_deg, int16_t max_pwm);
void stop(void);

#endif
