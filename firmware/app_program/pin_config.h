/**
 * @file    pin_config.h
 * @brief   Pin map for the Cytron Robo Pico + Pico W robot car.
 *
 * The ONE place pin numbers are defined.  Checked against the Robo Pico
 * datasheet.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

/* Motor driver (Robo Pico, hard-wired).
 * Rewired 23 Oct to match the Week 6 design review: LEFT wheel (seen from
 * behind) on M1, RIGHT wheel on M2.  The motors are mounted mirror-image, so
 * the left motor's wires are crossed: left 05 -> M1B, 06 -> M1A; right
 * 05 -> M2A, 06 -> M2B.  Checked with the test buttons: M1A and M2A both
 * turn their wheel forwards. */
#define MOTOR_LEFT_PWM_A      (8u)   /* M1A - forward  */
#define MOTOR_LEFT_PWM_B      (9u)   /* M1B - backward */
#define MOTOR_RIGHT_PWM_A     (10u)  /* M2A - forward  */
#define MOTOR_RIGHT_PWM_B     (11u)  /* M2B - backward */

/* Wheel encoders, one Grove cable each (rewired 24 Oct).  Motor cable pins:
 * 01 GND -> Grove GND, 04 power -> Grove 3V3, 02 phase A -> GPIO below,
 * 03 phase B -> not used by the code yet.
 * Right encoder is on Grove 4 using GP17 only: the kernel drives GP16 as its
 * board-LED output at boot (BOARD_LED_PIN), so no encoder output may be
 * connected to GP16.  Leave the right motor's pin 03 unconnected. */
#define ENCODER_LEFT_A        (2u)   /* Grove 2: GP2 (phase B may go on GP3) */
#define ENCODER_RIGHT_A       (17u)  /* Grove 4: GP17 (GP16 left empty)     */

/* Servo for the ultrasonic scanner (Robo Pico servo header) */
#define SERVO_PWM_PIN         (15u)  /* GP15 */

/* HC-SR04+ ultrasonic sensor on Grove 1 (GND, 3V3, GP0, GP1), as in the
 * Week 6 design review.  GP0/GP1 are free because the console is USB. */
#ifndef ULTRASONIC_TRIG_PIN        /* can be overridden with ROBOT_CFLAGS */
#define ULTRASONIC_TRIG_PIN   (0u)   /* Grove 1: GP0 = Trig */
#define ULTRASONIC_ECHO_PIN   (1u)   /* Grove 1: GP1 = Echo */
#endif

/* LSM303DLHC accelerometer + compass on I2C0 (Grove 3) */
#define IMU_I2C_PORT          (0u)   /* i2c0 */
#define IMU_I2C_BAUDRATE      (100000u)
#define IMU_SDA_PIN           (4u)   /* GP4 */
#define IMU_SCL_PIN           (5u)   /* GP5 */
#define LSM303_ACCEL_ADDR     (0x19u)
#define LSM303_MAG_ADDR       (0x1Eu)

/* IR line sensors (analog) */
#define IR_SENSOR_LEFT_PIN    (26u)  /* ADC0 */
#define IR_SENSOR_CENTER_PIN  (27u)  /* ADC1 */
#define IR_SENSOR_RIGHT_PIN   (28u)  /* ADC2 */

#define IR_ADC_CHANNEL_LEFT   (0u)
#define IR_ADC_CHANNEL_CENTER (1u)
#define IR_ADC_CHANNEL_RIGHT  (2u)

/* Robo Pico on-board push buttons (active low) */
#define BUTTON_LINE_PIN       (21u)  /* GP21: start/stop line following     */

#endif /* PIN_CONFIG_H */
