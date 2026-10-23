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
 * On our car the LEFT wheel (seen from behind) is wired to M2 and the RIGHT
 * wheel to M1 - confirmed 10 Oct with a wheels-up line-follow test. */
#define MOTOR_LEFT_PWM_A      (10u)  /* M2A - forward  */
#define MOTOR_LEFT_PWM_B      (11u)  /* M2B - backward */
#define MOTOR_RIGHT_PWM_A     (8u)   /* M1A - forward  */
#define MOTOR_RIGHT_PWM_B     (9u)   /* M1B - backward */

/* Wheel encoders: the left green channel on GP3 produced no pulses, so use
 * the other left encoder channel already wired to GP2 (red).  Right uses
 * its red encoder channel on GP14. */
#define ENCODER_LEFT_A        (2u)   /* GP2: left encoder red */
#define ENCODER_RIGHT_A       (14u)  /* GP14: right encoder red */

/* Servo for the ultrasonic scanner (Robo Pico servo header) */
#define SERVO_PWM_PIN         (15u)  /* GP15 */

/* HC-SR04+ ultrasonic sensor (middle Grove socket).
 * Socket order: GND, 3V3, GP16, GP17.  The connected sensor uses
 * Echo = GP16 (white wire) and Trig = GP17 (yellow wire). */
#ifndef ULTRASONIC_TRIG_PIN        /* can be overridden with ROBOT_CFLAGS */
#define ULTRASONIC_TRIG_PIN   (17u)  /* GP17, yellow Trig wire */
#define ULTRASONIC_ECHO_PIN   (16u)  /* GP16, white Echo wire  */
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
