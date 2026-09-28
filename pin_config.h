#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

/* Motor Pins (Cytron Robo Pico Hardwired Mapping) */
#define MOTOR_LEFT_PWM_A      (8u)   /* M1A */
#define MOTOR_LEFT_PWM_B      (9u)   /* M1B */
#define MOTOR_RIGHT_PWM_A     (10u)  /* M2A */
#define MOTOR_RIGHT_PWM_B     (11u)  /* M2B */

/* Servo Motor Pin */
#define SERVO_PWM_PIN         (15u)  /* GP15 */

/* Ultrasonic Sensor Pins */
#define ULTRASONIC_TRIG_PIN   (0u)   /* GP0 */
#define ULTRASONIC_ECHO_PIN   (1u)   /* GP1 */

/* OKY3231-2 (LSM303DLHC) E-Compass (I2C0 on Grove 4) */
#define COMPASS_I2C_PORT      (i2c0)
#define COMPASS_SDA_PIN       (4u)   /* GP4 */
#define COMPASS_SCL_PIN       (5u)   /* GP5 */
#define LSM303_ACCEL_ADDR     (0x19u)
#define LSM303_MAG_ADDR       (0x1Eu)

/* IR Line Sensor ADC Channels */
#define IR_SENSOR_LEFT_PIN    (26u)  /* ADC0 */
#define IR_SENSOR_CENTER_PIN  (27u)  /* ADC1 */
#define IR_SENSOR_RIGHT_PIN   (28u)  /* ADC2 */

#define IR_ADC_CHANNEL_LEFT   (0u)
#define IR_ADC_CHANNEL_CENTER (1u)
#define IR_ADC_CHANNEL_RIGHT  (2u)

/* Encoder Pins */
#define ENCODER_LEFT_A        (2u)   /* GP2 */
#define ENCODER_RIGHT_A       (3u)   /* GP3 */

#endif /* PIN_CONFIG_H */