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
#define MOTOR_LEFT_PWM_A  (8u)  /* M1A - forward  */
#define MOTOR_LEFT_PWM_B  (9u)  /* M1B - backward */
#define MOTOR_RIGHT_PWM_A (10u) /* M2A - forward  */
#define MOTOR_RIGHT_PWM_B (11u) /* M2B - backward */

/* Wheel encoders (26 Oct).  Motor cable pins: 01 GND, 02 phase A,
 * 03 phase B, 04 power (3V3), 05/06 motor.  Each encoder needs 01 -> GND,
 * 04 -> 3V3 and ONE phase on the GPIO below; the code counts rising edges of
 * that one phase only.
 *   Left  -> Grove 2: 02 (phase A) -> GP2.  Pin 03 (phase B) and the
 *            Grove 2 GP3 wire are NOT connected.
 *   Right -> Grove 4: 02 (phase A) -> GP17.  Pin 03 and the Grove 4 GP16
 *            wire are NOT connected.
 * Pin safety: the kernel claims GP0/GP1 (UART0) and GP16 (board LED) at
 * boot.  usermain() first releases every Grove signal pin (GP0-GP3,
 * GP16-GP17) as a pulled-down input; modules then set up only the pins
 * listed in this file.  Still, never wire a sensor or encoder OUTPUT to a
 * pin this file does not read. */
#define ENCODER_LEFT_A  (2u)  /* Grove 2: GP2 = left phase A (pin 02) */
#define ENCODER_RIGHT_A (17u) /* Grove 4: GP17 = right phase A        */

/* Servo for the ultrasonic scanner (Robo Pico servo port GP15) */
#define SERVO_PWM_PIN (15u) /* GP15 */

/* HC-SR04+ ultrasonic sensor on Grove 1 (GND, 3V3, GP0, GP1), as in the
 * Week 6 design review.  GP0 (Trig) is a Pico OUTPUT and GP1 (Echo) an
 * input; the driver takes both pins back from the kernel's UART at start-up.
 * Sensor Vcc -> 3V3, Gnd -> GND: a reversed pair shorts the 3.3 V supply. */
#ifndef ULTRASONIC_TRIG_PIN      /* can be overridden with ROBOT_CFLAGS */
#define ULTRASONIC_TRIG_PIN (0u) /* Grove 1: GP0 = Trig */
#define ULTRASONIC_ECHO_PIN (1u) /* Grove 1: GP1 = Echo */
#endif

/* LSM303DLHC accelerometer + compass on I2C0 (Grove 3) */
#define IMU_I2C_PORT      (0u) /* i2c0 */
#define IMU_I2C_BAUDRATE  (100000u)
#define IMU_SDA_PIN       (4u) /* GP4 */
#define IMU_SCL_PIN       (5u) /* GP5 */
#define LSM303_ACCEL_ADDR (0x19u)
#define LSM303_MAG_ADDR   (0x1Eu)

/* IR line sensors (analog) */
#define IR_SENSOR_LEFT_PIN   (26u) /* ADC0 */
#define IR_SENSOR_CENTER_PIN (27u) /* ADC1 */
#define IR_SENSOR_RIGHT_PIN  (28u) /* ADC2 */

#define IR_ADC_CHANNEL_LEFT   (0u)
#define IR_ADC_CHANNEL_CENTER (1u)
#define IR_ADC_CHANNEL_RIGHT  (2u)

/* Barcode sensor: its digital DO output is wired to GP28 and reads HIGH on
 * black. Both front sensors stay on their analog outputs. */
#define BARCODE_DIGITAL_PIN         IR_SENSOR_RIGHT_PIN
#define BARCODE_DIGITAL_BLACK_LEVEL (1u)

/* Robo Pico on-board push buttons (active low) */
#define BUTTON_LINE_PIN (21u) /* GP21: start/stop line following     */

#endif /* PIN_CONFIG_H */

/*** end of file ***/

