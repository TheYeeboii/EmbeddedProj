/**
 * @file main.c
 * @brief Complete Subsystem Diagnostic Hardware Test with OKY3231-2 Compass.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pin_config.h"

/* Volatile pulse counters for quadrature encoders updated via ISR */
static volatile int32_t g_left_encoder_ticks = 0;
static volatile int32_t g_right_encoder_ticks = 0;

/* Subsystem function prototypes */
static void test_motors_init(void);
static void test_motors_drive(uint16_t left_speed, uint16_t right_speed);
static void test_encoders_init(void);
static void gpio_callback(uint gpio, uint32_t events);
static void test_ir_sensors_init(void);
static void test_ir_sensors_read(uint16_t * p_left, uint16_t * p_center, uint16_t * p_right);
static void test_ultrasonic_init(void);
static uint32_t test_ultrasonic_read_cm(void);
static void test_servo_init(void);
static void test_servo_set_angle(uint8_t angle_deg);
static void test_compass_init(void);
static bool test_compass_read_mag(int16_t * p_x, int16_t * p_y, int16_t * p_z);

int main(void)
{
    /* Initialize stdio over USB */
    stdio_init_all();
    
    /* Startup delay for power rail stabilization */
    sleep_ms(2000);
    printf("=== ROVER HARDWARE DIAGNOSTIC TEST (FULL SUBSYSTEM) ===\n");

    /* Initialize active hardware peripherals */
    test_motors_init();
    test_encoders_init();
    test_ir_sensors_init();
    test_ultrasonic_init();
    test_servo_init();
    test_compass_init();

    printf("Diagnostic loop started.\n");

    uint16_t ir_left = 0u;
    uint16_t ir_center = 0u;
    uint16_t ir_right = 0u;
    uint32_t distance_cm = 0u;
    uint8_t sweep_angle = 0u;

    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;

    while (true)
    {
        /* 1. Read IR Line Sensors */
        test_ir_sensors_read(&ir_left, &ir_center, &ir_right);
        printf("IR Raw -> Left(GP26): %u | Center(GP27): %u | Right(GP28): %u\n", 
               ir_left, ir_center, ir_right);

        /* 2. Sweep Servo Motor & Read Ultrasonic Distance */
        test_servo_set_angle(sweep_angle);
        sleep_ms(100);
        distance_cm = test_ultrasonic_read_cm();
        printf("Servo: %u deg | Distance: %lu cm\n", sweep_angle, (unsigned long)distance_cm);

        sweep_angle = (uint8_t)((sweep_angle + 45u) % 180u);

        /* 3. Read Wheel Encoder Counts */
        printf("Encoder Ticks -> Left: %ld | Right: %ld\n", 
               (long)g_left_encoder_ticks, (long)g_right_encoder_ticks);

        /* 4. Read OKY3231-2 Compass Magnetometer */
        if (test_compass_read_mag(&mag_x, &mag_y, &mag_z))
        {
            float heading_rad = atan2f((float)mag_y, (float)mag_x);
            float heading_deg = heading_rad * (180.0f / 3.14159265f);
            if (heading_deg < 0.0f)
            {
                heading_deg += 360.0f;
            }
            printf("Compass -> Mag X: %d | Y: %d | Z: %d | Heading: %.1f deg\n", 
                   mag_x, mag_y, mag_z, heading_deg);
        }
        else
        {
            printf("Compass -> [ERROR] Read failed!\n");
        }

        /* 5. Pulse Motors Forward Briefly */
        printf("Pulsing Motors Forward...\n");
        test_motors_drive(25000u, 25000u);
        sleep_ms(250);
        test_motors_drive(0u, 0u);

        printf("------------------------------------------------\n");
        sleep_ms(1000);
    }

    return 0;
}

static void test_motors_init(void)
{
    gpio_set_function(MOTOR_LEFT_PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_LEFT_PWM_B, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM_B, GPIO_FUNC_PWM);

    uint slice_l = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM_A);
    uint slice_r = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM_A);

    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);
}

static void test_motors_drive(uint16_t left_speed, uint16_t right_speed)
{
    pwm_set_gpio_level(MOTOR_LEFT_PWM_A, left_speed);
    pwm_set_gpio_level(MOTOR_LEFT_PWM_B, 0u);
    pwm_set_gpio_level(MOTOR_RIGHT_PWM_A, right_speed);
    pwm_set_gpio_level(MOTOR_RIGHT_PWM_B, 0u);
}

static void gpio_callback(uint gpio, uint32_t events)
{
    if (ENCODER_LEFT_A == gpio)
    {
        g_left_encoder_ticks++;
    }
    else if (ENCODER_RIGHT_A == gpio)
    {
        g_right_encoder_ticks++;
    }
}

static void test_encoders_init(void)
{
    gpio_init(ENCODER_LEFT_A);
    gpio_set_dir(ENCODER_LEFT_A, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT_A);

    gpio_init(ENCODER_RIGHT_A);
    gpio_set_dir(ENCODER_RIGHT_A, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT_A);

    gpio_set_irq_enabled_with_callback(ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE, true);
}

static void test_ir_sensors_init(void)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_LEFT_PIN);
    adc_gpio_init(IR_SENSOR_CENTER_PIN);
    adc_gpio_init(IR_SENSOR_RIGHT_PIN);
}

static void test_ir_sensors_read(uint16_t * p_left, uint16_t * p_center, uint16_t * p_right)
{
    if ((NULL != p_left) && (NULL != p_center) && (NULL != p_right))
    {
        adc_select_input(IR_ADC_CHANNEL_LEFT);
        *p_left = adc_read();

        adc_select_input(IR_ADC_CHANNEL_CENTER);
        *p_center = adc_read();

        adc_select_input(IR_ADC_CHANNEL_RIGHT);
        *p_right = adc_read();
    }
}

static void test_ultrasonic_init(void)
{
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, false);

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
}

static uint32_t test_ultrasonic_read_cm(void)
{
    gpio_put(ULTRASONIC_TRIG_PIN, true);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_PIN, false);

    uint64_t start_wait = time_us_64();
    while (!gpio_get(ULTRASONIC_ECHO_PIN))
    {
        if ((time_us_64() - start_wait) > 30000u)
        {
            return 0u;
        }
    }

    uint64_t start_time = time_us_64();
    while (gpio_get(ULTRASONIC_ECHO_PIN))
    {
        if ((time_us_64() - start_time) > 30000u)
        {
            return 0u;
        }
    }
    uint64_t elapsed_time = time_us_64() - start_time;

    return (uint32_t)(elapsed_time / 58u);
}

static void test_servo_init(void)
{
    gpio_set_function(SERVO_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PWM_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_config_set_wrap(&config, 39062u);
    pwm_init(slice_num, &config, true);
}

static void test_servo_set_angle(uint8_t angle_deg)
{
    if (angle_deg > 180u)
    {
        angle_deg = 180u;
    }
    uint16_t duty_ticks = (uint16_t)(1000u + (((uint32_t)angle_deg * 2000u) / 180u));
    pwm_set_gpio_level(SERVO_PWM_PIN, duty_ticks);
}

static void test_compass_init(void)
{
    i2c_init(COMPASS_I2C_PORT, 100u * 1000u);
    gpio_set_function(COMPASS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(COMPASS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(COMPASS_SDA_PIN);
    gpio_pull_up(COMPASS_SCL_PIN);

    /* Configure LSM303 Magnetometer: CRA_REG_M (0x00) -> 15 Hz update rate */
    uint8_t cra_cfg[2] = {0x00u, 0x10u};
    i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, cra_cfg, 2u, false);

    /* Configure LSM303 Magnetometer: MR_REG_M (0x02) -> Continuous conversion mode */
    uint8_t mr_cfg[2] = {0x02u, 0x00u};
    i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, mr_cfg, 2u, false);
}

static bool test_compass_read_mag(int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
    if ((NULL == p_x) || (NULL == p_y) || (NULL == p_z))
    {
        return false;
    }

    uint8_t reg = 0x03u; /* OUT_X_H_M register address */
    if (i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, &reg, 1u, true) < 0)
    {
        return false;
    }

    uint8_t buffer[6] = {0};
    if (i2c_read_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, buffer, 6u, false) < 6)
    {
        return false;
    }

    /* LSM303DLHC Magnetometer output order: X, Z, Y (16-bit big endian) */
    *p_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *p_z = (int16_t)((buffer[2] << 8) | buffer[3]);
    *p_y = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}