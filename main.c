/**
 * @file main.c
 * @brief Buddy 1 rover firmware: Wi-Fi, command handling, and telemetry.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include "pin_config.h"
#include "buddy2_motion.h"
#include "buddy3_ir.h"

#ifndef WIFI_SSID
#define WIFI_SSID "WEISONG 4706"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "2W73)39f"
#endif

#define BUDDY1_ROBOT_ID "buddy1"
#define MQTT_TOPIC_COMMAND "rover/commands"
#define MQTT_TOPIC_STATUS "rover/status"
#define MQTT_TOPIC_HEARTBEAT "rover/heartbeat"
#define MQTT_TOPIC_TELEMETRY "rover/telemetry"
#define MQTT_TOPIC_DEBUG "rover/debug"
#define BUDDY1_MQTT_ENABLED 1
#define MOTOR_COMMAND_TIMEOUT_MS 1000u

static volatile int32_t g_left_encoder_ticks = 0;
static volatile int32_t g_right_encoder_ticks = 0;
static volatile uint16_t g_last_left_motor = 0u;
static volatile uint16_t g_last_right_motor = 0u;
static volatile uint32_t g_last_command_ms = 0u;

static void test_motors_init(void);
static void test_motors_drive(uint16_t left_speed, uint16_t right_speed);
static void test_motors_reverse(uint16_t left_speed, uint16_t right_speed);
static void test_encoders_init(void);
static void gpio_callback(uint gpio, uint32_t events);
static void test_ir_sensors_init(void);
static void test_ir_sensors_read(uint16_t *p_left, uint16_t *p_center, uint16_t *p_right);
static void test_ultrasonic_init(void);
static uint32_t test_ultrasonic_read_cm(void);
static void test_servo_init(void);
static void test_servo_set_angle(uint8_t angle_deg);
static void test_compass_init(void);
static bool test_compass_read_mag(int16_t *p_x, int16_t *p_y, int16_t *p_z);
static bool wifi_connect(void);
static bool mqtt_connect_broker(void);
static bool mqtt_publish_message(const char *topic, const char *payload);
static bool mqtt_subscribe_commands(void);
static bool mqtt_receive_commands(char *buffer, size_t buffer_size);
static bool read_console_command(char *payload, size_t payload_size);
static void build_telemetry_payload(char *payload,
                                   size_t payload_size,
                                   uint16_t ir_left,
                                   uint16_t ir_center,
                                   uint16_t ir_right,
                                   uint32_t distance_cm,
                                   bool compass_ok,
                                   float heading_deg,
                                   uint32_t ts_ms);
static bool parse_json_command(const char *payload, char *command_name, size_t command_name_size, int *left_val, int *right_val);
static void handle_command_payload(const char *payload);
static void print_telemetry(const char *telemetry);
static void print_heartbeat(uint32_t now_ms);

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("=== ROVER BUDDY 1: WIFI COMMUNICATION, COMMAND AND TELEMETRY ===\n");
    printf("Wi-Fi commands over USB serial: stop, forward, reverse, left, right, drive <left> <right>\n");

    test_motors_init();
    test_encoders_init();
    buddy3_init();
    test_ultrasonic_init();
    test_servo_init();
    test_compass_init();

    uint32_t next_telemetry_ms = 0u;
    uint32_t next_heartbeat_ms = 0u;
    uint32_t next_buddy3_debug_ms = 0u;
    uint8_t sweep_angle = 0u;
    uint16_t ir_left = 0u;
    uint16_t ir_center = 0u;
    uint16_t ir_right = 0u;
    uint32_t distance_cm = 0u;
    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;
    buddy2_motion_state_t motion_state;
    char telemetry_payload[256];
    char command_payload[128];
#if BUDDY1_MQTT_ENABLED
    char mqtt_command_buffer[128];
#endif

    while (true)
    {
        if (!wifi_connect())
        {
            printf("Wi-Fi unavailable; continuing in offline mode.\n");
            sleep_ms(1000);
            continue;
        }

    #if BUDDY1_MQTT_ENABLED
        if (!mqtt_connect_broker())
        {
            printf("MQTT broker unavailable; retrying in 5s.\n");
            sleep_ms(5000);
            continue;
        }
    #endif

        if (read_console_command(command_payload, sizeof(command_payload)))
        {
            printf("Command received: %s\n", command_payload);
            handle_command_payload(command_payload);
        }

    #if BUDDY1_MQTT_ENABLED
        if (mqtt_receive_commands(mqtt_command_buffer, sizeof(mqtt_command_buffer)))
        {
            printf("MQTT command received: %s\n", mqtt_command_buffer);
            handle_command_payload(mqtt_command_buffer);
        }
    #endif

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        buddy2_update(now_ms);
        buddy3_update(now_ms);
        uint8_t barcode_command = 0u;
        if (buddy3_take_barcode(&barcode_command))
        {
            buddy3_stop_line_follow();
            if (barcode_command == 1u)
            {
                printf("Barcode navigation: turn left\n");
                buddy2_turn_left(90.0f, 20000);
            }
            else if (barcode_command == 2u)
            {
                printf("Barcode navigation: turn right\n");
                buddy2_turn_right(90.0f, 20000);
            }
            else if (barcode_command == 3u)
            {
                printf("Barcode navigation: go straight\n");
                buddy2_move_forward(300.0f, 20000);
            }
        }
        buddy2_get_motion_state(&motion_state);
        g_left_encoder_ticks = motion_state.left_ticks;
        g_right_encoder_ticks = motion_state.right_ticks;
        if (now_ms >= next_buddy3_debug_ms)
        {
            buddy3_ir_state_t buddy3_state;
            buddy3_get_state(&buddy3_state);
            printf("Buddy3: raw=%u,%u,%u norm=%u,%u,%u position=%d line=%s junction=%s barcode=%s cmd=%u\n",
                   buddy3_state.raw[0], buddy3_state.raw[1], buddy3_state.raw[2],
                   buddy3_state.normalized[0], buddy3_state.normalized[1], buddy3_state.normalized[2],
                   buddy3_state.line_seen ? buddy3_state.line_position : 0,
                   buddy3_state.line_seen ? "yes" : "no",
                   buddy3_state.junction_seen ? "yes" : "no",
                   buddy3_state.barcode_ready ? "yes" : "no",
                   buddy3_state.barcode_command);
        #if BUDDY1_MQTT_ENABLED
                 char buddy3_debug_payload[256];
                 snprintf(buddy3_debug_payload,
                    sizeof(buddy3_debug_payload),
                    "{\"raw_left\":%u,\"raw_center\":%u,\"raw_right\":%u,\"norm_left\":%u,\"norm_center\":%u,\"norm_right\":%u,\"position\":%d,\"line\":%s,\"junction\":%s,\"barcode\":%s,\"command\":%u}",
                    buddy3_state.raw[0], buddy3_state.raw[1], buddy3_state.raw[2],
                    buddy3_state.normalized[0], buddy3_state.normalized[1], buddy3_state.normalized[2],
                    buddy3_state.line_position,
                    buddy3_state.line_seen ? "true" : "false",
                    buddy3_state.junction_seen ? "true" : "false",
                    buddy3_state.barcode_ready ? "true" : "false",
                    buddy3_state.barcode_command);
                 mqtt_publish_message(MQTT_TOPIC_DEBUG, buddy3_debug_payload);
        #endif
            next_buddy3_debug_ms = now_ms + 1000u;
        }
        if (!buddy2_motion_active() && !buddy3_line_following() &&
            ((g_last_left_motor != 0u) || (g_last_right_motor != 0u)) &&
            ((now_ms - g_last_command_ms) > MOTOR_COMMAND_TIMEOUT_MS))
        {
            test_motors_drive(0u, 0u);
            printf("Safety stop: motor command timed out.\n");
        }

        if (now_ms >= next_telemetry_ms)
        {
            test_ir_sensors_read(&ir_left, &ir_center, &ir_right);
            test_servo_set_angle(sweep_angle);
            sleep_ms(50);
            distance_cm = test_ultrasonic_read_cm();
            bool compass_ok = test_compass_read_mag(&mag_x, &mag_y, &mag_z);
            float heading_deg = 0.0f;
            if (compass_ok)
            {
                float heading_rad = atan2f((float)mag_y, (float)mag_x);
                heading_deg = heading_rad * (180.0f / 3.14159265f);
                if (heading_deg < 0.0f)
                {
                    heading_deg += 360.0f;
                }
            }

            build_telemetry_payload(telemetry_payload,
                                    sizeof(telemetry_payload),
                                    ir_left,
                                    ir_center,
                                    ir_right,
                                    distance_cm,
                                    compass_ok,
                                    heading_deg,
                                    now_ms);
            print_telemetry(telemetry_payload);
#if BUDDY1_MQTT_ENABLED
            mqtt_publish_message(MQTT_TOPIC_TELEMETRY, telemetry_payload);
#endif
            sweep_angle = (uint8_t)((sweep_angle + 45u) % 180u);
            next_telemetry_ms = now_ms + 3000u;
        }

        if (now_ms >= next_heartbeat_ms)
        {
            print_heartbeat(now_ms);
#if BUDDY1_MQTT_ENABLED
            bool heartbeat_published = mqtt_publish_message(MQTT_TOPIC_HEARTBEAT, "{\"robot_id\":\"buddy1\",\"state\":\"online\"}");
            if (!heartbeat_published)
            {
                printf("Heartbeat MQTT publish failed.\n");
            }
#endif
            next_heartbeat_ms = now_ms + 10000u;
        }

        cyw43_arch_poll();
        sleep_ms(10);
    }

    return 0;
}

static void test_motors_init(void)
{
    buddy2_motion_init();
}

static void test_motors_drive(uint16_t left_speed, uint16_t right_speed)
{
    g_last_left_motor = left_speed;
    g_last_right_motor = right_speed;
    g_last_command_ms = to_ms_since_boot(get_absolute_time());

    buddy2_stop();
    buddy2_set_motor_pwm((int16_t)(left_speed > 32767u ? 32767u : left_speed),
                         (int16_t)(right_speed > 32767u ? 32767u : right_speed));
}

static void test_motors_reverse(uint16_t left_speed, uint16_t right_speed)
{
    g_last_left_motor = left_speed;
    g_last_right_motor = right_speed;
    g_last_command_ms = to_ms_since_boot(get_absolute_time());

    buddy2_stop();
    buddy2_set_motor_pwm(-(int16_t)(left_speed > 32767u ? 32767u : left_speed),
                         -(int16_t)(right_speed > 32767u ? 32767u : right_speed));
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
}

static void test_ir_sensors_init(void)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_LEFT_PIN);
    adc_gpio_init(IR_SENSOR_CENTER_PIN);
    adc_gpio_init(IR_SENSOR_RIGHT_PIN);
}

static void test_ir_sensors_read(uint16_t *p_left, uint16_t *p_center, uint16_t *p_right)
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

    uint8_t cra_cfg[2] = {0x00u, 0x10u};
    i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, cra_cfg, 2u, false);

    uint8_t mr_cfg[2] = {0x02u, 0x00u};
    i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, mr_cfg, 2u, false);
}

static bool test_compass_read_mag(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
    if ((NULL == p_x) || (NULL == p_y) || (NULL == p_z))
    {
        return false;
    }

    uint8_t reg = 0x03u;
    if (i2c_write_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, &reg, 1u, true) < 0)
    {
        return false;
    }

    uint8_t buffer[6] = {0};
    if (i2c_read_blocking(COMPASS_I2C_PORT, LSM303_MAG_ADDR, buffer, 6u, false) < 6)
    {
        return false;
    }

    *p_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *p_z = (int16_t)((buffer[2] << 8) | buffer[3]);
    *p_y = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

static bool wifi_connect(void)
{
    static bool wifi_initialized = false;
    static bool wifi_connected = false;

    if (!wifi_initialized)
    {
        if (cyw43_arch_init())
        {
            printf("Failed to initialize CYW43 Wi-Fi stack.\n");
            return false;
        }

        cyw43_arch_enable_sta_mode();
        wifi_initialized = true;
    }

    int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status == CYW43_LINK_UP)
    {
        wifi_connected = true;
        return true;
    }

    if (wifi_connected && link_status != CYW43_LINK_DOWN && link_status != CYW43_LINK_FAIL)
    {
        return true;
    }

    wifi_connected = false;
    printf("Connecting to Wi-Fi SSID=%s ...\n", WIFI_SSID);
    int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (result != 0)
    {
        printf("Wi-Fi connection failed.\n");
        wifi_connected = false;
        return false;
    }

    wifi_connected = true;
    printf("Wi-Fi connected.\n");
    return true;
}

/* The raw lwIP MQTT client is used below; the earlier BSD-socket prototype
 * is retained only as a reference while the protocol callbacks are migrated. */
#if 0
static bool mqtt_connect_broker(void)
{
    static int mqtt_socket = -1;
    static bool mqtt_connected = false;

    if (mqtt_connected && mqtt_socket >= 0)
    {
        return true;
    }

    mqtt_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mqtt_socket < 0)
    {
        printf("MQTT socket creation failed.\n");
        return false;
    }

    struct hostent *host = gethostbyname(MQTT_BROKER_IP);
    if (host == NULL)
    {
        printf("MQTT DNS lookup failed for %s\n", MQTT_BROKER_IP);
        closesocket(mqtt_socket);
        mqtt_socket = -1;
        return false;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1883);
    memcpy(&addr.sin_addr, host->h_addr_list[0], sizeof(addr.sin_addr));

    if (connect(mqtt_socket, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        printf("MQTT broker connection failed.\n");
        closesocket(mqtt_socket);
        mqtt_socket = -1;
        return false;
    }

    const char *client_id = BUDDY1_ROBOT_ID;
    uint8_t packet[64];
    size_t index = 0u;
    packet[index++] = 0x10u;
    uint8_t remaining_length = (uint8_t)(10u + 2u + strlen(client_id));
    packet[index++] = remaining_length;
    packet[index++] = 0x00u;
    packet[index++] = 0x04u;
    packet[index++] = 'M';
    packet[index++] = 'Q';
    packet[index++] = 'T';
    packet[index++] = 'T';
    packet[index++] = 0x04u;
    packet[index++] = 0x02u;
    packet[index++] = 0x00u;
    packet[index++] = 0x3Cu;
    packet[index++] = 0x00u;
    packet[index++] = (uint8_t)(strlen(client_id) >> 8u);
    packet[index++] = (uint8_t)(strlen(client_id) & 0xFFu);
    memcpy(&packet[index], client_id, strlen(client_id));
    index += strlen(client_id);

    if (send(mqtt_socket, packet, index, 0) < 0)
    {
        printf("MQTT CONNECT send failed.\n");
        closesocket(mqtt_socket);
        mqtt_socket = -1;
        return false;
    }

    mqtt_connected = true;
    printf("MQTT stage: connected callback\n");
    printf("MQTT connected to broker %s.\n", MQTT_BROKER_IP);
    mqtt_subscribe_commands();
    return true;
}

static bool mqtt_publish_message(const char *topic, const char *payload)
{
    int mqtt_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mqtt_socket < 0)
    {
        return false;
    }

    static bool mqtt_connected = false;
    if (!mqtt_connected)
    {
        struct hostent *host = gethostbyname(MQTT_BROKER_IP);
        if (host == NULL)
        {
            closesocket(mqtt_socket);
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(1883);
        memcpy(&addr.sin_addr, host->h_addr_list[0], sizeof(addr.sin_addr));

        if (connect(mqtt_socket, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        {
            closesocket(mqtt_socket);
            return false;
        }

        mqtt_connected = true;
    }

    if (topic == NULL || payload == NULL)
    {
        closesocket(mqtt_socket);
        return false;
    }

    size_t topic_len = strlen(topic);
    size_t payload_len = strlen(payload);
    size_t total_len = 2u + topic_len + payload_len;
    uint8_t *packet = (uint8_t *)malloc(5u + total_len);
    if (packet == NULL)
    {
        closesocket(mqtt_socket);
        return false;
    }

    size_t idx = 0u;
    packet[idx++] = 0x30u;
    uint32_t remaining = (uint32_t)total_len;
    if (remaining < 128u)
    {
        packet[idx++] = (uint8_t)remaining;
    }
    else
    {
        packet[idx++] = 0x80u | (uint8_t)(remaining % 128u);
        packet[idx++] = (uint8_t)(remaining / 128u);
    }
    packet[idx++] = (uint8_t)(topic_len >> 8u);
    packet[idx++] = (uint8_t)(topic_len & 0xFFu);
    memcpy(&packet[idx], topic, topic_len);
    idx += topic_len;
    memcpy(&packet[idx], payload, payload_len);
    idx += payload_len;

    ssize_t sent = send(mqtt_socket, packet, idx, 0);
    free(packet);
    closesocket(mqtt_socket);
    mqtt_connected = (sent >= 0);
    return (sent >= 0);
}

static bool mqtt_subscribe_commands(void)
{
    static int mqtt_socket = -1;
    if (mqtt_socket < 0)
    {
        mqtt_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (mqtt_socket < 0)
        {
            return false;
        }

        struct hostent *host = gethostbyname(MQTT_BROKER_IP);
        if (host == NULL)
        {
            closesocket(mqtt_socket);
            mqtt_socket = -1;
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(1883);
        memcpy(&addr.sin_addr, host->h_addr_list[0], sizeof(addr.sin_addr));
        if (connect(mqtt_socket, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        {
            closesocket(mqtt_socket);
            mqtt_socket = -1;
            return false;
        }
    }

    const char *topic = MQTT_TOPIC_COMMAND;
    size_t topic_len = strlen(topic);
    uint8_t packet[64];
    size_t index = 0u;
    packet[index++] = 0x82u;
    packet[index++] = (uint8_t)(2u + 2u + topic_len + 1u);
    packet[index++] = 0x00u;
    packet[index++] = 0x01u;
    packet[index++] = (uint8_t)(topic_len >> 8u);
    packet[index++] = (uint8_t)(topic_len & 0xFFu);
    memcpy(&packet[index], topic, topic_len);
    index += topic_len;
    packet[index++] = 0x00u;

    ssize_t sent = send(mqtt_socket, packet, index, 0);
    if (sent < 0)
    {
        closesocket(mqtt_socket);
        mqtt_socket = -1;
        return false;
    }

    closesocket(mqtt_socket);
    mqtt_socket = -1;
    return true;
}

static bool mqtt_receive_commands(char *buffer, size_t buffer_size)
{
    static int mqtt_socket = -1;
    if (buffer == NULL || buffer_size == 0u)
    {
        return false;
    }

    if (mqtt_socket < 0)
    {
        mqtt_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (mqtt_socket < 0)
        {
            return false;
        }

        struct hostent *host = gethostbyname(MQTT_BROKER_IP);
        if (host == NULL)
        {
            closesocket(mqtt_socket);
            mqtt_socket = -1;
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(1883);
        memcpy(&addr.sin_addr, host->h_addr_list[0], sizeof(addr.sin_addr));
        if (connect(mqtt_socket, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        {
            closesocket(mqtt_socket);
            mqtt_socket = -1;
            return false;
        }
    }

    uint8_t temp[128];
    ssize_t received = recv(mqtt_socket, temp, sizeof(temp), MSG_DONTWAIT);
    if (received <= 0)
    {
        return false;
    }

    if (received < 2)
    {
        return false;
    }

    uint8_t header = temp[0];
    if ((header & 0xF0u) != 0x30u)
    {
        return false;
    }

    uint8_t remaining_length = temp[1];
    if (remaining_length < 2u)
    {
        return false;
    }

    size_t offset = 2u;
    uint16_t topic_len = (uint16_t)(((uint16_t)temp[offset] << 8u) | (uint16_t)temp[offset + 1u]);
    offset += 2u;
    if (offset + topic_len > (size_t)received)
    {
        return false;
    }

    const char *topic = (const char *)&temp[offset];
    (void)topic;
    offset += topic_len;
    if (offset >= (size_t)received)
    {
        return false;
    }

    size_t payload_len = (size_t)received - offset;
    if (payload_len >= buffer_size)
    {
        payload_len = buffer_size - 1u;
    }

    memcpy(buffer, &temp[offset], payload_len);
    buffer[payload_len] = '\0';
    closesocket(mqtt_socket);
    mqtt_socket = -1;
    return true;
}
#endif

static mqtt_client_t *g_mqtt_client = NULL;
static bool g_mqtt_connecting = false;
static bool g_mqtt_reconnect_pending = false;
static bool g_mqtt_subscription_pending = false;
static char g_mqtt_command[128];
static size_t g_mqtt_command_length = 0u;
static bool g_mqtt_command_ready = false;

static void mqtt_request_complete(void *arg, err_t error)
{
    (void)arg;
    if (error != ERR_OK)
    {
        printf("MQTT request failed: %d\n", error);
    }
}

static void mqtt_incoming_data(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    (void)arg;
    if (data == NULL || len == 0u)
    {
        return;
    }

    size_t available = (sizeof(g_mqtt_command) - 1u) - g_mqtt_command_length;
    size_t copy_len = len;
    if (copy_len > available)
    {
        copy_len = available;
    }
    memcpy(&g_mqtt_command[g_mqtt_command_length], data, copy_len);
    g_mqtt_command_length += copy_len;
    g_mqtt_command[g_mqtt_command_length] = '\0';
    g_mqtt_command_ready = ((flags & MQTT_DATA_FLAG_LAST) != 0u);
}

static void mqtt_incoming_publish(void *arg, const char *topic, u32_t total_length)
{
    (void)arg;
    g_mqtt_command_length = 0u;
    g_mqtt_command_ready = false;
    printf("MQTT message received on %s (%lu bytes)\n",
           topic != NULL ? topic : "(unknown)",
           (unsigned long)total_length);
}

static void mqtt_connected(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;
    g_mqtt_connecting = false;
    if (status != MQTT_CONNECT_ACCEPTED)
    {
        g_mqtt_reconnect_pending = true;
        g_mqtt_subscription_pending = false;
        printf("MQTT connection failed: %d\n", (int)status);
        return;
    }

    g_mqtt_reconnect_pending = false;
    printf("MQTT connected to broker %s.\n", MQTT_BROKER_IP);
    g_mqtt_subscription_pending = true;
}

static void mqtt_dns_resolved(const char *name, const ip_addr_t *ipaddr, void *arg)
{
    (void)name;
    (void)arg;
    if (ipaddr == NULL || g_mqtt_client == NULL)
    {
        g_mqtt_connecting = false;
        printf("MQTT DNS lookup failed for %s\n", MQTT_BROKER_IP);
        return;
    }

    static const struct mqtt_connect_client_info_t client_info = {
        .client_id = BUDDY1_ROBOT_ID,
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive = 60u,
        .will_topic = MQTT_TOPIC_STATUS,
        .will_msg = "{\"robot_id\":\"buddy1\",\"state\":\"offline\"}",
        .will_msg_len = 0u,
        .will_qos = 0u,
        .will_retain = 1u
    };
    cyw43_arch_lwip_begin();
    printf("MQTT stage: connect start\n");
    err_t connect_result = mqtt_client_connect(g_mqtt_client, ipaddr, MQTT_PORT, mqtt_connected, NULL, &client_info);
    cyw43_arch_lwip_end();
    printf("MQTT stage: connect queued result=%d\n", connect_result);
}

static bool mqtt_connect_broker(void)
{
    if (g_mqtt_client == NULL)
    {
        cyw43_arch_lwip_begin();
        g_mqtt_client = mqtt_client_new();
        if (g_mqtt_client == NULL)
        {
            cyw43_arch_lwip_end();
            printf("MQTT client allocation failed.\n");
            return false;
        }
        mqtt_set_inpub_callback(g_mqtt_client, mqtt_incoming_publish, mqtt_incoming_data, NULL);
        cyw43_arch_lwip_end();
    }

    if (g_mqtt_reconnect_pending)
    {
        cyw43_arch_lwip_begin();
        mqtt_disconnect(g_mqtt_client);
        cyw43_arch_lwip_end();
        g_mqtt_reconnect_pending = false;
    }

    cyw43_arch_lwip_begin();
    bool client_connected = mqtt_client_is_connected(g_mqtt_client) != 0u;
    cyw43_arch_lwip_end();
    if (client_connected)
    {
        if (g_mqtt_subscription_pending)
        {
            if (mqtt_subscribe_commands())
            {
                g_mqtt_subscription_pending = false;
            }
        }
        return true;
    }
    if (g_mqtt_connecting)
    {
        return true;
    }

    g_mqtt_connecting = true;
    ip_addr_t broker_address;
    if (ipaddr_aton(MQTT_BROKER_IP, &broker_address))
    {
        mqtt_dns_resolved(MQTT_BROKER_IP, &broker_address, NULL);
    }
    else
    {
        err_t dns_result = dns_gethostbyname(MQTT_BROKER_IP, &broker_address, mqtt_dns_resolved, NULL);
        if (dns_result != ERR_OK && dns_result != ERR_INPROGRESS)
        {
            g_mqtt_connecting = false;
            printf("MQTT DNS request failed: %d\n", dns_result);
        }
    }
    return true;
}

static bool mqtt_publish_message(const char *topic, const char *payload)
{
    if (g_mqtt_client == NULL || topic == NULL || payload == NULL)
    {
        return false;
    }

    cyw43_arch_lwip_begin();
    bool client_connected = mqtt_client_is_connected(g_mqtt_client) != 0u;
    cyw43_arch_lwip_end();
    if (!client_connected)
    {
        return false;
    }

    cyw43_arch_lwip_begin();
    printf("MQTT stage: publish start topic=%s\n", topic);
    err_t result = mqtt_publish(g_mqtt_client,
                                topic,
                                payload,
                                (u16_t)strlen(payload),
                                0u,
                                0u,
                                mqtt_request_complete,
                                NULL);
    cyw43_arch_lwip_end();
    printf("MQTT stage: publish result=%d\n", result);
    return result == ERR_OK;
}

static bool mqtt_subscribe_commands(void)
{
    if (g_mqtt_client == NULL)
    {
        return false;
    }
    cyw43_arch_lwip_begin();
    printf("MQTT stage: subscribe start\n");
    err_t result = mqtt_subscribe(g_mqtt_client, MQTT_TOPIC_COMMAND, 0u, mqtt_request_complete, NULL);
    cyw43_arch_lwip_end();
    printf("MQTT stage: subscribe result=%d\n", result);
    return result == ERR_OK;
}

static bool mqtt_receive_commands(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0u || !g_mqtt_command_ready)
    {
        return false;
    }
    snprintf(buffer, buffer_size, "%s", g_mqtt_command);
    g_mqtt_command_length = 0u;
    g_mqtt_command_ready = false;
    return true;
}

static bool read_console_command(char *payload, size_t payload_size)
{
    static char command_buffer[64];
    static size_t command_index = 0u;

    if (payload == NULL || payload_size == 0u)
    {
        return false;
    }

    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT)
    {
        payload[0] = '\0';
        return false;
    }

    if (ch == '\r' || ch == '\n')
    {
        if (command_index == 0u)
        {
            payload[0] = '\0';
            return false;
        }

        command_buffer[command_index] = '\0';
        snprintf(payload, payload_size, "%s", command_buffer);
        command_index = 0u;
        return true;
    }

    if (command_index < (sizeof(command_buffer) - 1u))
    {
        command_buffer[command_index++] = (char)ch;
    }

    payload[0] = '\0';
    return false;
}

static bool extract_json_int(const char *payload, const char *key, int *value)
{
    if ((payload == NULL) || (key == NULL) || (value == NULL))
    {
        return false;
    }

    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *key_ptr = strstr(payload, pattern);
    if (key_ptr == NULL)
    {
        return false;
    }

    const char *colon = strchr(key_ptr, ':');
    if (colon == NULL)
    {
        return false;
    }

    const char *num_ptr = colon + 1;
    while ((*num_ptr == ' ') || (*num_ptr == '\t') || (*num_ptr == '\r') || (*num_ptr == '\n'))
    {
        num_ptr++;
    }

    char *end_ptr = NULL;
    long parsed = strtol(num_ptr, &end_ptr, 10);
    if (end_ptr == num_ptr)
    {
        return false;
    }

    *value = (int)parsed;
    return true;
}

static bool parse_json_command(const char *payload, char *command_name, size_t command_name_size, int *left_val, int *right_val)
{
    if ((payload == NULL) || (command_name == NULL) || (left_val == NULL) || (right_val == NULL))
    {
        return false;
    }

    const char *cmd_ptr = strstr(payload, "\"cmd\"");
    if (cmd_ptr == NULL)
    {
        cmd_ptr = strstr(payload, "\"command\"");
    }
    if (cmd_ptr == NULL)
    {
        return false;
    }

    const char *colon = strchr(cmd_ptr, ':');
    if (colon == NULL)
    {
        return false;
    }

    const char *value = colon + 1;
    while ((*value == ' ') || (*value == '\t') || (*value == '\r') || (*value == '\n'))
    {
        value++;
    }
    if (*value != '"')
    {
        return false;
    }

    value++;
    const char *end = strchr(value, '"');
    if (end == NULL)
    {
        return false;
    }

    size_t len = (size_t)(end - value);
    if (len >= command_name_size)
    {
        len = command_name_size - 1u;
    }
    memcpy(command_name, value, len);
    command_name[len] = '\0';

    *left_val = -1;
    *right_val = -1;

    int parsed_left = 0;
    int parsed_right = 0;
    bool left_found = extract_json_int(payload, "left", &parsed_left);
    bool right_found = extract_json_int(payload, "right", &parsed_right);
    if (left_found)
    {
        *left_val = parsed_left;
    }
    if (right_found)
    {
        *right_val = parsed_right;
    }

    return true;
}

static void build_telemetry_payload(char *payload,
                                   size_t payload_size,
                                   uint16_t ir_left,
                                   uint16_t ir_center,
                                   uint16_t ir_right,
                                   uint32_t distance_cm,
                                   bool compass_ok,
                                   float heading_deg,
                                   uint32_t ts_ms)
{
    if ((payload == NULL) || (payload_size == 0u))
    {
        return;
    }

    const char *state = compass_ok ? "online" : "compass_error";
    snprintf(payload,
             payload_size,
             "{\"robot_id\":\"%s\",\"ts_ms\":%lu,\"state\":\"%s\",\"left_ir\":%u,\"center_ir\":%u,\"right_ir\":%u,\"distance_cm\":%lu,\"heading_deg\":%.1f,\"left_ticks\":%ld,\"right_ticks\":%ld,\"motor_left\":%u,\"motor_right\":%u,\"status\":\"%s\"}",
             BUDDY1_ROBOT_ID,
             (unsigned long)ts_ms,
             state,
             ir_left,
             ir_center,
             ir_right,
             (unsigned long)distance_cm,
             heading_deg,
             (long)g_left_encoder_ticks,
             (long)g_right_encoder_ticks,
             g_last_left_motor,
             g_last_right_motor,
             state);
}

static void handle_command_payload(const char *payload)
{
    if (payload == NULL || payload[0] == '\0')
    {
        return;
    }

    char json_command[16];
    int left_val = 0;
    int right_val = 0;
    if (payload[0] == '{' && parse_json_command(payload, json_command, sizeof(json_command), &left_val, &right_val))
    {
        if (strcmp(json_command, "line_follow") == 0)
        {
            buddy3_start_line_follow();
            printf("Buddy 3 line following enabled\n");
            return;
        }
        if (strcmp(json_command, "line_stop") == 0)
        {
            buddy3_stop_line_follow();
            printf("Buddy 3 line following stopped\n");
            return;
        }
        if (strcmp(json_command, "stop") == 0)
        {
            test_motors_drive(0u, 0u);
            printf("Command accepted: stop via JSON\n");
            return;
        }
        if (strcmp(json_command, "forward") == 0)
        {
            test_motors_drive(30000u, 30000u);
            printf("Command accepted: forward via JSON\n");
            return;
        }
        if (strcmp(json_command, "reverse") == 0)
        {
            test_motors_reverse(20000u, 20000u);
            printf("Command accepted: reverse via JSON\n");
            return;
        }
        if (strcmp(json_command, "left") == 0)
        {
            test_motors_drive(0u, 30000u);
            printf("Command accepted: left via JSON\n");
            return;
        }
        if (strcmp(json_command, "right") == 0)
        {
            test_motors_drive(30000u, 0u);
            printf("Command accepted: right via JSON\n");
            return;
        }
        if (strcmp(json_command, "drive") == 0 &&
            left_val >= 0 && left_val <= 65535 &&
            right_val >= 0 && right_val <= 65535)
        {
            test_motors_drive((uint16_t)left_val, (uint16_t)right_val);
            printf("Command accepted: drive %d %d via JSON\n", left_val, right_val);
            return;
        }
        int distance_mm = 0;
        int angle_deg = 0;
        int motion_pwm = 20000;
        extract_json_int(payload, "distance_mm", &distance_mm);
        extract_json_int(payload, "angle_deg", &angle_deg);
        int parsed_pwm = 0;
        if (extract_json_int(payload, "max_pwm", &parsed_pwm))
        {
            motion_pwm = parsed_pwm;
        }
        if (strcmp(json_command, "move_forward") == 0 && distance_mm > 0 && motion_pwm > 0 && motion_pwm <= 32767)
        {
            printf("Motion accepted: forward %d mm\n", distance_mm);
            buddy2_move_forward((float)distance_mm, (int16_t)motion_pwm);
            return;
        }
        if (strcmp(json_command, "move_backward") == 0 && distance_mm > 0 && motion_pwm > 0 && motion_pwm <= 32767)
        {
            printf("Motion accepted: backward %d mm\n", distance_mm);
            buddy2_move_backward((float)distance_mm, (int16_t)motion_pwm);
            return;
        }
        if (strcmp(json_command, "turn_left") == 0 && angle_deg > 0 && motion_pwm > 0 && motion_pwm <= 32767)
        {
            printf("Motion accepted: left %d degrees\n", angle_deg);
            buddy2_turn_left((float)angle_deg, (int16_t)motion_pwm);
            return;
        }
        if (strcmp(json_command, "turn_right") == 0 && angle_deg > 0 && motion_pwm > 0 && motion_pwm <= 32767)
        {
            printf("Motion accepted: right %d degrees\n", angle_deg);
            buddy2_turn_right((float)angle_deg, (int16_t)motion_pwm);
            return;
        }
        return;
    }

    if (strcmp(payload, "stop") == 0)
    {
        test_motors_drive(0u, 0u);
        printf("Command accepted: stop\n");
        return;
    }

    if (strcmp(payload, "forward") == 0)
    {
        test_motors_drive(30000u, 30000u);
        printf("Command accepted: forward\n");
        return;
    }

    if (strcmp(payload, "reverse") == 0)
    {
        test_motors_reverse(20000u, 20000u);
        printf("Command accepted: reverse\n");
        return;
    }

    if (strcmp(payload, "left") == 0)
    {
        test_motors_drive(0u, 30000u);
        printf("Command accepted: left\n");
        return;
    }

    if (strcmp(payload, "right") == 0)
    {
        test_motors_drive(30000u, 0u);
        printf("Command accepted: right\n");
        return;
    }

    int distance_mm = 0;
    int angle_deg = 0;
    int motion_pwm = 20000;
    if (sscanf(payload, "move_forward %d %d", &distance_mm, &motion_pwm) >= 1 &&
        strncmp(payload, "move_forward", 12) == 0 && distance_mm > 0 && motion_pwm > 0 && motion_pwm <= 32767)
    {
        buddy2_move_forward((float)distance_mm, (int16_t)motion_pwm);
        return;
    }
    if (sscanf(payload, "move_backward %d %d", &distance_mm, &motion_pwm) >= 1 &&
        strncmp(payload, "move_backward", 13) == 0 && distance_mm > 0 && motion_pwm > 0 && motion_pwm <= 32767)
    {
        buddy2_move_backward((float)distance_mm, (int16_t)motion_pwm);
        return;
    }
    if (sscanf(payload, "turn_left %d %d", &angle_deg, &motion_pwm) >= 1 &&
        strncmp(payload, "turn_left", 9) == 0 && angle_deg > 0 && motion_pwm > 0 && motion_pwm <= 32767)
    {
        buddy2_turn_left((float)angle_deg, (int16_t)motion_pwm);
        return;
    }
    if (sscanf(payload, "turn_right %d %d", &angle_deg, &motion_pwm) >= 1 &&
        strncmp(payload, "turn_right", 10) == 0 && angle_deg > 0 && motion_pwm > 0 && motion_pwm <= 32767)
    {
        buddy2_turn_right((float)angle_deg, (int16_t)motion_pwm);
        return;
    }

    char *copy = (char *)malloc(strlen(payload) + 1u);
    if (copy != NULL)
    {
        strcpy(copy, payload);
        char *token = strtok(copy, " \t");
        if (token != NULL && strcmp(token, "drive") == 0)
        {
            char *left_str = strtok(NULL, " \t");
            char *right_str = strtok(NULL, " \t");
            if (left_str != NULL && right_str != NULL)
            {
                long left_val = strtol(left_str, NULL, 10);
                long right_val = strtol(right_str, NULL, 10);
                if (left_val < 0L)
                {
                    left_val = 0L;
                }
                if (right_val < 0L)
                {
                    right_val = 0L;
                }
                if (left_val > 65535L)
                {
                    left_val = 65535L;
                }
                if (right_val > 65535L)
                {
                    right_val = 65535L;
                }
                test_motors_drive((uint16_t)left_val, (uint16_t)right_val);
                printf("Command accepted: drive %ld %ld\n", left_val, right_val);
            }
        }
        free(copy);
    }
}

static void print_telemetry(const char *telemetry)
{
    printf("Telemetry: %s\n", telemetry);
}

static void print_heartbeat(uint32_t now_ms)
{
    printf("Heartbeat: topic=%s status=online robot=%s uptime_ms=%lu\n",
           MQTT_TOPIC_HEARTBEAT,
           BUDDY1_ROBOT_ID,
           (unsigned long)now_ms);
}
