#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "robot_packed.h"

/* Minimal nonblocking kernel mailbox model for the actual comms.c. */
#define TEST_TKERNEL_H
typedef unsigned int UINT;
typedef int INT;
typedef int ID;
typedef int TMO;
typedef unsigned char UB;
typedef unsigned long SZ;
typedef struct { uint32_t hi; uint32_t lo; } SYSTIM;
typedef struct { void *exinf; unsigned mbfatr; SZ bufsz; INT maxmsz; } T_CMBF;
#define E_OK 0
#define TA_TFIFO 0
#define TMO_POL 0
int tk_get_otm(SYSTIM *);
int tk_cre_mbf(const T_CMBF *);
int tk_snd_mbf(ID, const void *, INT, int);
int tk_rcv_mbf(ID, void *, int);

#include "../../app_program/subsystem1/comms.c"
#define TM_WIFI_MQTT 1
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"

typedef struct {
    uint8_t data[4][COMMS_PAYLOAD_MAX + COMMS_MESSAGE_HEADER];
    unsigned sizes[4];
    unsigned head;
    unsigned count;
} fake_mailbox_t;
static fake_mailbox_t boxes[2];
static unsigned created;
static uint32_t clock_ms;
static uint64_t clock_us;
static mqtt_client_t client = {1};
static err_t publish_result;
static unsigned attempts;
static unsigned delivered;
static uint8_t received[16][COMMS_PAYLOAD_MAX];
static uint32_t received_size[16];
static char received_topic[16][32];

int tk_get_otm(SYSTIM * time)
{
    time->hi = 0;
    time->lo = clock_ms;
    return E_OK;
}
int tk_cre_mbf(const T_CMBF * config)
{
    assert(config->maxmsz <= (INT) sizeof boxes[0].data[0]);
    return (int) ++created;
}
int tk_snd_mbf(ID id, const void * data, INT length, int timeout)
{
    fake_mailbox_t * box = &boxes[id - 1];
    assert(timeout == TMO_POL);
    if (box->count == 4) return -50;
    unsigned slot = (box->head + box->count) % 4;
    memcpy(box->data[slot], data, (unsigned) length);
    box->sizes[slot] = (unsigned) length;
    box->count++;
    return E_OK;
}
int tk_rcv_mbf(ID id, void * data, int timeout)
{
    fake_mailbox_t * box = &boxes[id - 1];
    assert(timeout == TMO_POL);
    if (!box->count) return -50;
    unsigned length = box->sizes[box->head];
    memcpy(data, box->data[box->head], length);
    box->head = (box->head + 1) % 4;
    box->count--;
    return (int) length;
}
void robot_fault_clear(uint8_t mask) { (void) mask; }
void robot_fault_set(uint8_t mask) { (void) mask; }
void robot_fault_update(uint8_t clear, uint8_t set) { (void) clear; (void) set; }
uint64_t time_us_64(void) { return clock_us; }
int ipaddr_aton(const char * text, ip_addr_t * address)
{
    (void) text; address->addr = 1; return 1;
}
mqtt_client_t * mqtt_client_new(void) { return &client; }
int mqtt_client_is_connected(mqtt_client_t * pointer) { return pointer->connected; }
void mqtt_set_inpub_callback(mqtt_client_t * pointer,
 void (*publish)(void *, const char *, u32_t),
 void (*data)(void *, const u8_t *, u16_t, u8_t), void * arg)
{
    (void) pointer; (void) publish; (void) data; (void) arg;
}
err_t mqtt_client_connect(mqtt_client_t * pointer, const ip_addr_t * address,
 u16_t port, void (*callback)(mqtt_client_t *, void *, mqtt_connection_status_t),
 void * arg, const struct mqtt_connect_client_info_t * config)
{
    (void) pointer; (void) address; (void) port; (void) callback;
    (void) arg; (void) config;
    return ERR_CONN;
}
err_t mqtt_subscribe(mqtt_client_t * pointer, const char * topic, u8_t qos,
 void * callback, void * arg)
{
    (void) pointer; (void) topic; (void) qos; (void) callback; (void) arg;
    return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t * pointer, const char * topic,
 const void * data, u16_t length, u8_t qos, u8_t retain, void * callback, void * arg)
{
    (void) pointer; (void) qos; (void) retain; (void) callback; (void) arg;
    attempts++;
    if (publish_result != ERR_OK) return publish_result;
    assert(delivered < 16 && length <= COMMS_PAYLOAD_MAX);
    memcpy(received[delivered], data, length);
    received_size[delivered] = length;
    strcpy(received_topic[delivered], topic);
    delivered++;
    return ERR_OK;
}
static void advance(unsigned ms)
{
    clock_ms += ms;
    clock_us += (uint64_t) ms * 1000;
}

static void test_lag_fields(void)
{
    const uint32_t types[] = {9, 10, 11, 12};
    const uint32_t offsets[] = {104, 155, 40, 88};
    for (unsigned i = 0; i < 4; ++i) {
        uint8_t bytes[32];
        robot_packed_writer_t writer;
        robot_packed_reader_t reader;
        memset(bytes, 0x55, sizeof bytes);
        bytes[0] = (uint8_t) ((types[i] << 4) | 1);
        uint8_t original[32];
        memcpy(original, bytes, sizeof bytes);
        robot_packed_reader_init(&reader, bytes, sizeof bytes);
        reader.bit_pos = offsets[i];
        unsigned base = robot_packed_get(&reader, 16);
        robot_wire_add_transport_age(bytes, sizeof bytes, 9);
        robot_packed_reader_init(&reader, bytes, sizeof bytes);
        reader.bit_pos = offsets[i];
        assert(robot_packed_get(&reader, 16) == base + 9);
        for (unsigned bit = 0; bit < sizeof bytes * 8; ++bit) {
            if (bit >= offsets[i] && bit < offsets[i] + 16) continue;
            unsigned mask = 1u << (7 - bit % 8);
            assert((bytes[bit/8] & mask) == (original[bit/8] & mask));
        }
        robot_wire_add_transport_age(bytes, sizeof bytes, UINT32_MAX);
        robot_packed_reader_init(&reader, bytes, sizeof bytes);
        reader.bit_pos = offsets[i];
        assert(robot_packed_get(&reader, 16) == 65535);
        robot_packed_writer_init(&writer, bytes, 1);
        bytes[0] = (uint8_t) ((types[i] << 4) | 1);
        robot_wire_add_transport_age(bytes, 1, 2000); /* Truncated: preserved. */
        assert(bytes[0] == (uint8_t) ((types[i] << 4) | 1));
    }
}

int main(void)
{
    struct netif netif = {1, 1};
    const uint8_t first[] = {0x21, 0x00, 0xff, 0x00};
    const uint8_t second[] = {0x31};
    uint8_t command[COMMS_COMMAND_MAX];
    uint32_t length;

    assert(comms_init() == 0);
    robot_mqtt_poll(&netif);
    assert(comms_publish(COMMS_TOPIC_DEBUG, first, sizeof first));
    assert(comms_publish(COMMS_TOPIC_HEARTBEAT, second, sizeof second));
    advance(50);
    publish_result = ERR_MEM;
    robot_mqtt_poll(&netif);
    assert(attempts == 1 && delivered == 0 && boxes[0].count == 1);
    advance(1000);
    robot_mqtt_poll(&netif);
    assert(attempts == 2 && boxes[0].count == 1);
    /* Commands continue to arrive while outgoing MQTT is backpressured. */
    robot_mqtt_incoming_publish(NULL, "rover/commands", 1);
    robot_mqtt_incoming_data(NULL, (const u8_t *) "\x10", 1, MQTT_DATA_FLAG_LAST);
    assert(comms_wait_command(command, sizeof command, &length, TMO_POL));
    assert(length == 1 && command[0] == 0x10);
    publish_result = ERR_OK;
    robot_mqtt_poll(&netif);
    assert(delivered == 2 && !g_b_outgoing_pending && !boxes[0].count);
    assert(received_size[0] == sizeof first && !memcmp(received[0], first, sizeof first));
    assert(!strcmp(received_topic[0], "rover/debug"));
    assert(received_size[1] == 1 && received[1][0] == 0x31);
    robot_mqtt_poll(&netif);
    assert(delivered == 2); /* No duplicate after retry success. */

    /* Queue age + retry time, counted once, with payload bytes preserved. */
    uint8_t status[13] = {0xc1};
    assert(comms_publish(COMMS_TOPIC_DEBUG, status, sizeof status));
    advance(75);
    publish_result = ERR_MEM;
    robot_mqtt_poll(&netif);
    advance(925);
    publish_result = ERR_OK;
    robot_mqtt_poll(&netif);
    robot_packed_reader_t reader;
    robot_packed_reader_init(&reader, received[2], received_size[2]);
    reader.bit_pos = 88;
    assert(robot_packed_get(&reader, 16) == 1000);

    /* The four-slot producer queue remains bounded under an outage. */
    publish_result = ERR_MEM;
    assert(comms_publish(COMMS_TOPIC_DEBUG, first, sizeof first));
    robot_mqtt_poll(&netif);
    for (unsigned i = 0; i < 4; ++i)
        assert(comms_publish(COMMS_TOPIC_HEARTBEAT, second, sizeof second));
    assert(!comms_publish(COMMS_TOPIC_HEARTBEAT, second, sizeof second));
    netif.link = 0;
    advance(1000);
    robot_mqtt_poll(&netif);
    assert(g_b_outgoing_pending);
    assert(!comms_publish(COMMS_TOPIC_HEARTBEAT, second, sizeof second));
    netif.link = 1;
    publish_result = ERR_OK;
    robot_mqtt_connected(&client, NULL, MQTT_CONNECT_ACCEPTED);
    unsigned before = delivered;
    robot_mqtt_poll(&netif);
    assert(!strcmp(received_topic[before], "rover/status")); /* Online first. */
    assert(!memcmp(received[before+1], first, sizeof first)); /* Pending first. */
    robot_mqtt_poll(&netif);
    robot_mqtt_poll(&netif);
    assert(!boxes[0].count && !g_b_outgoing_pending);

    clock_ms = UINT32_MAX - 5;
    assert(comms_publish(COMMS_TOPIC_HEARTBEAT, second, sizeof second));
    advance(20);
    uint32_t topic, age;
    uint8_t outgoing[COMMS_PAYLOAD_MAX];
    assert(comms_take_outgoing_timed(&topic, outgoing, sizeof outgoing, &length, &age));
    assert(age == 20 && length == 1 && outgoing[0] == 0x31);
    test_lag_fields();
    puts("PASS: real MQTT+mailbox retry, ordering, binary zeros, bounded backpressure, reconnect, command responsiveness, accurate/saturated age and clock wrap");
    return 0;
}
