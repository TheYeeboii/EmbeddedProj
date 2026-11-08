/**
 * @file    comms.c
 * @brief   Buddy 1: mailboxes between the robot tasks and the network.
 *
 * See comms.h.  Outgoing messages are stored as [topic byte][payload],
 * incoming ones as the raw command bytes, each in a micro T-Kernel message
 * buffer.  The kernel returns each message's size, so no terminator is
 * needed and payloads may contain 0x00.
 */

#include <tk/tkernel.h>

#include "comms.h"
#include "robot_fault.h"

#define COMMS_OUTBOX_MESSAGES (4u)
#define COMMS_INBOX_MESSAGES  (4u)
/* The kernel stores a small header with each message. */
#define COMMS_MSG_OVERHEAD (8u)
/* One topic byte in front of each outgoing payload. */
#define COMMS_TOPIC_BYTES (1u)

static ID g_outbox_id = 0;
static ID g_inbox_id  = 0;
/* Written only by the radio task (comms_set_link_state); an aligned byte
 * read is atomic, so readers need no lock. */
static volatile uint8_t g_link_state = COMMS_LINK_OFFLINE;

static uint32_t comms_strlen (const char * p_text, uint32_t limit);
static void     comms_update_link_faults (uint8_t state);

int32_t comms_init (void)
{
    T_CMBF  cmbf       = {0};
    ID      mailbox_id = 0;
    int32_t result     = 0;

    /* The robot starts offline. */
    comms_update_link_faults(COMMS_LINK_OFFLINE);
    cmbf.exinf  = NULL;
    cmbf.mbfatr = TA_TFIFO;
    /* Sizes are small compile-time constants, well inside SZ and INT. */
    cmbf.bufsz =
        (SZ) (COMMS_OUTBOX_MESSAGES *
              (COMMS_PAYLOAD_MAX + COMMS_TOPIC_BYTES + COMMS_MSG_OVERHEAD));
    cmbf.maxmsz = (INT) (COMMS_PAYLOAD_MAX + COMMS_TOPIC_BYTES);
    mailbox_id  = tk_cre_mbf(&cmbf);
    if (mailbox_id < E_OK)
    {
        result = (int32_t) mailbox_id; /* Negative kernel error code. */
    }
    else
    {
        g_outbox_id = mailbox_id;

        cmbf.bufsz  = (SZ) (COMMS_INBOX_MESSAGES *
                            (COMMS_COMMAND_MAX + COMMS_MSG_OVERHEAD));
        cmbf.maxmsz = (INT) COMMS_COMMAND_MAX;
        mailbox_id  = tk_cre_mbf(&cmbf);
        if (mailbox_id < E_OK)
        {
            result = (int32_t) mailbox_id;
        }
        else
        {
            g_inbox_id = mailbox_id;
        }
    }

    return result;
}

bool comms_publish (uint32_t topic, const uint8_t * p_data, uint32_t length)
{
    UB       message[COMMS_PAYLOAD_MAX + COMMS_TOPIC_BYTES];
    uint32_t index    = 0u;
    bool     b_queued = false;

    if ((g_outbox_id > 0) && (NULL != p_data) && (0u != length) &&
        (length <= COMMS_PAYLOAD_MAX) && (topic < COMMS_TOPIC_COUNT) &&
        (COMMS_LINK_MQTT_UP == g_link_state))
    {
        /* topic < COMMS_TOPIC_COUNT, so it fits in one byte. */
        message[0] = (UB) topic;
        for (index = 0u; index < length; index++)
        {
            message[index + COMMS_TOPIC_BYTES] = p_data[index];
        }

        /* TMO_POL: never wait - if the network is slow, drop this message.
         * length + 1 <= COMMS_PAYLOAD_MAX + 1, so it fits in INT. */
        b_queued =
            (E_OK == tk_snd_mbf(g_outbox_id, message,
                                (INT) (length + COMMS_TOPIC_BYTES), TMO_POL));
    }

    return b_queued;
}

bool comms_publish_text (uint32_t topic, const char * p_text)
{
    bool b_queued = false;

    if (NULL != p_text)
    {
        /* Characters are stored as bytes; the cast only changes the pointer
         * type, not the data. */
        b_queued = comms_publish(topic, (const uint8_t *) p_text,
                                 comms_strlen(p_text, COMMS_PAYLOAD_MAX));
    }

    return b_queued;
}

bool comms_take_outgoing (uint32_t * p_topic, uint8_t * p_payload,
                          uint32_t buffer_size, uint32_t * p_length)
{
    UB       message[COMMS_PAYLOAD_MAX + COMMS_TOPIC_BYTES];
    INT      size    = 0;
    uint32_t length  = 0u;
    uint32_t index   = 0u;
    bool     b_taken = false;

    if ((g_outbox_id > 0) && (NULL != p_topic) && (NULL != p_payload) &&
        (NULL != p_length) && (buffer_size >= COMMS_PAYLOAD_MAX))
    {
        size = tk_rcv_mbf(g_outbox_id, message, TMO_POL);
        if (size > (INT) COMMS_TOPIC_BYTES)
        {
            /* size > 1 here, so the unsigned conversion is exact, and
             * maxmsz keeps it at most COMMS_PAYLOAD_MAX + 1. */
            length = (uint32_t) size - COMMS_TOPIC_BYTES;
            for (index = 0u; index < length; index++)
            {
                p_payload[index] = message[index + COMMS_TOPIC_BYTES];
            }
            *p_topic  = (uint32_t) message[0];
            *p_length = length;
            b_taken   = true;
        }
    }

    return b_taken;
}

void comms_put_incoming (const uint8_t * p_data, uint32_t length)
{
    if ((g_inbox_id > 0) && (NULL != p_data) && (0u != length) &&
        (length <= COMMS_COMMAND_MAX))
    {
        /* length <= COMMS_COMMAND_MAX, so it fits in INT.  The kernel copies
         * the bytes; the cast drops const for the kernel API only. */
        (void) tk_snd_mbf(g_inbox_id, (void *) p_data, (INT) length, TMO_POL);
    }
}

bool comms_wait_command (uint8_t * p_buffer, uint32_t buffer_size,
                         uint32_t * p_length, int32_t timeout_ms)
{
    INT  size       = 0;
    bool b_received = false;

    /* The kernel writes up to maxmsz (COMMS_COMMAND_MAX) bytes. */
    if ((g_inbox_id > 0) && (NULL != p_buffer) && (NULL != p_length) &&
        (buffer_size >= COMMS_COMMAND_MAX))
    {
        size = tk_rcv_mbf(g_inbox_id, p_buffer, (TMO) timeout_ms);
        if (size > 0)
        {
            *p_length  = (uint32_t) size; /* size > 0, conversion exact. */
            b_received = true;
        }
    }

    return b_received;
}

uint8_t comms_get_link_state (void)
{
    return g_link_state;
}

void comms_set_link_state (uint8_t state)
{
    /* Called every radio poll: only touch the fault byte on a change. */
    if (state != g_link_state)
    {
        g_link_state = state;
        comms_update_link_faults(state);
    }
}

/* Count characters before the NUL, at most "limit". */
static uint32_t comms_strlen (const char * p_text, uint32_t limit)
{
    uint32_t length = 0u;

    while ((length < limit) && ('\0' != p_text[length]))
    {
        length++;
    }

    return length;
}

/* Wi-Fi loss: not joined.  MQTT loss: no broker (also true without Wi-Fi). */
static void comms_update_link_faults (uint8_t state)
{
    if (COMMS_LINK_MQTT_UP == state)
    {
        robot_fault_clear(ROBOT_FAULT_WIFI_LOSS | ROBOT_FAULT_MQTT_LOSS);
    }
    else if (COMMS_LINK_WIFI_UP == state)
    {
        robot_fault_clear(ROBOT_FAULT_WIFI_LOSS);
        robot_fault_set(ROBOT_FAULT_MQTT_LOSS);
    }
    else
    {
        robot_fault_set(ROBOT_FAULT_WIFI_LOSS | ROBOT_FAULT_MQTT_LOSS);
    }
}

/*** end of file ***/

