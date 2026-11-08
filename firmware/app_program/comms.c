/**
 * @file    comms.c
 * @brief   Buddy 1: mailboxes between the robot tasks and the network.
 *
 * See comms.h.  Outgoing messages are stored as [topic byte][JSON text],
 * incoming ones as plain text, each in a micro T-Kernel message buffer.
 */

#include <tk/tkernel.h>

#include "comms.h"

#define COMMS_OUTBOX_MESSAGES (4u)
#define COMMS_INBOX_MESSAGES  (4u)
/* The kernel stores a small header with each message. */
#define COMMS_MSG_OVERHEAD (8u)

static ID                g_outbox_id  = 0;
static ID                g_inbox_id   = 0;
static volatile uint32_t g_link_state = COMMS_LINK_OFFLINE;

static uint32_t comms_strlen (const char * p_text, uint32_t limit)
{
    uint32_t length = 0u;

    while ((length < limit) && ('\0' != p_text[length]))
    {
        length++;
    }
    return length;
}

int32_t comms_init (void)
{
    T_CMBF  cmbf   = {0};
    ID      id     = 0;
    int32_t result = 0;

    cmbf.exinf  = NULL;
    cmbf.mbfatr = TA_TFIFO;
    /* Sizes are small compile-time constants, well inside SZ and INT. */
    cmbf.bufsz  = (SZ) (COMMS_OUTBOX_MESSAGES *
                        (COMMS_PAYLOAD_MAX + 1u + COMMS_MSG_OVERHEAD));
    cmbf.maxmsz = (INT) (COMMS_PAYLOAD_MAX + 1u);
    id          = tk_cre_mbf(&cmbf);
    if (id < E_OK)
    {
        result = (int32_t) id; /* Negative kernel error code. */
    }
    else
    {
        g_outbox_id = id;

        cmbf.bufsz  = (SZ) (COMMS_INBOX_MESSAGES *
                            (COMMS_COMMAND_MAX + COMMS_MSG_OVERHEAD));
        cmbf.maxmsz = (INT) COMMS_COMMAND_MAX;
        id          = tk_cre_mbf(&cmbf);
        if (id < E_OK)
        {
            result = (int32_t) id;
        }
        else
        {
            g_inbox_id = id;
        }
    }

    return result;
}

bool comms_publish (uint32_t topic, const char * p_payload)
{
    UB       message[COMMS_PAYLOAD_MAX + 1u];
    uint32_t length   = 0u;
    uint32_t index    = 0u;
    bool     b_queued = false;

    if ((g_outbox_id > 0) && (NULL != p_payload) &&
        (topic < COMMS_TOPIC_COUNT) && (COMMS_LINK_MQTT_UP == g_link_state))
    {
        length = comms_strlen(p_payload, COMMS_PAYLOAD_MAX - 1u);
        /* topic < COMMS_TOPIC_COUNT and characters are stored as bytes. */
        message[0] = (UB) topic;
        for (index = 0u; index < length; index++)
        {
            message[index + 1u] = (UB) p_payload[index];
        }
        message[length + 1u] = (UB) '\0';

        /* TMO_POL: never wait - if the network is slow, drop this message.
         * length + 2 <= COMMS_PAYLOAD_MAX + 1, so it fits in INT. */
        b_queued = (E_OK == tk_snd_mbf(g_outbox_id, message,
                                       (INT) (length + 2u), TMO_POL));
    }

    return b_queued;
}

bool comms_take_outgoing (uint32_t * p_topic, char * p_payload,
                          uint32_t buffer_size)
{
    UB       message[COMMS_PAYLOAD_MAX + 1u];
    INT      size    = 0;
    uint32_t index   = 0u;
    bool     b_taken = false;

    if ((g_outbox_id > 0) && (NULL != p_topic) && (NULL != p_payload) &&
        (0u != buffer_size))
    {
        size = tk_rcv_mbf(g_outbox_id, message, TMO_POL);
        if (size >= 2)
        {
            *p_topic = (uint32_t) message[0];
            /* size >= 2 here, so the unsigned conversion is exact. */
            for (index = 0u; ((index + 1u) < (uint32_t) size) &&
                             ((index + 1u) < buffer_size);
                 index++)
            {
                p_payload[index] = (char) message[index + 1u];
            }
            p_payload[index] = '\0';
            b_taken          = true;
        }
    }

    return b_taken;
}

void comms_put_incoming (const char * p_data, uint32_t length)
{
    UB       message[COMMS_COMMAND_MAX];
    uint32_t index = 0u;

    if ((g_inbox_id > 0) && (NULL != p_data) && (0u != length))
    {
        if (length > (COMMS_COMMAND_MAX - 1u))
        {
            length = COMMS_COMMAND_MAX - 1u;
        }
        for (index = 0u; index < length; index++)
        {
            message[index] = (UB) p_data[index];
        }
        message[length] = (UB) '\0';

        /* length + 1 <= COMMS_COMMAND_MAX, so it fits in INT. */
        (void) tk_snd_mbf(g_inbox_id, message, (INT) (length + 1u), TMO_POL);
    }
}

bool comms_wait_command (char * p_buffer, uint32_t buffer_size,
                         int32_t timeout_ms)
{
    UB       message[COMMS_COMMAND_MAX];
    INT      size       = 0;
    uint32_t index      = 0u;
    bool     b_received = false;

    if ((g_inbox_id > 0) && (NULL != p_buffer) && (0u != buffer_size))
    {
        size = tk_rcv_mbf(g_inbox_id, message, (TMO) timeout_ms);
        if (size > 0)
        {
            /* size > 0 here, so the unsigned conversion is exact. */
            for (index = 0u;
                 (index < (uint32_t) size) && ((index + 1u) < buffer_size);
                 index++)
            {
                p_buffer[index] = (char) message[index];
            }
            p_buffer[index] = '\0';
            b_received      = true;
        }
    }

    return b_received;
}

uint32_t comms_get_link_state (void)
{
    return g_link_state;
}

void comms_set_link_state (uint32_t state)
{
    g_link_state = state;
}

/*** end of file ***/

