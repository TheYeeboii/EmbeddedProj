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
    T_CMBF cmbf;
    ID     id;

    cmbf.exinf  = NULL;
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) (COMMS_OUTBOX_MESSAGES *
                        (COMMS_PAYLOAD_MAX + 1u + COMMS_MSG_OVERHEAD));
    cmbf.maxmsz = (INT) (COMMS_PAYLOAD_MAX + 1u);
    id          = tk_cre_mbf(&cmbf);
    if (id < E_OK)
    {
        return (int32_t) id;
    }
    g_outbox_id = id;

    cmbf.bufsz =
        (SZ) (COMMS_INBOX_MESSAGES * (COMMS_COMMAND_MAX + COMMS_MSG_OVERHEAD));
    cmbf.maxmsz = (INT) COMMS_COMMAND_MAX;
    id          = tk_cre_mbf(&cmbf);
    if (id < E_OK)
    {
        return (int32_t) id;
    }
    g_inbox_id = id;

    return 0;
}

bool comms_publish (uint32_t topic, const char * p_payload)
{
    UB       message[COMMS_PAYLOAD_MAX + 1u];
    uint32_t length;
    uint32_t index;

    if ((g_outbox_id <= 0) || (NULL == p_payload) ||
        (topic >= COMMS_TOPIC_COUNT) || (COMMS_LINK_MQTT_UP != g_link_state))
    {
        return false;
    }

    length     = comms_strlen(p_payload, COMMS_PAYLOAD_MAX - 1u);
    message[0] = (UB) topic;
    for (index = 0u; index < length; index++)
    {
        message[index + 1u] = (UB) p_payload[index];
    }
    message[length + 1u] = (UB) '\0';

    /* TMO_POL: never wait - if the network is slow, drop this message. */
    return (E_OK ==
            tk_snd_mbf(g_outbox_id, message, (INT) (length + 2u), TMO_POL));
}

bool comms_take_outgoing (uint32_t * p_topic, char * p_payload,
                          uint32_t buffer_size)
{
    UB       message[COMMS_PAYLOAD_MAX + 1u];
    INT      size;
    uint32_t index;

    if ((g_outbox_id <= 0) || (NULL == p_topic) || (NULL == p_payload) ||
        (0u == buffer_size))
    {
        return false;
    }

    size = tk_rcv_mbf(g_outbox_id, message, TMO_POL);
    if (size < 2)
    {
        return false;
    }

    *p_topic = (uint32_t) message[0];
    for (index = 0u;
         (index + 1u < (uint32_t) size) && (index + 1u < buffer_size); index++)
    {
        p_payload[index] = (char) message[index + 1u];
    }
    p_payload[index] = '\0';
    return true;
}

void comms_put_incoming (const char * p_data, uint32_t length)
{
    UB       message[COMMS_COMMAND_MAX];
    uint32_t index;

    if ((g_inbox_id <= 0) || (NULL == p_data) || (0u == length))
    {
        return;
    }
    if (length > (COMMS_COMMAND_MAX - 1u))
    {
        length = COMMS_COMMAND_MAX - 1u;
    }
    for (index = 0u; index < length; index++)
    {
        message[index] = (UB) p_data[index];
    }
    message[length] = (UB) '\0';

    (void) tk_snd_mbf(g_inbox_id, message, (INT) (length + 1u), TMO_POL);
}

bool comms_wait_command (char * p_buffer, uint32_t buffer_size,
                         int32_t timeout_ms)
{
    UB       message[COMMS_COMMAND_MAX];
    INT      size;
    uint32_t index;

    if ((g_inbox_id <= 0) || (NULL == p_buffer) || (0u == buffer_size))
    {
        return false;
    }

    size = tk_rcv_mbf(g_inbox_id, message, (TMO) timeout_ms);
    if (size <= 0)
    {
        return false;
    }

    for (index = 0u; (index < (uint32_t) size) && (index + 1u < buffer_size);
         index++)
    {
        p_buffer[index] = (char) message[index];
    }
    p_buffer[index] = '\0';
    return true;
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

