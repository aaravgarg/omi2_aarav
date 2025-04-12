#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/printk.h>
#include "transport.h"
#include "config.h"
#include "utils.h"

LOG_MODULE_REGISTER(transport, CONFIG_LOG_DEFAULT_LEVEL);

static const struct bt_gatt_attr *audio_notify_attr = NULL;

static struct bt_conn *current_connection = NULL;
static uint16_t current_mtu = 0;

#define NET_BUFFER_HEADER_SIZE 3
#define RING_BUFFER_HEADER_SIZE 2

static uint8_t tx_buffer[CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE];
static uint8_t pusher_temp_data[CODEC_OUTPUT_MAX_BYTES + NET_BUFFER_HEADER_SIZE];
static struct ring_buf ring_buf;

int broadcast_audio_packets(uint8_t *data, size_t size)
{
    if (size > CODEC_OUTPUT_MAX_BYTES) return false;

    tx_buffer[0] = size & 0xFF;
    tx_buffer[1] = (size >> 8) & 0xFF;
    memcpy(tx_buffer + RING_BUFFER_HEADER_SIZE, data, size);

    int written = ring_buf_put(&ring_buf, tx_buffer, size + RING_BUFFER_HEADER_SIZE);
    return written == (size + RING_BUFFER_HEADER_SIZE);
}

static bool read_from_tx_queue()
{
    uint32_t size = ring_buf_get(&ring_buf, tx_buffer, CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE);
    if (size < RING_BUFFER_HEADER_SIZE) return false;

    uint16_t payload_size = tx_buffer[0] + (tx_buffer[1] << 8);
    if (payload_size == 0 || payload_size > CODEC_OUTPUT_MAX_BYTES) return false;

    memcpy(pusher_temp_data + NET_BUFFER_HEADER_SIZE, tx_buffer + RING_BUFFER_HEADER_SIZE, payload_size);
    return true;
}

static bool push_to_gatt(struct bt_conn *conn)
{
    if (!read_from_tx_queue()) return false;

    uint16_t payload_size = tx_buffer[0] + (tx_buffer[1] << 8);
    static uint16_t packet_index = 0;

    pusher_temp_data[0] = packet_index & 0xFF;
    pusher_temp_data[1] = (packet_index >> 8) & 0xFF;
    pusher_temp_data[2] = 0;  // single frame

    LOG_INF("Transport: notifying packet of size %d", payload_size + NET_BUFFER_HEADER_SIZE);
    int err = bt_gatt_notify(conn, audio_notify_attr, pusher_temp_data, payload_size + NET_BUFFER_HEADER_SIZE);
    if (err)
    {
        LOG_ERR("bt_gatt_notify failed: %d", err);
        return false;
    }
    packet_index++;
    return true;
}

static void connection_cb(struct bt_conn *conn, uint8_t err)
{
    if (err == 0)
    {
        current_connection = bt_conn_ref(conn);
        bt_conn_get_info(conn, NULL);
        current_mtu = bt_gatt_get_mtu(conn);
        LOG_INF("Transport connected, MTU: %d", current_mtu);
    }
}

static void disconnection_cb(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Transport disconnected");
    if (current_connection)
    {
        bt_conn_unref(current_connection);
        current_connection = NULL;
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connection_cb,
    .disconnected = disconnection_cb,
};

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY)
        LOG_INF("Client subscribed for notifications");
    else
        LOG_INF("Client unsubscribed from notifications");
}

static struct bt_uuid_128 audio_service_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10000, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_data_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10001, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

static ssize_t audio_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

BT_GATT_SERVICE_DEFINE(audio_service,
    BT_GATT_PRIMARY_SERVICE(&audio_service_uuid),
    BT_GATT_CHARACTERISTIC(&audio_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           audio_read, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

K_THREAD_STACK_DEFINE(pusher_stack, 2048);
static struct k_thread pusher_thread;

static void pusher_thread_fn(void *a, void *b, void *c)
{
    while (1)
    {
        if (current_connection)
        {
            push_to_gatt(current_connection);
        }
        k_sleep(K_MSEC(10));
    }
}

int transport_start()
{
    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    audio_notify_attr = &audio_service.attrs[1];

    bt_conn_cb_register(&conn_callbacks);
    k_thread_create(&pusher_thread, pusher_stack, K_THREAD_STACK_SIZEOF(pusher_stack), pusher_thread_fn, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

    struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x14, 0x21, 0xA1, 0x68, 0x4F, 0x6C, 0x7E, 0x53, 0xF2, 0xE8, 0x00, 0x00, 0xB1, 0x19, 0x00, 0x00),
    };

    err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

    ring_buf_init(&ring_buf, sizeof(tx_buffer), tx_buffer);

    LOG_INF("Transport bluetooth initialized");
    return 0;
}