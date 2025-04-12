#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include "codec.h"
#include "config.h"
#include "utils.h"
#ifdef CODEC_OPUS
#include "lib/opus-1.2.1/opus.h"
#endif

LOG_MODULE_REGISTER(codec, CONFIG_LOG_DEFAULT_LEVEL);

static volatile codec_callback _callback = NULL;

void set_codec_callback(codec_callback callback)
{
    _callback = callback;
}

uint8_t codec_ring_buffer_data[AUDIO_BUFFER_SAMPLES * 2];
struct ring_buf codec_ring_buf;

int codec_receive_pcm(int16_t *data, size_t len)
{
    LOG_INF("Received PCM %d bytes", len * 2);
    int written = ring_buf_put(&codec_ring_buf, (uint8_t *)data, len * 2);
    if (written != len * 2)
    {
        LOG_ERR("Failed to write %d bytes to codec ring buffer", len * 2);
        return -1;
    }
    return 0;
}

int16_t codec_input_samples[CODEC_PACKAGE_SAMPLES];
uint8_t codec_output_bytes[CODEC_OUTPUT_MAX_BYTES];
K_THREAD_STACK_DEFINE(codec_stack, 32000);
static struct k_thread codec_thread;
uint16_t execute_codec();

#if CODEC_OPUS
__ALIGN(4)
static uint8_t m_opus_encoder[7180];
static OpusEncoder *const m_opus_state = (OpusEncoder *)m_opus_encoder;
#endif

void codec_entry()
{
    LOG_INF("Codec thread started");
    uint16_t output_size;
    while (1)
    {
        if (ring_buf_size_get(&codec_ring_buf) < CODEC_PACKAGE_SAMPLES * 2)
        {
            k_sleep(K_MSEC(10));
            continue;
        }

        LOG_INF("Codec thread: got enough PCM data");
        ring_buf_get(&codec_ring_buf, (uint8_t *)codec_input_samples, CODEC_PACKAGE_SAMPLES * 2);

        output_size = execute_codec();
        LOG_INF("Codec thread: encoded %d bytes", output_size);

        if (_callback)
        {
            LOG_INF("Codec thread: sending encoded data to callback");
            _callback(codec_output_bytes, output_size);
        }

        k_yield();
    }
}

int codec_start()
{
#if CODEC_OPUS
    opus_encoder_init(m_opus_state, 16000, 1, CODEC_OPUS_APPLICATION);
    opus_encoder_ctl(m_opus_state, OPUS_SET_BITRATE(CODEC_OPUS_BITRATE));
#endif

    ring_buf_init(&codec_ring_buf, sizeof(codec_ring_buffer_data), codec_ring_buffer_data);
    k_thread_create(&codec_thread, codec_stack, K_THREAD_STACK_SIZEOF(codec_stack), (k_thread_entry_t)codec_entry, NULL, NULL, NULL, 4, 0, K_NO_WAIT);

    LOG_INF("Codec started");
    return 0;
}

#if CODEC_OPUS
uint16_t execute_codec()
{
    opus_int32 size = opus_encode(m_opus_state, codec_input_samples, CODEC_PACKAGE_SAMPLES, codec_output_bytes, sizeof(codec_output_bytes));
    if (size < 0)
    {
        LOG_WRN("Opus encoding failed: %d", size);
        return 0;
    }
    return size;
}
#endif