#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include "codec.h"
#include "config.h"
#include "utils.h"
#ifdef CODEC_OPUS
#include "lib/opus-1.2.1/opus.h"
#endif

LOG_MODULE_REGISTER(codec, CONFIG_LOG_DEFAULT_LEVEL);

//
// Output
//

static volatile codec_callback _callback = NULL;

void set_codec_callback(codec_callback callback)
{
    _callback = callback;
}

//
// Input
//

uint8_t codec_ring_buffer_data[AUDIO_BUFFER_SAMPLES * 2]; // 2 bytes per sample
struct ring_buf codec_ring_buf;

int codec_receive_pcm(int16_t *data, size_t len)
{
    int written = ring_buf_put(&codec_ring_buf, (uint8_t *)data, len * 2);
    if (written != len * 2)
    {
        LOG_ERR("Failed to write %d bytes to codec ring buffer", len * 2);
        return -1;
    }

    LOG_INF("PCM written to codec ring buffer: %d bytes", written);
    return 0;
}

//
// Thread
//

int16_t codec_input_samples[CODEC_PACKAGE_SAMPLES];
uint8_t codec_output_bytes[CODEC_OUTPUT_MAX_BYTES];
K_THREAD_STACK_DEFINE(codec_stack, 32000);
static struct k_thread codec_thread;
uint16_t execute_codec();

#if CODEC_OPUS
#if (CONFIG_OPUS_MODE == CONFIG_OPUS_MODE_CELT)
#define OPUS_ENCODER_SIZE 7180
#endif
#if (CONFIG_OPUS_MODE == CONFIG_OPUS_MODE_HYBRID)
#define OPUS_ENCODER_SIZE 10916
#endif
__ALIGN(4)
static uint8_t m_opus_encoder[OPUS_ENCODER_SIZE];
static OpusEncoder *const m_opus_state = (OpusEncoder *)m_opus_encoder;
#endif

// void codec_entry()
// {

//     LOG_INF("Codec: PCM data received, encoding now...");

//     uint16_t output_size;
//     while (1)
//     {

//         // Check if we have enough data
//         if (ring_buf_size_get(&codec_ring_buf) < CODEC_PACKAGE_SAMPLES * 2)
//         {
//             // LOG_PRINTK("waiting on data....\n");
//             k_sleep(K_MSEC(10));
//             continue;
//         }
//         // Read package
//         ring_buf_get(&codec_ring_buf, (uint8_t *)codec_input_samples, CODEC_PACKAGE_SAMPLES * 2);

//         // Run Codec
//         output_size = execute_codec();

//         // Notify
//         if (_callback)
//         {
//             _callback(codec_output_bytes, output_size);
//         }

//         // Yield
//         k_yield();
//     }
// }

void codec_entry()
{
    uint16_t output_size;
    while (1)
    {
        size_t available = ring_buf_size_get(&codec_ring_buf);
        if (available < CODEC_PACKAGE_SAMPLES * 2)
        {
            LOG_WRN("Not enough PCM data in buffer: %d bytes available", available);
            k_sleep(K_MSEC(10));
            continue;
        }

        LOG_INF("Enough PCM data available: %d bytes", available);
        ring_buf_get(&codec_ring_buf, (uint8_t *)codec_input_samples, CODEC_PACKAGE_SAMPLES * 2);
        output_size = execute_codec();

        if (_callback)
        {
            LOG_INF("Calling codec callback with %d bytes", output_size);
            _callback(codec_output_bytes, output_size);
        }
        else
        {
            LOG_ERR("Codec callback is NULL");
        }

        k_yield();
    }
}


int codec_start()
{

// OPUS
#if CODEC_OPUS
    ASSERT_TRUE(opus_encoder_get_size(1) == sizeof(m_opus_encoder));
    ASSERT_TRUE(opus_encoder_init(m_opus_state, 16000, 1, CODEC_OPUS_APPLICATION) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_BITRATE(CODEC_OPUS_BITRATE)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_VBR(CODEC_OPUS_VBR)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_VBR_CONSTRAINT(0)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_COMPLEXITY(CODEC_OPUS_COMPLEXITY)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_LSB_DEPTH(16)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_DTX(0)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_INBAND_FEC(0)) == OPUS_OK);
    ASSERT_TRUE(opus_encoder_ctl(m_opus_state, OPUS_SET_PACKET_LOSS_PERC(0)) == OPUS_OK);

    LOG_INF("Codec initialized");
#endif

    // Thread
    ring_buf_init(&codec_ring_buf, sizeof(codec_ring_buffer_data), codec_ring_buffer_data);
    k_thread_create(&codec_thread, codec_stack, K_THREAD_STACK_SIZEOF(codec_stack), (k_thread_entry_t)codec_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);

    LOG_INF("Codec started");

    // Success
    return 0;
}

//
// Opus codec
//

#if CODEC_OPUS

uint16_t execute_codec()
{
    opus_int32 size = opus_encode(m_opus_state, codec_input_samples, CODEC_PACKAGE_SAMPLES, codec_output_bytes, sizeof(codec_output_bytes));
    if (size < 0)
    {
        LOG_INF("Opus encoding failed: %d", size);
        return 0;
    }
    LOG_INF("Opus encoding success, size: %d", size);
    return size;
}

#endif
