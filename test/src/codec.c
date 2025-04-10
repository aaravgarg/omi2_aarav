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

uint8_t codec_ring_buffer_data[AUDIO_BUFFER_SAMPLES * 2 * 2]; // Original buffer size: 2 bytes per sample
struct ring_buf codec_ring_buf;
int codec_receive_pcm(int16_t *data, size_t len) //this gets called after mic data is finished 
{   
    uint32_t space_available = ring_buf_space_get(&codec_ring_buf);
    uint32_t data_available = ring_buf_size_get(&codec_ring_buf);
    
    // If buffer is full, discard enough data to make room plus a safety margin
    if (space_available < len * 2) {
        LOG_WRN("Buffer full (%d/%d bytes used): discarding oldest data", 
                data_available, sizeof(codec_ring_buffer_data));
        
        // Discard half of the buffer or enough for new data plus margin, whichever is larger
        uint32_t to_discard = data_available / 2;
        if (to_discard < (len * 2 + CODEC_PACKAGE_SAMPLES * 2)) {
            to_discard = len * 2 + CODEC_PACKAGE_SAMPLES * 2;
        }
        
        // Make sure we don't try to discard more than what's available
        to_discard = MIN(to_discard, data_available);
        
        LOG_WRN("Discarding %d bytes of oldest audio data", to_discard);
        ring_buf_get(&codec_ring_buf, NULL, to_discard);
        
        // Verify we now have space
        space_available = ring_buf_space_get(&codec_ring_buf);
        if (space_available < len * 2) {
            LOG_ERR("Buffer still full after discarding %d bytes, space: %d, need: %d", 
                    to_discard, space_available, len * 2);
            // Emergency measure: reset the entire buffer
            ring_buf_reset(&codec_ring_buf);
            LOG_WRN("Emergency buffer reset performed");
            // Now we definitely have space
            space_available = sizeof(codec_ring_buffer_data);
        }
    }
   
    int written = ring_buf_put(&codec_ring_buf, (uint8_t *)data, len * 2);
    if (written != len * 2)
    {
        LOG_ERR("Failed to write %d bytes to codec ring buffer, only wrote %d bytes", 
                len * 2, written);
        return -1;
    }
    
    LOG_DBG("Successfully wrote %d bytes, buffer usage: %d/%d", 
            written, ring_buf_size_get(&codec_ring_buf), sizeof(codec_ring_buffer_data));
    
    return 0;
}

//
// Thread
//

int16_t codec_input_samples[CODEC_PACKAGE_SAMPLES];
uint8_t codec_output_bytes[CODEC_OUTPUT_MAX_BYTES];
K_THREAD_STACK_DEFINE(codec_stack, 40000);
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

void codec_entry()
{
    uint16_t output_size;
    uint32_t data_available;
    uint32_t consecutive_processing = 0;
    
    while (1)
    {
        // Check how much data we have
        data_available = ring_buf_size_get(&codec_ring_buf);
        
        if (data_available >= CODEC_PACKAGE_SAMPLES * 2)
        {
            // Read package
            ring_buf_get(&codec_ring_buf, (uint8_t *)codec_input_samples, CODEC_PACKAGE_SAMPLES * 2);

            // Run Codec
            output_size = execute_codec();

            // Notify
            if (_callback)
            {
                _callback(codec_output_bytes, output_size);
            }
            
            // Process consecutively if buffer is filling up
            consecutive_processing++;
            
            // If buffer has a lot of data or we haven't processed too many consecutive packages,
            // continue processing without sleeping
            if (data_available > CODEC_PACKAGE_SAMPLES * 6 || consecutive_processing < 3) {
                continue; // Skip sleep and process more data
            }
            
            // Reset counter after consecutive processing
            consecutive_processing = 0;
        }
        else
        {
            // Reset the consecutive processing counter when we run out of data
            consecutive_processing = 0;
            
            // Very short sleep when no data
            k_sleep(K_MSEC(1));
        }
        
        // Always yield after processing or sleeping
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
#endif

    // Thread
    ring_buf_init(&codec_ring_buf, sizeof(codec_ring_buffer_data), codec_ring_buffer_data);
    LOG_INF("Codec ring buffer initialized with size %d bytes", sizeof(codec_ring_buffer_data));
    k_thread_create(&codec_thread, codec_stack, K_THREAD_STACK_SIZEOF(codec_stack), 
                   (k_thread_entry_t)codec_entry, NULL, NULL, NULL, 
                   K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    LOG_INF("Codec thread started with priority %d", 3);

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
        LOG_WRN("Opus encoding failed: %d", size);
        return 0;
    }
    LOG_DBG("Opus encoding success: %i", size);
    return size;
}

#endif
