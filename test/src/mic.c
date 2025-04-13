#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/audio/dmic.h>
#include <stdlib.h>

#include "config.h"
#include "codec.h"
#include "mic.h"

LOG_MODULE_REGISTER(mic, CONFIG_LOG_DEFAULT_LEVEL);

// --- Configuration ---
#define SAMPLE_RATE_HZ 16000
#define SAMPLE_BITS 16
#define BYTES_PER_SAMPLE (SAMPLE_BITS / 8)
#define DMIC_TIMEOUT_MS 1000
#define DMIC_BLOCK_SIZE_BYTES (MIC_BUFFER_SAMPLES * BYTES_PER_SAMPLE)
// Need enough blocks for double buffering at least
#define DMIC_MEM_SLAB_BLOCK_COUNT 8

// --- Device and GPIO ---
static const struct device *const dmic_dev = DEVICE_DT_GET(DT_ALIAS(dmic0));

// Get GPIO specs from Devicetree (make sure these exist in your DTS)
static const struct gpio_dt_spec mic_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_en_pin), gpios, {0});
static const struct gpio_dt_spec mic_thsel = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_thsel_pin), gpios, {0});
static const struct gpio_dt_spec mic_wake = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_wake_pin), gpios, {0});

// --- Memory Management ---
K_MEM_SLAB_DEFINE_STATIC(dmic_mem_slab, DMIC_BLOCK_SIZE_BYTES, DMIC_MEM_SLAB_BLOCK_COUNT, 4);

// --- DMIC Configuration Structs ---
static struct pcm_stream_cfg stream_cfg = {
    .pcm_rate = SAMPLE_RATE_HZ,
    .pcm_width = SAMPLE_BITS,
    .block_size = DMIC_BLOCK_SIZE_BYTES,
    .mem_slab = &dmic_mem_slab,
};

// Configure for MONO operation on the LEFT channel
static struct dmic_cfg drv_cfg = {
    .io = {
        /* These values depend on board, copied from example, adjust if needed */
        .min_pdm_clk_freq = 1000000,
        .max_pdm_clk_freq = 3500000,
        .min_pdm_clk_dc = 40,
        .max_pdm_clk_dc = 60,
    },
    .streams = &stream_cfg,
    .channel = {
        .req_num_streams = 1,
        .req_num_chan = 1, // Requesting MONO
        // Map logical channel 0 to physical channel 0 (LEFT)
        // req_chan_map_lo will be set in mic_start
        .req_chan_map_hi = 0, // Not used for MONO
    },
};

// --- State ---
static atomic_t mic_active = ATOMIC_INIT(0); // Track if mic processing is active

// --- Power Control ---
static int mic_power_off(void)
{
    int ret = 0;
    LOG_INF("Powering off microphone");

    // Configure pins according to example OFF state
    if (mic_thsel.port) {
        ret = gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT_INACTIVE);
         if (ret) LOG_ERR("Failed to configure THSEL for OFF: %d", ret);
    }
    if (mic_wake.port) {
         // Wake pin often acts as input when mic is off/sleeping
        ret = gpio_pin_configure_dt(&mic_wake, GPIO_INPUT);
         if (ret) LOG_ERR("Failed to configure WAKE for OFF: %d", ret);
    }
    if (mic_en.port) {
        ret = gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT_INACTIVE);
         if (ret) LOG_ERR("Failed to configure EN for OFF: %d", ret);
    }
    return ret; // Return last error code
}

static int mic_power_on(void)
{
    int ret = 0;
    LOG_INF("Powering on microphone");

    // Configure pins according to example ON state
    if (mic_thsel.port) {
         // Example sets THSEL high for ON
        ret = gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT_ACTIVE);
         if (ret) LOG_ERR("Failed to configure THSEL for ON: %d", ret);
    }
    if (mic_wake.port) {
        // Wake pin might be input or output depending on mic usage
        ret = gpio_pin_configure_dt(&mic_wake, GPIO_INPUT); // Example uses input
         if (ret) LOG_ERR("Failed to configure WAKE for ON: %d", ret);
    }
    if (mic_en.port) {
        ret = gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT_ACTIVE);
         if (ret) LOG_ERR("Failed to configure EN for ON: %d", ret);
    }
    k_sleep(K_MSEC(5)); // Allow time for mic to power up
    return ret; // Return last error code
}


// --- Data Reading Thread ---
K_THREAD_STACK_DEFINE(dmic_reader_stack, 2048); // Adjust stack size if needed
static struct k_thread dmic_reader_thread_data;

static void dmic_reader_thread(void *p1, void *p2, void *p3)
{
    int ret;
    void *buffer;
    uint32_t buffer_size;

    LOG_INF("DMIC reader thread started");

    while (atomic_get(&mic_active))
    {
        // Start capturing - restart DMIC for each block
        ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
        if (ret < 0) {
            LOG_ERR("Failed to start DMIC trigger: %d", ret);
            k_sleep(K_MSEC(5));
            continue;
        }

        // Blocking read for the next buffer
        ret = dmic_read(dmic_dev, 0, &buffer, &buffer_size, DMIC_TIMEOUT_MS);
        
        // Stop capture after each read attempt
        dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);

        if (ret == 0)
        {
            if (buffer_size != DMIC_BLOCK_SIZE_BYTES) {
                 LOG_WRN("DMIC read unexpected size: %u (expected %u)", buffer_size, DMIC_BLOCK_SIZE_BYTES);
                 // Still process if callback exists, maybe partial buffer?
            }

            int16_t *samples = (int16_t *)buffer;
            size_t total_samples = buffer_size / BYTES_PER_SAMPLE;

            for (size_t i = 0; i < total_samples; i += CODEC_PACKAGE_SAMPLES) {
                size_t chunk_samples = CODEC_PACKAGE_SAMPLES;
                if (i + CODEC_PACKAGE_SAMPLES > total_samples)
                    chunk_samples = total_samples - i;

                int ret = codec_receive_pcm(&samples[i], chunk_samples);
                int retry = 0;
                while (ret != 0 && retry++ < 5) {
                    LOG_WRN("Retrying chunk write to codec (%d/%d)...", retry, 5);
                    k_sleep(K_MSEC(2));
                    ret = codec_receive_pcm(&samples[i], chunk_samples);
                }

                if (ret != 0) {
                    LOG_ERR("Dropping audio chunk after retries");
                    break;
                }
            }

            // Free the buffer back to the slab after all chunks
            k_mem_slab_free(&dmic_mem_slab, buffer);
            buffer = NULL;
        }
        else if (ret == -EAGAIN) {  // -11: temporary condition, no data available
            LOG_WRN("DMIC read returned -EAGAIN (no data available); retrying...");
            k_sleep(K_MSEC(5));  // Short delay before retrying
            continue;
        }
        else if (ret == -ETIMEDOUT) {
            LOG_WRN("DMIC read timed out");
            k_sleep(K_MSEC(5));
            continue;
        }
        else {
            LOG_ERR("DMIC read failed: %d", ret);
            // Don't exit the thread, just retry after a delay
            k_sleep(K_MSEC(100));
            continue;
        }
    }

    LOG_INF("DMIC reader thread exiting");
    // Ensure trigger is stopped if loop terminates
    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    if (ret < 0) {
         LOG_ERR("DMIC STOP trigger failed on exit: %d", ret);
    }
}


// --- Public API ---

int mic_start()
{
    int ret;

    if (!device_is_ready(dmic_dev)) {
        LOG_ERR("DMIC device %s not ready", dmic_dev->name);
        return -ENODEV;
    }

    // Check if GPIOs were found (port != NULL)
    if (!mic_en.port || !mic_thsel.port /* || !mic_wake.port */) { // Wake might be optional
         LOG_WRN("One or more MIC control GPIOs not found in DTS!");
         // Decide if this is fatal or not based on hardware requirements
    }

    // Power on the microphone
    ret = mic_power_on();
    if (ret != 0) {
        LOG_ERR("Failed to power on microphone pins: %d", ret);
        return ret;
    }

    // Set the channel map before configuring
    drv_cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

    // Configure the DMIC driver
    ret = dmic_configure(dmic_dev, &drv_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure DMIC driver: %d", ret);
        mic_power_off(); // Clean up
        return ret;
    }
    LOG_INF("DMIC driver configured successfully");

    // Start the reader thread only if configuration is successful
    atomic_set(&mic_active, 1);
    k_thread_create(&dmic_reader_thread_data, dmic_reader_stack,
                    K_THREAD_STACK_SIZEOF(dmic_reader_stack),
                    dmic_reader_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(7), 0, K_NO_WAIT); // Adjust priority if needed
    k_thread_name_set(&dmic_reader_thread_data, "dmic_reader");

    // Triggering is now handled in the reader thread

    LOG_INF("Microphone started using Zephyr DMIC driver");
    return 0;
}

// Simple ON/OFF control - assumes mic_start was successful
void mic_on()
{
    // If already active or driver not ready, do nothing?
    if (!atomic_get(&mic_active) || !device_is_ready(dmic_dev)) {
         LOG_WRN("Mic cannot be turned ON (not started or driver not ready)");
         // Potentially call mic_start() here? Or require mic_start first?
         // For now, let's just power the pin if available.
         if(mic_en.port) {
             mic_power_on(); // Might be redundant if already on
         }
         return;
    }

    // Resume capture if it was stopped
    int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
         LOG_ERR("Failed to trigger START in mic_on: %d", ret);
    } else {
         LOG_INF("Mic capture resumed.");
    }
}

void mic_off()
{
    // If not active or driver not ready, do nothing?
    if (!atomic_get(&mic_active) || !device_is_ready(dmic_dev)) {
         LOG_WRN("Mic cannot be turned OFF (not active or driver not ready)");
          // Power off pins just in case
         if(mic_en.port) {
             mic_power_off();
         }
         return;
    }

    // Stop capture
    int ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    if (ret < 0) {
        LOG_ERR("Failed to trigger STOP in mic_off: %d", ret);
    } else {
         LOG_INF("Mic capture stopped.");
    }

    // Consider stopping the thread? For now, just stopping trigger.
    // Powering off might be too aggressive if user wants to quickly toggle.
    // Let's just power off the enable pin for simplicity.
    if(mic_en.port) {
        gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT_INACTIVE);
    }

}
