/*
 * Production Microphone Driver Implementation
 * Uses the Zephyr DMIC (Digital Microphone) driver interface
 */
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/audio/dmic.h>     /* Digital microphone interface driver */
#include <zephyr/sys/util.h>
#include "mic.h"

LOG_MODULE_REGISTER(mic, CONFIG_LOG_DEFAULT_LEVEL);

/* Audio configuration parameters */
#define BITS_PER_BYTE 8
#define SAMPLE_RATE_HZ 16000       /* 16kHz sampling rate */
#define SAMPLE_BITS 16             /* 16-bit samples */
#define TIMEOUT_MS 1000            /* Read timeout in milliseconds */
#define BLOCK_SIZE 1600  /* Use the existing buffer size definition */
#define BLOCK_COUNT 4              /* Number of memory blocks for buffering (double-buffer) */

/* Memory allocation for audio buffers */
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE * sizeof(int16_t), BLOCK_COUNT, 4);

/* Device references */
static const struct device *const dmic = DEVICE_DT_GET(DT_ALIAS(dmic0));  /* Get DMIC device from device tree */

/* Microphone control GPIO pins from device tree */
static const struct gpio_dt_spec mic_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_en_pin), gpios, {0});        /* Enable pin */
static const struct gpio_dt_spec mic_thsel = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_thsel_pin), gpios, {0});  /* Threshold select pin */
static const struct gpio_dt_spec mic_wake = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pdm_wake_pin), gpios, {0});    /* Wake pin */

/* PCM stream configuration */
static struct pcm_stream_cfg stream = {
	.pcm_rate = SAMPLE_RATE_HZ,
	.pcm_width = SAMPLE_BITS,
	.block_size = BLOCK_SIZE * sizeof(int16_t),
	.mem_slab = &mem_slab,
};

/* DMIC configuration */
static struct dmic_cfg cfg = {
	.io = {
		.min_pdm_clk_freq = 1000000,  /* Minimum PDM clock frequency (1MHz) */
		.max_pdm_clk_freq = 3500000,  /* Maximum PDM clock frequency (3.5MHz) */
		.min_pdm_clk_dc = 40,         /* Minimum clock duty cycle percentage */
		.max_pdm_clk_dc = 60,         /* Maximum clock duty cycle percentage */
	},
	.streams = &stream,
	.channel = {
		.req_num_streams = 1,         /* Request one audio stream */
		.req_num_chan = 2,            /* Request stereo (2 channels) */
	},
};

/* Initialization flag */
static bool initialized = false;

/* Callback function for audio processing */
static volatile mix_handler _callback = NULL;

/* Audio processing thread data */
#define STACK_SIZE 1024
#define THREAD_PRIORITY 5
static struct k_thread mic_thread;
static K_THREAD_STACK_DEFINE(mic_stack, STACK_SIZE);
static volatile bool mic_running = false;

/* Audio processing thread function */
static void mic_processing_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    void *buffer;
    uint32_t size;
    
    while (mic_running) {
        /* Read audio data from the microphone */
        if (dmic_read(dmic, 0, &buffer, &size, TIMEOUT_MS) == 0) {
            /* Call user callback with audio data if registered */
            if (_callback) {
                /* Cast buffer to int16_t* as expected by callback */
                _callback((int16_t *)buffer);
            }
            
            /* Free the buffer */
            k_mem_slab_free(&mem_slab, buffer);
        } else {
            LOG_ERR("DMIC read failed");
            k_msleep(10); /* Short delay to avoid CPU spinning on failures */
        }
    }
}

/* 
 * Power off the microphone to save power
 * Configures GPIO pins to appropriate states for power off
 */
void mic_off()
{
    if (mic_running) {
        mic_running = false;
        dmic_trigger(dmic, DMIC_TRIGGER_STOP);
        k_thread_abort(&mic_thread);
    }
    
    /* Configure and set GPIO pins for power off */
    if (device_is_ready(mic_en.port)) {
        gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT);
        gpio_pin_set_dt(&mic_en, 0);
    }
    
    if (device_is_ready(mic_thsel.port)) {
        gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT);
        gpio_pin_set_dt(&mic_thsel, 0);
    }
    
    if (device_is_ready(mic_wake.port)) {
        gpio_pin_configure_dt(&mic_wake, GPIO_INPUT);
    }
    
    LOG_INF("Microphone powered off");
}

/* 
 * Power on the microphone
 * Configures GPIO pins to appropriate states for power on
 */
void mic_on()
{
    /* Configure and set GPIO pins for power on */
    if (device_is_ready(mic_en.port)) {
        gpio_pin_configure_dt(&mic_en, GPIO_OUTPUT);
        gpio_pin_set_dt(&mic_en, 1);
    }
    
    if (device_is_ready(mic_thsel.port)) {
        gpio_pin_configure_dt(&mic_thsel, GPIO_OUTPUT);
        gpio_pin_set_dt(&mic_thsel, 1);
    }
    
    if (device_is_ready(mic_wake.port)) {
        gpio_pin_configure_dt(&mic_wake, GPIO_INPUT);
    }
    
    LOG_INF("Microphone powered on");
}

/*
 * Start the microphone
 * Initializes the DMIC peripheral and starts audio capture
 */
int mic_start()
{
    /* Check if DMIC device is ready */
    if (!device_is_ready(dmic)) {
        LOG_ERR("DMIC device not ready");
        return -ENODEV;
    }
    
    /* Check if already initialized and running */
    if (initialized && mic_running) {
        LOG_INF("Microphone already started");
        return 0;
    }
    
    /* Stop any previous instance if running */
    if (mic_running) {
        mic_running = false;
        dmic_trigger(dmic, DMIC_TRIGGER_STOP);
        k_thread_abort(&mic_thread);
    }

    /* Power on the microphone */
    mic_on();
	k_msleep(50);
    
    /* Set up the channel mapping configuration */
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) | 
                                 dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
    
    /* Configure the DMIC with our settings */
    int ret = dmic_configure(dmic, &cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure DMIC (%d)", ret);
        return ret;
    }
    
    /* Start the microphone */
    ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("START trigger failed (%d)", ret);
        return ret;
    }
    
    /* Start audio processing thread */
    mic_running = true;
    k_thread_create(&mic_thread, mic_stack, STACK_SIZE,
                   mic_processing_thread, NULL, NULL, NULL,
                   THREAD_PRIORITY, 0, K_NO_WAIT);
    
    initialized = true;
    LOG_INF("Audio microphone started");
    return 0;
}

/*
 * Register callback function for processing audio data
 * This function will be called whenever new audio data is available
 */
void set_mic_callback(mix_handler callback) 
{
    _callback = callback;
}
