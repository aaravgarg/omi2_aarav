#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <haly/nrfy_gpio.h>
#include "nrfx_clock.h"
#include "nrfx_pdm.h"
#include "config.h"
#include "mic.h"

LOG_MODULE_REGISTER(mic, CONFIG_LOG_DEFAULT_LEVEL);

static int16_t _buffer_0[MIC_BUFFER_SAMPLES];
static int16_t _buffer_1[MIC_BUFFER_SAMPLES];
static volatile uint8_t _next_buffer_index = 0;
static volatile mix_handler _callback = NULL;

static void pdm_irq_handler(nrfx_pdm_evt_t const *event)
{
    if (event->error)
    {
        LOG_ERR("PDM error: %d", event->error);
        return;
    }

    if (event->buffer_requested)
    {
        LOG_INF("PDM IRQ: buffer requested");
        if (_next_buffer_index == 0)
        {
            nrfx_pdm_buffer_set(NULL, _buffer_0, MIC_BUFFER_SAMPLES);
            _next_buffer_index = 1;
        }
        else
        {
            nrfx_pdm_buffer_set(NULL, _buffer_1, MIC_BUFFER_SAMPLES);
            _next_buffer_index = 0;
        }
    }

    if (event->buffer_released && _callback)
    {
        LOG_INF("PDM IRQ: buffer released: %p", event->buffer_released);

        if (event->buffer_released == _buffer_0)
        {
            LOG_INF("PDM: Callback invoked with buffer_0");
            _callback(_buffer_0);
        }
        else if (event->buffer_released == _buffer_1)
        {
            LOG_INF("PDM: Callback invoked with buffer_1");
            _callback(_buffer_1);
        }
        else
        {
            LOG_ERR("Unknown buffer released: %p", event->buffer_released);
        }
    }
}

int mic_start()
{
    if (!nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY))
    {
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    }

    nrfy_gpio_cfg_output(PDM_PWR_PIN);
    nrfy_gpio_pin_set(PDM_PWR_PIN);

    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    pdm_config.gain_l = MIC_GAIN;
    pdm_config.gain_r = MIC_GAIN;
    pdm_config.interrupt_priority = MIC_IRC_PRIORITY;
    pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
    pdm_config.mode = NRF_PDM_MODE_MONO;
    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    pdm_config.ratio = NRF_PDM_RATIO_80X;

    if (nrfx_pdm_init(NULL, &pdm_config, pdm_irq_handler) != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to initialize nrfx PDM");
        return -1;
    }

    if (nrfx_pdm_buffer_set(NULL, _buffer_0, MIC_BUFFER_SAMPLES) != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to prime initial PDM buffer");
        return -1;
    }

    if (nrfx_pdm_start(NULL) != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to start PDM capture");
        return -1;
    }

    LOG_INF("Microphone started with nrfx_pdm");
    return 0;
}

void set_mic_callback(mix_handler callback)
{
    _callback = callback;
    LOG_INF("Mic callback set to %p", callback);
}