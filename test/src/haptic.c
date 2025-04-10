/**
 * @file haptic.c
 * @brief Implementation of haptic feedback functionality
 * 
 * This file provides functions to control a haptic actuator (vibration motor)
 * connected to a GPIO pin on the device.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "haptic.h"

LOG_MODULE_REGISTER(haptic, CONFIG_LOG_DEFAULT_LEVEL);

/* GPIO pin for haptic control using gpio_dt_spec */
static const struct gpio_dt_spec haptic_gpio_pin = 
    GPIO_DT_SPEC_GET_OR(DT_NODELABEL(motor_pin), gpios, {0});

/* Maximum haptic duration in milliseconds */
#define MAX_HAPTIC_DURATION 5000

/* Work item for turning off the haptic actuator */
static struct k_work_delayable haptic_off_work;

/**
 * @brief Handler for the haptic off work item
 *
 * Called when the haptic duration has elapsed to turn off the haptic actuator
 *
 * @param work Pointer to the work item
 */
static void haptic_off_work_handler(struct k_work *work)
{
    gpio_pin_set_dt(&haptic_gpio_pin, 0);
    LOG_DBG("Haptic turned off by work handler");
}

/**
 * @brief Initialize the haptic feedback pin
 *
 * Configures the GPIO pin used to control the haptic actuator
 * and initializes the work item for timing
 *
 * @return 0 on success, negative error code on failure
 */
int init_haptic_pin(void)
{
    /* Initialize the work item for turning off the haptic */
    k_work_init_delayable(&haptic_off_work, haptic_off_work_handler);
    
    if (!device_is_ready(haptic_gpio_pin.port)) {
        LOG_ERR("Haptic GPIO device not ready");
        return -ENODEV;
    }
    
    /* Configure the haptic pin as output, initially low */
    int err = gpio_pin_configure_dt(&haptic_gpio_pin, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure haptic pin: %d", err);
        return err;
    }
    
    LOG_INF("Haptic pin initialized");
    return 0;
}

/**
 * @brief Trigger haptic feedback for specified duration
 *
 * Activates the haptic actuator for the specified duration in milliseconds
 * Uses a work item to turn off the haptic after the duration has elapsed,
 * which avoids blocking the calling thread.
 *
 * @param duration_ms Duration in milliseconds to activate the haptic feedback
 */
void play_haptic_milli(uint32_t duration_ms)
{
    if (!device_is_ready(haptic_gpio_pin.port)) {
        LOG_ERR("Haptic device not initialized");
        return;
    }
    
    if (duration_ms > MAX_HAPTIC_DURATION) {
        LOG_WRN("Requested duration %u ms exceeds maximum allowed %u ms, limiting duration", 
                duration_ms, MAX_HAPTIC_DURATION);
        duration_ms = MAX_HAPTIC_DURATION;
    }
    
    /* Cancel any pending work to turn off the haptic */
    k_work_cancel_delayable(&haptic_off_work);
    
    /* Activate haptic */
    gpio_pin_set_dt(&haptic_gpio_pin, 1);
    
    /* Schedule the work to turn off the haptic after the duration */
    k_work_schedule(&haptic_off_work, K_MSEC(duration_ms));
    
    LOG_DBG("Haptic activated for %u ms", duration_ms);
} 