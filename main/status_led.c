#include "status_led.h"

#include "dongle_state.h"

#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef CONFIG_DONGLE_LED_ENABLED
#define CONFIG_DONGLE_LED_ENABLED 0
#endif

#ifndef CONFIG_DONGLE_LED_GPIO
#define CONFIG_DONGLE_LED_GPIO 48
#endif

#ifndef CONFIG_DONGLE_LED_BRIGHTNESS
#define CONFIG_DONGLE_LED_BRIGHTNESS 32
#endif

#if !defined(CONFIG_DONGLE_LED_TYPE_ADDRESSABLE) && !defined(CONFIG_DONGLE_LED_TYPE_NOOP)
#define CONFIG_DONGLE_LED_TYPE_ADDRESSABLE 1
#endif

#ifndef CONFIG_DONGLE_LED_TYPE_ADDRESSABLE
#define CONFIG_DONGLE_LED_TYPE_ADDRESSABLE 0
#endif

#define RMT_LED_RESOLUTION_HZ 10000000
#define LED_TASK_PERIOD_MS 50
#define WAKE_FLASH_US 250000

static const char *TAG = "LED";

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static rmt_channel_handle_t s_led_chan;
static rmt_encoder_handle_t s_led_encoder;
static int64_t s_wake_flash_until_us;

static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1,
    .duration0 = 0.3 * RMT_LED_RESOLUTION_HZ / 1000000,
    .level1 = 0,
    .duration1 = 0.9 * RMT_LED_RESOLUTION_HZ / 1000000,
};

static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1,
    .duration0 = 0.9 * RMT_LED_RESOLUTION_HZ / 1000000,
    .level1 = 0,
    .duration1 = 0.3 * RMT_LED_RESOLUTION_HZ / 1000000,
};

static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 0,
    .duration0 = RMT_LED_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level1 = 0,
    .duration1 = RMT_LED_RESOLUTION_HZ / 1000000 * 50 / 2,
};

static size_t led_encoder_cb(const void *data, size_t data_size,
                             size_t symbols_written, size_t symbols_free,
                             rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    (void)arg;
    if (symbols_free < 8) return 0;

    const size_t data_pos = symbols_written / 8;
    const uint8_t *bytes = data;
    if (data_pos < data_size) {
        size_t pos = 0;
        for (int bit = 0x80; bit != 0; bit >>= 1) {
            symbols[pos++] = (bytes[data_pos] & bit) ? ws2812_one : ws2812_zero;
        }
        return pos;
    }

    symbols[0] = ws2812_reset;
    *done = true;
    return 1;
}

static uint8_t scale_component(uint8_t v)
{
    return (uint8_t)(((uint16_t)v * CONFIG_DONGLE_LED_BRIGHTNESS) / 255);
}

static rgb_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return (rgb_t) {
        .r = scale_component(r),
        .g = scale_component(g),
        .b = scale_component(b),
    };
}

static uint8_t triangle_u8(uint32_t elapsed_ms, uint32_t period_ms, uint8_t min_v, uint8_t max_v)
{
    if (period_ms == 0) return max_v;
    uint32_t phase = elapsed_ms % period_ms;
    uint32_t half = period_ms / 2;
    uint32_t span = max_v - min_v;
    if (phase < half) {
        return min_v + (uint8_t)((span * phase) / half);
    }
    return max_v - (uint8_t)((span * (phase - half)) / half);
}

static bool blink_on(uint32_t elapsed_ms, uint32_t period_ms, uint32_t on_ms)
{
    return (elapsed_ms % period_ms) < on_ms;
}

static rgb_t color_for_state(const dongle_status_t *st, uint32_t elapsed_ms)
{
    // PC sleeping: dim purple heartbeat
    if (st->usb_suspended && st->state != DONGLE_STATE_OTA_UPDATE &&
        st->state != DONGLE_STATE_ERROR) {
        uint8_t pulse = triangle_u8(elapsed_ms, 2400, 8, 45);
        return rgb(pulse * 2 / 3, 0, pulse);
    }

    // Mouse mode active on any controller: orange heartbeat
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (st->controllers[i].mouse_mode) {
            uint8_t pulse = triangle_u8(elapsed_ms, 2000, 30, 180);
            return rgb(pulse, pulse / 3, 0);
        }
    }

    if (st->state == DONGLE_STATE_BOOTING) {
        uint8_t pulse = triangle_u8(elapsed_ms, 1800, 12, 120);
        return rgb(pulse, pulse, pulse);
    }

    if (st->state == DONGLE_STATE_OTA_UPDATE) {
        return blink_on(elapsed_ms, 300, 150) ? rgb(160, 0, 220) : rgb(255, 180, 0);
    }

    if (st->state == DONGLE_STATE_ERROR) {
        uint8_t pulse = triangle_u8(elapsed_ms, 800, 10, 255);
        return rgb(pulse, 0, 0);
    }

    // Scanning / connected based on controller count.
    if (st->connected_count >= 2) {
        // Two controllers: faster green heartbeat
        return rgb(0, triangle_u8(elapsed_ms, 2000, 80, 200), 0);
    } else if (st->connected_count >= 1) {
        // One controller: green heartbeat
        return rgb(0, triangle_u8(elapsed_ms, 4000, 80, 160), 0);
    } else if (st->webui_active) {
        // Scanning + Wi-Fi AP on: blue-yellow alternating heartbeat
        bool blue = (elapsed_ms % 2400) < 1200;
        uint8_t pulse = triangle_u8(elapsed_ms, 2400, 20, 100);
        return blue ? rgb(0, 0, pulse) : rgb(pulse, pulse, 0);
    } else {
        // Scanning / no controller: dim blue heartbeat
        uint8_t pulse = triangle_u8(elapsed_ms, 2400, 20, 100);
        return rgb(0, 0, pulse);
    }
}

static void led_set(rgb_t c)
{
#if CONFIG_DONGLE_LED_ENABLED && CONFIG_DONGLE_LED_TYPE_ADDRESSABLE
    if (!s_led_chan || !s_led_encoder) return;

    const uint8_t grb[3] = { c.g, c.r, c.b };
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    esp_err_t err = rmt_transmit(s_led_chan, s_led_encoder, grb, sizeof(grb), &tx_config);
    if (err == ESP_OK) {
        rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(10));
    }
#else
    (void)c;
#endif
}

static bool led_hw_init(void)
{
#if CONFIG_DONGLE_LED_ENABLED && CONFIG_DONGLE_LED_TYPE_ADDRESSABLE
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = CONFIG_DONGLE_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_RESOLUTION_HZ,
        .trans_queue_depth = 1,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_led_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RMT channel init failed: %s", esp_err_to_name(err));
        return false;
    }

    rmt_simple_encoder_config_t encoder_cfg = {
        .callback = led_encoder_cb,
    };
    err = rmt_new_simple_encoder(&encoder_cfg, &s_led_encoder);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RMT encoder init failed: %s", esp_err_to_name(err));
        return false;
    }

    err = rmt_enable(s_led_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RMT enable failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Status LED enabled on GPIO %d brightness %d", CONFIG_DONGLE_LED_GPIO,
             CONFIG_DONGLE_LED_BRIGHTNESS);
    return true;
#else
    ESP_LOGI(TAG, "Status LED disabled/no-op");
    return false;
#endif
}

static void status_led_task(void *arg)
{
    (void)arg;
    const bool hw_ok = led_hw_init();
    const int64_t start_us = esp_timer_get_time();

    while (1) {
        dongle_status_t st;
        dongle_state_get_status(&st);

        int64_t now_us = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now_us - start_us) / 1000);
        rgb_t c = now_us < s_wake_flash_until_us ? rgb(255, 255, 255)
                                                  : color_for_state(&st, elapsed_ms);
        if (hw_ok) led_set(c);
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

void status_led_init(void)
{
#if CONFIG_DONGLE_LED_ENABLED
    xTaskCreate(status_led_task, "status_led", 3072, NULL, 1, NULL);
#endif
}

void status_led_flash_remote_wake(void)
{
    s_wake_flash_until_us = esp_timer_get_time() + WAKE_FLASH_US;
}
