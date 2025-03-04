/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"

#include "bsp/esp_bsp_st7735.h"
#include "bsp_err_check.h"

#if CONFIG_BSP_DISPLAY_ENABLED
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "bsp/display.h"

#if CONFIG_BSP_DISPLAY_DRIVER_ILI9341
#include "esp_lcd_ili9341.h"
#elif CONFIG_BSP_DISPLAY_DRIVER_GC9A01
#include "esp_lcd_gc9a01.h"
#elif CONFIG_BSP_DISPLAY_DRIVER_ST7735
#include "esp_lcd_st7735.h"
#endif

#endif

#if CONFIG_BSP_TOUCH_ENABLED
#include "bsp/touch.h"
#if CONFIG_BSP_TOUCH_DRIVER_TT21100
#include "esp_lcd_touch_tt21100.h"
#elif CONFIG_BSP_TOUCH_DRIVER_GT1151
#include "esp_lcd_touch_gt1151.h"
#elif CONFIG_BSP_TOUCH_DRIVER_GT911
#include "esp_lcd_touch_gt911.h"
#elif CONFIG_BSP_TOUCH_DRIVER_CST816S
#include "esp_lcd_touch_cst816s.h"
#elif CONFIG_BSP_TOUCH_DRIVER_FT5X06
#include "esp_lcd_touch_ft5x06.h"
#endif
#endif

static const char *TAG = "BSP-st7735";

#if CONFIG_BSP_DISPLAY_ENABLED
static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
#endif

#if CONFIG_BSP_TOUCH_ENABLED
static esp_lcd_touch_handle_t tp;   // LCD touch handle
#endif

#if CONFIG_BSP_DISPLAY_ENABLED
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           CONFIG_BSP_DISPLAY_CMD_BITS
#define LCD_PARAM_BITS         CONFIG_BSP_DISPLAY_PARAM_BITS

esp_err_t bsp_display_brightness_init(void)
{
#if CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));
#endif
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
#if CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

#if CONFIG_BSP_DISPLAY_BRIGHTNESS_INVERT
    brightness_percent = (100 - brightness_percent);
#endif

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH));
#endif
    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_PCLK,
        .mosi_io_num = BSP_LCD_DATA0,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
#if CONFIG_BSP_DISPLAY_DRIVER_ST7789
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");
    ESP_LOGI(TAG, "Initialize LCD: ST7789");
#elif CONFIG_BSP_DISPLAY_DRIVER_ILI9341
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");
    ESP_LOGI(TAG, "Initialize LCD: ILI9341");
#elif CONFIG_BSP_DISPLAY_DRIVER_GC9A01
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_gc9a01(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");
    ESP_LOGI(TAG, "Initialize LCD: GC9A01");
#elif CONFIG_BSP_DISPLAY_DRIVER_ST7735
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7735(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");
    ESP_LOGI(TAG, "Initialize LCD: ST7735");
#endif
    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);

    bool disp_swap_xy = false;
    bool disp_mirror_x = false;
    bool disp_mirror_y = false;
    bool disp_invert_color = false;
#if CONFIG_BSP_DISPLAY_ROTATION_SWAP_XY
    disp_swap_xy = true;
#endif
#if CONFIG_BSP_DISPLAY_ROTATION_MIRROR_X
    disp_mirror_x = true;
#endif
#if CONFIG_BSP_DISPLAY_ROTATION_MIRROR_Y
    disp_mirror_y = true;
#endif
#if CONFIG_BSP_DISPLAY_INVERT_COLOR
    disp_invert_color = true;
#endif

    esp_lcd_panel_mirror(*ret_panel, disp_mirror_x, disp_mirror_y);
    esp_lcd_panel_swap_xy(*ret_panel, disp_swap_xy);
    esp_lcd_panel_invert_color(*ret_panel, disp_invert_color);
    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

static lv_display_t *bsp_display_lcd_init(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = (BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = 1,
#else
        .double_buffer = 0,
#endif
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
#if CONFIG_BSP_DISPLAY_ROTATION_SWAP_XY
            .swap_xy = true,
#endif
#if CONFIG_BSP_DISPLAY_ROTATION_MIRROR_X
            .mirror_x = true,
#endif
#if CONFIG_BSP_DISPLAY_ROTATION_MIRROR_Y
            .mirror_y = true,
#endif
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };
#if BSP_LCD_H_OFFSET || BSP_LCD_V_OFFSET
    esp_lcd_panel_set_gap(panel_handle, (BSP_LCD_H_OFFSET), (BSP_LCD_V_OFFSET));
#endif
    return lvgl_port_add_disp(&disp_cfg);
}

#if CONFIG_BSP_TOUCH_ENABLED
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = BSP_LCD_TOUCH_RST,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
#if CONFIG_BSP_TOUCH_ROTATION_SWAP_XY
            .swap_xy = true,
#endif
#if CONFIG_BSP_TOUCH_ROTATION_MIRROR_X
            .mirror_x = true,
#endif
#if CONFIG_BSP_TOUCH_ROTATION_MIRROR_Y
            .mirror_y = true,
#endif
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
#if CONFIG_BSP_TOUCH_DRIVER_TT21100
    ESP_LOGI(TAG, "Initialize LCD Touch: TT21100");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_TT21100_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_tt21100(tp_io_handle, &tp_cfg, ret_touch);
#elif CONFIG_BSP_TOUCH_DRIVER_GT1151
    ESP_LOGI(TAG, "Initialize LCD Touch: GT1151");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT1151_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt1151(tp_io_handle, &tp_cfg, ret_touch);
#elif CONFIG_BSP_TOUCH_DRIVER_GT911
    ESP_LOGI(TAG, "Initialize LCD Touch: GT911");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
#elif CONFIG_BSP_TOUCH_DRIVER_CST816S
    ESP_LOGI(TAG, "Initialize LCD Touch: CST816S");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, ret_touch);
#elif CONFIG_BSP_TOUCH_DRIVER_FT5X06
    ESP_LOGI(TAG, "Initialize LCD Touch: FT5X06");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5X06_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ; // This parameter was introduced together with I2C Driver-NG in IDF v5.2
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, ret_touch);
#endif
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}
#endif //CONFIG_BSP_TOUCH_ENABLED

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG()
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);
#if CONFIG_BSP_TOUCH_ENABLED
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);
#endif //CONFIG_BSP_TOUCH_ENABLED

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
#endif //CONFIG_BSP_DISPLAY_ENABLED