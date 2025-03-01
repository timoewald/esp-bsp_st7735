#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"  // standard LCD command definitions (MADCTL, etc)
#include "esp_lcd_st7735.h"

#include "soc/soc_caps.h"
#include "esp_lcd_types.h"


#include <sys/cdefs.h>
#include "esp_lcd_panel_vendor.h"



static const char *TAG = "st7735";

/* Internal structure for ST7735 panel state */
typedef struct {
    esp_lcd_panel_t base;              // Base panel interface
    esp_lcd_panel_io_handle_t io;      // LCD panel IO handle (SPI)
    int x_gap;                         // Optional x offset (column start offset)
    int y_gap;                         // Optional y offset (row start offset)
    uint8_t fb_bits_per_pixel;         // Frame buffer color depth (bits per pixel)
    uint8_t madctl_val;                // Current MADCTL register value (orientation/color order)
    uint8_t colmod_val;                // Current COLMOD register value (color mode)
    const uint8_t *init_cmds;          // Pointer to init commands array (if provided via vendor_config)
    uint16_t init_cmds_size;           // Number of init commands
} st7735_panel_t;

/* Type for an initialization command entry */
typedef struct {
    uint8_t cmd;
    uint8_t data_bytes;    // Number of data bytes; 0xFF denotes end of init sequence
    uint8_t data[];        // Data to send after command (variable length)
} st7735_lcd_init_cmd_t;

/* Default initialization commands for ST7735 (if no custom sequence provided) */
static const st7735_lcd_init_cmd_t vendor_specific_init_default[] = {
    { ST7735_SWRESET, 0, { } },                      // Software reset (no args)
    { ST7735_SLPOUT,  0, { } },                      // Exit sleep mode
    // Frame Rate Control (normal/idle/partial modes)
    { ST7735_FRMCTR1, 3, { 0x05, 0x3A, 0x3A } },     // FRMCTR1: frame rate = fosc/(1*2+40) * (LINE+2C+2D)
    { ST7735_FRMCTR2, 3, { 0x05, 0x3A, 0x3A } },     // FRMCTR2: same as FRMCTR1 (idle mode)
    { ST7735_FRMCTR3, 6, { 0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A } }, // FRMCTR3: dot inversion mode
    { ST7735_INVCTR,  1, { 0x03 } },                 // INVCTR: no inversion
    // Power control
    { ST7735_PWCTR1,  3, { 0x62, 0x02, 0x04 } },     // Power control -4.6V, Auto mode
    { ST7735_PWCTR2,  1, { 0xC0 } },                 // VGH25=2.4C, VGSEL=-10, etc.
    { ST7735_PWCTR3,  2, { 0x0D, 0x00 } },
    { ST7735_PWCTR4,  2, { 0x8D, 0x6A } },
    { ST7735_PWCTR5,  2, { 0x8D, 0xEE } },
    { ST7735_VMCTR1,  1, { 0x0E } },                 // VCOM control
    // Display settings
    { ST7735_INVON,   0, { } },                      // Invert colors ON (use ST7735_INVOFF for normal colors)
    { ST7735_COLMOD,  1, { 0x05 } },                 // Color mode: 16-bit/pixel
    // Gamma correction tables
    { ST7735_GMCTRP1, 16, { 0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07,
                             0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10 } },
    { ST7735_GMCTRN1, 16, { 0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08,
                             0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10 } },
    { ST7735_NORON,   0, { } },                      // Normal display mode ON
    { ST7735_DISPON,  0, { } }                       // Display ON
    // End of sequence (data_bytes == 0xFF could be used to mark end if needed)
};

/* Forward declarations of panel functions (implement esp_lcd_panel_t interface) */
static esp_err_t panel_st7735_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7735_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7735_draw_bitmap(esp_lcd_panel_t *panel,
                                         int x_start, int y_start, int x_end, int y_end,
                                         const void *color_data);
static esp_err_t panel_st7735_invert_color(esp_lcd_panel_t *panel, bool invert);
static esp_err_t panel_st7735_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7735_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7735_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7735_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

/* Create a new ST7735 panel instance */
esp_err_t esp_lcd_new_panel_st7735(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7735_panel_t *st7735 = NULL;
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    st7735 = calloc(1, sizeof(st7735_panel_t));
    ESP_RETURN_ON_FALSE(st7735, ESP_ERR_NO_MEM, TAG, "no mem for st7735 panel");

    // Configure RST GPIO (if used)
    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST failed");
        // Reset the display by toggling the line
        gpio_set_level(panel_dev_config->reset_gpio_num, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(panel_dev_config->reset_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // Set initial MADCTL value based on RGB endianness 
    switch (panel_dev_config->rgb_endian) {
        case LCD_RGB_ENDIAN_RGB:
            st7735->madctl_val = 0x00;
            break;
        case LCD_RGB_ENDIAN_BGR:
            st7735->madctl_val = 0x00 | LCD_CMD_BGR_BIT;  // set BGR bit if available
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
    }
    // Set initial color mode (COLMOD) value based on bits_per_pixel 
    switch (panel_dev_config->bits_per_pixel) {
        case 16: // RGB565
            st7735->colmod_val = 0x55;   // 16-bit mode command value for COLMOD
            st7735->fb_bits_per_pixel = 16;
            break;
        case 18: // RGB666
            st7735->colmod_val = 0x66;   // 18-bit mode command value
            st7735->fb_bits_per_pixel = 24;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
    }

    st7735->io = io;
    // Save custom init commands if provided via vendor_config
    if (panel_dev_config->vendor_config) {
        const st7735_vendor_config_t *vcfg = (const st7735_vendor_config_t *) panel_dev_config->vendor_config;
        st7735->init_cmds = vcfg->init_cmds;
        st7735->init_cmds_size = vcfg->init_cmds_size;
    }

    // Hook function pointers to esp_lcd_panel_t interface
    st7735->base.reset = panel_st7735_reset;
    st7735->base.init = panel_st7735_init;
    st7735->base.draw_bitmap = panel_st7735_draw_bitmap;
    st7735->base.invert_color = panel_st7735_invert_color;
    st7735->base.set_gap = panel_st7735_set_gap;
    st7735->base.mirror = panel_st7735_mirror;
    st7735->base.swap_xy = panel_st7735_swap_xy;
    st7735->base.disp_off = panel_st7735_disp_on_off;

    *ret_panel = &(st7735->base);
    ESP_LOGD(TAG, "new ST7735 panel @%p", st7735);
    return ESP_OK;
err:
    if (st7735) free(st7735);
    return ret;
}

/* Reset callback (via esp_lcd_panel_reset) */
static esp_err_t panel_st7735_reset(esp_lcd_panel_t *panel)
{
    // The reset is already handled in esp_lcd_new_panel_st7735 via GPIO toggling.
    // We can optionally resend a SW reset command:
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    return esp_lcd_panel_io_tx_param(st7735->io, ST7735_SWRESET, NULL, 0);
}

/* Initialization sequence: sends all the commands to configure the display */
static esp_err_t panel_st7735_init(esp_lcd_panel_t *panel)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735->io;
    ESP_LOGI(TAG, "Initializing ST7735 panel...");

    // Exit sleep mode
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG, "SLPOUT send failed");
    vTaskDelay(pdMS_TO_TICKS(100));  // wait 100 ms after sleep out

    // Set MADCTL (orientation/color order)
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                         (uint8_t[]){ st7735->madctl_val }, 1),
                         TAG, "MADCTL send failed");
    // Set COLMOD (color mode) 
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD,
                         (uint8_t[]){ st7735->colmod_val }, 1),
                         TAG, "COLMOD send failed");

    // Determine which init command sequence to use
    const st7735_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    if (st7735->init_cmds) {
        init_cmds      = (const st7735_lcd_init_cmd_t*) st7735->init_cmds;
        init_cmds_size = st7735->init_cmds_size;
    } else {
        init_cmds      = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(st7735_lcd_init_cmd_t);
    }
    ESP_LOGD(TAG, "Sending %d init commands", init_cmds_size);
    // Send each command in sequence
    for (int i = 0; i < init_cmds_size; ++i) {
        uint8_t cmd = init_cmds[i].cmd;
        uint8_t num_data = init_cmds[i].data_bytes;
        const uint8_t *data = init_cmds[i].data;
        // If command conflicts with already sent MADCTL/COLMOD, log warning 
        if (cmd == ST7735_MADCTL || cmd == ST7735_COLMOD) {
            ESP_LOGW(TAG, "Init sequence contains 0x%02X, which will override previous setting", cmd);
        }
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_param(io, cmd, data, num_data),
            TAG, "Init command 0x%02X failed", cmd
        );
        // Apply specific delays if needed (some commands may require wait)
        // In our init array, delays are already accounted by separate entries or general short delay
        // Example: after certain commands like SLPOUT or DISPON a delay is recommended.
        if (cmd == ST7735_SLPOUT) {
            vTaskDelay(pdMS_TO_TICKS(500)); // Sleep Out needs ample delay
        } else if (cmd == ST7735_DISPON) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Display ON delay
        } else if (cmd == ST7735_SWRESET) {
            vTaskDelay(pdMS_TO_TICKS(150)); // Software reset delay
        }
    }
    ESP_LOGI(TAG, "ST7735 initialization sequence done");
    return ESP_OK;
}

/* Draw bitmap (flush color data to a region of the screen) */
static esp_err_t panel_st7735_draw_bitmap(esp_lcd_panel_t *panel,
                                         int x_start, int y_start, int x_end, int y_end,
                                         const void *color_data)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    assert(x_start < x_end && y_start < y_end && "Invalid bitmap coordinates");
    esp_lcd_panel_io_handle_t io = st7735->io;

    // Apply any set gap/offset (offsets are added to the coordinates)
    x_start += st7735->x_gap;
    x_end   += st7735->x_gap;
    y_start += st7735->y_gap;
    y_end   += st7735->y_gap;

    // Set Column Address Window (CASET)
    uint8_t col_data[] = {
        (uint8_t)((x_start >> 8) & 0xFF),
        (uint8_t)(x_start & 0xFF),
        (uint8_t)(((x_end - 1) >> 8) & 0xFF),
        (uint8_t)((x_end - 1) & 0xFF)
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, col_data, 4),
                        TAG, "CASET failed");
    // Set Row Address Window (RASET)
    uint8_t row_data[] = {
        (uint8_t)((y_start >> 8) & 0xFF),
        (uint8_t)(y_start & 0xFF),
        (uint8_t)(((y_end - 1) >> 8) & 0xFF),
        (uint8_t)((y_end - 1) & 0xFF)
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, row_data, 4),
                        TAG, "RASET failed");
    // Transfer pixel data
    size_t len = (x_end - x_start) * (y_end - y_start) * (st7735->fb_bits_per_pixel / 8);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len),
                        TAG, "RAMWR (color data) failed");
    return ESP_OK;
}

/* Invert colors on/off */
static esp_err_t panel_st7735_invert_color(esp_lcd_panel_t *panel, bool invert)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    uint8_t command = invert ? LCD_CMD_INVON : LCD_CMD_INVOFF;  // use standard INVON/INVOFF
    return esp_lcd_panel_io_tx_param(st7735->io, command, NULL, 0);
}

/* Mirror (flip) display horizontally and/or vertically */
static esp_err_t panel_st7735_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    // MADCTL bits for mirroring: BIT0 = column order, BIT1 = row order
    uint8_t madctl = st7735->madctl_val;
    if (mirror_x) madctl |= LCD_CMD_MX_BIT;
    else          madctl &= ~LCD_CMD_MX_BIT;
    if (mirror_y) madctl |= LCD_CMD_MY_BIT;
    else          madctl &= ~LCD_CMD_MY_BIT;
    st7735->madctl_val = madctl;
    return esp_lcd_panel_io_tx_param(st7735->io, LCD_CMD_MADCTL, &madctl, 1);
}

/* Swap X and Y axes (for rotation by 90/270 degrees) */
static esp_err_t panel_st7735_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    uint8_t madctl = st7735->madctl_val;
    if (swap_axes) madctl |= LCD_CMD_MV_BIT;
    else           madctl &= ~LCD_CMD_MV_BIT;
    st7735->madctl_val = madctl;
    return esp_lcd_panel_io_tx_param(st7735->io, LCD_CMD_MADCTL, &madctl, 1);
}

/* Set gap/offset (for non-zero start coordinates) */
static esp_err_t panel_st7735_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    st7735->x_gap = x_gap;
    st7735->y_gap = y_gap;
    return ESP_OK;
}

/* Turn the display on or off (enable/disable display output) */
static esp_err_t panel_st7735_disp_on_off(esp_lcd_panel_t *panel, bool off)
{
    st7735_panel_t *st7735 = __containerof(panel, st7735_panel_t, base);
    uint8_t cmd = off ? LCD_CMD_DISPOFF : LCD_CMD_DISPON;
    return esp_lcd_panel_io_tx_param(st7735->io, cmd, NULL, 0);
}
