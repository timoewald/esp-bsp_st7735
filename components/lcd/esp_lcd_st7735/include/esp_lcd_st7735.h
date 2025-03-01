#pragma once
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_ops.h"

#include "hal/lcd_types.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_idf_version.h"


/* ST7735 specific command definitions (if not already in esp_lcd_panel_commands.h) */
#define ST7735_SWRESET   0x01  // Software Reset 
#define ST7735_SLPOUT    0x11  // Sleep Out 
#define ST7735_COLMOD    0x3A  // Interface Pixel format 
#define ST7735_MADCTL    0x36  // Memory Data Access Control 
#define ST7735_CASET     0x2A  // Column Address Set 
#define ST7735_RASET     0x2B  // Row Address Set 
#define ST7735_RAMWR     0x2C  // Memory Write 
#define ST7735_DISPON    0x29  // Display ON 
#define ST7735_NORON     0x13  // Normal Display Mode ON 
#define ST7735_INVON     0x21  // Invert ON 
#define ST7735_INVOFF    0x20  // Invert OFF 
#define ST7735_FRMCTR1   0xB1  // Frame Rate Control 1 (In normal mode) 
#define ST7735_FRMCTR2   0xB2  // Frame Rate Control 2 (In idle mode) 
#define ST7735_FRMCTR3   0xB3  // Frame Rate Control 3 (In partial mode) 
#define ST7735_INVCTR    0xB4  // Display Inversion Control 
#define ST7735_PWCTR1    0xC0  // Power Control 1 
#define ST7735_PWCTR2    0xC1  // Power Control 2 
#define ST7735_PWCTR3    0xC2  // Power Control 3 
#define ST7735_PWCTR4    0xC3  // Power Control 4 
#define ST7735_PWCTR5    0xC4  // Power Control 5 
#define ST7735_VMCTR1    0xC5  // VCOM Control 1 
#define ST7735_GMCTRP1   0xE0  // Gamma Adjustments (Positive) 
#define ST7735_GMCTRN1   0xE1  // Gamma Adjustments (Negative) 

#ifdef __cplusplus
extern "C" {
#endif

/** Optional vendor config for ST7735 panel init (to override default init commands) */
typedef struct {
    const void *init_cmds;    // Pointer to array of init commands (typecast to internal struct) 
    uint16_t init_cmds_size;  // Number of commands in the init array 
} st7735_vendor_config_t;

/**
 * @brief Create a new ST7735 LCD panel instance
 *
 * @param io LCD panel IO handle (SPI bus)
 * @param panel_dev_config general panel config (reset GPIO, color depth, etc.)
 * @param ret_panel out parameter for panel handle
 * @return esp_err_t 
 */
esp_err_t esp_lcd_new_panel_st7735(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
