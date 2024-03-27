#pragma once


#if CONFIG_DOUBLE_FB
#define LCD_NUM_FB             2
#else
#define LCD_NUM_FB             1
#endif // CONFIG_DOUBLE_FB

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

#define MATOUCH_7IN_RGB 1
//#define SD 1
#define UHD 1

#if MATOUCH_7IN_RGB
#define LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  0
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT       10
#define PIN_NUM_HSYNC          39
#define PIN_NUM_VSYNC          41
#define PIN_NUM_DE             40
#define PIN_NUM_PCLK           42
#define PIN_NUM_DATA0          8 // B0
#define PIN_NUM_DATA1          3 // B1
#define PIN_NUM_DATA2          46 // B2
#define PIN_NUM_DATA3          9 // B3
#define PIN_NUM_DATA4          1 // B4
#define PIN_NUM_DATA5          5 // G0
#define PIN_NUM_DATA6          6 // G1
#define PIN_NUM_DATA7          7 // G2
#define PIN_NUM_DATA8          15 // G3
#define PIN_NUM_DATA9          16 // G4
#define PIN_NUM_DATA10         4 // G5
#define PIN_NUM_DATA11         45  // R0
#define PIN_NUM_DATA12         48 // R1
#define PIN_NUM_DATA13         47 // R2
#define PIN_NUM_DATA14         21 // R3
#define PIN_NUM_DATA15         14 // R4
#define PIN_NUM_DISP_EN        -1
// The pixel number in horizontal and vertical
#if UHD
#define LCD_H_RES              1024
#define LCD_V_RES              600
#elif SD
#define LCD_H_RES              
#define LCD_V_RES              
#endif

#define LCD_HSYNC_BACK_PORCH   128
#define LCD_HSYNC_FRONT_PORCH  40
#define LCD_HSYNC_PULSE_WIDTH  48
#define LCD_VSYNC_BACK_PORCH   45
#define LCD_VSYNC_FRONT_PORCH  13
#define LCD_VSYNC_PULSE_WIDTH  3
#define LCD_PIXEL_CLK_ACT_NEG  true

#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

esp_lcd_panel_io_i2c_config_t lcd_io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
esp_lcd_touch_config_t tp_cfg = {
    .x_max = LCD_H_RES,
    .y_max = LCD_V_RES,
    .rst_gpio_num = -1,
    .int_gpio_num = -1,
    .levels = {
        .reset = 0,
        .interrupt = 0,
    },
    .flags = {
        .swap_xy = 0,
        .mirror_x = 0,
        .mirror_y = 0,
    },
};

esp_lcd_touch_handle_t tp;

i2c_master_bus_config_t i2c_mst_cfg =
{
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 17,    
    .scl_io_num = 18,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    //.intr_priority = 0,
    //.trans_queue_depth = 1,
    .flags.enable_internal_pullup = 1,    
};

i2c_master_bus_handle_t bus0;
esp_lcd_panel_io_handle_t tp_i2c_handle;

esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,

        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = PIN_NUM_DISP_EN,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .data_gpio_nums = {
            PIN_NUM_DATA0,
            PIN_NUM_DATA1,
            PIN_NUM_DATA2,
            PIN_NUM_DATA3,
            PIN_NUM_DATA4,
            PIN_NUM_DATA5,
            PIN_NUM_DATA6,
            PIN_NUM_DATA7,
            PIN_NUM_DATA8,
            PIN_NUM_DATA9,
            PIN_NUM_DATA10,
            PIN_NUM_DATA11,
            PIN_NUM_DATA12,
            PIN_NUM_DATA13,
            PIN_NUM_DATA14,
            PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = LCD_HSYNC_BACK_PORCH,
            .hsync_front_porch = LCD_HSYNC_FRONT_PORCH,
            .hsync_pulse_width = LCD_HSYNC_PULSE_WIDTH,
            .vsync_back_porch = LCD_VSYNC_BACK_PORCH,
            .vsync_front_porch = LCD_VSYNC_FRONT_PORCH,
            .vsync_pulse_width = LCD_VSYNC_PULSE_WIDTH,
            .flags.pclk_active_neg = LCD_PIXEL_CLK_ACT_NEG,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };

    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    
#endif
