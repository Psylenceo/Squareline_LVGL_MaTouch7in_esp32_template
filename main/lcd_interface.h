#pragma once

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
#endif