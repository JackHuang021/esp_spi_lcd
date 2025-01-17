/**
 * @file lcd.h
 * @author Jack
 * @brief 
 * @version 0.1
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_st7789.h"
#include "i2c_new.h"

/**
 * @brief config info of lvgl
 * 
 */
struct lvgl_config {
    int task_priority;      /* LVGL任务的优先级 */
    int task_stack;         /* LVGL任务的栈大小 */
    int task_max_delay_ms;
    int task_min_delay_ms;
    int timer_period_ms;    /* LVGL定时器周期 */
};

/**
 * @brief config info of display
 * 
 */
struct display_config {
    uint32_t frame_buffer_size;
    uint32_t trans_size;
    uint32_t hres;
    uint32_t vres;
    uint32_t pclk_hz;
    spi_host_device_t spi_host;

    gpio_num_t dc_gpio_num;
    gpio_num_t rst_gpio_num;
    gpio_num_t cs_gpio_num;
    gpio_num_t bl_gpio_num;
    gpio_num_t te_gpio_num;

    gpio_int_type_t tear_intr_type;
    lv_display_rotation_t sw_rotate;
    lv_color_format_t color_format;
    struct lvgl_config lvgl_cfg;

    struct i2c_obj *i2c;

    int lcd_cmd_bits;
    int lcd_param_bits;
};

esp_err_t lcd_init(struct display_config *disp_cfg);
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);
