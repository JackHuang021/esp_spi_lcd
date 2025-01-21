/**
 * @file lcd.c
 * @author Jack Huang (jackhuang021@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-01-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_panel_st7789.h"
#include "lcd.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lcd_panel_commands.h"
#include "lv_demos.h"
#include "sdkconfig.h"

static const char *tag = "lcd";

struct display_ctx {
    esp_lcd_panel_io_handle_t panel_io_handle;
    esp_lcd_panel_io_handle_t tp_io_handle;
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_touch_handle_t tp_handle;
    lv_disp_t *lv_disp;
    lv_indev_t *lv_indev;
    esp_timer_handle_t lvgl_timer;


    uint32_t trans_size;
    uint8_t *frame_buffer;
    uint8_t *trans_buf;
    uint8_t bpp;
    SemaphoreHandle_t trans_done_sem;

    uint32_t lvgl_timer_period_ms;
    uint32_t lvgl_task_max_delay_ms;
    uint32_t lvgl_task_min_delay_ms;

    gpio_num_t te_gpio_num;

    SemaphoreHandle_t lvgl_mux;
    SemaphoreHandle_t te_v_sync_sem;
    SemaphoreHandle_t te_catch_sem;

    bool lvgl_running;
};

/* display context */
static struct display_ctx *disp_ctx =  NULL;

/**
 * @brief TE interrupt handle
 * 
 * @param arg point to display context
 */
static void IRAM_ATTR display_tear_interrupt(void *arg)
{
    assert(arg);
    struct display_ctx *disp_ctx = (struct display_ctx *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (disp_ctx->te_v_sync_sem) {
        /* send vsync semphore */
        xSemaphoreGiveFromISR(disp_ctx->te_v_sync_sem, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
    }
    gpio_isr_handler_remove(disp_ctx->te_gpio_num);
}

/**
 * @brief notify lvgl do next flush when last flush finished
 * 
 * @param panel_io lcd panel io pointer
 * @param edata Panel IO event data
 * @param user_ctx lv_disp_drv_t pointer
 * @return true a high priority task has been waken up by this function
 */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    struct display_ctx *disp_ctx = (struct display_ctx *)user_ctx;

    lv_disp_flush_ready(disp_ctx->lv_disp);

    return false;
}

static void lvgl_flush_callback(lv_display_t *disp, const lv_area_t *area,
                                uint8_t *px_map)
{
    assert(disp != NULL);
    struct display_ctx *disp_ctx =
                    (struct display_ctx * )lv_display_get_driver_data(disp);
    assert(disp_ctx != NULL);

    const int x_start = area->x1;
    const int x_end = area->x2;
    const int y_start = area->y1;
    const int y_end = area->y2;
    static uint8_t set_te_scanline_val[2];
    uint16_t scanline_val_tmp = (y_end - y_start + 1) / 2 + y_start;
    set_te_scanline_val[0] = scanline_val_tmp & 0xFF;
    set_te_scanline_val[1] = scanline_val_tmp >> 8;

    // set te interrupt line
    esp_lcd_panel_io_tx_param(disp_ctx->panel_io_handle, LCD_CMD_STE,
                              (uint8_t[]) {set_te_scanline_val[0], set_te_scanline_val[1]}, 2);
    gpio_isr_handler_add(GPIO_NUM_2, display_tear_interrupt, disp_ctx);
    // wait te sem
    xSemaphoreTake(disp_ctx->te_v_sync_sem, portMAX_DELAY);
    esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, x_start, y_start,
                                    x_end + 1, y_end + 1, px_map);
}

/**
 * @brief timer callback to increase lvgl tick
 * 
 * @param arg point to display context
 */
static void lvgl_tick_increment(void *arg)
{
    struct display_ctx *disp_ctx = (struct display_ctx *)arg;

    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(disp_ctx->lvgl_timer_period_ms);
}

/**
 * @brief lock lvgl
 * 
 * @param timeout_ms timeout time
 * @return true lock lvgl success
 * @return false fail to lock lvgl
 */
bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks =
                (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(disp_ctx->lvgl_mux, timeout_ticks) == pdTRUE);
}

/**
 * @brief unlock lvgl
 * 
 */
void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(disp_ctx->lvgl_mux);
}

/**
 * @brief lvgl task
 * 
 * @param arg not used
 */
static void lvgl_port_task(void *arg)
{
    struct display_ctx *disp_ctx = (struct display_ctx *)arg;
    uint32_t task_delay_ms = disp_ctx->lvgl_task_max_delay_ms;

    ESP_LOGI(tag, "start lvgl task");
    disp_ctx->lvgl_running = true;
    while (disp_ctx->lvgl_running) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > disp_ctx->lvgl_task_max_delay_ms) {
            task_delay_ms = disp_ctx->lvgl_task_max_delay_ms;
        } else if (task_delay_ms < disp_ctx->lvgl_task_min_delay_ms) {
            task_delay_ms = disp_ctx->lvgl_task_min_delay_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }

    vTaskDelete(NULL);
}

/**
 * @brief update touch data
 * 
 * @param indev point to input device
 * @param data point to input data
 */
static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    esp_lcd_touch_handle_t tp_handle =
                        (esp_lcd_touch_handle_t)lv_indev_get_driver_data(indev);

    /* Read touch controller data */
    esp_lcd_touch_read_data(tp_handle);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp_handle, touchpad_x,
                                        touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(tag, "x: %ld, y: %ld", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/**
 * @brief lcd & lvgl init
 * 
 * @param disp_cfg point to display config parameters
 * @return esp_err_t 
 */
esp_err_t lcd_init(struct display_config *disp_cfg)
{
    esp_err_t ret = ESP_OK;

    disp_ctx = malloc(sizeof(struct display_ctx));
    ESP_GOTO_ON_FALSE(disp_ctx, ESP_ERR_NO_MEM, err, tag,
                      "not enough memory for lvgl context allocation");

    disp_ctx->lvgl_timer_period_ms = disp_cfg->lvgl_cfg.timer_period_ms;
    disp_ctx->lvgl_task_min_delay_ms = disp_cfg->lvgl_cfg.task_min_delay_ms;
    disp_ctx->lvgl_task_max_delay_ms = disp_cfg->lvgl_cfg.task_max_delay_ms;
    disp_ctx->te_gpio_num = disp_cfg->te_gpio_num;
    disp_ctx->bpp = LV_COLOR_DEPTH / 8;
    disp_ctx->trans_size = disp_cfg->trans_size;

    /* lcd vsync sem */
    disp_ctx->te_v_sync_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(disp_ctx->te_v_sync_sem, ESP_ERR_NO_MEM, err, tag,
                      "failed to create te sync semaphore");

    /* spi data transfer done sem */
    disp_ctx->trans_done_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(disp_ctx->trans_done_sem, ESP_ERR_NO_MEM, err, tag,
                      "failed to create te sync semaphore");

    /* ready to catch te signal sem */
    disp_ctx->te_catch_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(disp_ctx->te_catch_sem, ESP_ERR_NO_MEM, err, tag,
                      "failed to create te sync semaphore");

    /* init lcd backlight io */
    ESP_LOGI(tag, "turn off lcd backlight");
    gpio_config_t lcd_bl_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << disp_cfg->bl_gpio_num,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&lcd_bl_gpio_config), err, tag,
                      "te gpio config failed");
    gpio_set_level(disp_cfg->bl_gpio_num, 1);

    /* init lcd */
    ESP_LOGI(tag, "install lcd panel IO");
    esp_lcd_panel_io_handle_t panel_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = disp_cfg->dc_gpio_num,
        .cs_gpio_num = disp_cfg->cs_gpio_num,
        .pclk_hz = disp_cfg->pclk_hz,
        .lcd_cmd_bits = disp_cfg->lcd_cmd_bits,
        .lcd_param_bits = disp_cfg->lcd_param_bits,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = disp_ctx,
    };

    /* Attach the LCD to the SPI bus */
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)disp_cfg->spi_host,
                                   &io_config, &panel_io_handle);
    ESP_GOTO_ON_ERROR(ret, err, tag, "failed to attach lcd to spi bus");
    disp_ctx->panel_io_handle = panel_io_handle;

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = disp_cfg->rst_gpio_num,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
    };

    ESP_LOGI(tag, "install ST7789 panel driver");
    ret = esp_lcd_new_panel_st7789(panel_io_handle, &panel_config,
                                   &disp_ctx->panel_handle);
    ESP_GOTO_ON_ERROR(ret, err, tag, "insatll lcd panel driver failed");

    esp_lcd_panel_reset(disp_ctx->panel_handle);
    esp_lcd_panel_init(disp_ctx->panel_handle);
    esp_lcd_panel_invert_color(disp_ctx->panel_handle, true);
    esp_lcd_panel_swap_xy(disp_ctx->panel_handle, false);
    esp_lcd_panel_mirror(disp_ctx->panel_handle, false, false);

    if (disp_cfg->te_gpio_num) {
        disp_cfg->tear_intr_type = GPIO_INTR_NEGEDGE;

        const gpio_config_t te_gpio_config = {
            .intr_type = disp_cfg->tear_intr_type,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(disp_cfg->te_gpio_num),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&te_gpio_config);
        gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    }

    /* consider to turn on lcd after lvgl first flash cb */
    esp_lcd_panel_disp_on_off(disp_ctx->panel_handle, true);

    esp_lcd_panel_io_i2c_config_t tp_io_config =
                                    ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    tp_io_config.scl_speed_hz = 400000;
    esp_lcd_touch_config_t tp_config = {
        .x_max = disp_cfg->hres,
        .y_max = disp_cfg->vres,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
#if CONFIG_LCD_DAXIAN_2_0
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
        },
#elif CONFIG_LCD_DAXIAN_2_4
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 0,
        },
#endif
    };
    ret = esp_lcd_new_panel_io_i2c_v2(disp_cfg->i2c->bus_handle,
                                      &tp_io_config, &disp_ctx->tp_io_handle);
    ESP_GOTO_ON_ERROR(ret, err, tag, "failed to init i2c panel io");
    

    ESP_LOGI(tag, "init touch controller ft6336\n");
    ret = esp_lcd_touch_new_i2c_ft5x06(disp_ctx->tp_io_handle, &tp_config,
                                       &disp_ctx->tp_handle);
    ESP_GOTO_ON_ERROR(ret, err, tag, "failed to init ft6336");

    ESP_LOGI(tag, "init lvgl library");
    lv_init();

    /* memory for lvgl render */
    disp_ctx->frame_buffer = heap_caps_malloc(disp_cfg->frame_buffer_size,
                                                 MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(disp_ctx->frame_buffer, ESP_ERR_NO_MEM, err, tag,
                      "not enough memory for frame buffer allocation");

    // /* memory for DMA trans */
    // disp_ctx->trans_buf = heap_caps_malloc(disp_cfg->trans_size,
    //                                           MALLOC_CAP_DMA);
    // ESP_GOTO_ON_FALSE(disp_ctx->trans_buf, ESP_ERR_NO_MEM, err, tag,
    //                   "not enough memory for buf1 allocation");

    disp_ctx->lv_disp = lv_display_create(disp_cfg->hres, disp_cfg->vres);
    ESP_GOTO_ON_FALSE(disp_ctx->lv_disp, ESP_ERR_NO_MEM, err, tag,
                      "not enough memory for lv_disp_t");

    lv_display_set_color_format(disp_ctx->lv_disp, disp_cfg->color_format);
    lv_display_set_buffers(disp_ctx->lv_disp, disp_ctx->frame_buffer, NULL,
                           disp_cfg->frame_buffer_size,
                           LV_DISP_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp_ctx->lv_disp, lvgl_flush_callback);
    lv_display_set_driver_data(disp_ctx->lv_disp, disp_ctx);

    ESP_LOGI(tag, "install lvgl tick timer");
    /* lvgl timer, add 2ms for lvgl tick each timer interrupt */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_increment,
        .name = "lvgl_tick",
        .arg = disp_ctx,
    };

    ret = esp_timer_create(&lvgl_tick_timer_args, &disp_ctx->lvgl_timer);
    ESP_GOTO_ON_ERROR(ret, err, tag, "create lvgl timer failed");
    ret = esp_timer_start_periodic(disp_ctx->lvgl_timer,
                                   disp_cfg->lvgl_cfg.timer_period_ms * 1000);
    ESP_GOTO_ON_ERROR(ret, err, tag, "start lvgl timer failed");

    ESP_LOGI(tag, "register lvgl input device");
    disp_ctx->lv_indev = lv_indev_create();
    ESP_GOTO_ON_FALSE(disp_ctx->lv_indev, ESP_ERR_NO_MEM, err, tag,
                      "not enough memory for lv_indev_t");

    lv_indev_set_type(disp_ctx->lv_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(disp_ctx->lv_indev, lvgl_touch_cb);
    lv_indev_set_disp(disp_ctx->lv_indev, disp_ctx->lv_disp);
    lv_indev_set_driver_data(disp_ctx->lv_indev, disp_ctx->tp_handle);

    disp_ctx->lvgl_mux = xSemaphoreCreateRecursiveMutex();
    ESP_GOTO_ON_FALSE(disp_ctx->lvgl_mux, ESP_ERR_NO_MEM, err, tag,
                      "create lvgl mutex fail");

    ESP_LOGI(tag, "create lvgl task");
    BaseType_t res = 0;
    res = xTaskCreate(lvgl_port_task, "lvgl", disp_cfg->lvgl_cfg.task_stack,
                      disp_ctx, disp_cfg->lvgl_cfg.task_priority, NULL);
    ESP_GOTO_ON_FALSE(res, ESP_FAIL, err, tag, "create lvgl task failed");
    ESP_LOGI(tag, "turn on lcd backlight");
    gpio_set_level(disp_cfg->bl_gpio_num, 0);

err:
    if (ret != ESP_OK) {

        if (disp_ctx->lvgl_mux)
            vSemaphoreDelete(disp_ctx->lvgl_mux);

        if (disp_ctx->te_catch_sem)
            vSemaphoreDelete(disp_ctx->te_catch_sem);

        if (disp_ctx->te_v_sync_sem)
            vSemaphoreDelete(disp_ctx->te_v_sync_sem);

        if (disp_ctx->trans_done_sem)
            vSemaphoreDelete(disp_ctx->trans_done_sem);

        if (disp_ctx->panel_handle)
            esp_lcd_panel_del(disp_ctx->panel_handle);

        if (disp_ctx->panel_io_handle)
            esp_lcd_panel_io_del(disp_ctx->panel_io_handle);

        if (disp_ctx->tp_io_handle)
            esp_lcd_panel_io_del(disp_ctx->tp_io_handle);

        if (disp_ctx->tp_handle)
            esp_lcd_touch_del(disp_ctx->tp_handle);

        if (disp_ctx->lv_disp)
            lv_display_delete(disp_ctx->lv_disp);

        if (disp_ctx->frame_buffer)
            free(disp_ctx->frame_buffer);

        if (disp_ctx->trans_buf)
            free(disp_ctx->trans_buf);

        if (disp_ctx->lvgl_timer) {
            esp_timer_stop(disp_ctx->lvgl_timer);
            esp_timer_delete(disp_ctx->lvgl_timer);
            disp_ctx->lvgl_timer = NULL;
        }

        if (disp_ctx->lv_indev)
            lv_indev_delete(disp_ctx->lv_indev);

        if (disp_ctx)
            free(disp_ctx);
    }

    return ret;
}
