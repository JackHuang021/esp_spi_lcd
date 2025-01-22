/**
 * @file lvgl_test.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lcd.h"

static const char tag[] = "test";

static uint32_t colors[] = {
    0xFF0000,
    0x00FF00,
    0x0000FF,
};

static char *strs[] = {
    "千姿百态",
    "千虑一得",
    "千载难逢",
};
static lv_obj_t *label = NULL;

static void lv_example_tiny_ttf_2(void)
{
    /*Create style with the new font*/
    static lv_style_t style;
    lv_style_init(&style);
    lv_font_t * font = lv_tiny_ttf_create_file("A:/sdcard/alibaba.ttf", 16);
    lv_style_set_text_font(&style, font);
    lv_style_set_text_align(&style, LV_TEXT_ALIGN_CENTER);

    /*Create a label with the new style*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_obj_add_style(label, &style, 0);
    lv_label_set_text(label, "你好\n这里是TinyTTF测试");
    lv_obj_center(label);
}

static void lv_example_freetype_1(void)
{
    /*Create a font*/
    lv_font_t * font = lv_freetype_font_create("/sdcard/alibaba.ttf",
                                               LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                               16,
                                               LV_FREETYPE_FONT_STYLE_NORMAL);

    if(!font) {
        ESP_LOGE(tag, "freetype font create failed.");
        return;
    }

    /*Create style with the new font*/
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_font(&style, font);
    lv_style_set_text_align(&style, LV_TEXT_ALIGN_CENTER);

    /*Create a label with the new style*/
    label = lv_label_create(lv_screen_active());
    lv_obj_add_style(label, &style, 0);
    lv_label_set_text(label, "你好\n这里是Freetype字体测试");
    lv_obj_center(label);
}

static void lvgl_test_task(void *arg)
{
    static int color_index;
    int64_t t1 = 0;
    int64_t t2 = 0;

    // lvgl_lock(-1);
    // t1 = esp_timer_get_time();
    // lv_example_freetype_1();
    // t2 = esp_timer_get_time();
    // ESP_LOGI(tag, "t: %lld uS",t2 - t1);
    // lvgl_unlock();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        lvgl_lock(-1);
        lv_obj_t *active_screen = lv_scr_act();
        lv_obj_set_style_bg_color(active_screen, lv_color_hex(colors[color_index]), 0);
        // lv_label_set_text(label, strs[color_index]);
        lvgl_unlock();
        color_index = (color_index + 1) % 3;
    }
}

static void lvgl_fs_test(void)
{
    lv_fs_file_t fd;
    lv_fs_res_t res;
    char *buf = NULL;
    uint32_t size = 0;
    uint32_t rsize = 0;

    res = lv_fs_open(&fd, "A:/sdcard/test.txt", LV_FS_MODE_RD);
    if (res != LV_FS_RES_OK) {
        ESP_LOGE(tag, "lvgl fs open failed");
        return;
    }

    lv_fs_seek(&fd, 0, LV_FS_SEEK_END);
    lv_fs_tell(&fd, &size);
    ESP_LOGI(tag, "lvgl fs file size %lu bytes", size);
    lv_fs_seek(&fd, 0,LV_FS_SEEK_SET);

    buf = malloc(size + 1);
    res = lv_fs_read(&fd, buf, size, &rsize);
    if (res != LV_FS_RES_OK) {
        ESP_LOGE(tag, "lvgl fs read failed");
        return;
    }
    buf[size] = '\0';
    ESP_LOGI(tag, "read contents: %s", buf);

    lv_fs_close(&fd);

    /*Create a label with the new style*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "lvgl fs test OK");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}

void lvgl_test(void)
{
    lv_obj_t *active_screen = lv_scr_act();

    lvgl_lock(-1);
    lv_obj_set_size(active_screen, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(active_screen, lv_palette_main(LV_PALETTE_GREEN), 0);
    lvgl_fs_test();
    lvgl_unlock();
    xTaskCreate(&lvgl_test_task, "lvgl_test_task", 8192, NULL, 1, NULL);
}

