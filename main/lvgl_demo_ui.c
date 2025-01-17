/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_smartconfig.h"

LV_FONT_DECLARE(alipuhui);

static const char *tag = "ui";

static lv_obj_t *label_wifi = NULL;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(tag, "got ip");
        lv_label_set_text(label_wifi, "WiFi连接成功");
    }

    if (event_base ==  SC_EVENT) {
        switch (event_id) {
        case SC_EVENT_SCAN_DONE:
            ESP_LOGI(tag, "Scan done");
            break;
        case SC_EVENT_FOUND_CHANNEL:
            ESP_LOGI(tag, "Found channel");
            break;
        case SC_EVENT_GOT_SSID_PSWD:
            ESP_LOGI(tag, "got SSID and password");
            break;
        default:
            break;
        }
    }
}

void example_lvgl_demo_ui(void)
{
    // 注册IP_EVENT和SC_EVENT的事件处理
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        event_handler, NULL, NULL);
    esp_event_handler_instance_register(SC_EVENT, ESP_EVENT_ANY_ID,
                                        event_handler, NULL, NULL);

    label_wifi = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(label_wifi, &alipuhui, 0);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x888888), LV_PART_MAIN);
    lv_label_set_text(label_wifi, "正在连接WiFi...");
    lv_obj_set_pos(label_wifi, 85 ,110);
}
