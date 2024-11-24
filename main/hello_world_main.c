/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "console_wifi.h"
#include "esp_psram.h"
#include "esp_chip_info.h"
#include "lvgl.h"
#include "iic.h"
#include "xl9555.h"
#include "lcd.h"

static const char *tag = "main";

/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
static void monitoring_task(void *pvParameter)
{
    for(;;){
        ESP_LOGI(tag, "free heap: %ld", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t flash_size;
    esp_chip_info_t chip_info;
    struct i2c_obj i2c = {0};

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_flash_get_size(NULL, &flash_size);
    esp_chip_info(&chip_info);
    ESP_LOGI(tag, "cpu num: %d", chip_info.cores);
    ESP_LOGI(tag, "flash size: %ld MB", flash_size >> 20);
    ESP_LOGI(tag, "psram size: %d bytes", esp_psram_get_size());

    i2c.sda = IIC0_SDA_GPIO_PIN;
    i2c.scl = IIC0_SCL_GPIO_PIN;
    i2c.port = I2C_NUM_0;
    i2c_init(&i2c);
    xl9555_init(&i2c);

    xTaskCreatePinnedToCore(&monitoring_task, "monitor_task", 2048, NULL, 1, NULL, 1);

    // while (1) {
    //     vTaskDelay(100);
    // }
}
