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
#include "i2c_new.h"
#include "xl9555.h"
#include "lcd.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "spi.h"
#include "esp_littlefs.h"
#include "smartconfig.h"
#include "lv_demos.h"

static const char *tag = "main";

extern esp_err_t load_font_alipuhui(void);
extern void lvgl_test(void);

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


static void sdcard_init(void)
{
    esp_err_t ret = ESP_OK;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card = NULL;
    const char mount_point[] = "/sdcard";
    ESP_LOGI(tag, "init sd card using spi");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 20000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_SPISD_CS_PIN;
    slot_config.host_id = host.slot;

    ESP_LOGI(tag, "mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "filesystem mount failed");
        return;
    }
    ESP_LOGI(tag, "filesystem mounted");
    sdmmc_card_print_info(stdout, card);
}

static void littlefs_test(void)
{
    esp_err_t ret = ESP_OK;

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL)
            ESP_LOGE(tag, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE(tag, "Failed to find LittleFS partition");
        else
            ESP_LOGE(tag, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(tag, "init littlefs done");
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(conf.partition_label);
    } else {
        ESP_LOGI(tag, "Partition size: total: %d, used: %d", total, used);
    }
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t flash_size;
    esp_chip_info_t chip_info;
    struct i2c_obj i2c0 = {0};
    struct spi_master spi = {0};

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

    i2c0.sda_io_num = CONFIG_I2C0_SDA_PIN;
    i2c0.scl_io_num = CONFIG_I2C0_SCL_PIN;
    i2c0.port = I2C_NUM_0;

    spi.bus_config.miso_io_num = CONFIG_SPI_MISO_PIN;
    spi.bus_config.mosi_io_num = CONFIG_SPI_MOSI_PIN;
    spi.bus_config.sclk_io_num = CONFIG_SPI_SCK_PIN;
    spi.bus_config.quadhd_io_num = -1;
    spi.bus_config.quadwp_io_num = -1;
    spi.bus_config.max_transfer_sz = 320 * 240 * 2;
    spi.spi_port = SPI2_HOST;
    spi.dma_channel = SPI_DMA_CH_AUTO;

    i2c_init(&i2c0);
    spi_init(&spi);
    sdcard_init();

    littlefs_test();
    // load_font_alipuhui();
    init_wifi();

    /* 显示参数配置 */
    struct display_config disp_cfg = {
        .hres = 240,
        .vres = 320,
        .pclk_hz = 80 * 1000 * 1000,
        .frame_buffer_size = 240 * 320 * LV_COLOR_DEPTH / 8,
        .trans_size = 240 * 320 / 8 * LV_COLOR_DEPTH / 8,
        .sw_rotate = LV_DISPLAY_ROTATION_0,
        .lvgl_cfg.task_priority = 4,
        .lvgl_cfg.task_stack = 8192,
        .lvgl_cfg.task_min_delay_ms = 2,
        .lvgl_cfg.task_max_delay_ms = 500,
        .lvgl_cfg.timer_period_ms = 2,
        .cs_gpio_num = CONFIG_SPI_LCD_CS_PIN,
        .dc_gpio_num = CONFIG_SPI_LCD_DC_PIN,
        .rst_gpio_num = CONFIG_SPI_LCD_RST_PIN,
        .te_gpio_num = CONFIG_SPI_LCD_TE_PIN,
        .bl_gpio_num = CONFIG_SPI_LCD_BL_PIN,
        .tear_intr_type = GPIO_INTR_NEGEDGE,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_host = SPI2_HOST,
        .i2c = &i2c0,
        .color_format = LV_COLOR_FORMAT_RGB565,
    };
    ESP_LOGI(tag, "lv_color_t size: %d", sizeof(lv_color_t));
    lcd_init(&disp_cfg);

    // Lock the mutex due to the LVGL APIs are not thread-safe
    lvgl_lock(-1);
    lvgl_test();
    lvgl_unlock();

    xTaskCreatePinnedToCore(&monitoring_task, "monitor_task", 2048, NULL, 1, NULL, 1);
}
