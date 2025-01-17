/**
 * @file iic.c
 * @author Jack Huang
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "i2c_new.h"
#include "esp_log.h"

static const char *tag = "i2c";

/**
 * @brief init i2c bus
 * 
 * @param i2c 
 * @return esp_err_t 
 */
esp_err_t i2c_init(struct i2c_obj *i2c)
{
    esp_err_t ret = ESP_OK;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c->port,
        .scl_io_num = i2c->scl_io_num,
        .sda_io_num = i2c->sda_io_num,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&i2c_bus_config, &i2c->bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(tag, "i2c%d init done", i2c_bus_config.i2c_port);
        i2c->is_init = true;
    }
    else
        ESP_LOGE(tag, "i2c%d init failed, return %s", i2c_bus_config.i2c_port,
                 esp_err_to_name(ret));

    return ret;
}

