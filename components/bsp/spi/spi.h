#ifndef __SPI_H_
#define __SPI_H_

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"


struct spi_master {
    spi_bus_config_t bus_config;
    spi_host_device_t spi_port;
    spi_common_dma_t dma_channel;
    bool is_init;
};

esp_err_t spi_init(struct spi_master *spi);

#endif
