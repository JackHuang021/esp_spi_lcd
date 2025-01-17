#include "spi.h"
#include "esp_log.h"

static const char *tag = "spi";

esp_err_t spi_init(struct spi_master *spi)
{
    esp_err_t ret = ESP_OK;

    ret = spi_bus_initialize(spi->spi_port, &spi->bus_config, spi->dma_channel);
    if (ret == ESP_OK) {
        ESP_LOGI(tag, "spi%d init done", spi->spi_port);
        spi->is_init = true;
    }
    else
        ESP_LOGE(tag, "spi%d init failed, return %s", spi->spi_port,
                 esp_err_to_name(ret));

    return ret;
}
