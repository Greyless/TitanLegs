#include "drv8323r.h"

static const char* drv_TAG = "drv8323r";

esp_err_t drv8323r_init(drv8323r_t *dev)
{
    CHECK_ARG(dev);

    spi_bus_config_t cfg = {
        .mosi_io_num = dev->DRV_MOSI_SDI_pin,
        .miso_io_num = dev->DRV_MISO_SDO_pin,
        .sclk_io_num = dev->DRV_SCLK_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0
    };

    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    gpio_set_direction(dev->DRV_N_FAULT_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->DRV_EN_GATE_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->DRV_SCS_pin, GPIO_MODE_OUTPUT);

    gpio_set_level(dev->DRV_EN_GATE_pin, 1);
    gpio_set_level(dev->DRV_SCS_pin, 1);

    ESP_LOGD(drv_TAG, "enGate Pin enabled");
    ESP_LOGD(drv_TAG, "chip select Pin enabled");
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_err_t err = spi_bus_initialize(dev->spi_host, &cfg, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(drv_TAG, "SPI init failed : %d", err);
        return ESP_FAIL;
    }

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = dev->DRV_SCS_pin;
    dev->spi_cfg.clock_speed_hz = dev->max_spi_clockspeed;
    dev->spi_cfg.mode = 1;
    dev->spi_cfg.queue_size = 1;

    err = spi_bus_add_device(dev->spi_host, &dev->spi_cfg, &dev->spi_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(drv_TAG, "Unable to add DRV8323R to SPI bus : %d", err);
        return ESP_FAIL;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(drv_TAG, "DRV8323R initialised successfully");
    
    return ESP_OK;
}

esp_err_t drv8323r_write_register(drv8323r_t *dev, uint8_t register_address, uint16_t value)
{
    CHECK_ARG(dev);

    spi_transaction_t packet;
    memset(&packet, 0, sizeof(spi_transaction_t));

    uint16_t out = 0;
    out = out | (register_address & 0x0F) << 11;
    out = out | (value & 0x07FF);

    uint8_t tx[] = { (out >> 8 & 0xFF), (out & 0xFF) };
    uint8_t rx[sizeof(tx)];

    packet.tx_buffer = tx;
    packet.rx_buffer = rx;
    packet.length = sizeof(tx) * 8;

    return spi_device_transmit(dev->spi_dev, &packet);
}

esp_err_t drv8323r_read_register(drv8323r_t *dev, uint8_t register_address, uint16_t *value)
{
    CHECK_ARG(dev && value);

    spi_transaction_t packet;
    memset(&packet, 0, sizeof(spi_transaction_t));

    uint16_t out = 0;
    out = out | (1 << 15);
    out = out | (register_address & 0x0F) << 11;
    out = out | 0x807F;
    
    uint8_t tx[] = { (out >> 8 & 0xFF), (out & 0xFF) };
    uint8_t rx[sizeof(tx)];

    packet.tx_buffer = tx;
    packet.rx_buffer = rx;
    packet.length = sizeof(tx) * 8;

    esp_err_t err = spi_device_transmit(dev->spi_dev, &packet);
    *value = (rx[0] << 8 | rx[1]);
    
    return err;
}

esp_err_t drv8323r_read_status_00_register(drv8323r_t *dev, drv8323r_status_00_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_STATUS_00_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_status_01_register(drv8323r_t *dev, drv8323r_status_01_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_STATUS_01_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_02_register(drv8323r_t *dev, drv8323r_control_02_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_02_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_03_register(drv8323r_t *dev, drv8323r_control_03_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_03_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_04_register(drv8323r_t *dev, drv8323r_control_04_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_04_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_05_register(drv8323r_t *dev, drv8323r_control_05_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_05_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_06_register(drv8323r_t *dev, drv8323r_control_06_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_06_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_read_control_07_register(drv8323r_t *dev, drv8323r_control_07_reg_t *value)
{
    return drv8323r_read_register(dev, DRV8323R_CONTROL_07_REG_ADDR, (uint16_t*)value);
}

esp_err_t drv8323r_write_control_02_register(drv8323r_t *dev, drv8323r_control_02_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_02_REG_ADDR, *((uint16_t*)&value));
}

esp_err_t drv8323r_write_control_03_register(drv8323r_t *dev, drv8323r_control_03_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_03_REG_ADDR, *((uint16_t*)&value));
}

esp_err_t drv8323r_write_control_04_register(drv8323r_t *dev, drv8323r_control_04_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_04_REG_ADDR, *((uint16_t*)&value));
}

esp_err_t drv8323r_write_control_05_register(drv8323r_t *dev, drv8323r_control_05_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_05_REG_ADDR, *((uint16_t*)&value));
}

esp_err_t drv8323r_write_control_06_register(drv8323r_t *dev, drv8323r_control_06_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_06_REG_ADDR, *((uint16_t*)&value));
}

esp_err_t drv8323r_write_control_07_register(drv8323r_t *dev, drv8323r_control_07_reg_t value)
{
    return drv8323r_write_register(dev, DRV8323R_CONTROL_07_REG_ADDR, *((uint16_t*)&value));
}
