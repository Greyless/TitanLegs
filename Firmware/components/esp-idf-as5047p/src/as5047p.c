#include "as5047p.h"


static const char* as_TAG = "as5047p";

esp_err_t as5047p_init(as5047p_t *dev)
{
    CHECK_ARG(dev);

    spi_bus_config_t cfg = {
        .mosi_io_num = dev->AS_MOSI_SDI_pin,
        .miso_io_num = dev->AS_MISO_SDO_pin,
        .sclk_io_num = dev->AS_SCLK_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10,
        .flags = 0
    };

    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    gpio_set_direction(dev->AS_SCS_pin, GPIO_MODE_OUTPUT);

    gpio_set_level(dev->AS_SCS_pin, 0);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_err_t err = spi_bus_initialize(dev->spi_host, &cfg, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(as_TAG, "SPI init failed : %d", err);
        return ESP_FAIL;
    }

    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = dev->AS_SCS_pin;
    dev->spi_cfg.clock_speed_hz = dev->max_spi_clockspeed;
    dev->spi_cfg.mode = 1;
    dev->spi_cfg.queue_size = 1;
    dev->spi_cfg.duty_cycle_pos = 128;

   
   err = spi_bus_add_device(dev->spi_host, &dev->spi_cfg, &dev->spi_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(as_TAG, "Unable to add DRV8305 to SPI bus : %d", err);
        return ESP_FAIL;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(as_TAG, "SPI init failed : %d", err);
        return ESP_FAIL;
    }


    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(as_TAG, "AS5047P initialised successfully");
    
    return ESP_OK;
}

static uint8_t as5047p_calc_parity(uint32_t v)
{
  v ^= v >> 1;
  v ^= v >> 2;
  v = (v & 0x11111111U) * 0x11111111U;
  return (v >> 28) & 1;
}




esp_err_t as5047p_write_register(as5047p_t *dev, uint16_t regAddr, uint16_t newRegContent)
{
    int16_t tmpResponse;
    int16_t curRegContent;
    if(!( (regAddr == AS5047P_PROG ) || (regAddr == AS5047P_ZPOSL )  || (regAddr == AS5047P_ZPOSM ) || (regAddr == AS5047P_SETTINGS1 ) || (regAddr == AS5047P_SETTINGS2 )) )
    {
	return ESP_FAIL;
    }

    as5047p_read_write(dev, regAddr, 0);
    tmpResponse = as5047p_read_write(dev, newRegContent, 0);
    if( tmpResponse < 0 )
    {
        return ESP_FAIL;
    }
    
    curRegContent = as5047p_read_write(dev, AS5047P_DIAAGC , 1);

    if( curRegContent < 0 )
    { 
        return ESP_FAIL;
    }


    return ESP_OK;
}


esp_err_t as5047p_read_register(as5047p_t *dev, uint16_t regAddr, uint16_t *value)
{
    spi_transaction_t packet;
    memset(&packet, 0, sizeof(spi_transaction_t));
    
    uint16_t out;
    out = regAddr & 0x3FFF;
    out |= 1 << 14;
    out |= as5047p_calc_parity(out) << 15;

    

    uint8_t tx[] = { (out >> 8 & 0xFF), ((out) & 0xFF) };
    uint8_t rx[sizeof(tx)];

    packet.tx_buffer = tx;
    packet.rx_buffer = rx;
    packet.length = 2 * 8;

    esp_err_t err = spi_device_transmit(dev->spi_dev, &packet);

    *value = (rx[0] << 8 | rx[1]);

    ESP_LOGI(as_TAG, "OUT = %x", *value);
    
    return err;
}

esp_err_t as5047p_read_diaagc_reg(as5047p_t *dev, as5047p_diaagc_reg_t *value)
{
    return as5047p_read_register(dev, AS5047P_DIAAGC, (uint16_t*)value);
}

esp_err_t as5047p_read_angleunc_reg(as5047p_t *dev, as5047p_angleunc_reg_t *value)
{
    return as5047p_read_register(dev, AS5047P_ANGLEUNC, (uint16_t*)value);
}

esp_err_t as5047p_read_anglecom_reg(as5047p_t *dev, as5047p_anglecom_reg_t *value)
{
    return as5047p_read_register(dev, AS5047P_ANGLECOM, (uint16_t*)value);
}

esp_err_t as5047p_read_mag_reg(as5047p_t *dev, as5047p_mag_reg_t *value)
{
    return as5047p_read_register(dev, AS5047P_MAG, (uint16_t*)value);
}

esp_err_t as5047p_read_errfl_reg(as5047p_t *dev, as5047p_errfl_reg_t *value)
{
    return as5047p_read_register(dev, AS5047P_ERRFL, (uint16_t*)value);
}
