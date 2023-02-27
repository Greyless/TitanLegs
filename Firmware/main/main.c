#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "as5047p.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MISO_PIN 19
#define MOSI_PIN 23
#define SCLK_PIN 18
#define CS_PIN 15
#define HANDSHAKE 2

static const char TAG[] = "Encoder Angle";

// void init_gpio()
// {
//     as5047p_t dev = {
//                    .AS_MISO_SDO_pin = MISO_PIN,
//                    .AS_MOSI_SDI_pin = MOSI_PIN,
//                    .AS_SCLK_pin = SCLK_PIN,
//                    .AS_SCS_pin = CS_PIN,
//                    .spi_host = SPI2_HOST,
//                    .max_spi_clockspeed = 1000000};

//     as5047p_init(&dev);
//     as5047p_set_factory_settings(&dev);
//     as5047p_set_zero_position(&dev);
// }

void app_main()
{
    // init_gpio();

    as5047p_t dev = {
                   .AS_MISO_SDO_pin = MISO_PIN,
                   .AS_MOSI_SDI_pin = MOSI_PIN,
                   .AS_SCLK_pin = SCLK_PIN,
                   .AS_SCS_pin = CS_PIN,
                   .spi_host = SPI3_HOST,
                   .max_spi_clockspeed = 1000000};

    as5047p_init(&dev);
    //as5047p_set_factory_settings(&dev);
    // as5047p_set_zero_position(&dev);

    as5047p_diaagc_reg_t diaagc_reg;
    as5047p_anglecom_reg_t anglecom_reg;
    as5047p_angleunc_reg_t angleunc_reg;
    as5047p_mag_reg_t mag_reg;
    as5047p_errfl_reg_t errfl_reg;
    

    while(1)
    {
        // int16_t currPosition = as5047p_read_register(&dev, AS5047P_ANGLECOM);
        // ESP_LOGI(TAG, "Position %x", currPosition);
        as5047p_read_diaagc_reg(&dev, &diaagc_reg);
        //as5047p_read_anglecom_reg(&dev, &anglecom_reg);
        as5047p_read_errfl_reg(&dev, &errfl_reg);
        // as5047p_read_mag_reg(&dev, &mag_reg);
        // as5047p_read_angleunc_reg(&dev, &angleunc_reg);
        ESP_LOGI(TAG, "MAGL = %x", diaagc_reg.MAGL);
        ESP_LOGI(TAG, "MAGH = %x", diaagc_reg.MAGH);
        ESP_LOGI(TAG, "PARERR = %x", errfl_reg.PARERR);
        ESP_LOGI(TAG, "INVCOMM = %x", errfl_reg.INVCOMM);
        ESP_LOGI(TAG, "FRERR = %x", errfl_reg.FRERR);
        ESP_LOGI(TAG, "RSVDs = %x", errfl_reg.RSVD);
        // ESP_LOGI(TAG, "COF = %x", diaagc_reg.COF);
        // ESP_LOGI(TAG, "LF = %x", diaagc_reg.LF);        
        // ESP_LOGI(TAG, "AGC = %x", diaagc_reg.AGC);
        // ESP_LOGI(TAG, "ANGLEUNC = %x", angleunc_reg.ANGLEUNC);
        //ESP_LOGI(TAG, "ANGLE = %x", anglecom_reg.ANGLECOM);

        // ESP_LOGI(TAG, "MAG = %x", mag_reg.MAG);
        
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}