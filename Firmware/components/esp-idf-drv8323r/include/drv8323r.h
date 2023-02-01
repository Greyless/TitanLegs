#ifndef DRV8323R_H
#define DRV8323R_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define DRV8323R_STATUS_00_REG_ADDR 0x00
#define DRV8323R_STATUS_01_REG_ADDR 0x01
#define DRV8323R_CONTROL_02_REG_ADDR 0x02
#define DRV8323R_CONTROL_03_REG_ADDR 0x03
#define DRV8323R_CONTROL_04_REG_ADDR 0x04
#define DRV8323R_CONTROL_05_REG_ADDR 0x05
#define DRV8323R_CONTROL_06_REG_ADDR 0x06
#define DRV8323R_CONTROL_07_REG_ADDR 0x07


// struct which holds data for SPI driver
typedef struct drv8323r_t_ {
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    gpio_num_t DRV_EN_GATE_pin;
    gpio_num_t DRV_N_FAULT_pin;
    gpio_num_t DRV_MISO_SDO_pin;
    gpio_num_t DRV_MOSI_SDI_pin;
    gpio_num_t DRV_SCLK_pin;
    gpio_num_t DRV_SCS_pin;
    spi_host_device_t spi_host;
    uint32_t max_spi_clockspeed;
} drv8323r_t;

// Structs defining the registers on drv8323r
typedef struct drv8323r_status_00_reg_t_
{
    uint8_t VDS_LC         :1 ; // Bits 0
    uint8_t VDS_HC         :1 ; // Bits 1
    uint8_t VDS_LB         :1 ; // Bits 2
    uint8_t VDS_HB         :1 ; // Bits 3
    uint8_t VDS_HA         :1 ; // Bits 4
    uint8_t VDS_LA         :1 ; // Bits 5
    uint8_t OTSD           :1 ; // Bits 6
    uint8_t UVLO           :1 ; // Bits 7
    uint8_t GDF            :1 ; // Bits 8
    uint8_t VDS_OCP        :1 ; // Bits 9
    uint8_t FAULT          :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_status_00_reg_t;

typedef struct drv8323r_status_01_reg_t_
{
    uint8_t VGS_LC         :1 ; // Bits 0
    uint8_t VGS_HC         :1 ; // Bits 1
    uint8_t VGS_LB         :1 ; // Bits 2
    uint8_t VGS_HB         :1 ; // Bits 3
    uint8_t VGS_LA         :1 ; // Bits 4
    uint8_t VGS_HA         :1 ; // Bits 5
    uint8_t CPUV           :1 ; // Bits 6
    uint8_t OTW            :1 ; // Bits 7
    uint8_t SC_OC          :1 ; // Bits 8
    uint8_t SB_OC          :1 ; // Bits 9
    uint8_t SA_OC          :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_status_01_reg_t;

typedef struct drv8323r_control_02_reg_t_
{
    uint8_t CLR_FLT        :1 ; // Bits 0
    uint8_t BRAKE          :1 ; // Bits 1
    uint8_t COAST          :1 ; // Bits 2
    uint8_t PWM_DIR        :1 ; // Bits 3
    uint8_t PWM_COM        :1 ; // Bits 4
    uint8_t PWM_MODE       :2 ; // Bits 5:6
    uint8_t OTW_REP        :1 ; // Bits 7
    uint8_t DIS_GDF        :1 ; // Bits 8
    uint8_t DIS_CPUV       :1 ; // Bits 9
    uint8_t STAT02_RSV1    :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_control_02_reg_t;

typedef struct drv8323r_control_03_reg_t_
{
    uint8_t IDRIVEN_HS     :4 ; // Bits 0:3
    uint8_t IDRIVEP_HS     :4 ; // Bits 4:7
    uint8_t LOCK           :3 ; // Bits 8:10
} __attribute__((packed, aligned(2))) drv8323r_control_03_reg_t;

typedef struct drv8323r_control_04_reg_t_
{
    uint8_t IDRIVEP_LS     :4 ; // Bits 0:3
    uint8_t IDRIVEN_LS     :4 ; // Bits 3:7
    uint8_t TDRIVE         :2 ; // Bits 8:9
    uint8_t CBC            :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_control_04_reg_t;

typedef struct drv8323r_control_05_reg_t_
{
    uint8_t VDS_LVL        :4 ; // Bits 0:3
    uint8_t OCP_DEG        :2 ; // Bits 4:5
    uint8_t OCP_MODE       :2 ; // Bits 6:7
    uint8_t DEAD_TIME      :2 ; // Bits 8:9
    uint8_t TRETRY         :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_control_05_reg_t;

typedef struct drv8323r_control_06_reg_t_
{
    uint8_t SEN_LVL        :2 ; // Bits 0:1
    uint8_t CSA_CAL_C      :1 ; // Bits 2
    uint8_t CSA_CAL_B      :1 ; // Bits 3
    uint8_t CSA_CAL_A      :1 ; // Bits 4
    uint8_t DIS_SEN        :1 ; // Bits 5
    uint8_t CSA_GAIN       :2 ; // Bits 6:7
    uint8_t LS_REF         :1 ; // Bits 8
    uint8_t VREF_DIV       :1 ; // Bits 9
    uint8_t CSA_FET        :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8323r_control_06_reg_t;

typedef struct drv8323r_reserved_07_reg_t_
{
    uint16_t STAT07_RSV    :11 ; // Bits 0:10
} __attribute__((packed, aligned(2))) drv8323r_reserved_07_reg_t;


esp_err_t drv8323r_init(drv8323r_t *dev);
esp_err_t drv8323r_write_register(drv8323r_t *dev, uint8_t register_address, uint16_t value);
esp_err_t drv8323r_read_register(drv8323r_t *dev, uint8_t register_address, uint16_t *value);

esp_err_t drv8323r_read_status_00_register(drv8323r_t *dev, drv8323r_status_00_reg_t *value);
esp_err_t drv8323r_read_status_01_register(drv8323r_t *dev, drv8323r_status_01_reg_t *value);

esp_err_t drv8323r_read_control_02_register(drv8323r_t *dev, drv8323r_control_02_reg_t *value);
esp_err_t drv8323r_read_control_03_register(drv8323r_t *dev, drv8323r_control_03_reg_t *value);
esp_err_t drv8323r_read_control_04_register(drv8323r_t *dev, drv8323r_control_04_reg_t *value);
esp_err_t drv8323r_read_control_05_register(drv8323r_t *dev, drv8323r_control_05_reg_t *value);
esp_err_t drv8323r_read_control_06_register(drv8323r_t *dev, drv8323r_control_06_reg_t *value);

esp_err_t drv8323r_write_control_02_register(drv8323r_t *dev, drv8323r_control_02_reg_t value);
esp_err_t drv8323r_write_control_03_register(drv8323r_t *dev, drv8323r_control_03_reg_t value);
esp_err_t drv8323r_write_control_04_register(drv8323r_t *dev, drv8323r_control_04_reg_t value);
esp_err_t drv8323r_write_control_05_register(drv8323r_t *dev, drv8323r_control_05_reg_t value);
esp_err_t drv8323r_write_control_06_register(drv8323r_t *dev, drv8323r_control_06_reg_t value);
#endif