#ifndef AS5047P_H
#define AS5047P_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define AS5047P_ACCESS_WRITE 		false
#define AS5047P_ACCESS_READ 		true

#define AS5047P_FRAME_PARD		( 1 << 15)
#define AS5047P_FRAME_EF 		( 1 << 14)
#define AS5047P_FRAME_DATA		0x3FFF

#define AS5047P_ABIRES_100 	100
#define AS5047P_ABIRES_200 	200
#define AS5047P_ABIRES_400 	400
#define AS5047P_ABIRES_800 	800
#define AS5047P_ABIRES_1200 	1200
#define AS5047P_ABIRES_1600 	1600
#define AS5047P_ABIRES_2000 	2000
#define AS5047P_ABIRES_4000 	4000
#define AS5047P_ABIRES_1024 	1024
#define AS5047P_ABIRES_2048 	2048
#define AS5047P_ABIRES_4096 	4096

// --- Volatile registers
#define AS5047P_NOP          	0x0000
#define AS5047P_ERRFL        	0x0001
#define AS5047P_PROG        	0x0003
#define AS5047P_DIAAGC       	0x3FFC
#define AS5047P_MAG          	0x3FFD
#define AS5047P_ANGLEUNC     	0x3FFE
#define AS5047P_ANGLECOM     	0x3FFF

// --- Non-volatile registers
#define AS5047P_ZPOSM        	0x0016
#define AS5047P_ZPOSL        	0x0017
#define AS5047P_SETTINGS1    	0x0018
#define AS5047P_SETTINGS2    	0x0019

// --- Fields in registers
#define AS5047P_ERRFL_PARERR		( 1 << 2)
#define AS5047P_ERRFL_INVCOMM		( 1 << 1)
#define AS5047P_ERRFL_FRERR		( 1 << 0)
#define AS5047P_PROG_PROGVER		( 1 << 6)
#define AS5047P_PROG_PROGOTP		( 1 << 3)
#define AS5047P_PROG_OTPREF		( 1 << 2)
#define AS5047P_PROG_PROGEN		( 1 << 0)
#define AS5047P_DIAAGC_MAGL		( 1 << 11)
#define AS5047P_DIAAGC_MAGH		( 1 << 10)
#define AS5047P_DIAAGC_COF		( 1 << 9)
#define AS5047P_DIAAGC_LF		( 1 << 8)
#define AS5047P_DIAAGC_AGC		( 0x00FF << 0)
#define AS5047P_MAG_CMAG		( 0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG	( 0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG	( 0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM		( 0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN	( 1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN	( 1 << 6)
#define AS5047P_ZPOSL_ZPOSL		( 0x003F << 0)
#define AS5047P_SETTINGS1_BIT0		( 1 << 0)
#define AS5047P_SETTINGS1_NOISESET	( 1 << 1)
#define AS5047P_SETTINGS1_DIR		( 1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI	( 1 << 3)
#define AS5047P_SETTINGS1_DAECDIS	( 1 << 4)
#define AS5047P_SETTINGS1_ABIBIN	( 1 << 5)
#define AS5047P_SETTINGS1_DATASEL	( 1 << 6)
#define AS5047P_SETTINGS1_PWMON		( 1 << 7)
#define AS5047P_SETTINGS2_UVWPP		( 0x0007 << 0)
#define AS5047P_SETTINGS2_HYS		( 0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES	( 0x0007 << 5)


typedef struct as5047p_t_ {
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    gpio_num_t AS_MISO_SDO_pin;
    gpio_num_t AS_MOSI_SDI_pin;
    gpio_num_t AS_SCLK_pin;
    gpio_num_t AS_SCS_pin;
    gpio_num_t AS_HANDSHAKE;
    spi_host_device_t spi_host;
    uint32_t max_spi_clockspeed;
} as5047p_t;

typedef struct as5047p_diaagc_reg_t_
{
    uint16_t AGC            :8 ; // Bits 0-7
    uint16_t LF             :1 ; // Bits 8
    uint16_t COF            :1 ; // Bits 9
    uint16_t MAGH           :1 ; // Bits 10
    uint16_t MAGL           :1 ; // Bits 11
    
} __attribute__((packed, aligned(2))) as5047p_diaagc_reg_t;

typedef struct as5047p_errfl_reg_t_
{
    uint16_t PARERR          :1 ; // Bits 0
    uint16_t INVCOMM         :1 ; // Bits 1
    uint16_t FRERR           :1 ; // Bits 2
    uint16_t RSVD            :13 ;
    
} __attribute__((packed, aligned(2))) as5047p_errfl_reg_t;

typedef struct as5047p_angleunc_reg_t_
{
    uint16_t ANGLEUNC           :16 ; // Bits 0-13
    
} __attribute__((packed, aligned(2))) as5047p_angleunc_reg_t;


typedef struct as5047p_anglecom_reg_t_
{
    uint16_t ANGLECOM           :16 ; // Bits 0-13
    
} __attribute__((packed, aligned(2))) as5047p_anglecom_reg_t;

typedef struct as5047p_mag_reg_t_
{
    uint16_t MAG           :16 ; // Bits 0-13
    
} __attribute__((packed, aligned(2))) as5047p_mag_reg_t;



esp_err_t as5047p_init(as5047p_t *dev);
esp_err_t as5047p_read_write(as5047p_t *dev, uint16_t frameTx, bool rw);
esp_err_t as5047p_read_register(as5047p_t *dev, uint16_t regAddr, uint16_t *value);
esp_err_t as5047p_write_register(as5047p_t *dev, uint16_t regAddr, uint16_t newRegContent);
// esp_err_t as5047p_set_factory_settings(as5047p_t *dev);
// esp_err_t as5047p_set_zero_position(as5047p_t *dev);
esp_err_t as5047p_read_diaagc_reg(as5047p_t *dev, as5047p_diaagc_reg_t *value);
esp_err_t as5047p_read_angleunc_reg(as5047p_t *dev, as5047p_angleunc_reg_t *value);
esp_err_t as5047p_read_anglecom_reg(as5047p_t *dev, as5047p_anglecom_reg_t *value);
esp_err_t as5047p_read_mag_reg(as5047p_t *dev, as5047p_mag_reg_t *value);
esp_err_t as5047p_read_errfl_reg(as5047p_t *dev, as5047p_errfl_reg_t *value);
//esp_err_t as5047p_read_position(as5047p_t *dev, _Bool extendedDiag);


#endif
