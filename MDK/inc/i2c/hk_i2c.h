
#ifndef __HK_I2C_H__
#define __HK_I2C_H__

#include "gd32f4xx.h"
#include "systick.h"
#include "gd32f4xx_i2c.h"
#include <stdio.h>
#include <string.h>

// #define I2C_TIME_OUT (10000UL)
#define I2C_TIME_OUT (1000UL)

#define I2C_BAUDRATE (100000ul)
//#define I2C_BAUDRATE (400000ul) //400K
#define DEFAULT_IIC_ADDR (0x20)

#define IPMB1_SCL_PORT (GPIOB)
#define IPMB1_SCL_PIN (GPIO_PIN_6)
#define IPMB1_SDA_PORT (GPIOB)
#define IPMB1_SDA_PIN (GPIO_PIN_7)

#define IPMB2_SCL_PORT (GPIOB)
#define IPMB2_SCL_PIN (GPIO_PIN_10)
#define IPMB2_SDA_PORT (GPIOB)
#define IPMB2_SDA_PIN (GPIO_PIN_3)


#define BMC_IIC2_SCL_PORT (GPIOA)
#define BMC_IIC2_SCL_PIN (GPIO_PIN_8)

#define BMC_IIC2_SDA_PORT (GPIOC)
#define BMC_IIC2_SDA_PIN (GPIO_PIN_9)

void hk_i2c_init(const uint32_t iic_id);
int i2c_master_initialize(const uint32_t iic_id, const uint32_t i2c_addr);
int i2c_master_transmit(const uint32_t iic_id, uint16_t dev_addr, uint8_t *tx_data, uint32_t size, int time_out);
int i2c_master_receive(const uint32_t iic_id, uint16_t dev_addr, uint8_t *rx_data, uint32_t size, int time_out);
int i2c_slave_initialize(const uint32_t iic_id, const uint32_t iic_addr);
int i2c_slave_receive(const uint32_t iic_id, uint8_t *rx_data, const uint32_t size, uint8_t block_en, const int time_out);

#endif
