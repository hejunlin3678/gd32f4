/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_sys_warn.c
 **
 ** \brief  sensor
 **
 ** \author macy@schkzy.cn
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include "syswarn/hk_sys_warn.h"

#if 0
extern uint8_t g_sensor_number;
void get_slots_gpio_init() // io board get sllot status
{
    for (uint8_t i = 0; i < SLOT_GPIO_NUM; i++)
    {
        rcu_periph_clock_enable((rcu_periph_enum)SLOT_GPIO_CLK[i]);
        gpio_mode_set(SLOT_GPIO_PORT[i], GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SLOT_GPIO_PIN[i]);
    }
}

uint8_t get_slots_status(uint8_t slot_id)
{
    return gpio_input_bit_get(SLOT_GPIO_PORT[slot_id], SLOT_GPIO_PIN[slot_id]);
}

void sensor_i2c_init(void)
{
    /* configure RCU */
    i2c_rcu_config(I2C_SENSOR);
    /* configure GPIO */
    i2c_gpio_config(I2C_SENSOR);
    /* configure I2C */
    i2c_config(I2C_SENSOR);
}

uint8_t i2c_read_temp(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, int timeout)
{
    uint8_t enRet = 0;
    uint8_t tx_buffer[1] = {0};
    enRet = I2c_Master_Transmit(I2C_SENSOR, u16DevAddr, tx_buffer, 1u, timeout);
    enRet = I2c_Master_Receive(I2C_SENSOR, u16DevAddr, pu8RxData, u32Size, timeout);
    // hk_print_msg(HK_DEBUG, "temp sensor value:0x%x,0x%x", pu8RxData[0], pu8RxData[1]);
    return enRet;
}

uint8_t i2c_write_sensor_reg(uint16_t u16DevAddr, uint8_t u8RegAddr, uint8_t *pu8TxData, uint32_t u32Size, int timeout)
{
    uint8_t enRet = 0;
    // __disable_irq();
    uint32_t i;
    I2c_Master_Initialize(I2C_SENSOR, IPMB_MASTER_ADDR);
    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C_SENSOR, I2C_FLAG_I2CBSY))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C_SENSOR);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_SBSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(I2C_SENSOR, u16DevAddr, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_ADDSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C_SENSOR, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_TBE))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }

    i2c_data_transmit(I2C_SENSOR, (uint8_t)u8RegAddr);
    /* wait until the TBE bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_TBE))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }

    for (i = 0; i < u32Size; i++)
    {
        /* data transmission */
        i2c_data_transmit(I2C_SENSOR, pu8TxData[i]);
        /* wait until the TBE bit is set */
        while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_TBE))
        {
            timeout--;
            if (timeout <= 0)
                break;
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C_SENSOR);
    while (I2C_CTL0(I2C_SENSOR) & I2C_CTL0_STOP)
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    // i2c_enable(I2C_SENSOR);
    if (timeout <= 0)
        enRet = 1;
    // __enable_irq();
    return enRet;
}

uint8_t i2c_read_sensor_reg(uint16_t u16DevAddr, uint8_t u8RegAddr, uint8_t *pu8RxData, uint32_t u32Size, int timeout)
{
    uint8_t enRet = 0;
    // __disable_irq();
    I2c_Master_Initialize(I2C_SENSOR, IPMB_MASTER_ADDR);
    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C_SENSOR, I2C_FLAG_I2CBSY))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C_SENSOR);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_SBSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C_SENSOR, (uint8_t)u16DevAddr, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_ADDSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C_SENSOR, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_TBE))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* data transmission */
    i2c_data_transmit(I2C_SENSOR, (uint8_t)u8RegAddr);
    /* wait until the TBE bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_TBE))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    i2c_start_on_bus(I2C_SENSOR);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_SBSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    i2c_master_addressing(I2C_SENSOR, u16DevAddr, I2C_RECEIVER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_ADDSEND))
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C_SENSOR, I2C_FLAG_ADDSEND);
    for (uint32_t i = 0; i < u32Size; i++)
    {

        /* wait until the RBNE bit is set */
        // timeout = 0x500; //定时待调 todo
        while (!i2c_flag_get(I2C_SENSOR, I2C_FLAG_RBNE))
        {
            timeout--;
            if (timeout <= 0)
                break;
        }

        /* read a data from I2C_DATA */
        pu8RxData[i] = i2c_data_receive(I2C_SENSOR);
        // i2c_ack_config(I2C_SENSOR, I2C_ACK_DISABLE);
    }
    i2c_ack_config(I2C_SENSOR, I2C_ACK_DISABLE);
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C_SENSOR);
    /* wait until stop condition generate */
    while (I2C_CTL0(I2C_SENSOR) & I2C_CTL0_STOP)
    {
        timeout--;
        if (timeout <= 0)
            break;
    }
    /* enable acknowledge */
    // i2c_ack_config(I2C_SENSOR, I2C_ACK_DISABLE);
    if (timeout <= 0)
        enRet = 1;
    // __enable_irq();
    return enRet;
}

#define GET_BIT(value, bit) ((value) & (1 << (bit)))    // read bit value
#define CPL_BIT(value, bit) ((value) ^= (1 << (bit)))   // negate bit value
#define SET0_BIT(value, bit) ((value) &= ~(1 << (bit))) // set bit is 0
#define SET1_BIT(value, bit) ((value) |= (1 << (bit)))  // set bit is 1

static uint8_t set_time_format(uint8_t value, unsigned int bitl, unsigned int bith, uint8_t data)
{
    uint8_t v = value;
    if (bitl <= bith)
    {
        unsigned int bcount = bith - bitl + 1;
        unsigned int cbit = 0;
        unsigned int cdata = 0;
        for (unsigned int i = 0; i < bcount; i++)
        {
            cdata |= (1 << i);
            cbit |= (1 << (bitl + i));
        }
        v &= ~(cbit);
        v |= ((data & cdata) << bitl);
    }
    return v;
}

static int bcd_int(uint8_t data)
{
    int h = (data >> 4) & 0x0F;
    int l = data & 0x0F;
    int s = h * 10 + l;
    return s;
}

static uint8_t int_bcd(int data)
{
    int h = data / 10;
    int l = data % 10;
    int s = h << 4 | l;
    return s;
}

static uint8_t get_now_time(void)
{
    static uint8_t g_now_time[6] = {0};
    uint8_t pr_time[7] = {0};
    uint8_t ret = 0;
    ret = i2c_read_sensor_reg(BM8563_ADDR, 0x02, pr_time, 7, I2C_TIME_OUT);
    if (ret == 0)
    {
        g_now_time[5] = bcd_int(set_time_format(pr_time[0], 7, 7, 0)); // second
        g_now_time[4] = bcd_int(set_time_format(pr_time[1], 7, 7, 0)); // min
        g_now_time[3] = bcd_int(set_time_format(pr_time[2], 6, 7, 0)); // hour
        g_now_time[2] = bcd_int(set_time_format(pr_time[3], 6, 7, 0)); // day
        g_now_time[1] = bcd_int(set_time_format(pr_time[5], 5, 7, 0)); // month
        g_now_time[0] = bcd_int(pr_time[6]);                           // year
        return 0; // ok
    }
    else
        // hk_print_msg(HK_ERROR, "get time fail!!!");
    return 1; // ng
}

uint8_t set_time[9] = {0, 0, 0, 13, 10, 8, 2, 2, 22}; // test data 2022/02/08-10:13:00
static void set_now_time(uint8_t *new_time, uint8_t time_len)
{
    uint8_t write_time[9] = {0};
    uint8_t ret = 0;
    for (uint8_t i = 0; i < time_len; i++)
    {
        write_time[i] = int_bcd(new_time[i]);
    }

    ret = i2c_write_sensor_reg(BM8563_ADDR, 0x00, write_time, time_len, I2C_TIME_OUT);
    if (ret == 0)
        printf("set time ok!!!");
    else
        printf("set time fail!!!");
}

void sys_warn_task(void) // set  cycle time to get sensor value
{
    //adc0_init();
    // hk_print_msg(HK_DEBUG, "running...");
    //while (1)
    //{
        //g_sensor_number = g_sensor_data_read_func();
        //get_now_time();
        // // hk_print_msg(HK_DEBUG, "work time: %d min,sensor num :%d", rtc_get_work_time(), g_sensor_number);
        // vTaskDelay(5000);
   // }
}
#endif
