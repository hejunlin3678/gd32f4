#include "FreeRTOS.h"
#include "task.h"
#include "i2c/hk_i2c.h"
#include "hk_common.h"

/*注意事项：
  1.iic通信中断异常后，易出现死锁。
  主机IO板处理办法：多次通信检测到SDA线拉低后，发生9个clk复位总线，建议复位。
  从机刀片处理办法：设置iic通信线程”看门狗“，长时间未收到信息判断，进行iic初始化。

  2.3路iic-IO需根据工程实际应用配置。

  3.使用轮询收发,接收阻塞判断时注意任务优先级&CPU占用
*/

uint8_t g_i2cbus_error_fg = 0; // 总线错误次数标识,错误次数达阈值，重置总线。

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hk_i2c_rcu_config(const uint32_t iic_id)
{
    switch (iic_id)
    {
    case I2C0:
        /* enable GPIOB clock */
        rcu_periph_clock_enable(RCU_GPIOB);
        /* enable I2C0 clock */
        rcu_periph_clock_enable(RCU_I2C0);
        break;

    case I2C1:
        /* enable GPIOB clock */
        rcu_periph_clock_enable(RCU_GPIOB);
        /* enable I2C1 clock */
        rcu_periph_clock_enable(RCU_I2C1);
        break;

    case I2C2:
        /* enable GPIOA/GPIOC clock */
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_GPIOC);
        /* enable I2C0 clock */
        rcu_periph_clock_enable(RCU_I2C2);
        break;

    default:
        break;
    }
}

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hk_i2c_gpio_config(const uint32_t iic_id)
{

    switch (iic_id)
    {

    case I2C0:
        /* I2C0 GPIO ports */
        /* connect PB6 to I2C0_SCL */
        gpio_af_set(IPMB1_SCL_PORT, GPIO_AF_4, IPMB1_SCL_PIN); // 引脚跟工程匹配!!!!如:IIC0还可为PB9,PB10
        /* connect PB7 to I2C0_SDA */
        gpio_af_set(IPMB1_SDA_PORT, GPIO_AF_4, IPMB1_SDA_PIN);

        /* configure I2C0 GPIO */
        gpio_mode_set(IPMB1_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IPMB1_SCL_PIN);
        gpio_output_options_set(IPMB1_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IPMB1_SCL_PIN);
        gpio_mode_set(IPMB1_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IPMB1_SDA_PIN);
        gpio_output_options_set(IPMB1_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IPMB1_SDA_PIN);
        break;

    case I2C1:
        /* I2C1 GPIO ports */
        /* connect PB10 to I2C1_SCL */
        gpio_af_set(IPMB2_SCL_PORT, GPIO_AF_4, IPMB2_SCL_PIN); // 引脚跟工程匹配!!!!
        /* connect PB3 to I2C1_SDA */
        gpio_af_set(IPMB2_SDA_PORT, GPIO_AF_9, IPMB2_SDA_PIN);

        /* configure I2C1 GPIO */
        gpio_mode_set(IPMB2_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IPMB2_SCL_PIN);
        gpio_output_options_set(IPMB2_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IPMB2_SCL_PIN);
        gpio_mode_set(IPMB2_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, IPMB2_SDA_PIN);
        gpio_output_options_set(IPMB2_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IPMB2_SDA_PIN);
        break;

    case I2C2:
        /* I2C1 GPIO ports */
        /* connect PA8 to I2C2_SCL */
        gpio_af_set(BMC_IIC2_SCL_PORT, GPIO_AF_4, BMC_IIC2_SCL_PIN); // 引脚跟工程匹配!!!!
        /* connect PC9 to I2C2_SDA */
        gpio_af_set(BMC_IIC2_SDA_PORT, GPIO_AF_4, BMC_IIC2_SDA_PIN);

        /* configure I2C2 GPIO */
        gpio_mode_set(BMC_IIC2_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMC_IIC2_SCL_PIN);
        gpio_output_options_set(BMC_IIC2_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, BMC_IIC2_SCL_PIN);
        gpio_mode_set(BMC_IIC2_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMC_IIC2_SDA_PIN);
        gpio_output_options_set(BMC_IIC2_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, BMC_IIC2_SDA_PIN);
        break;

    default:
        hk_print_msg(HK_ERROR, "EEROR: hk_i2c_gpio_config");
        break;
    }
}

/*!
    \brief      configure the i2c master interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hk_i2c_config(const uint32_t iic_id)
{
    switch (iic_id)
    {

    case I2C0:
        /* configure I2C clock */
        i2c_clock_config(I2C0, I2C_BAUDRATE, I2C_DTCY_2);
        /* configure I2C address */
        i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, DEFAULT_IIC_ADDR);
        /* enable I2C0 */
        i2c_enable(I2C0);
        /* enable acknowledge */
        i2c_ack_config(I2C0, I2C_ACK_ENABLE);
        break;

    case I2C1:
        /* configure I2C clock */
        i2c_clock_config(I2C1, I2C_BAUDRATE, I2C_DTCY_2);
        /* configure I2C address */
        i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, DEFAULT_IIC_ADDR);
        /* enable I2C1 */
        i2c_enable(I2C1);
        /* enable acknowledge */
        i2c_ack_config(I2C1, I2C_ACK_ENABLE);
        break;

    case I2C2:
        /* configure I2C clock */
        i2c_clock_config(I2C2, I2C_BAUDRATE, I2C_DTCY_2);
        /* configure I2C address */
        i2c_mode_addr_config(I2C2, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, DEFAULT_IIC_ADDR);
        /* enable I2C2 */
        i2c_enable(I2C2);
        /* enable acknowledge */
        i2c_ack_config(I2C2, I2C_ACK_ENABLE);
        break;

    default:
        hk_print_msg(HK_ERROR, "EEROR: hk_i2c_config");
        break;
    }
}

void hk_i2c_init(const uint32_t iic_id)
{
    /* configure RCU */
    hk_i2c_rcu_config(iic_id);
    /* configure GPIO */
    hk_i2c_gpio_config(iic_id);
    /* configure I2C */
    hk_i2c_config(iic_id);

	 /*因为焊接电阻，电流识别错误(西安改)*/
	  I2C_CTL0(iic_id) |= 0X0080;
	
    // todo+初始化检测总线是否异常，是-->9个clk，复位IIC总线.
}

int i2c_master_initialize(const uint32_t iic_id, const uint32_t i2c_addr)
{
    int ret = 0;
    /* configure I2C clock */
    i2c_deinit(iic_id);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    i2c_clock_config(iic_id, I2C_BAUDRATE, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(iic_id, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, i2c_addr);
    /* enable I2C0 */
    i2c_enable(iic_id);
    /* enable acknowledge */
    i2c_ack_config(iic_id, I2C_ACK_ENABLE);

    return ret;
}

int i2c_master_transmit(const uint32_t iic_id, uint16_t dev_addr, uint8_t *tx_data, uint32_t size, int time_out)
{
    uint32_t i;
    int ret = 0;
    int time = 0;
    // i2c_master_initialize(iic_id, DEFAULT_IIC_ADDR);
    /* wait until I2C bus is idle */
    int wait_time_ms = 1000;                      // 等待总线空闲时长不超1秒
    while (i2c_flag_get(iic_id, I2C_FLAG_I2CBSY)) // 发送等待总线空闲
    {
        wait_time_ms--;
        if (wait_time_ms <= 0)
        {
            g_i2cbus_error_fg++;
            hk_print_msg(HK_DEBUG, "iic bus is busy");
            i2c_flag_clear(iic_id, I2C_FLAG_I2CBSY);
            return -1;
        }
        // vTaskDelay(1); // zy+
        portYIELD(); // 向高优先级交出cpu占用
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(iic_id);
    /* wait until SBSEND bit is set */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_SBSEND))
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }

    /* send slave address to I2C bus */
    i2c_master_addressing(iic_id, dev_addr, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_ADDSEND))
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(iic_id, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_TBE))
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }

    for (i = 0; i < size; i++)
    {
        /* data transmission */
        i2c_data_transmit(iic_id, tx_data[i]);
        /* wait until the TBE bit is set */
        time = time_out;
        while (!i2c_flag_get(iic_id, I2C_FLAG_TBE))
        {
            time--;
            if (time <= 0)
            {
                ret--;
                break;
            }
            portYIELD(); // 向高优先级交出cpu占用
        }
    }

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(iic_id);
    time = time_out;
    while (I2C_CTL0(iic_id) & I2C_CTL0_STOP)
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }
    return ret;
}

int i2c_master_receive(const uint32_t iic_id, uint16_t dev_addr, uint8_t *rx_data, uint32_t size, int time_out)
{
    int ret = 0;
    int time = 0;
    // i2c_master_initialize(iic_id, DEFAULT_IIC_ADDR);
    /* wait until I2C bus is idle */

    int wait_time_ms = 1000;                      // 等待总线空闲时长不超1秒
    while (i2c_flag_get(iic_id, I2C_FLAG_I2CBSY)) // 发送等待总线空闲
    {
        wait_time_ms--;
        if (wait_time_ms <= 0)
        {
            g_i2cbus_error_fg++;
            hk_print_msg(HK_DEBUG, "iic bus is busy");
            i2c_flag_clear(iic_id, I2C_FLAG_I2CBSY);
            return -1;
        }
        portYIELD(); // 向高优先级交出cpu占用,注意其他地方cpu使用
        //        vTaskDelay(1);
    }

    /* send a start condition to I2C bus */
    i2c_start_on_bus(iic_id);
    /* wait until SBSEND bit is set */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_SBSEND))
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }
    i2c_master_addressing(iic_id, dev_addr, I2C_RECEIVER);

    /* wait until ADDSEND bit is set */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_ADDSEND))
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }

    /* clear ADDSEND bit */
    i2c_flag_clear(iic_id, I2C_FLAG_ADDSEND);
    for (uint32_t i = 0; i < size; i++)
    {
        /* wait until the RBNE bit is set */
        time = time_out;
        while (!i2c_flag_get(iic_id, I2C_FLAG_RBNE))
        {
            time--;
            if (time <= 0)
            {
                ret--;
                break;
            }
            portYIELD(); // 向高优先级交出cpu占用
        }
        /* read a data from I2C_DATA */
        rx_data[i] = i2c_data_receive(iic_id);
        i2c_ack_config(iic_id, I2C_ACK_DISABLE);
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(iic_id);
    /* wait until stop condition generate */
    time = time_out;
    while (I2C_CTL0(iic_id) & I2C_CTL0_STOP)
    {
        time--;
        if (time <= 0)
        {
            ret--;
            break;
        }
        portYIELD(); // 向高优先级交出cpu占用
    }
    /* enable acknowledge */
    i2c_ack_config(iic_id, I2C_ACK_DISABLE);

    return ret;
}

int i2c_slave_initialize(const uint32_t iic_id, const uint32_t iic_addr)
{
    int ret = 0;
    /* configure I2C clock */
    i2c_deinit(iic_id);
    __NOP(); // 时钟延时,zy+
    __NOP();
    __NOP();
    __NOP();
    i2c_clock_config(iic_id, I2C_BAUDRATE, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(iic_id, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, iic_addr);
    /* enable I2C0 */
    i2c_enable(iic_id);
    /* enable acknowledge */
    i2c_ack_config(iic_id, I2C_ACK_ENABLE);

    return ret;
}

// return rev_data len
// depends on block_en.if block_en ==0,it all was blocking. if block_en !=0, set the time_out blocking;
int i2c_slave_receive(const uint32_t iic_id, uint8_t *rx_data, const uint32_t size, uint8_t block_en, const int time_out)
{
    uint8_t ret = 0;
    /* clear all status */
    uint32_t i;
    int time = 0;
    time = time_out;
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(iic_id, I2C_FLAG_ADDSEND)) // IO板不能阻塞！！！！
    {
        // 未使用中断接收    //zy+
        // 通道做从时，必须阻塞等待I2C_FLAG_ADDSEND状态！！！！，
        // 通道做主时，可不阻塞！！！

        if (block_en != 0)
        {
            time--;
            if (time <= 0)
            {
                hk_print_msg(HK_DEBUG, "=====return===");
                i2c_flag_clear(iic_id, I2C_FLAG_ADDSEND);
                i2c_stop_on_bus(iic_id);
                return 0;
                break;
            }
            portYIELD();
        }
        else
        {
            // time--;
            // if (time <= 0)
            // {
            //     break;
            // }
            // vTaskDelay(1);
            portYIELD(); // 向高优先级交出cpu占用
        }
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(iic_id, I2C_FLAG_ADDSEND);
    for (i = 0; i < size; i++)
    {
        /* wait until the RBNE bit is set */
        time = time_out;
        while (!i2c_flag_get(iic_id, I2C_FLAG_RBNE))
        {
            time--;
            if (time <= 0)
            {
                break;
            }
            portYIELD(); // 向高优先级交出cpu占用
        }
        if (time <= 0) // 超时情况下退出,并返回接收到的长度
        {
            ret = i;
            break;
        }
        /* read a data byte from I2C_DATA */
        rx_data[i] = i2c_data_receive(iic_id);
        // i2c_ack_config(iic_id, I2C_ACK_ENABLE);
    }
    /* wait until the STPDET bit is set */
    time = time_out;
    while (!i2c_flag_get(iic_id, I2C_FLAG_STPDET))
    {
        time--;
        if (time <= 0)
            break;
    }
    /* clear the STPDET bit */
    i2c_enable(iic_id);
    return ret;
}
