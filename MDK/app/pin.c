

#include "gd32F4xx.h"
#include "pin.h"
#include "systick.h"
#include "comm/basecpld.h"
#include "i2c/hk_i2c.h"

void gpio_init(void)
{
    /* enable the gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* input */
    gpio_mode_set(PWR_GD_12V_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PWR_GD_12V_PIN);   // 后电12V切換标志 未使用
    gpio_mode_set(VPX_GA0_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GA0_PIN);
    gpio_mode_set(VPX_GA1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GA1_PIN);
    gpio_mode_set(VPX_GA2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GA2_PIN);
    gpio_mode_set(VPX_GA3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GA3_PIN);
    gpio_mode_set(VPX_GA4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GA4_PIN);
    gpio_mode_set(VPX_GAP_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VPX_GAP_PIN);
    /* IIC NOTICE */  //Initialize as input first
    gpio_mode_set(IPMB1_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IPMB1_SCL_PIN);
    gpio_mode_set(IPMB1_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IPMB1_SDA_PIN);
    gpio_mode_set(IPMB2_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IPMB2_SCL_PIN);
    gpio_mode_set(IPMB2_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IPMB2_SDA_PIN);
    gpio_mode_set(BMC_IIC2_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BMC_IIC2_SCL_PIN);
    gpio_mode_set(BMC_IIC2_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BMC_IIC2_SDA_PIN);

    /* output */
    gpio_mode_set(BMC_CPLD_GPIO0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO0_PIN);
    gpio_mode_set(CPU_BIOS_SW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, CPU_BIOS_SW_PIN);
    gpio_mode_set(BMC_CPLD_GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO1_PIN);
    gpio_mode_set(BMC_CPLD_GPIO2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO2_PIN);
    gpio_mode_set(BMC_CPLD_GPIO3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO3_PIN);
    gpio_mode_set(BMC_CPLD_GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO4_PIN);
    gpio_mode_set(BMC_CPLD_GPIO5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO5_PIN);
    gpio_mode_set(BMC_CPLD_GPIO6_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO6_PIN);
    gpio_mode_set(BMC_CPLD_GPIO7_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO7_PIN);
    gpio_mode_set(BMC_CPLD_GPIO8_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMC_CPLD_GPIO8_PIN);
    gpio_mode_set(CPU_BMC_COM_SW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, CPU_BMC_COM_SW_PIN);   // COM 串口切換
    gpio_mode_set(FPGA_JTAG_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, FPGA_JTAG_EN_PIN);
    
    gpio_output_options_set(BMC_CPLD_GPIO0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BMC_CPLD_GPIO0_PIN);
    gpio_output_options_set(CPU_BIOS_SW_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, CPU_BIOS_SW_PIN);
    gpio_output_options_set(FPGA_JTAG_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, FPGA_JTAG_EN_PIN);

    gpio_bit_set(FPGA_JTAG_EN_PORT, FPGA_JTAG_EN_PIN);

    gpio_spi_select_default();
}

// 00
void gpio_spi_select_default(void)
{
    gpio_bit_reset(CPU_BIOS_SW_PORT, CPU_BIOS_SW_PIN);
    gpio_bit_reset(BMC_CPLD_GPIO0_PORT, BMC_CPLD_GPIO0_PIN);
}

// 10
void gpio_spi_select_upgrade(void)
{
    gpio_bit_set(CPU_BIOS_SW_PORT, CPU_BIOS_SW_PIN);
    gpio_bit_reset(BMC_CPLD_GPIO0_PORT, BMC_CPLD_GPIO0_PIN);
}

/* SPACE LINE FOR END OF FILE */

