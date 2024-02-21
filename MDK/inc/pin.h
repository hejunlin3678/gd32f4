#ifndef __PIN_H__
#define __PIN_H__

#define BMC_CPLD_GPIO0_PORT           (GPIOE)
#define BMC_CPLD_GPIO0_PIN            (GPIO_PIN_2)

#define CPU_BIOS_SW_PORT              (GPIOD)
#define CPU_BIOS_SW_PIN               (GPIO_PIN_14)

#define BMC_CPLD_GPIO1_PORT           (GPIOE)
#define BMC_CPLD_GPIO1_PIN            (GPIO_PIN_3)

#define BMC_CPLD_GPIO2_PORT           (GPIOE)
#define BMC_CPLD_GPIO2_PIN            (GPIO_PIN_4)

#define BMC_CPLD_GPIO3_PORT           (GPIOE)
#define BMC_CPLD_GPIO3_PIN            (GPIO_PIN_5)

#define BMC_CPLD_GPIO4_PORT           (GPIOE)
#define BMC_CPLD_GPIO4_PIN            (GPIO_PIN_6)

#define BMC_CPLD_GPIO5_PORT           (GPIOC)
#define BMC_CPLD_GPIO5_PIN            (GPIO_PIN_13)

#define BMC_CPLD_GPIO6_PORT           (GPIOC)
#define BMC_CPLD_GPIO6_PIN            (GPIO_PIN_14)

#define BMC_CPLD_GPIO7_PORT           (GPIOC)
#define BMC_CPLD_GPIO7_PIN            (GPIO_PIN_15)

#define BMC_CPLD_GPIO8_PORT           (GPIOA)
#define BMC_CPLD_GPIO8_PIN            (GPIO_PIN_10)

#define VPX_GA0_PORT                  (GPIOE)
#define VPX_GA0_PIN                   (GPIO_PIN_9)

#define VPX_GA1_PORT                  (GPIOE)
#define VPX_GA1_PIN                   (GPIO_PIN_10)

#define VPX_GA2_PORT                  (GPIOE)
#define VPX_GA2_PIN                   (GPIO_PIN_11)

#define VPX_GA3_PORT                  (GPIOE)
#define VPX_GA3_PIN                   (GPIO_PIN_12)

#define VPX_GA4_PORT                  (GPIOE)
#define VPX_GA4_PIN                   (GPIO_PIN_13)

#define VPX_GAP_PORT                  (GPIOE)
#define VPX_GAP_PIN                   (GPIO_PIN_14)

#define CPU_BMC_COM_SW_PORT           (GPIOD)
#define CPU_BMC_COM_SW_PIN            (GPIO_PIN_15)

#define PWR_GD_12V_PORT               (GPIOC)
#define PWR_GD_12V_PIN                (GPIO_PIN_10)

#define BMC_UART4_TX_PORT          (GPIOC)
#define BMC_UART4_TX_PIN           (GPIO_PIN_12)
#define BMC_UART4_RX_PORT          (GPIOD)
#define BMC_UART4_RX_PIN           (GPIO_PIN_2)

#define BMC_USART5_TX_PORT         (GPIOA)
#define BMC_USART5_TX_PIN          (GPIO_PIN_11)
#define BMC_USART5_RX_PORT         (GPIOA)
#define BMC_USART5_RX_PIN          (GPIO_PIN_12)

#define BMC_UART1_TX_PORT          (GPIOD)
#define BMC_UART1_TX_PIN           (GPIO_PIN_5)
#define BMC_UART1_RX_PORT          (GPIOD)
#define BMC_UART1_RX_PIN           (GPIO_PIN_6)

#define FPGA_JTAG_EN_PORT          (GPIOE)
#define FPGA_JTAG_EN_PIN           (GPIO_PIN_15)

//9545 RST 
#define RST_NCA9545_PORT          (GPIOE)
#define RST_NCA9545_PIN           (GPIO_PIN_7)

void gpio_init(void);
void gpio_spi_select_default(void);
void gpio_spi_select_upgrade(void);
void power_12v_on(void);
void power_12v_off(void);

#endif /* __PIN_H__ */

