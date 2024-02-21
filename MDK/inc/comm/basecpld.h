#ifndef __BASE_CPLD_H__
#define __BASE_CPLD_H__

#include "gd32f4xx.h"


#define BASE_CPLD_READ      (0x03)
#define BASE_CPLD_WRITE     (0x02)

#define BASE_CPLD_VERSION           (0x01)
#define BASE_CPLD_CPU_PWR_STA       (0x02)
#define BASE_CPLD_CPU_PWR_EN        (0x03)
#define BASE_CPLD_PRESENT_RST       (0x04)
#define BASE_CPLD_PWR_STA           (0x05)
#define BASE_CPLD_PWR_OPS           (0x06)

#define BASE_CPLD_INLENT_TEMP       (0x07)
#define BASE_CPLD_OUTLENT_TEMP      (0x08)
#define BASE_CPLD_WAVE              (0x09)
#define BASE_CPLD_2_MSATA_STA       (0x0a)
#define BASE_CPLD_1_MSATA_STA       (0x0b)

#define BASE_BIOS_VERSION_1          (0x0c)
#define BASE_BIOS_VERSION_2          (0x0d)
#define BASE_BIOS_VERSION_3          (0x0e)
#define BASE_BIOS_VERSION_4          (0x0f)

#define BASE_CPU_MAX_FREQ_H          (0x10)
#define BASE_CPU_MAX_FREQ_L          (0x11)
#define BASE_CPU_CUR_FREQ_H          (0x12)
#define BASE_CPU_CUR_FREQ_L          (0x13)
#define BASE_CPU_MODEL_NUM           (0x14)

#define BASE_MEM_MANUFACTURER        (0x18)
#define BASE_MEM_MAX_FREQ_H          (0x19)
#define BASE_MEM_MAX_FREQ_L          (0x1a)
#define BASE_MEM_CUR_FREQ_H          (0x1b)
#define BASE_MEM_CUR_FREQ_L          (0x20)
#define BASE_MEM_SINGLE_CAP          (0x21)
#define BASE_MEM_CAPACITY_H          (0x22)
#define BASE_MEM_CAPACITY_L          (0x23)
#define BASE_MEM_TYPE                (0x24)

#define BASE_GET_BMC_IP_1            (0x25)
#define BASE_GET_BMC_IP_2            (0x26)
#define BASE_GET_BMC_IP_3            (0x27)
#define BASE_GET_BMC_IP_4            (0x28)

#define BASE_GET_BMC_MAC_1           (0x29)
#define BASE_GET_BMC_MAC_2           (0x2A)
#define BASE_GET_BMC_MAC_3           (0x2B)
#define BASE_GET_BMC_MAC_4           (0x2C)
#define BASE_GET_BMC_MAC_5           (0x2D)
#define BASE_GET_BMC_MAC_6           (0x2E)

#define BASE_GET_BMC_MASK_1          (0x2F)
#define BASE_GET_BMC_MASK_2          (0x30)
#define BASE_GET_BMC_MASK_3          (0x31)
#define BASE_GET_BMC_MASK_4          (0x32)

#define BASE_GET_BMC_GATEWAY_1       (0x33)
#define BASE_GET_BMC_GATEWAY_2       (0x34)
#define BASE_GET_BMC_GATEWAY_3       (0x35)
#define BASE_GET_BMC_GATEWAY_4       (0x36)

#define BASE_SET_BMC_IP_1            (0x37)
#define BASE_SET_BMC_IP_2            (0x38)
#define BASE_SET_BMC_IP_3            (0x39)
#define BASE_SET_BMC_IP_4            (0x3A)

#define BASE_SET_BMC_MASK_1          (0x3B)
#define BASE_SET_BMC_MASK_2          (0x3C)
#define BASE_SET_BMC_MASK_3          (0x3D)
#define BASE_SET_BMC_MASK_4          (0x3E)

#define BASE_SET_BMC_GATEWAY_1       (0x3F)
#define BASE_SET_BMC_GATEWAY_2       (0x40)
#define BASE_SET_BMC_GATEWAY_3       (0x41)
#define BASE_SET_BMC_GATEWAY_4       (0x42)

#define BASE_BOARD_BOARD_ID          (0x43)
#define BASE_BOARD_BOM_ID            (0x44)
#define BASE_BOARD_PCB_ID            (0x45)

#define BASE_MODULE1_CPLD_VER        (0x50)
#define BASE_MODULE1_VENDOR_H        (0x51)
#define BASE_MODULE1_VENDOR_L        (0x52)
#define BASE_MODULE1_BOARD_ID        (0x53)
#define BASE_MODULE1_100M_CLOCK      (0x54)
#define BASE_MODULE1_50M_CLOCK       (0x55)
#define BASE_MODULE1_33M_CLOCK       (0x56)
#define BASE_MODULE1_VRD_VER         (0x57)
#define BASE_MODULE1_SYSTEM_STA      (0x58)
#define BASE_MODULE1_SYSTEM_ERROR    (0x59)
#define BASE_MODULE1_BOARD_RST       (0x5B)

#define MOD1_GE_XGE0_STATUS          (0x5C)
#define MOD1_GE_XGE1_STATUS          (0x5D)
#define MOD1_GE_XGE2_STATUS          (0x5E)
#define MOD1_GE_XGE3_STATUS          (0x5F)
#define MOD1_GE_XGE4_STATUS          (0x60)
#define MOD1_GE_XGE5_STATUS          (0x61)
#define MOD1_GE_XGE6_STATUS          (0x62)
#define MOD1_GE_XGE7_STATUS          (0x63)

#define MOD1_VCC_12V_VALUE_H              (0x64)
#define MOD1_VCC_12V_VALUE_L              (0x65)
#define MOD1_CPU_1V8_VALUE_H              (0x66)
#define MOD1_CPU_1V8_VALUE_L              (0x67)
#define MOD1_DDR_0V9_VALUE_H              (0x68)
#define MOD1_DDR_0V9_VALUE_L              (0x69)
#define MOD1_VDDQ_1V2_AC_VALUE_H          (0x6A)
#define MOD1_VDDQ_1V2_AC_VALUE_L          (0x6B)
#define MOD1_VDDQ_1V2_BD_VALUE_H          (0x6C)
#define MOD1_VDDQ_1V2_BD_VALUE_L          (0x6D)
#define MOD1_DDR_2V5_VALUE_H              (0x6E)
#define MOD1_DDR_2V5_VALUE_L              (0x6F)
#define MOD1_VCC_3V3_VALUE_H              (0x70)
#define MOD1_VCC_3V3_VALUE_L              (0x71)
#define MOD1_VCC_5V_DRMOS_VALUE_H         (0x72)
#define MOD1_VCC_5V_DRMOS_VALUE_L         (0x73)
#define MOD1_VCC1_0V9_DRMOS_VALUE_H       (0x74)
#define MOD1_VCC1_0V9_DRMOS_VALUE_L       (0x75)
#define MOD1_VCC2_0V9_DRMOS_VALUE_H       (0x76)
#define MOD1_VCC2_0V9_DRMOS_VALUE_L       (0x77)

#define MOD1_TEMPER1_VALUE           (0x78)
#define MOD1_TEMPER2_VALUE           (0x79)
#define MOD1_TEMPER3_VALUE           (0x7A)
#define MOD1_TEMPER4_VALUE           (0x7B)
#define MOD1_TEMPER5_VALUE           (0x7C)
#define MOD1_TEMPER6_VALUE           (0x7D)

#define MOD1_INTERRUPT_ALARM_CODE    (0x7E)
#define MOD1_ABNORMAL_POWER_REPORT   (0x7F)
#define MOD1_BMC_INTERRUPT_CLEAR     (0x80)

#define BASE_MODULE2_CPLD_VER        (0x81)
#define BASE_MODULE2_VENDOR_H        (0x82)
#define BASE_MODULE2_VENDOR_L        (0x83)
#define BASE_MODULE2_BOARD_ID        (0x84)
#define BASE_MODULE2_100M_CLOCK      (0x85)
#define BASE_MODULE2_50M_CLOCK       (0x86)
#define BASE_MODULE2_33M_CLOCK       (0x87)
#define BASE_MODULE2_VRD_VER         (0x88)
#define BASE_MODULE2_SYSTEM_STA      (0x89)
#define BASE_MODULE2_SYSTEM_ERROR    (0x8A)
#define BASE_MODULE2_BOARD_RST       (0x8B)

#define MOD2_GE_XGE0_STATUS          (0x8C)
#define MOD2_GE_XGE1_STATUS          (0x8D)
#define MOD2_GE_XGE2_STATUS          (0x8E)
#define MOD2_GE_XGE3_STATUS          (0x8F)
#define MOD2_GE_XGE4_STATUS          (0x90)
#define MOD2_GE_XGE5_STATUS          (0x91)
#define MOD2_GE_XGE6_STATUS          (0x92)
#define MOD2_GE_XGE7_STATUS          (0x93)

#define MOD2_VCC_12V_VALUE_H              (0x94)
#define MOD2_VCC_12V_VALUE_L              (0x95)
#define MOD2_CPU_1V8_VALUE_H              (0x96)
#define MOD2_CPU_1V8_VALUE_L              (0x97)
#define MOD2_DDR_0V9_VALUE_H              (0x98)
#define MOD2_DDR_0V9_VALUE_L              (0x99)
#define MOD2_VDDQ_1V2_AC_VALUE_H          (0x9A)
#define MOD2_VDDQ_1V2_AC_VALUE_L          (0x9B)
#define MOD2_VDDQ_1V2_BD_VALUE_H          (0x9C)
#define MOD2_VDDQ_1V2_BD_VALUE_L          (0x9D)
#define MOD2_DDR_2V5_VALUE_H              (0x9E)
#define MOD2_DDR_2V5_VALUE_L              (0x9F)
#define MOD2_VCC_3V3_VALUE_H              (0xA0)
#define MOD2_VCC_3V3_VALUE_L              (0xA1)
#define MOD2_VCC_5V_DRMOS_VALUE_H         (0xA2)
#define MOD2_VCC_5V_DRMOS_VALUE_L         (0xA3)
#define MOD2_VCC1_0V9_DRMOS_VALUE_H       (0xA4)
#define MOD2_VCC1_0V9_DRMOS_VALUE_L       (0xA5)
#define MOD2_VCC2_0V9_DRMOS_VALUE_H       (0xA6)
#define MOD2_VCC2_0V9_DRMOS_VALUE_L       (0xA7)

#define MOD2_TEMPER1_VALUE           (0xA8)
#define MOD2_TEMPER2_VALUE           (0xA9)
#define MOD2_TEMPER3_VALUE           (0xAA)
#define MOD2_TEMPER4_VALUE           (0xAB)
#define MOD2_TEMPER5_VALUE           (0xAC)
#define MOD2_TEMPER6_VALUE           (0xAD)

#define MOD2_INTERRUPT_ALARM_CODE    (0xAE)
#define MOD2_ABNORMAL_POWER_REPORT   (0xAF)
#define MOD2_BMC_INTERRUPT_CLEAR     (0xB0)

uint8_t bc_get_version(void);
uint8_t bc_get_cpu_pwr_status(void);
uint8_t bc_get_cpu_power_en(void);
uint8_t bc_get_cpu_present_reset(void);
uint8_t bc_get_power_status(void);
uint8_t bc_get_power_ops(void);
uint8_t bc_get_inlent_temp(void);
uint8_t bc_get_outlent_temp(void);
uint8_t bc_get_wave(void);
uint8_t bc_get_cpu2_msata_status(void);
uint8_t bc_get_cpu1_msata_status(void);
void bc_cpu_power_on(void);
void bc_cpu_power_off(void);
void bc_heat_on(void);
void bc_heat_off(void);
uint8_t bc_read(uint8_t addr);
uint8_t bc_write(uint8_t addr, uint8_t value);

#endif /* __BASE_CPLD_H__ */
