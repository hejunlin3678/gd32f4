#ifndef __HK_SENSOR_H__
#define __HK_SENSOR_H__

#include "gd32f4xx_i2c.h"
#include "adc/hk_adc.h"

#define SENSOR_MASTER (0x20)
#define SENSOR_IIC (I2C2)
#define TEMP_OUT (0x9a)
#define TEMP_IN (0x92)

#define SENSOR_ALARM_VALUE_ADDR (0x082E0000ul) //sensor alarm addr

// [6:7] 11b 不使用[0:5]超限情况
// [5]–1b危险超上限值Danger exceeds the upper limit
// [4]–1b严重超上限值Seriously exceeds the upper limit
// [3]–1b轻微超上限值Slightly over the upper limit
// [2]–1b危险超下限值The danger exceeds the lower limit
// [1]–1b严重超下限值Seriously exceeds the lower limit
// [0]–1b轻微超下限值Slightly over lower limit

/* 上限和下限 阀值 */
typedef enum
{
    SLIGHTLY_OVER_LOWER_LIMIT = 0U, //轻微超下限值
    SERIOUSLY_OVER_LOWER_LIMIT,     //严重超下限值
    DANGER_OVER_LOWER_LIMIT,        //危险超下限值
    SLIGHTLY_OVER_UPPER_LIMIT,      //轻微超上限值
    SERIOUSLY_OVER_UPPER_LIMIT,     //严重超上限值
    DANGER_OVER_UPPER_LIMIT,        //危险超上限值
    NO_LIMIT,                       //没有限制
    NORMAL,
} boundary_value_enum;

typedef struct
{
    uint16_t adc_value; // AD 值

    float V1;               //换算后的电压值
    float V2;               //采出的电压值
    float A;                //电流值
} ADC_t;

// Sensor  struct
typedef struct
{
    ADC_t AD[ADC_NUMS];

} hkADC_t;

typedef struct
{
    char    name[16];   //sensor name
    uint8_t unit;       // 0-温度 1-电压 2-电流
    union 
    {
        float    fValue;
        int      iValue;
    }data;
    float   slightVal;  //轻微
    float   seriousVal; //严重
    float   dangerVal;  //危险
    float   lowSlightVal;  //轻微
    float   lowSeriousVal; //严重
    float   lowDangerVal;  //危险
    boundary_value_enum  curAlarm;
} Sensor_t;

typedef enum
{
    SENSOR_MXM_12V = 0,
    SENSOR_CPU_12V,
    SENSOR_CPU3V3,
    SENSOR_BOARD_5V,
    SENSOR_BOARD_3V3,
    SENSOR_BOARD_WAVE,
    SENSOR_INLENT_TEMP,
    SENSOR_OUTLENT_TEMP,
    SENSOR_MCU_TEMP,
    SENSOR_CPU_CORE_TEMP,
    SENSOR_CPU_PRESENT,
    SENSOR_MXM_PRESENT,
    SENSOR_AT1_PRESENT,
    SENSOR_AT2_PRESENT,
    SENSOR_MSATA1_PRESENT,
    SENSOR_MSATA2_PRESENT,
} Sensor_type;

extern Sensor_t g_sensor_list[16];

int read_lm75_temp(const uint32_t iic_id, uint8_t sensor_addr);

void read_sensor_thread(uint8_t uxPriority);
#endif
