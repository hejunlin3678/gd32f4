
#include "stdint.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "console/serial.h"
#include "hk_sensor/hk_sensor.h"
#include "i2c/hk_i2c.h"
#include "hk_common.h"
#include "online_check/online_check.h"
#include "sensor_list.h"

// ADC  struct
volatile hkADC_t g_adc_data;
int dbg_level = HK_INFO;

int iic_9545_switch_channel(const uint32_t iic_id, uint8_t sensor_addr, uint8_t channel)
{
    int ret = -1;
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    ret = i2c_master_receive(iic_id, sensor_addr, rx_buffer, 1, I2C_TIME_OUT);
    vTaskDelay(5);  //to 9545 seitch channel 
    tx_buffer[0] = (channel == 0 ? 0x01 : (channel == 1 ? 0x02 : (channel == 2 ? 0x04 : 0x08)));
    ret = i2c_master_transmit(iic_id, sensor_addr, tx_buffer, 1, I2C_TIME_OUT);
    vTaskDelay(5); //to 9545 seitch channel
    return ret;
}

// IIC 温度获取
int read_lm75_temp(const uint32_t iic_id, uint8_t sensor_addr) // return signed char!!!
{
    int ret;
    uint8_t tx_buffer[1] = {0};
    uint8_t rx_buffer[2] = {0};

    
    iic_9545_switch_channel(iic_id, 0xe6, 0x01);
    vTaskDelay(5); 
    ret = i2c_master_receive(iic_id, sensor_addr, rx_buffer, 1, I2C_TIME_OUT);
    vTaskDelay(5);
    if (ret == 0)
    {
        hk_print_msg(HK_DEBUG, "temp : %d 'C", (signed char)rx_buffer[0]);
        return (signed char)rx_buffer[0];
    }

    //cpu重新上电，lm75这路iic会挂掉(用的后电)，检测电压大于10V，重新初始化sensor_iic,
    if(g_sensor_list[SENSOR_CPU_12V].data.iValue > 10)  
    {
        hk_i2c_init(SENSOR_IIC); // IIC2
        i2c_master_initialize(SENSOR_IIC, SENSOR_MASTER);
    }

    return HK_EEROR_INT; // return error
}

// ADC  电压和电流采集
static void adc_sample(void)
{
    ADC_M ch; // channel enum
    float temp_mcu = 0; //mcu temp

    for (uint8_t i = 0; i < ADC_NUMS; i++)
    {
        ch = i;

        g_adc_data.AD[ch].adc_value = hk_adc_filter(HK_ADC_BASE, adcChannel[ch]);
        g_adc_data.AD[ch].V2 =
            (((float)g_adc_data.AD[ch].adc_value * ADC_VREF) / (float)ADC1_ACCURACY);

        // taskENTER_CRITICAL();
        if (ch == VPX_12V_A) // 电流
        {
            g_adc_data.AD[ch].A = xNum[ch] * g_adc_data.AD[ch].V2;//2023.12.01修改，硬件改动，修改电流采集算法
            g_sensor_list[ch].data.fValue = g_adc_data.AD[ch].A;
        }
        else // 电压
        {
            g_adc_data.AD[ch].V1 = xNum[ch] * g_adc_data.AD[ch].V2;
            g_sensor_list[ch].data.fValue = g_adc_data.AD[ch].V1;
        }
        // taskEXIT_CRITICAL();
    }

    /* mcu temperature */
    temp_mcu = hk_adc_filter(HK_ADC_BASE, MCU_TEMP);
    taskENTER_CRITICAL();
    temp_mcu = temp_mcu * 3.3 / 4096;
    g_sensor_list[SENSOR_MCU_TEMP].data.iValue = (1.43 - temp_mcu) / 0.0043 + 25;
    taskEXIT_CRITICAL();
}

static float alarm_value[10][3] = {
    {12.60, 13.20, 13.80}, // MXM_12V0
    {12.60, 13.20, 13.80}, // CPU_12V0
    {3.46, 3.63, 3.80},    // CPU_3V3
    {5.25, 5.5, 5.75},     // BOARD_5V0
    {3.46, 3.63, 3.80},    // BOARD_3V3
    {15.00, 15.20, 15.80}, // BOARD_WAVE
    {60, 70, 80},          // Inlet_Temp
    {90, 100, 110},        // Outlet_Temp
    {60, 70, 80},          // BMC_Temp
    {80, 90, 102},         // CPU_Core_Temp
};

static void sensor_alarm_data_init(void)
{
    uint8_t read_data[4] = {0};
    uint8_t i = 0;

    for (i = 0; i < 10; i++)
    {
        memset(read_data, 0x00, 4);
        flash_read(SENSOR_ALARM_VALUE_ADDR + (4 * i), 4, read_data);
        // printf("flash_read addr [0x%X]\r\n",SENSOR_ALARM_VALUE_ADDR + (4 * i));
        if (read_data[0] == 0xFF) // flash中没有存入数据
        {
            g_sensor_list[i].slightVal = alarm_value[i][0];
            g_sensor_list[i].seriousVal = alarm_value[i][1];
            g_sensor_list[i].dangerVal = alarm_value[i][2];
        }
        else
        {
            g_sensor_list[i].slightVal = read_data[0];
            g_sensor_list[i].seriousVal = read_data[1];
            g_sensor_list[i].dangerVal = read_data[2];
        }
    }
}

static void read_sensor_task(void)
{
    hk_adc_init(); // ADC0

    hk_i2c_init(SENSOR_IIC); // IIC2
    i2c_master_initialize(SENSOR_IIC, SENSOR_MASTER);

    sensor_alarm_data_init(); // alarm data init
    
    while (1)
    {
        // IIC 温度获取
        uint8_t temp1 = read_lm75_temp(SENSOR_IIC, TEMP_OUT);
        uint8_t temp2 = read_lm75_temp(SENSOR_IIC, TEMP_IN);

        g_sensor_list[SENSOR_INLENT_TEMP].data.iValue = temp2;
        g_sensor_list[SENSOR_OUTLENT_TEMP].data.iValue = temp1;
        g_sensor_list[SENSOR_CPU_CORE_TEMP].data.iValue = spi_1_read(0x50);
        g_sensor_list[SENSOR_CPU_PRESENT].data.iValue = cpu_online_check();
        g_sensor_list[SENSOR_MXM_PRESENT].data.iValue = mxm_online_check();
        g_sensor_list[SENSOR_AT1_PRESENT].data.iValue = alt1_online_check();
        g_sensor_list[SENSOR_AT2_PRESENT].data.iValue = alt2_online_check();
        g_sensor_list[SENSOR_MSATA1_PRESENT].data.iValue = msata1_online_check();
        g_sensor_list[SENSOR_MSATA2_PRESENT].data.iValue = msata2_online_check();
        
        // ADC  电压和电流采集
        adc_sample();
        vTaskDelay(1000);
    }
}

// todo:发送ipmb同时读sensor，iic接口验证
void read_sensor_thread(uint8_t uxPriority)
{
    xTaskCreate((TaskFunction_t)read_sensor_task, "read_sensor_task",
                configMINIMAL_STACK_SIZE * 2, NULL, uxPriority, NULL);
}
