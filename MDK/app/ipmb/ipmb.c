#include <stdio.h>
#include <string.h>

#include "pin.h"
#include "ipmb/ipmb.h"
#include "i2c/hk_i2c.h"

static inline uint8_t CalcChecksum(uint8_t *data, int len);
uint32_t g_ipmb_slave_addr = 0; // ipmb地址，根据刀片io检测获取
uint32_t ipmb_master_addr = 0;  // 获取管理子卡地址

void ipmb_calc_checksum(IpmbMsg *msg)
{
    msg->checksum1 = CalcChecksum(&msg->rspAddr, 2U);
    msg->payload[msg->len - 7] = CalcChecksum(&msg->reqAddr, msg->len - 4);
}

uint8_t ipmb_checksum_check(IpmbMsg *msg)
{
    uint8_t checksum = 0;

    checksum = CalcChecksum(&msg->rspAddr, 2U);
    if (checksum != msg->checksum1)
    {
        printf("\r\n[IPMB] msg checksum1 error [recv_len:%d] \r\n", msg->len);
        return CHECKSUM1_ERR;
    }

    checksum = CalcChecksum(&msg->reqAddr, msg->len - 4);
    if (checksum != msg->payload[msg->len - PACKET_LEN])
    {
        printf("\r\n[IPMB] msg checksum2 error [recv_len:%d] \r\n", msg->len);
        return CHECKSUM2_ERR;
    }

    return CHECKSUM_OK;
}

void ipmb_send(uint32_t i2c_periph, IpmbMsg *msg)
{
    // printf("==== ipmb_send ====\r\n");
    i2c_master_initialize(i2c_periph, ipmb_master_addr);
    i2c_master_transmit(i2c_periph, ipmb_master_addr, (uint8_t *)msg, msg->len, I2C_TIME_OUT);
}

uint8_t ipmb_recv(uint32_t i2c_periph, uint8_t bolck_en, IpmbMsg *msg)
{
    uint8_t rev_len = 0;
    // printf("==== ipmb_recv ====\r\n");
    // 默认接收64字节数据  //todo:
    i2c_slave_initialize(i2c_periph, g_ipmb_slave_addr);
    rev_len = i2c_slave_receive(i2c_periph, (uint8_t *)msg, 64, bolck_en, I2C_TIME_OUT);

    return rev_len;
}

static inline uint8_t CalcChecksum(uint8_t *data, int len)
{
    uint8_t Checksum = 0;
    register int i = 0;

    if (data == 0 || len == 0)
        return Checksum;

    for (i = 0; i < len; i++)
        Checksum = (Checksum + data[i]) % 256;

    return (-Checksum);
}

IPMB_SLOT_TYPE slot = 0; // 槽位

#if LAISI_OR_32
//莱斯
uint8_t getReqAddr(void) // return ipmb savle addr
{
    uint8_t GAP = 0u;

    if (1 == gpio_input_bit_get(VPX_GA0_PORT, VPX_GA0_PIN))
    {
        GAP |= 0x1;
    }
    if (1 == gpio_input_bit_get(VPX_GA1_PORT, VPX_GA1_PIN))
    {
        GAP |= 0x2;
    }
    if (1 == gpio_input_bit_get(VPX_GA2_PORT, VPX_GA2_PIN))
    {
        GAP |= 0x4;
    }
    if (1 == gpio_input_bit_get(VPX_GA3_PORT, VPX_GA3_PIN))
    {
        GAP |= 0x8;
    }
    if (1 == gpio_input_bit_get(VPX_GA4_PORT, VPX_GA4_PIN))
    {
        GAP |= 0x10;
    }
    if (1 == gpio_input_bit_get(VPX_GAP_PORT, VPX_GAP_PIN))
    {
        GAP |= 0x20;
    }

    switch (GAP)
    {
    case 0x3B:
        slot = SLOT4;
        break;
    case 0x1A:
        slot = SLOT5;
        break;
    case 0x37:
        slot = SLOT8;
        break;
    case 0x16:
        slot = SLOT9;
        break;
    default:
        slot = 0; // 读不到，默认IIC地址:0xC0
    }
    printf("\r\nGAP=[0x%X] ipmb_addr=[0x%X]\r\n", GAP, ipmb_addr[slot]);
    return ipmb_addr[slot];
}
#else
//32所
uint8_t getReqAddr(void) // return ipmb savle addr
{
    uint8_t GAP = 0u;

    if (1 == gpio_input_bit_get(VPX_GA0_PORT, VPX_GA0_PIN))
    {
        GAP |= 0x1;
    }
    if (1 == gpio_input_bit_get(VPX_GA1_PORT, VPX_GA1_PIN))
    {
        GAP |= 0x2;
    }
    if (1 == gpio_input_bit_get(VPX_GA2_PORT, VPX_GA2_PIN))
    {
        GAP |= 0x4;
    }
    if (1 == gpio_input_bit_get(VPX_GA3_PORT, VPX_GA3_PIN))
    {
        GAP |= 0x8;
    }
    if (1 == gpio_input_bit_get(VPX_GA4_PORT, VPX_GA4_PIN))
    {
        GAP |= 0x10;
    }
    if (1 == gpio_input_bit_get(VPX_GAP_PORT, VPX_GAP_PIN)) //高位
    {
        GAP |= 0x20;
    }

    switch (GAP)
    {
    case 0x3E://111110
        slot = SLOT1;
        break;
    case 0x3D://111101
        slot = SLOT2;
        break;
    case 0x34://110100
        slot = SLOT3;
        break;
    case 0x19://011001
        slot = SLOT4;
        break;
    case 0x1A://011010
        slot = SLOT5;
        break;
    case 0x15://0X0101
        slot = SLOT6;
        break;
    case 0x05:
        slot = SLOT6;
        break;
    case 0x38://1X1000
        slot = SLOT7;
        break;
    case 0x28:
        slot = SLOT7;
        break;
    case 0x3B://111011
        slot = SLOT8;
        break;
    case 0x1C://011100
        slot = SLOT9;
        break;
    default:
        slot = 0; // 读不到，默认IIC地址:0xC0
    }
    printf("\r\nGAP=[0x%X] ipmb_addr=[0x%X]\r\n", GAP, ipmb_addr[slot]);
    return ipmb_addr[slot];
}


#endif

void ipmb_print_msg(IpmbMsg *msg)
{
    uint8_t *temp_data = (uint8_t *)msg;

    for (uint8_t i = 0; i < msg->len; i++)
        printf("0x%.2X ", temp_data[i]);
    
    printf("\r\n");
}


// 电压，电流
// y = (25*x+(-128*10))/100
// x = 4*y + 256/5
uint8_t ipmb_calc_rsp_voltage_value(float value)
{
    uint8_t ret = ( 4*value + (256/5) );
    
    // printf("voltage value=[%.2f] calc=[0x%X]\r\n", value, ret);

    return ret;
}

// 温度
// y = (15*x+(-32*10))/10
// x = (2/3)*y + 64/3
uint8_t ipmb_calc_rsp_temp_value(float value)
{
    uint8_t ret = ( value + 64/3 );
    
    // printf("temp value=[%.2f] calc=[0x%X]\r\n", value, ret);

    return ret;
}

