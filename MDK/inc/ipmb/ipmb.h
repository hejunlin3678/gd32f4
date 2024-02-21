#ifndef __IPMB_H__
#define __IPMB_H__

#include "gd32f4xx.h"
#include <stdio.h>

#define IPMB1 (I2C0)
#define IPMB2 (I2C1)
#define DEVICE_ID (0xFF)    //厂家自己定义

// #define IPMB_NETFN_GET_DEV_SDR                  (0x04 << 2)
// #define IPMB_NETFN_GET_SENSOR_R                 (0x04 << 2)
// #define IPMB_NETFN_SET_FRU_ACT                  (0x2C << 2)
// #define IPMB_NETFN_FSSEM_REQ                    (0x04 << 2)
// #define IPMB_NETFN_FSSEM_RSP                    (0x05 << 2)

// get
#define IPMB_NETFN_GET_DEV_ID (0x18)
#define IPMB_NETFN_GET_DEV_SDR (0x10)
#define IPMB_NETFN_GET_SENSOR_R (0x10)
#define IPMB_NETFN_SET_FRU_ACT (0xB0)
#define IPMB_NETFN_FRU_STATE_EVENT_MSG (0x14)
#define IPMB_NETFN_SET_START_DEL (0x18)
#define IPMB_NETFN_GET_START_DEL (0x18)

// send
#define NETFN_RSP_DEV_ID (0x1C)
#define NETFN_RSP_SENSOR_R (0x14)
#define NETFN_RSP_DEV_SDR (0x14)
#define NETFN_RSP_SET_FRU (0xB4)
#define NETFN_RSP_GET_FRU (0x14)
#define NETFN_RSP_SET_DELAY (0x1C)
#define NETFN_RSP_GET_DELAY (0x1C)

#define IPMB_CMD_GET_DEV_ID (0x01)
#define IPMB_CMD_GET_DEV_SDR (0x21)
#define IPMB_CMD_GET_SENSOR_R (0x2D)
#define IPMB_CMD_SET_FRU_ACT (0x0C)
#define IPMB_CMD_FRU_STATE_SENSOR_EVENT_MSG (0x02)
#define IPMB_CMD_SET_START_DEL (0x18)
#define IPMB_CMD_GET_START_DEL (0x19)

#define MAX_MSG_LEN (240U)
#define IPMB_REQUEST_LEN_MIN (7U)
#define IPMB_MSG_PAYLOAD_LEN_MAX (MAX_MSG_LEN - IPMB_REQUEST_LEN_MIN - 1)

// #define IPMB_MASTER_ADDR (0xCD) //背板
// #define IPMB_SLAVE_ADDR (0x88)  //主板

#define FMC_START_DELAY_ADDRESS (0x082D0000ul)

/*! Memory clear */
#define MEM_ZERO_STRUCT(x)                     \
    do                                         \
    {                                          \
        memset((void *)&(x), 0l, (sizeof(x))); \
    } while (0)


#define WARN_NUM (9)    //sensor num

#define PACKET_LEN (7)  //数据包固定长度

//0:32所  1：莱斯
#define LAISI_OR_32 1


typedef enum
{
    CHECKSUM_OK = 0,
    CHECKSUM1_ERR,
    CHECKSUM2_ERR

} CHECK_SUM;

#if LAISI_OR_32
//莱斯
static uint8_t ipmb_addr[11] = {
    0x80, // GAP、GP3、GP2、GA1、GA0 : 无
    0x82, // 0B 00001
    0x84, // 0B 00010
    0x86, // 0B 00011
    0x88, // 0B 00100
    0x8A, // 0B 00101
    0x8C, // 0B 00110
    0x8E, // 0B 00111
    0x90, // 0B 01000
    0x92, // 0B 01001
    0x80, // 0B 11010
};

//槽个数为11个
typedef enum
{
    SLOT1 = 1,
    SLOT2,
    SLOT3,
    SLOT4,
    SLOT5,
    SLOT6,
    SLOT7,
    SLOT8,
    SLOT9,
    SLOT10,
} IPMB_SLOT_TYPE;
#else
//32所
static uint8_t ipmb_addr[11] = {
    0xC0,   //slot0:风扇
    0x80,   //slot1:电源1           111110
    0x82,   //slot2:电源2           111101
    0x84,   //slot3:串口 BLADE10    110100
    0x86,   //slot4:计算 BLADE9     011001
    0x88,   //slot5:计算 BLADE8     011010
    0x20,   //slot6:主控 BLADE7     0X0101
    0x22,   //slot7:备主控 BLADE6   1X1000
    0x8a,   //slot8:计算 BLADE5     111011
    0x8c,   //slot9:计算 BLADE4     011100
};

//槽位
typedef enum
{
    SLOT0 = 0,
    SLOT1 = 1,
    SLOT2,
    SLOT3,
    SLOT4,
    SLOT5,
    SLOT6,
    SLOT7,
    SLOT8,
    SLOT9,
} IPMB_SLOT_TYPE;
#endif

typedef enum
{
    TEMP_INLET = 1,
    TEMP_OUTLET = 2,
    TEMP_CPU = 3,
    CPU_12V = 4,
    CPU_3V3 = 5,
    BOARD_5V = 6,
    BOARD_3V3 = 7,
    MXM_12V = 8,
    VPX_A = 9,  
} SENSOR_M;

typedef enum
{
    M0 = 0,
    M1 = 1,
    M4 = 4,
    M6 = 6,
    M7 = 7,
} M_U;

typedef struct
{
    uint8_t id_l; //读取记录ID  //sensor id
    uint8_t id_h;

    uint8_t sensor_num;
    uint8_t sensor_type; // 1：温度，2：电压，3：电流，4：风扇
    uint8_t sensor_unit; // 1:摄氏度，4：伏特，5：安培

} SDR_type_t;

typedef struct
{
    uint8_t rspAddr;                           // 01h 响应地址
    uint8_t netFn;                             // 02h 网络功能码
    uint8_t checksum1;                         // 03h 头部校验码
    uint8_t reqAddr;                           // 04h 请求地址
    uint8_t rqSeq;                             // 05h 请求序列号
    uint8_t cmd;                               // 06h 命令码
    uint8_t payload[IPMB_MSG_PAYLOAD_LEN_MAX]; //.....校验码
    uint8_t len;
} IpmbMsg;

IpmbMsg *ipmb_rsp_test_msg_init(uint8_t dataLen);
void ipmb_calc_checksum(IpmbMsg *msg);
uint8_t ipmb_checksum_check(IpmbMsg *msg);
void ipmb_send(uint32_t i2c_periph, IpmbMsg *msg);
uint8_t ipmb_recv(uint32_t i2c_periph, uint8_t bolck_en, IpmbMsg *msg);
uint8_t getReqAddr(void); // return ipmb savle addr

void ipmb_rsp_ipmb1_thread(uint8_t uxPriority);
void ipmb_rsp_ipmb2_thread(uint8_t uxPriority);
void ipmb_req_thread(uint8_t uxPriority);

void ipmb_print_msg(IpmbMsg *msg);
uint8_t ipmb_calc_rsp_voltage_value(float value);
uint8_t ipmb_calc_rsp_temp_value(float value);

#endif /* __IPMB_H__ */
