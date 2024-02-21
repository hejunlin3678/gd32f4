#include <stdio.h>
#include <string.h>

#include "task.h"
#include "ipmb/ipmb.h"
#include "i2c/hk_i2c.h"
#include "fmc/fmc.h"
#include "hk_sensor/hk_sensor.h"
#include "comm/basecpld.h"
#include "hk_common.h"
#include "spi/gd25qxx.h"
// #include "sensor_list.h"

static IpmbMsg *getDeviceID(IpmbMsg *req);
static IpmbMsg *getSensorReading(IpmbMsg *req);

static IpmbMsg *getDeviceSDR(IpmbMsg *req);
static IpmbMsg *setFruActivation(IpmbMsg *req);
static IpmbMsg *getFruStateSensorEventMsg(IpmbMsg *req);
static IpmbMsg *setBoardStartDelay(IpmbMsg *req);
static IpmbMsg *getBoardStartDelay(IpmbMsg *req);

static IpmbMsg *msgReslove(IpmbMsg *msg);

// ipmb地址，根据刀片io检测获取
extern uint32_t g_ipmb_slave_addr;
// 获取管理子卡地址
extern uint32_t ipmb_master_addr; // 获取管理子卡地址
// 上下电标志位
static uint8_t pwr_flg = 1;
// 设置启动延时标志位
static uint8_t set_delay_flag = 0;
// 板卡槽位、启动延时时间
static uint8_t write_delay_data[4] = {0,0,0,0}; 

// Event上报超时
static uint16_t fru_timeout = 0;
// cpu状态： 上电M4，断电M1
static uint8_t cpu_state[2] = {4, 4};      
// getDeviceSdr data
// SDR_type_t sdr_data;

static void ipmb_rsp_ipmb1(void); // IPMB1
static void ipmb_rsp_ipmb2(void); // IPMB2备用

void fru_req_task(void);

void ipmb_rsp_ipmb1_thread(uint8_t uxPriority)
{
    /* Create that task that handles the console itself. */
    xTaskCreate((TaskFunction_t)ipmb_rsp_ipmb1, /* The task that implements the command console*/
                "ipmb_rsp1_task",               /* Text name assigned to the task*/
                configMINIMAL_STACK_SIZE * 5,   /* The size of the stack allocated to the task*/
                NULL,                           /* The parameter is not used, so NULL is passed*/
                uxPriority,                     /* The priority allocated to the task*/
                NULL);                          /* A handle is not required, so just pass NULL*/
}

void ipmb_rsp_ipmb2_thread(uint8_t uxPriority)
{
    /* Create that task that handles the console itself. */
    xTaskCreate((TaskFunction_t)ipmb_rsp_ipmb2, /* The task that implements the command console*/
                "ipmb_rsp2_task",               /* Text name assigned to the task*/
                configMINIMAL_STACK_SIZE * 5,   /* The size of the stack allocated to the task*/
                NULL,                           /* The parameter is not used, so NULL is passed*/
                uxPriority,                     /* The priority allocated to the task*/
                NULL);                          /* A handle is not required, so just pass NULL*/
}

static void power_ops_check(IpmbMsg *req)
{
    static uint8_t pwr_op = 1;
    //写入flash的延时数据
    // uint8_t delay_data[4] = {write_delay_data[0]};
    if(set_delay_flag == 1)
    {
        flash_erase_sector(FMC_START_DELAY_ADDRESS, FMC_START_DELAY_ADDRESS + 0x04);
        flash_write(FMC_START_DELAY_ADDRESS, 4, write_delay_data);
        set_delay_flag = 0;
    }

    if (pwr_flg & pwr_op)
    {
        vTaskDelay(req->payload[1]); //todo+hjl
        bc_cpu_power_on();
        vTaskDelay(10);
        pwr_op = 0;
    }
    if (pwr_flg == 0 & pwr_op == 0)
    {
        bc_cpu_power_off();
        vTaskDelay(10);
        pwr_op = 1;
    }
}

static void ipmb_rsp_func(uint32_t i2c_periph)
{
    static uint8_t rev_len = 0;
    
    g_ipmb_slave_addr = getReqAddr(); // the init iic_salve addr depend on slot'status

    hk_i2c_init(i2c_periph);          // i2c init
    while (1)
    {
        IpmbMsg *req = (IpmbMsg *)(malloc(sizeof(IpmbMsg)));
        IpmbMsg *rsp = NULL;
        if (req == NULL)
        {
            printf("=====ERROR: ipmb_rsp_func(): malloc failed======\r\n");
            goto exit;
        }
        memset(req, 0, sizeof(IpmbMsg));

        /* IPMB RECV */
        rev_len = ipmb_recv(i2c_periph, 0, req);
        req->len = rev_len;

        if (req->len != 0)
        {
            if (!ipmb_checksum_check(req)) // 接收数据校验
            {
                rsp = msgReslove(req); // 命令识别、判断、处理
                if (rsp == NULL)
                    goto exit;
                
                ipmb_calc_checksum(rsp);    // 校验码
                ipmb_send(i2c_periph, rsp); // 发送
            }
        }

        power_ops_check(req);
    exit:
        printf("RECV<<< ");
        ipmb_print_msg(req);
    
        if (rsp->len != 0)
        {
            printf("SEND>>> ");
            ipmb_print_msg(rsp);
            printf("\r\n");
        }

        if (req)
            free(req);
        if (rsp)
            free(rsp);

        fru_req_task();
    }
}

static void ipmb_rsp_ipmb1(void)
{
    ipmb_rsp_func(IPMB1);
}

static void ipmb_rsp_ipmb2(void)
{
    ipmb_rsp_func(IPMB2);
}

static IpmbMsg *msgReslove(IpmbMsg *msg)
{
    switch (msg->netFn)
    {
    case IPMB_NETFN_GET_DEV_ID: // IPMB_NETFN_SET_START_DEL//IPMB_NETFN_GET_START_DEL
        switch (msg->cmd)
        {
        case IPMB_CMD_GET_DEV_ID:
            msg = getDeviceID(msg);
            break;
        case IPMB_CMD_SET_START_DEL:
            msg = setBoardStartDelay(msg);
            break;
        case IPMB_CMD_GET_START_DEL:
            msg = getBoardStartDelay(msg);
            break;
        default:
            msg = NULL;
            break;
        }
        break;
    case IPMB_NETFN_GET_DEV_SDR: // IPMB_NETFN_GET_SENSOR_R //
        switch (msg->cmd)
        {
        case IPMB_CMD_GET_DEV_SDR:
            msg = getDeviceSDR(msg);
            break;
        case IPMB_CMD_GET_SENSOR_R:
            msg = getSensorReading(msg);
            break;
        default:
            msg = NULL;
            break;
        }
        break;
    case IPMB_NETFN_SET_FRU_ACT:
        msg = setFruActivation(msg);
        break;
    case IPMB_NETFN_FRU_STATE_EVENT_MSG:
        msg = getFruStateSensorEventMsg(msg);
        break;
    default:
        msg = NULL;
        break;
    }

    return msg;
}

static IpmbMsg *msgPublic(IpmbMsg *req)
{
    static IpmbMsg *msg = NULL;
    msg = (IpmbMsg *)(malloc(sizeof(IpmbMsg)));
    if (msg == NULL)
    {
        printf("=====ERROR: ipmb>>msgPublic(): malloc failed======\r\n");
        return msg;
    }
    memset(msg, 0, sizeof(IpmbMsg));

    ipmb_master_addr = req->reqAddr; // 获取主IIC地址
    msg->rspAddr = req->reqAddr;     // 响应地址
    msg->reqAddr = req->rspAddr;     // 请求地址
    msg->rqSeq = req->rqSeq;         // 请求序列号
    msg->cmd = req->cmd;             // 命令码

    return msg;
}

static IpmbMsg *getDeviceID(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;
    if (rsp == NULL)
    {
        return NULL;
    }

    // 响应数据填充
    rsp->netFn = NETFN_RSP_DEV_ID; // 网络码

    /* Completion Code */
    rsp->payload[i++] = 0x00;
    /* DeviceId */
    rsp->payload[i++] = DEVICE_ID;
    /* Device Revision */
    rsp->payload[i++] = 0x81; // 1000 0001
    /* software Revision1 */
    rsp->payload[i++] = 0x91; // 1001 0001  （如版本2.5-0xD2h）
    /* software Revision2 */
    rsp->payload[i++] = 0x01; // 1.0-0x01h，1.5-0x51h
    /* IPMI Revision */
    rsp->payload[i++] = 0x51;
    /* Device Support */ // 设备支持功能，机箱管理93h(CHMC)，功能板23h(IPMC)
    rsp->payload[i++] = 0x23;
    /* 板卡厂家ID1 */ // 低字节在前
    rsp->payload[i++] = 0x01;
    /* 板卡厂家ID2 */
    rsp->payload[i++] = 0x00;
    /* 板卡厂家ID3 */
    rsp->payload[i++] = 0x00;
    /* 产品类型ID1 */
    rsp->payload[i++] = 0x10;
    /* 产品类型ID2 */
    rsp->payload[i++] = 0x00;

    rsp->len = PACKET_LEN + i;

    return rsp;
}

// sensor 超限情况
uint8_t warn_level_func(float sample, uint8_t sensor_num)
{
    uint8_t data = 0;
    if (sample > g_sensor_list[sensor_num].dangerVal)
    {
        data = 0x20;
    }
    else if (sample > g_sensor_list[sensor_num].seriousVal)
    {
        data = 0x10;
    }
    else if (sample > g_sensor_list[sensor_num].slightVal)
    {
        data = 0x08;
    }
    return data;
}

static IpmbMsg *getSensorReading(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;
    float sensor_value = 0; // 传感器值
    uint16_t sensor_id = req->payload[0];
    uint8_t warning = 0;

    if (rsp == NULL)
    {
        return NULL;
    }

    if (g_sensor_list[sensor_id].unit == 1)
    {
        sensor_value = (float)g_sensor_list[sensor_id].data.iValue;
        warning = warn_level_func(g_sensor_list[sensor_id].data.iValue, sensor_id);
    }
    else
    {
        sensor_value = g_sensor_list[sensor_id].data.fValue;
        warning = warn_level_func(g_sensor_list[sensor_id].data.fValue, sensor_id);
    }

    // 响应数据填充
    rsp->netFn = NETFN_RSP_SENSOR_R; // 网络码

    /* Completion Code */
    rsp->payload[i++] = 0x00;
    /* Command data1 */
    // printf("g_sensor_list[sensor_id].unit = [%d]\r\n", g_sensor_list[sensor_id].unit);
    if (g_sensor_list[sensor_id].unit == 1)
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(sensor_value);
    else
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(sensor_value);
    /* Command data2 */
    rsp->payload[i++] = 0x80;
    /* Command data3 */
    // rsp->payload[i++] = 0xC0; // 1100 0000 不使用[0:5]超限情况
    rsp->payload[i++] = warning;

    rsp->len = PACKET_LEN + i;

    return rsp;
}

static IpmbMsg *getDeviceSDR(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;
    // 读取记录ID  //sensor id
    uint16_t sensor_id = req->payload[2] + (req->payload[3] << 8);

    if (rsp == NULL)
    {
        return NULL;
    }
    // printf("g_sensor_list[sensor_id].unit = [%d]\r\n", g_sensor_list[sensor_id].unit);
    // 响应数据填充
    rsp->netFn = NETFN_RSP_DEV_SDR;

    /* Completion Code */
    rsp->payload[i++] = 0x00;
    /* Next Record ID1 */
    if (sensor_id < 9)
    {
        rsp->payload[i++] = req->payload[2] + 1; // LS Byte
        /* Next Record ID2 */
        rsp->payload[i++] = req->payload[3]; // MS Byte
    }
    else
    {
        rsp->payload[i++] = 0xff;
        rsp->payload[i++] = 0xff;
    }
    /* Sensor ID */
    rsp->payload[i++] = req->payload[2];
    /* Sensor ID */
    rsp->payload[i++] = req->payload[3];
    /* SDR Version */
    rsp->payload[i++] = 0x51;
    /* Record Type */
    rsp->payload[i++] = 0x01;
    /* Record Length */
    rsp->payload[i++] = 0x2B; // 后面的字节长度 43
    /* IPMB addr */
    rsp->payload[i++] = rsp->reqAddr >> 1;
    /* Sensor Owner LUN */
    rsp->payload[i++] = 0x00;
    /* Sensor Number */
    rsp->payload[i++] = sensor_id;
    /* Entity ID */
    rsp->payload[i++] = 0x0A;
    /* Entity Instance */
    rsp->payload[i++] = 0x60;
    /* Sensor Initialization */
    rsp->payload[i++] = 0x7F;
    /* Sensor Capabilities */
    rsp->payload[i++] = 0x68;
    /* Sensor Type */
    rsp->payload[i++] = g_sensor_list[sensor_id].unit;
    /* Event / Reading Type Code */
    rsp->payload[i++] = 0x00;
    /* Assertion Event Mask */
    rsp->payload[i++] = 0x80;
    /* Lower Threshold Reading Mask */
    rsp->payload[i++] = 0x0A;
    /* Deassertion Event Mask */
    rsp->payload[i++] = 0x80;
    /* Upper Threshold Reading Mask */
    rsp->payload[i++] = 0x7A;
    /* Discrete Reading Mask */
    rsp->payload[i++] = 0x38;
    /* Settable Threshold Mask, Readable Threshold Mask */
    rsp->payload[i++] = 0x38;
    /* Sensor Units 1 */
    rsp->payload[i++] = 0x00;
    /* Sensor Units 2 - Base Unit */
    rsp->payload[i++] = g_sensor_list[sensor_id].unit;
    /* Sensor Units 3 - Modifier Unit */
    rsp->payload[i++] = 0x00;
    /* Linearization */
    rsp->payload[i++] = 0x00;
    // 温度
    // y = (15*x+(-32*10))/10
    if (g_sensor_list[sensor_id].unit == 1)
    {
        /* M */
        rsp->payload[i++] = 0x0F;
        /* M, Tolerance */
        rsp->payload[i++] = 0x00;
        /* B */ // 取反+1
        rsp->payload[i++] = 0xE0;
        /* B, Accuracy */
        rsp->payload[i++] = 0xFF;
        /* Accuracy, Accuracy exp */
        rsp->payload[i++] = 0x00;
        /* R exp, B exp */
        rsp->payload[i++] = 0xF1;
    }
    else
    // 电压，电流
    // y = (25*x+(-128*10))/100
    {
        /* M */
        rsp->payload[i++] = 0x19;
        /* M, Tolerance */
        rsp->payload[i++] = 0x00;
        /* B */ // 取反+1
        rsp->payload[i++] = 0x80;
        /* B, Accuracy */
        rsp->payload[i++] = 0xFF;
        /* Accuracy, Accuracy exp */
        rsp->payload[i++] = 0x00;
        /* R exp, B exp */
        rsp->payload[i++] = 0xE1;
    }
    /* Analog characteristic flags */
    rsp->payload[i++] = 0x00;
    /* Nominal Reading */
    rsp->payload[i++] = 0x99;
    /* Normal Maximum */
    rsp->payload[i++] = 0xBC;
    /* Normal Minimum */
    rsp->payload[i++] = 0x58;
    /* Sensor Maximum Reading */
    rsp->payload[i++] = 0xFF;
    /* Sensor Minimum Reading */
    rsp->payload[i++] = 0x00;
    /* Upper non-recoverable Threshold */
    if (g_sensor_list[sensor_id].unit == 1)
    {
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].slightVal);
        /* Upper critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].seriousVal);
        /* Upper non-critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].dangerVal);
        /* Lower non-recoverable Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].lowSlightVal);
        /* Lower critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].lowSeriousVal);
        /* Lower non-critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_temp_value(g_sensor_list[sensor_id].lowDangerVal);
    }
    else
    {
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].slightVal);
        /* Upper critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].seriousVal);
        /* Upper non-critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].dangerVal);
        /* Lower non-recoverable Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].lowSlightVal);
        /* Lower critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].lowSeriousVal);
        /* Lower non-critical Threshold */
        rsp->payload[i++] = ipmb_calc_rsp_voltage_value(g_sensor_list[sensor_id].lowDangerVal);
    }
    // rsp->payload[i++] = 0x80;
    /* Positive-going Threshold Hysteresis value */
    rsp->payload[i++] = 0x05;
    /* Negative-going Threshold Hysteresis value */
    rsp->payload[i++] = 0x05;
    /* reserved */
    rsp->payload[i++] = 0x00;
    /* reserved */
    rsp->payload[i++] = 0x00;
    /* OEM */
    rsp->payload[i++] = 0x00;
    /* ID String Type / Length Code  */
    rsp->payload[i++] = strlen(g_sensor_list[sensor_id].name);

    for (uint8_t j = 0; j < strlen(g_sensor_list[sensor_id].name); j++)
        rsp->payload[i++] = g_sensor_list[sensor_id].name[j];

    rsp->len = PACKET_LEN + i;

    return rsp;
}

static IpmbMsg *setFruActivation(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;

    if (rsp == NULL)
    {
        return NULL;
    }

    // 响应数据填充
    rsp->netFn = NETFN_RSP_SET_FRU; // 网络码

    /* Completion Code */
    switch (req->payload[2])
    {
    case 0x00: // 断电
        bc_cpu_power_off();
        rsp->payload[i++] = 0x00;
        break;
    case 0x01: // 上电
        bc_cpu_power_on();
        rsp->payload[i++] = 0x00;
        break;
    case 0x02: // 复位
        bc_cpu_power_off();
        vTaskDelay(5);
        bc_cpu_power_on();
        rsp->payload[i++] = 0x00;
        break;
    default: // 未知命令
        rsp->payload[i++] = 0xC1;
        break;
    }

    /* VOS */
    rsp->payload[i++] = 0x03;

    rsp->len = PACKET_LEN + i;

    return rsp;
}

static IpmbMsg *getFruStateSensorEventMsg(IpmbMsg *req)
{
    fru_timeout = 0;
    cpu_state[1] = cpu_state[0];

    return NULL;
}

static IpmbMsg *setBoardStartDelay(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;

    if (req->payload[1] == 0)
        pwr_flg = 0;
    else   
        pwr_flg = 1;

    if (rsp == NULL)
    {
        return NULL;
    }

    // 板卡启动延时时间
    set_delay_flag = 1;
    write_delay_data[0] = req->payload[1];

    // 响应数据填充
    rsp->netFn = NETFN_RSP_SET_DELAY; // 网络码

    /* Completion Code */
    rsp->payload[i++] = 0x00;

    rsp->len = PACKET_LEN + i;

    return rsp;
}

static IpmbMsg *getBoardStartDelay(IpmbMsg *req)
{
    IpmbMsg *rsp = msgPublic(req);
    uint8_t i = 0;
    uint8_t read_data[4] = {0};
    if (rsp == NULL)
    {
        return NULL;
    }

    flash_read(FMC_START_DELAY_ADDRESS, 4, read_data);
    // 响应数据填充
    rsp->netFn = NETFN_RSP_GET_DELAY; // 网络码

    /* Completion Code */
    rsp->payload[i++] = 0x00;
    /* 槽位 */
    rsp->payload[i++] = 0x01; // 从1开始依次累加
    /* 启动延时时间 */
    rsp->payload[i++] = read_data[0]; // 板卡延迟时间0-40,0表示不开机,1表示200ms

    rsp->len = PACKET_LEN + i;

    return rsp;
}

// Fru状态check，payload上下电状态
uint8_t fru_state_check(uint16_t voltage)
{
    if (voltage > 2)
        return M4;
    return M1;
}

// Fru
void fru_req_task(void)
{
    static uint8_t send_number = 4;  // 请求序列号，依次累加
    static uint16_t cpu_last_v = 0;  // temp save cpu_12V

    // cpu上下电状态
    /* cpu_state[0]:before state,  cpu_state[1]:now state*/
    cpu_state[0] = fru_state_check((uint16_t)g_sensor_list[SENSOR_CPU3V3].data.fValue);

    /* if not response,  timeout 3s continue send*/
    fru_timeout %= 3;

    // if cpu_state is different, start timeout count
    if (cpu_state[0] != cpu_state[1])
        fru_timeout++;

    // printf("cpu_state[0]=%d cpu_state[1]=%d fru_timeout=%d\r\n",cpu_state[0], cpu_state[1], fru_timeout);
    // 事件数据1和事件数据2不一样，payload上下电状态有变
    if ((cpu_state[0] != cpu_state[1]) && (fru_timeout > 0))
    {
        uint8_t i = 0;
        IpmbMsg *reqFru = (IpmbMsg *)(malloc(sizeof(IpmbMsg)));
        if ((reqFru == NULL))
        {
            printf("=====ERROR: fru event: malloc failed======\r\n");
            return;
        }
        memset(reqFru, 0, sizeof(IpmbMsg));
        
        send_number++;
        /* 管理子卡1 SLOT5 send */
        ipmb_master_addr = ipmb_addr[SLOT5];
        reqFru->rspAddr = ipmb_master_addr; 
        reqFru->netFn = 0x10;
        reqFru->reqAddr = g_ipmb_slave_addr;
        reqFru->rqSeq = send_number;
        reqFru->cmd = 0x02;
        reqFru->payload[i++] = 0x04;
        reqFru->payload[i++] = 0xF0;
        reqFru->payload[i++] = 0x00;
        reqFru->payload[i++] = 0x6F;
        reqFru->payload[i++] = cpu_state[0];
        reqFru->payload[i++] = cpu_state[1];
        reqFru->payload[i++] = 0x00;
        reqFru->len = PACKET_LEN + i;
        ipmb_calc_checksum(reqFru); // 校验码
        ipmb_send(IPMB1, reqFru);   // 发送  
        // printf(" EVENT SLOT5 SEND>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
        
        /* 管理子卡2 SLOT6 send */
        ipmb_master_addr = ipmb_addr[SLOT6];
        reqFru->rspAddr = ipmb_master_addr;
        ipmb_calc_checksum(reqFru); // 校验码
        ipmb_send(IPMB1, reqFru);   // 发送  
        // printf("EVENT SLOT6 SEND>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
        
        if (reqFru)
            free(reqFru);

        //IPMB1 send finished;  init slave:waiting to recv
        i2c_slave_initialize(IPMB1, g_ipmb_slave_addr);
    }
}
