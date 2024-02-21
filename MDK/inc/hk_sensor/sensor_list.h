
#include "hk_sensor/hk_sensor.h"

Sensor_t g_sensor_list[16] = {
    {
        .name = "MXM_12V0",
        .unit = 2,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "CPU_12V0",
        .unit = 2,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "CPU_3V3",
        .unit = 2,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "BOARD_5V0",
        .unit = 2,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "BOARD_3V3",
        .unit = 2,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "BOARD_WAVE",
        .unit = 3,
        .lowSlightVal = 0,
        .lowSeriousVal = 0,
        .lowDangerVal = 0,
    },
    {
        .name = "Inlet_Temp",
        .unit = 1,
        .lowSlightVal = -18,
        .lowSeriousVal = -20,
        .lowDangerVal = -25,
    },
    {
        .name = "Outlet_Temp",
        .unit = 1,
        .lowSlightVal = -18,
        .lowSeriousVal = -20,
        .lowDangerVal = -25,
    },
    {
        .name = "BMC_Temp",
        .unit = 1,
        .lowSlightVal = -18,
        .lowSeriousVal = -20,
        .lowDangerVal = -25,
    },
    {
        .name = "CPU_Core_Temp",
        .unit = 1,
        .lowSlightVal = -18,
        .lowSeriousVal = -20,
        .lowDangerVal = -25,
    },
    {
        .name = "CPU_Present",
        .unit = 4,
    },
    {
        .name = "MXM_Present",
        .unit = 4,
    },
    {
        .name = "AT200_1_Present",
        .unit = 4,
    },
    {
        .name = "AT200_2_Present",
        .unit = 4,
    },
    {
        .name = "mSATA_1_Present",
        .unit = 4,
    },
    {
        .name = "mSATA_2_Present",
        .unit = 4,
    },
};

