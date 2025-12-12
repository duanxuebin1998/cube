/*
 * cpu3_comm_display_params.c
 *
 *  Created on: 2025年12月10日
 *      Author: admin
 */


#include "cpu3_comm_display_params.h"
#include "usart.h"
#include <string.h>
#include "mb85rs2m.h"     // WriteMultiData / ReadMultiData
#include "my_crc.h"
#include "display_tankopera.h"
/* 这些在 usart.c 里定义 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE];
extern uint8_t UART3_RX_BUF[UART3_RX_BUF_SIZE];
extern uint8_t UART6_RX_BUF[UART6_RX_BUF_SIZE];

/* 全局实例 */
Cpu3CommAndDisplayParams g_cpu3_comm_display_params;

/* ================= CPU3 本机参数描述表 ================= */

typedef struct
{
    OperatingNumber opera_num;   /* 对应 COM_NUM_XXX */
    uint8_t        *name;        /* 中文名称（一般不用，菜单里已有） */
    int32_t         valuemin;    /* 最小值（= valuemax 时视为不校验） */
    int32_t         valuemax;    /* 最大值 */
    uint8_t         point;       /* 小数点位数 */
    uint8_t         bits;        /* 显示位数 */
    uint8_t        *unit;        /* 单位字符串，可为 NULL */
} Cpu3LocalParamDesc;

/* CPU3 本机参数元数据（只做范围/显示用，实际值在 g_cpu3_comm_display_params 里） */
static const Cpu3LocalParamDesc cpu3_local_param_table[] =
{
    /* ---------- 屏幕/界面显示类 ---------- */
    { COM_NUM_PARA_LANG,           (uint8_t*)"语言",         0,   1,    0, 1,  NULL },   /* 0:中文 1:英文 */
    { COM_NUM_SCREEN_DECIMAL,      (uint8_t*)"小数点位数",   0,   4,    0, 1,  NULL },
    { COM_NUM_SCREEN_PASSWARD,     (uint8_t*)"屏幕密码",     0, 9999,  0, 4,  NULL },
    { COM_NUM_SCREEN_OFF,          (uint8_t*)"是否息屏",     0,   1,    0, 1,  NULL },

    { COM_NUM_SCREEN_SOURCE_OIL,   (uint8_t*)"液位数据源",   0,   1,    0, 1,  NULL },   /* 0:设备 1:手输 */
    { COM_NUM_SCREEN_SOURCE_WATER, (uint8_t*)"水位数据源",   0,   1,    0, 1,  NULL },
    { COM_NUM_SCREEN_SOURCE_D,     (uint8_t*)"密度数据源",   0,   1,    0, 1,  NULL },
    { COM_NUM_SCREEN_SOURCE_T,     (uint8_t*)"温度数据源",   0,   1,    0, 1,  NULL },

    { COM_NUM_SCREEN_INPUT_OIL,    (uint8_t*)"液位手输值",   -999999, 999999, 1, 7, (uint8_t*)"mm" },
    { COM_NUM_SCREEN_INPUT_WATER,  (uint8_t*)"水位手输值",   -999999, 999999, 1, 7, (uint8_t*)"mm" },
    { COM_NUM_SCREEN_INPUT_D,      (uint8_t*)"密度手输值",   0, 2000,  3, 7, (uint8_t*)"kg/m3" },
    { COM_NUM_SCREEN_INPUT_D_SWITCH,(uint8_t*)"密度手输上传", 0,   1,  0, 1,  NULL },
    { COM_NUM_SCREEN_INPUT_T,      (uint8_t*)"温度手输值",   -500, 2000, 1, 7, (uint8_t*)"℃" },

    /* ---------- CPU3 串口配置（值一般用枚举索引） ---------- */
    /* 波特率用 0~N 索引映射到具体数值，由工具函数转换 */
    { COM_NUM_CPU3_COM1_BAUDRATE,  (uint8_t*)"COM1波特率",   0,   5,    0, 1,  NULL },
    { COM_NUM_CPU3_COM1_DATABITS,  (uint8_t*)"COM1数据位",   0,   1,    0, 1,  NULL },   /* 0:8 1:9 */
    { COM_NUM_CPU3_COM1_PARITY,    (uint8_t*)"COM1校验",     0,   2,    0, 1,  NULL },   /* 0:N 1:E 2:O */
    { COM_NUM_CPU3_COM1_STOPBITS,  (uint8_t*)"COM1停止位",   0,   1,    0, 1,  NULL },   /* 0:1 1:2 */
    { COM_NUM_CPU3_COM1_PROTOCOL,  (uint8_t*)"COM1协议",     0,   4,    0, 1,  NULL },   /* 自己定义 */

    { COM_NUM_CPU3_COM2_BAUDRATE,  (uint8_t*)"COM2波特率",   0,   5,    0, 1,  NULL },
    { COM_NUM_CPU3_COM2_DATABITS,  (uint8_t*)"COM2数据位",   0,   1,    0, 1,  NULL },
    { COM_NUM_CPU3_COM2_PARITY,    (uint8_t*)"COM2校验",     0,   2,    0, 1,  NULL },
    { COM_NUM_CPU3_COM2_STOPBITS,  (uint8_t*)"COM2停止位",   0,   1,    0, 1,  NULL },
    { COM_NUM_CPU3_COM2_PROTOCOL,  (uint8_t*)"COM2协议",     0,   4,    0, 1,  NULL },

    { COM_NUM_CPU3_COM3_BAUDRATE,  (uint8_t*)"COM3波特率",   0,   5,    0, 1,  NULL },
    { COM_NUM_CPU3_COM3_DATABITS,  (uint8_t*)"COM3数据位",   0,   1,    0, 1,  NULL },
    { COM_NUM_CPU3_COM3_PARITY,    (uint8_t*)"COM3校验",     0,   2,    0, 1,  NULL },
    { COM_NUM_CPU3_COM3_STOPBITS,  (uint8_t*)"COM3停止位",   0,   1,    0, 1,  NULL },
    { COM_NUM_CPU3_COM3_PROTOCOL,  (uint8_t*)"COM3协议",     0,   4,    0, 1,  NULL },
};

#define CPU3_LOCAL_PARAM_COUNT (sizeof(cpu3_local_param_table) / sizeof(cpu3_local_param_table[0]))


/* 判断当前操作是否 CPU3 本机参数 */
bool Cpu3Local_IsParam(OperatingNumber opera)
{
    return (opera >= COM_NUM_PARA_LOCAL_START);
}

/* ========== CPU3 通信/显示参数 <-> 菜单值 的转换 ========== */

/* 波特率索引 -> 实际波特率 */
static uint32_t Cpu3_BaudIndexToValue(int idx)
{
    switch (idx) {
    case 0: return 4800;
    case 1: return 9600;
    case 2: return 19200;
    case 3: return 38400;
    case 4: return 57600;
    case 5: return 115200;
    default: return 9600;
    }
}

/* 实际波特率 -> 索引 */
static int Cpu3_BaudValueToIndex(uint32_t baud)
{
    switch (baud) {
    case 4800:   return 0;
    case 9600:   return 1;
    case 19200:  return 2;
    case 38400:  return 3;
    case 57600:  return 4;
    case 115200: return 5;
    default:     return 1; /* 默认 9600 */
    }
}

/* 读取 CPU3 本机参数当前值（统一入口） */
static int32_t Cpu3Local_ReadValue(OperatingNumber opera)
{
    switch (opera)
    {
    /* 界面显示类 / 基本信息 */
    case COM_NUM_PARA_LANG:
        return g_cpu3_comm_display_params.language;

    case COM_NUM_SCREEN_DECIMAL:
        return g_cpu3_comm_display_params.screen_decimal;

    case COM_NUM_SCREEN_PASSWARD:
        return g_cpu3_comm_display_params.screen_password;

    case COM_NUM_SCREEN_OFF:
        return g_cpu3_comm_display_params.screen_off_time;

    /* 数据源选择 */
    case COM_NUM_SCREEN_SOURCE_OIL:
        return g_cpu3_comm_display_params.screen_source_oil;

    case COM_NUM_SCREEN_SOURCE_WATER:
        return g_cpu3_comm_display_params.screen_source_water;

    case COM_NUM_SCREEN_SOURCE_D:
        return g_cpu3_comm_display_params.screen_source_d;

    case COM_NUM_SCREEN_SOURCE_T:
        return g_cpu3_comm_display_params.screen_source_t;

    /* 手工输入 */
    case COM_NUM_SCREEN_INPUT_OIL:
        return g_cpu3_comm_display_params.screen_input_oil;

    case COM_NUM_SCREEN_INPUT_WATER:
        return g_cpu3_comm_display_params.screen_input_water;

    case COM_NUM_SCREEN_INPUT_D:
        return g_cpu3_comm_display_params.screen_input_d;

    case COM_NUM_SCREEN_INPUT_D_SWITCH:
        return g_cpu3_comm_display_params.screen_input_d_switch;

    case COM_NUM_SCREEN_INPUT_T:
        return g_cpu3_comm_display_params.screen_input_t;

    /* COM1 */
    case COM_NUM_CPU3_COM1_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com1.baudrate);
    case COM_NUM_CPU3_COM1_DATABITS:
        return (g_cpu3_comm_display_params.com1.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM1_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com1.parity;
    case COM_NUM_CPU3_COM1_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com1.stopbits;
    case COM_NUM_CPU3_COM1_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com1.protocol;

    /* COM2 */
    case COM_NUM_CPU3_COM2_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com2.baudrate);
    case COM_NUM_CPU3_COM2_DATABITS:
        return (g_cpu3_comm_display_params.com2.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM2_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com2.parity;
    case COM_NUM_CPU3_COM2_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com2.stopbits;
    case COM_NUM_CPU3_COM2_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com2.protocol;

    /* COM3 */
    case COM_NUM_CPU3_COM3_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com3.baudrate);
    case COM_NUM_CPU3_COM3_DATABITS:
        return (g_cpu3_comm_display_params.com3.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM3_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com3.parity;
    case COM_NUM_CPU3_COM3_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com3.stopbits;
    case COM_NUM_CPU3_COM3_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com3.protocol;

    default:
        return 0;
    }
}

/* 写 CPU3 本机参数（统一入口） */
static void Cpu3Local_WriteValue(OperatingNumber opera, int32_t v)
{
    switch (opera)
    {
    /* 界面显示类 */
    case COM_NUM_PARA_LANG:
        g_cpu3_comm_display_params.language = (uint8_t)v;
        break;

    case COM_NUM_SCREEN_DECIMAL:
        g_cpu3_comm_display_params.screen_decimal = (uint8_t)v;
        break;

    case COM_NUM_SCREEN_PASSWARD:
        g_cpu3_comm_display_params.screen_password = (uint16_t)v;
        break;

    case COM_NUM_SCREEN_OFF:
        g_cpu3_comm_display_params.screen_off_time = (uint8_t)v;
        break;

    case COM_NUM_SCREEN_SOURCE_OIL:
        g_cpu3_comm_display_params.screen_source_oil = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_WATER:
        g_cpu3_comm_display_params.screen_source_water = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_D:
        g_cpu3_comm_display_params.screen_source_d = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_T:
        g_cpu3_comm_display_params.screen_input_d = (uint8_t)v;
        break;

    case COM_NUM_SCREEN_INPUT_OIL:
        g_cpu3_comm_display_params.screen_input_oil = v;
        break;
    case COM_NUM_SCREEN_INPUT_WATER:
        g_cpu3_comm_display_params.screen_input_water = v;
        break;
    case COM_NUM_SCREEN_INPUT_D:
        g_cpu3_comm_display_params.screen_input_d = v;
        break;
    case COM_NUM_SCREEN_INPUT_D_SWITCH:
        g_cpu3_comm_display_params.screen_input_d_switch = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_INPUT_T:
        g_cpu3_comm_display_params.screen_input_t = v;
        break;

    /* COM1 */
    case COM_NUM_CPU3_COM1_BAUDRATE:
        g_cpu3_comm_display_params.com1.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM1_DATABITS:
        g_cpu3_comm_display_params.com1.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM1_PARITY:
        g_cpu3_comm_display_params.com1.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM1_STOPBITS:
        g_cpu3_comm_display_params.com1.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM1_PROTOCOL:
        g_cpu3_comm_display_params.com1.protocol = (ComProtocolType)v;
        break;

    /* COM2 */
    case COM_NUM_CPU3_COM2_BAUDRATE:
        g_cpu3_comm_display_params.com2.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM2_DATABITS:
        g_cpu3_comm_display_params.com2.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM2_PARITY:
        g_cpu3_comm_display_params.com2.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM2_STOPBITS:
        g_cpu3_comm_display_params.com2.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM2_PROTOCOL:
        g_cpu3_comm_display_params.com2.protocol = (ComProtocolType)v;
        break;

    /* COM3 */
    case COM_NUM_CPU3_COM3_BAUDRATE:
        g_cpu3_comm_display_params.com3.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM3_DATABITS:
        g_cpu3_comm_display_params.com3.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM3_PARITY:
        g_cpu3_comm_display_params.com3.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM3_STOPBITS:
        g_cpu3_comm_display_params.com3.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM3_PROTOCOL:
        g_cpu3_comm_display_params.com3.protocol = (ComProtocolType)v;
        break;

    default:
        break;
    }

    /* 这里可以顺手：重配串口 + 保存 FRAM */
    // Cpu3_Comm_ReInitAll();       // 你自己实现
    // Cpu3_SaveParamsToFRAM();     // 你自己实现
}


/* ================ 内部小工具：用一个 ComPortConfig 初始化一个 UART ================ */
static void Cpu3_ReinitOneUart(UART_HandleTypeDef *huart, const ComPortConfig *cfg)
{
    if (!huart || !cfg) return;

    HAL_UART_DeInit(huart);

    huart->Init.BaudRate = cfg->baudrate;
    huart->Init.WordLength = (cfg->databits == 9) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    /* 校验 */
    switch (cfg->parity) {
    case COM_PARITY_EVEN: huart->Init.Parity = UART_PARITY_EVEN; break;
    case COM_PARITY_ODD:  huart->Init.Parity = UART_PARITY_ODD;  break;
    case COM_PARITY_NONE:
    default:              huart->Init.Parity = UART_PARITY_NONE; break;
    }

    /* 停止位 */
    switch (cfg->stopbits) {
    case COM_STOPBITS_2: huart->Init.StopBits = UART_STOPBITS_2; break;
    case COM_STOPBITS_1:
    default:             huart->Init.StopBits = UART_STOPBITS_1; break;
    }

    huart->Init.Mode       = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================== 初始化默认值 ========================== */
void Cpu3_Params_InitDefaults(void)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));

    /* 屏幕基本信息 */
    g_cpu3_comm_display_params.local_led_version = 0x0100;
    g_cpu3_comm_display_params.language          = 0;

    /* 数据源默认 */
    g_cpu3_comm_display_params.screen_source_oil   = 0;
    g_cpu3_comm_display_params.screen_source_water = 0;
    g_cpu3_comm_display_params.screen_source_d     = 0;
    g_cpu3_comm_display_params.screen_source_t     = 0;

    /* 显示类默认 */
    g_cpu3_comm_display_params.screen_decimal  = 2;

    /* ========== 串口默认：保持和你现在 usart.c 一致 ========== */

    /* COM1 = USART6: 4800 8N1 DSM */
    g_cpu3_comm_display_params.com1.baudrate = 4800;
    g_cpu3_comm_display_params.com1.databits = 8;
    g_cpu3_comm_display_params.com1.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com1.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com1.protocol = COM_PROTO_DSM;

    /* COM2 = USART2: 4800 8N1 Modbus RTU */
    g_cpu3_comm_display_params.com2.baudrate = 4800;
    g_cpu3_comm_display_params.com2.databits = 8;
    g_cpu3_comm_display_params.com2.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com2.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com2.protocol = COM_PROTO_MODBUS_RTU;

    /* COM3 = USART3: 4800 8N2 Wartsila */
    g_cpu3_comm_display_params.com3.baudrate = 4800;
    g_cpu3_comm_display_params.com3.databits = 8;
    g_cpu3_comm_display_params.com3.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com3.stopbits = COM_STOPBITS_2;
    g_cpu3_comm_display_params.com3.protocol = COM_PROTO_WARTSILA;
}

/* ========================== 重配全部串口 ========================== */
void Cpu3_ReinitAllUarts(void)
{
    /* COM1 = USART6 */
    Cpu3_ReinitOneUart(&huart6, &g_cpu3_comm_display_params.com1);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);

    /* COM2 = USART2 */
    Cpu3_ReinitOneUart(&huart2, &g_cpu3_comm_display_params.com2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);

    /* COM3 = USART3 */
    Cpu3_ReinitOneUart(&huart3, &g_cpu3_comm_display_params.com3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
}

/* ==================== FRAM 存储结构定义 ==================== */
/*
 * 为了兼容后续版本，我们在 FRAM 里存储的是一个带头的结构：
 *  - magic   : 固定魔数，用来判断这个区域是不是 Cpu3 的参数
 *  - version : 版本号，后续结构体变大时可以用来做兼容
 *  - params  : 当前的 Cpu3CommAndDisplayParams
 *  - crc     : 对上述 (magic, version, params) 计算 CRC32
 */

#define CPU3_PARAM_MAGIC   0x43505533UL   // 'CPU3'
#define CPU3_PARAM_VERSION 0x0001U

typedef struct
{
    uint32_t                 magic;
    uint16_t                 version;
    uint16_t                 reserved;  // 对齐/预留
    Cpu3CommAndDisplayParams params;
    uint32_t                 crc;
} Cpu3ParamStorage;

void Cpu3_Params_SaveToFRAM(void)
{
    Cpu3ParamStorage stor;

    stor.magic   = CPU3_PARAM_MAGIC;
    stor.version = CPU3_PARAM_VERSION;
    stor.reserved = 0;
    stor.params  = g_cpu3_comm_display_params;

    /* 计算 CRC：不包含最后的 crc 字段本身 */
    uint32_t crc_len = sizeof(Cpu3ParamStorage) - sizeof(stor.crc);
    stor.crc = CRC32_HAL((uint8_t*)&stor, crc_len);

    WriteMultiData((uint8_t*)&stor, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));

    printf("Cpu3 params saved to FRAM. CRC=0x%08lX\r\n", (unsigned long)stor.crc);
}

void Cpu3_Params_LoadFromFRAM(void)
{
    Cpu3ParamStorage stor;
    memset(&stor, 0, sizeof(stor));

    ReadMultiData((uint8_t*)&stor, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));

    uint8_t use_default = 0;

    /* 检查 magic & version */
    if ((stor.magic != CPU3_PARAM_MAGIC) || (stor.version != CPU3_PARAM_VERSION)) {
        printf("Cpu3 FRAM params invalid magic/version. Use defaults.\r\n");
        use_default = 1;
    } else {
        /* 校验 CRC */
        uint32_t crc_len = sizeof(Cpu3ParamStorage) - sizeof(stor.crc);
        uint32_t crc_calc = CRC32_HAL((uint8_t*)&stor, crc_len);

        if (crc_calc != stor.crc) {
            printf("Cpu3 FRAM params CRC mismatch. FRAM=0x%08lX, CALC=0x%08lX\r\n",
                   (unsigned long)stor.crc, (unsigned long)crc_calc);
            use_default = 1;
        }
    }

    if (use_default) {
        /* 使用默认值并立刻写回 FRAM */
        Cpu3_Params_InitDefaults();
        Cpu3_Params_SaveToFRAM();
    } else {
        /* 正常加载 */
        g_cpu3_comm_display_params = stor.params;
        printf("Cpu3 params loaded from FRAM. CRC=0x%08lX\r\n", (unsigned long)stor.crc);
    }
}

