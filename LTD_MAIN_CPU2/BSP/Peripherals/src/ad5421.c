#include "ad5421.h"
#include "spi.h"
#include "stdio.h"
#include "main.h"
#include "system_parameter.h"
/* static int32_t SetCurrent(double current); */

static inline void AD5421_CS_LOW(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 执行AD5421 模拟输出中的 AD5421_CS_HIGH 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static inline void AD5421_CS_HIGH(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_SET);
}

#define AD5421_SPI_TIMEOUT_MS 10U

static volatile uint32_t ad5421_fault_flags = 0U;
static volatile uint32_t ad5421_fault_register = 0U;
static uint8_t ad5421_last_rx_data[3] = {0x00u, 0x00u, 0x00u};
/*
 * 函数用途：向 AD5421 写入 24 位寄存器数据。
 * 调用场景：AD5421 初始化、复位和 DAC 电流更新。
 * 关键约束：通过 SPI3 阻塞发送，带有限超时，不应在中断中调用。
 */
uint32_t AD5421_WriteReg(uint8_t reg, uint16_t value)
{
    HAL_StatusTypeDef status;
    uint8_t txData[3];

    txData[0] = reg & 0x7F;
    txData[1] = (uint8_t)((value >> 8) & 0xFFu);
    txData[2] = (uint8_t)(value & 0xFFu);

    AD5421_CS_LOW();
    status = HAL_SPI_Transmit(&hspi3, txData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();

    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_WRITE;
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    return NO_ERROR;
}

/*
 * 函数用途：从 AD5421 读取寄存器并返回通信状态。
 * 调用场景：控制寄存器回读校验和故障寄存器诊断。
 * 关键约束：通过 SPI3 阻塞收发，带有限超时，不应在中断中调用。
 */
static uint32_t AD5421_ReadRegChecked(uint8_t reg, uint16_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t commandData[3];
    uint8_t dummyData[3] = {0x00u, 0x00u, 0x00u};
    uint8_t rxData[3] = {0x00u, 0x00u, 0x00u};

    if (value == NULL) {
        return PARAM_ERROR;
    }

    commandData[0] = reg | 0x80u;
    commandData[1] = 0x00u;
    commandData[2] = 0x00u;

    AD5421_CS_LOW();
    status = HAL_SPI_Transmit(&hspi3, commandData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();
    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_READ;
        *value = 0U;
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    AD5421_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi3, dummyData, rxData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();
    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_READ;
        *value = 0U;
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    ad5421_last_rx_data[0] = rxData[0];
    ad5421_last_rx_data[1] = rxData[1];
    ad5421_last_rx_data[2] = rxData[2];
    *value = ((uint16_t)rxData[1] << 8) | rxData[2];
    return NO_ERROR;
}

/*
 * 函数用途：兼容旧接口读取 AD5421 寄存器值。
 * 调用场景：保留给历史状态读取路径。
 * 关键约束：通信失败时返回 0，详细故障通过驱动故障标志查询。
 */
uint16_t AD5421_ReadReg(uint8_t reg)
{
    uint16_t value = 0U;
    (void)AD5421_ReadRegChecked(reg, &value);
    return value;
}
/*
 * 函数用途：直接写入 AD5421 DAC 原始值。
 * 调用场景：电流换算完成后由电流设置接口调用。
 * 关键约束：不做电流范围换算，调用方必须先完成范围约束。
 */
uint32_t AD5421_SetDacRaw(uint16_t value)
{
    uint32_t ret = AD5421_WriteReg(WRITEDAC, value);
    if (ret != NO_ERROR) {
        return AD5421_WRITE_CURRENT_ERROR;
    }
    return NO_ERROR;
}

/*
 * 函数用途：按 mA 值换算并写入 AD5421 输出电流。
 * 调用场景：AO 服务需要刷新模拟电流输出时调用。
 * 关键约束：会把输入限制在 AD5421 允许的 3.2-24.0mA 范围内。
 */
uint32_t AD5421_SetCurrent(float mA)
{
    float scale;
    uint16_t dacValue;

    if (mA < 3.2f) {
        mA = 3.2f;
    }
    if (mA > 24.0f) {
        mA = 24.0f;
    }

    scale = (mA - 3.2f) / (24.0f - 3.2f);
    dacValue = (uint16_t)((scale * 65535.0f) + 0.5f);

    return AD5421_SetDacRaw(dacValue);
}

/*
 * 函数用途：按 0.01mA 单位设置 AD5421 输出电流。
 * 调用场景：AO 服务使用参数原始单位写入电流。
 * 关键约束：内部转为 mA 后复用 AD5421_SetCurrent()。
 */
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100)
{
    return AD5421_SetCurrent(((float)mA_x100) / 100.0f);
}

/*
 * 函数用途：读取 AD5421 驱动累计故障标志。
 * 调用场景：AO 运行态打包和故障分析。
 * 关键约束：只读内存状态，不访问外设。
 */
uint32_t AD5421_GetFaultFlags(void)
{
    return ad5421_fault_flags;
}

/*
 * 函数用途：读取最近一次 AD5421 READFAULT 原始值。
 * 调用场景：AO 运行态打包和故障分析。
 * 关键约束：只读缓存值，不主动刷新 AD5421。
 */
uint32_t AD5421_GetFaultRegister(void)
{
    return ad5421_fault_register;
}

/*
 * 函数用途：轮询 AD5421 故障寄存器和故障引脚。
 * 调用场景：AO 初始化和周期刷新时确认电流环/芯片状态。
 * 关键约束：会访问 SPI 和 GPIO，不应在中断中调用；未使能 AO 时上层不会调用。
 */
uint32_t AD5421_PollDiagnostics(void)
{
    uint16_t fault_reg = 0U;
    uint32_t ret;
    GPIO_PinState fault_pin;

    ret = AD5421_ReadRegChecked(READFAULT, &fault_reg);
    if (ret != NO_ERROR) {
        return AD5421_READFAULT_ERROR;
    }

    ad5421_fault_register = fault_reg;
    ad5421_fault_flags &= ~(AD5421_FAULT_FLAG_PIN | AD5421_FAULT_FLAG_STATUS);

    fault_pin = HAL_GPIO_ReadPin(AD5421_FAULT_GPIO_Port, AD5421_FAULT_Pin);
    if (fault_pin == GPIO_PIN_SET) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_PIN;
    }
    if (fault_reg != 0U) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_STATUS;
    }

    if ((ad5421_fault_flags & AD5421_FAULT_FLAG_PIN) != 0U) {
        return AD5421_FAULT_PIN_ERROR;
    }
    if ((ad5421_fault_flags & AD5421_FAULT_FLAG_STATUS) != 0U) {
        return AD5421_READFAULT_ERROR;
    }

    return NO_ERROR;
}

/* ============================== */
/* 初始化函数 */
/* ============================== */
void AD5421_Init(void)
{

}

/* ============================== */
/* 软件复位 */
/* ============================== */
void AD5421_Reset(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
    /* AD5421 模拟输出与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
    HAL_Delay(10);
}

/* ============================== */
/* 读取状态寄存器 */
/* ============================== */
uint16_t AD5421_GetStatus(void)
{
    (void)AD5421_PollDiagnostics();
    return (uint16_t)(ad5421_fault_register & 0xFFFFU);
}

/* ============================== */
/* 读取实际输出电流 (诊断值, 单位 mA) */
/* ============================== */
float AD5421_ReadCurrent(void)
{
    (void)AD5421_PollDiagnostics();
    return 0.0f;
}


/**
 * @brief 清除或复位AD5421 模拟输出中的 ResetAD5421 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void ResetAD5421(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
}
/*******************************************************
* Name    ReadControlRegister
* brief
* param   readbackcontrol:
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/
/* */
/* static void ReadControlRegister(uint16_t * readbackcontrol) */
/* { */
/* uint32_t i; */
/* uint16_t readback; */
/* */
/* for(i=0u;i<5u;i++) */
/* { */
/* SYNC = 0; */
/* DELAY30US; */
/* (void)SPI2_ReadWriteByte(READCONTROL); */
/* readback = SPI2WRITE00; */
/* readback <<= 8; */
/* readback += SPI2WRITE00; */
/* DELAY30US; */
/* SYNC = 1; */
/* DELAY30US; */
/* */
/* if((readback!=0u)&&(readback!=0xFFFFu)) */
/* { */
/* break; */
/* } */
/* } */
/* *readbackcontrol = readback; */
/* } */


/*
 * 函数用途：写入 AD5421 控制寄存器并回读校验。
 * 调用场景：AD5421 初始化时配置 SPI 看门狗模式。
 * 关键约束：会访问 SPI；回读不一致时返回专用读回错误。
 */
static uint32_t WriteControlRegister(uint16_t controldata)
{
    uint32_t ret;
    uint16_t readback = 0U;

    ret = AD5421_WriteReg(WRITECONTROL, controldata);
    if (ret != NO_ERROR) {
        printf("AD5421 control write fail: ret=0x%08lX\r\n", (unsigned long)ret);
        return AD5421_INIT_ERROR;
    }

    ret = AD5421_ReadRegChecked(READCONTROL, &readback);
    if (ret != NO_ERROR) {
        printf("AD5421 control read fail: ret=0x%08lX\r\n", (unsigned long)ret);
        return AD5421_INIT_ERROR;
    }

    if (readback != controldata) {
        printf("AD5421 control readback mismatch: write=0x%04X read=0x%04X rx=%02X %02X %02X\r\n",
               (unsigned int)controldata,
               (unsigned int)readback,
               (unsigned int)ad5421_last_rx_data[0],
               (unsigned int)ad5421_last_rx_data[1],
               (unsigned int)ad5421_last_rx_data[2]);
        ad5421_fault_flags |= AD5421_FAULT_FLAG_READBACK;
        return AD5421_READBACK_ERROR;
    }

    return NO_ERROR;
}

/*******************************************************
* Name    CalculateCurrentCRC
* brief
* param   data:
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/

/* static uint16_t CalculateCurrentCRC(uint32_t data) */
/* { */
/* uint32_t divisor = 0x107u; */
/* uint32_t res; */
/* uint32_t bitcount = 23u; */
/* uint32_t fir; */
/* */
/* for(;bitcount!=0u;bitcount--) */
/* { */
/* fir = ((data>>bitcount)&0x100u); */
/* if(fir!=0u) */
/* { */
/* break; */
/* } */
/* } */
/* */
/* res = (data>>bitcount)^divisor; */
/* for(;bitcount!=0u;bitcount--) */
/* { */
/* res = (res<<1u)+((data>>(bitcount-1u))&0x01u); */
/* if((res&0x100u)!=0u) */
/* { */
/* res ^= divisor; */
/* } */
/* } */
/* */
/* return (uint16_t )res; */
/* } */

/*
 * 函数用途：复位并初始化 AD5421，建立 AO 驱动可用状态。
 * 调用场景：AO 输出使能后首次刷新或初始化时调用。
 * 关键约束：会访问 SPI/GPIO 并读取诊断状态，不应在中断中调用。
 */
uint32_t Ad5421Init(void)
{
    uint32_t ret;

    ad5421_fault_flags = 0U;
    ad5421_fault_register = 0U;
    ResetAD5421();

    ret = WriteControlRegister(CUR_SPIOFF_COMMAND);
    if (ret != NO_ERROR) {
        return ret;
    }

    return AD5421_PollDiagnostics();
}

/*******************************************************
* Name    CurrentSelfTest
* brief
* param   None
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/
/* */
/* static uint32_t CurrentSelfTest(void) */
/* { */
/* uint32_t ret; */
/* if(FAULT_5421!=0u) */
/* { */
/* ReadFaultRegister();/ *read fault register* / */
/* } */
/* */
/* if(FAULT_5421!=0u) */
/* { */
/* ret = 1; */
/* (void)printf("AD5421 alarm!\r\n"); */
/* } */
/* else */
/* { */
/* ret = NO_ERROR; */
/* } */
/* */
/* return ret; */
/* } */




