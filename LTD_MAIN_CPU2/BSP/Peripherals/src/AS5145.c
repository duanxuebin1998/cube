/*
 * AS5145.c
 *
 * 实现方案说明：
 * 1. 本文件使用 TIM1 周期触发 + SPI5 DMA 接收 的方式读取 AS5145 的 SSI 数据。
 *    每次定时器到期后，都会由 Start_Read_SSI_Data() 发起一次 4 字节 DMA 读取。
 * 2. DMA 接收完成后，在 HAL_SPI_RxCpltCallback() 中关闭片选并解析帧数据。
 *    解析内容包括角度值 angle、OCF、COF、LIN 以及 parity 校验位。
 * 3. 为了提高“编码器未插/数据线悬空”时的可检测性，程序会先判断是否收到全 0 或全 1 的空总线模式。
 *    若检测到该模式，则直接按无响应处理，不进入正常角度累计流程。
 * 4. 正常帧到达后，会清零重试计数和错误上报锁存，并清除编码器类错误码；
 *    随后调用 Update_Encoder_Count() 完成跨零点修正、累计计数和位置换算。
 * 5. 当帧校验失败、OCF 未完成、COF 溢出或 SPI/DMA 本身异常时，会进入统一错误处理：
 *    - 前 3 次错误立即重试；
 *    - 超过 3 次只打印一次错误信息，但不会停止 TIM1，也不会停止后续通信；
 *    - 后续定时器仍持续拉起通信，直到再次收到有效帧后自动恢复。
 * 6. 该实现的目标是：既能在编码器异常时给出全局错误，又不会因为一次持续异常把通信链路彻底锁死。
 */

#include <mb85rs2m.h>
#include "AS5145.h"
#include "main.h"
#include "stdio.h"
#include "encoder.h"
#include "system_parameter.h"
#include "error_log.h"
#include "motor_ctrl.h"

#define SSI_FRAME_LENGTH     4u /* AS5145 SSI 单帧读取字节数。 */
#define SSI_RETRY_LIMIT      3u /* AS5145 SSI 通信参数：SSI 重试 限值。 */

/* SSI 通信状态：
 * retry_count   记录当前连续失败次数；
 * error_reported 用于限制持续故障期间只打印一次错误日志。
 */
typedef struct {
    uint8_t retry_count;
    bool error_reported;
} SSI_State;

static SSI_State ssi_state = { .retry_count = 0, .error_reported = false };
static volatile uint32_t ssi_last_error_code = NO_ERROR; /* AS5145 编码器故障记录，供恢复、显示或日志链路使用。 */
/* 首帧有效数据锁存：定时器启动不等于位置可信，运动门控必须等到这里置位。 */
static volatile bool ssi_first_valid_sample = false; /* AS5145 编码器模块级变量，保存跨函数共享的业务状态。 */
static volatile uint32_t ssi_last_ok_tick = 0U; /* AS5145 编码器模块级变量，保存跨函数共享的业务状态。 */
static uint8_t rxData[10] = { 0 };

static uint8_t Calculate_Even_Parity(uint32_t data);
static SSI_Data_t Parse_SSI_Data(const uint8_t *rxData);
static SSI_Data_t Process_SSI_Frame(uint8_t *rx_data, GPIO_TypeDef *cs_port, uint16_t cs_pin);
static bool Check_SSI_Error_Condition(const SSI_Data_t *data);
static bool Is_SSI_DisconnectedPattern(const uint8_t *rxData);
static uint32_t Get_SSI_Error_Code(const SSI_Data_t *data);
static bool Is_Encoder_Error_Code(uint32_t error_code);
static void Recover_SSI_Bus(void);
static HAL_StatusTypeDef Start_Read_SSI_Data(void);
static void Handle_SSI_Error(const SSI_Data_t *data);
static void Report_SSI_PersistentError(const SSI_Data_t *data);
static void Print_SSI_Error(const SSI_Data_t *data);

/**
 * @brief  计算 17 位有效数据的偶校验位
 */
static uint8_t Calculate_Even_Parity(uint32_t data) {
    uint8_t count = 0;

    for (uint8_t i = 0; i < 17; i++) {
        if (data & (1u << i)) {
            count++;
        }
    }

    return ((count % 2u) == 0u) ? 0u : 1u;
}

/**
 * @brief  解析 SSI 传感器返回的数据帧
 */
static SSI_Data_t Parse_SSI_Data(const uint8_t *rxData) {
    SSI_Data_t result;
    uint32_t raw_data = (((rxData[0] & 0x7Fu) << 11) | (rxData[1] << 3) | (rxData[2] >> 5));
    uint8_t calculated_parity;

    result.angle = (raw_data >> 6) & 0x0FFFu;
    result.OCF = (raw_data >> 5) & 0x01u;
    result.COF = (raw_data >> 4) & 0x01u;
    result.LIN = (raw_data >> 3) & 0x01u;
    result.MagINCn = (raw_data >> 2) & 0x01u;
    result.MagDECn = (raw_data >> 1) & 0x01u;
    result.parity = raw_data & 0x01u;

    calculated_parity = Calculate_Even_Parity(raw_data >> 1);
    result.parity_ok = (calculated_parity == result.parity);

    return result;
}

/**
 * @brief  处理一次完整 SSI 帧
 */
static SSI_Data_t Process_SSI_Frame(uint8_t *rx_data, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    SSI_Data_t parsed_data = { 0 };

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    if (Is_SSI_DisconnectedPattern(rx_data)) {
        Handle_SSI_Error(NULL);
        return parsed_data;
    }

    parsed_data = Parse_SSI_Data(rx_data);

    /* 先处理异常边界，避免AS5145 编码器状态机带故障继续运行。 */
    if (Check_SSI_Error_Condition(&parsed_data)) {
        Handle_SSI_Error(&parsed_data);
    } else {
        ssi_state.retry_count = 0;
        ssi_state.error_reported = false;
        ssi_last_error_code = NO_ERROR;
        /* 只有完整解析且无协议错误的帧，才能作为启动后的首个可信位置。 */
        ssi_first_valid_sample = true;
        ssi_last_ok_tick = HAL_GetTick();

        /* 先处理异常边界，避免AS5145 编码器状态机带故障继续运行。 */
        if (Is_Encoder_Error_Code(g_measurement.device_status.error_code)) {
            g_measurement.device_status.error_code = NO_ERROR;
        }

        Update_Encoder_Count(parsed_data.angle);
    }

    return parsed_data;
}

/**
 * @brief  判断当前 SSI 帧是否存在协议级错误
 */
static bool Check_SSI_Error_Condition(const SSI_Data_t *data) {
    return (!data->parity_ok || !data->OCF || data->COF);
}

/**
 * @brief  判断是否为编码器断开或数据线悬空时常见的空总线模式
 */
static bool Is_SSI_DisconnectedPattern(const uint8_t *rxData) {
    return ((rxData[0] == 0x00u) && (rxData[1] == 0x00u) &&
            (rxData[2] == 0x00u) && (rxData[3] == 0x00u)) ||
           ((rxData[0] == 0xFFu) && (rxData[1] == 0xFFu) &&
            (rxData[2] == 0xFFu) && (rxData[3] == 0xFFu));
}

/**
 * @brief  将当前 SSI 异常映射成全局错误码
 */
static uint32_t Get_SSI_Error_Code(const SSI_Data_t *data) {
    if (data == NULL) {
        return ENCODER_TIMEOUT;
    }
    if (!data->parity_ok) {
        return ENCODER_PARITY_ERROR;
    }
    if (!data->OCF) {
        return ENCODER_OCF_INCOMPLETE;
    }
    if (data->COF) {
        return ENCODER_CORDIC_OVERFLOW;
    }
    if (data->LIN) {
        return ENCODER_LINEARITY_WARNING;
    }
    return ENCODER_INVALID_DATA;
}

/**
 * @brief  判断错误码是否属于编码器通信错误范围
 */
static bool Is_Encoder_Error_Code(uint32_t error_code) {
    return (error_code >= ENCODER_TIMEOUT) && (error_code <= ENCODER_FIRST_SAMPLE_TIMEOUT);
}

/**
 * @brief  SPI5 异常时恢复片选和 DMA 状态，避免总线卡死
 */
static void Recover_SSI_Bus(void) {
    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);

    if (SSI.hdmarx != NULL) {
        (void)HAL_SPI_DMAStop(&SSI);
        __HAL_DMA_DISABLE(SSI.hdmarx);
        __HAL_DMA_CLEAR_FLAG(SSI.hdmarx, DMA_FLAG_TCIF3_7);
    }
}

/**
 * @brief  统一 SSI 错误处理：有限重试，超限后只上报一次，但持续保持通信
 */
static void Handle_SSI_Error(const SSI_Data_t *data) {
    uint32_t err = Get_SSI_Error_Code(data);

    ssi_last_error_code = err;
    if (!MotorCtrl_IsPositionSourceMotor()) {
        g_measurement.device_status.error_code = err;
    }

    if (++ssi_state.retry_count <= SSI_RETRY_LIMIT) {
        /* 错误 阶段：错误重试 模块：编码器 操作：通信诊断 原因：ErrorLog_GetReasonByCode(err) 尝试：ssi_state.retry_count/SSI_RETRY_LIMIT 错误码：err 错误名：ErrorLog_GetCodeName(err) */
        ErrorLog_Retry(ERROR_LOG_MODULE_ENCODER,
                       ERROR_LOG_OP_COMM_DIAG,
                       ErrorLog_GetReasonByCode(err),
                       (uint32_t)ssi_state.retry_count,
                       SSI_RETRY_LIMIT,
                       err);
        (void)Start_Read_SSI_Data();
        return;
    }

    /* 先处理异常边界，避免AS5145 编码器状态机带故障继续运行。 */
    if (!ssi_state.error_reported) {
        ssi_state.error_reported = true;
        if (!MotorCtrl_IsPositionSourceMotor()) {
            Report_SSI_PersistentError(data);
        }
    }
}

/**
 * @brief 接收AS5145 编码器中的 HAL_SPI_RxCpltCallback 逻辑。
 *
 * @param hspi 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &SSI) {
        Process_SSI_Frame(rxData, SSI_CSN_PORT, SSI_CSN_PIN);
    }
}

/**
 * @brief 执行AS5145 编码器中的 HAL_SPI_ErrorCallback 逻辑。
 *
 * @param hspi 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &SSI) {
        Handle_SSI_Error(NULL);
    }
}

/**
 * @brief  启动一次 SSI DMA 读取
 */
static HAL_StatusTypeDef Start_Read_SSI_Data(void) {
    HAL_StatusTypeDef status;

    if (HAL_SPI_GetState(&SSI) != HAL_SPI_STATE_READY) {
        Recover_SSI_Bus();
        if (HAL_SPI_GetState(&SSI) != HAL_SPI_STATE_READY) {
            if (!MotorCtrl_IsPositionSourceMotor()) {
                ssi_last_error_code = ENCODER_TIMEOUT;
                g_measurement.device_status.error_code = ENCODER_TIMEOUT;
                printf("SPI未就绪，当前状态: %d\n", HAL_SPI_GetState(&SSI));
            }
            return HAL_BUSY;
        }
    } else if (SSI.hdmarx != NULL) {
        __HAL_DMA_DISABLE(SSI.hdmarx);
        __HAL_DMA_CLEAR_FLAG(SSI.hdmarx, DMA_FLAG_TCIF3_7);
    }

    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_Receive_DMA(&SSI, rxData, SSI_FRAME_LENGTH);
    if (status != HAL_OK) {
        Recover_SSI_Bus();
        if (!MotorCtrl_IsPositionSourceMotor()) {
            ssi_last_error_code = ENCODER_TIMEOUT;
            g_measurement.device_status.error_code = ENCODER_TIMEOUT;
            printf("SPI错误码: 0x%08lX\n", SSI.ErrorCode);
            printf("SPI DMA 启动失败, 错误码: %d\n", status);
        }
    }

    return status;
}

/**
 * @brief 执行AS5145 编码器中的 AS5145_GetLastError 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t AS5145_GetLastError(void) {
    return ssi_last_error_code;
}

/**
 * @brief 查询 AS5145 是否已经收到过首帧有效 SSI 数据。
 *
 * 该函数只读锁存状态，不阻塞，可在普通任务流程中频繁调用。
 */
bool AS5145_HasValidSample(void) {
    return ssi_first_valid_sample;
}


/**
 * @brief 等待启动后的首帧有效 SSI 数据。
 *
 * 该函数会用 HAL_Delay() 短周期轮询，只能在任务上下文调用；
 * 不能在中断回调内调用，避免阻塞 SPI/DMA 和系统调度。
 */
uint32_t AS5145_WaitFirstValidSample(uint32_t timeout_ms) {
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (ssi_first_valid_sample) {
            return NO_ERROR;
        }
        /* AS5145 编码器与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(5U);
    }

    if (ssi_first_valid_sample) {
        return NO_ERROR;
    }
    /* 先处理异常边界，避免AS5145 编码器状态机带故障继续运行。 */
    if (ssi_last_error_code != NO_ERROR) {
        return ssi_last_error_code;
    }
    return ENCODER_FIRST_SAMPLE_TIMEOUT;
}

/**
 * @brief 执行AS5145 编码器中的 HAL_TIM_PeriodElapsedCallback 逻辑。
 *
 * @param htim 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        (void)Start_Read_SSI_Data();
    }
}

/**
 * @brief  持续错误首次超限时记录一次日志
 */
static void Report_SSI_PersistentError(const SSI_Data_t *data) {
    g_measurement.device_status.error_code = Get_SSI_Error_Code(data);
    Print_SSI_Error(data);
}

/**
 * @brief  打印 SSI 错误详情
 */
static void Print_SSI_Error(const SSI_Data_t *data) {
    if (data == NULL) {
        printf("编码器 SSI 无响应或数据线悬空，错误码: 0x%08lX\r\n", (unsigned long)SSI.ErrorCode);
        return;
    }

    if (!data->parity_ok) {
        printf("编码器 SSI 偶校验失败\r\n");
    }
    if (!data->OCF) {
        printf("编码器 SSI OCF 未完成\r\n");
    }
    if (data->COF) {
        printf("编码器 SSI CORDIC 溢出\r\n");
    }
    if (data->LIN) {
        printf("编码器 SSI 线性度报警\r\n");
    }
}

/**
 * @brief 执行AS5145 编码器中的 Start_Encoder_Collection_TIM 逻辑。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void) {
    HAL_StatusTypeDef status;

    ssi_state.retry_count = 0;
    ssi_state.error_reported = false;
    ssi_last_error_code = NO_ERROR;
    ssi_first_valid_sample = false;
    ssi_last_ok_tick = 0U;
    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);

    status = HAL_TIM_Base_Start_IT(&ENCODER_TIM_HANDLE);
    if (status != HAL_OK) {
        printf("[编码器][初始化][失败] 定时器启动失败：%d\r\n", status);
        return status;
    }

    printf("[编码器][初始化][成功] 定时器已启动，已触发首帧读取\r\n");
    /* 定时周期到来前先主动读一次，缩短上电后编码器不可用窗口。 */
    (void)Start_Read_SSI_Data();
    return status;
}
