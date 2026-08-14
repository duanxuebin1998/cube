/*
 * AS5145.h
 *
 * Created on: Dec 12, 2024
 * Author: Duan Xuebin
 */

#ifndef AS5145_H_
/* AS5145_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define AS5145_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* AS5145 SSI总线复用SPI5，只接收4字节帧，不发送配置命令。 */
#define SSI hspi5
/* AS5145片选为PF6，低电平开始一帧，DMA完成或错误时必须恢复高电平。 */
#define SSI_CSN_PIN GPIO_PIN_6
/* AS5145 SSI 片选信号所在 GPIO 端口 GPIOF；与 SSI_CSN_PIN 配套使用，切换时必须满足传感器时序。 */
#define SSI_CSN_PORT GPIOF
/* TIM1周期中断只发起下一次SPI5 DMA接收，不在定时器ISR解析帧。 */
#define ENCODER_TIM_HANDLE htim1

/* AS5145 18bit有效载荷的解析结果；原始帧只在PendSV中转换为本结构。 */
typedef struct
{
    uint16_t angle;   /* 12bit单圈绝对角，范围0至4095。 */
    uint8_t OCF;      /* Offset Compensation Finished，0表示上电补偿未完成。 */
    uint8_t COF;      /* CORDIC Overflow，1表示磁场或运算范围异常。 */
    uint8_t LIN;      /* Linearity Alarm，1表示线性度告警证据。 */
    uint8_t MagINCn;  /* 磁场增强告警位，低有效，保留用于诊断。 */
    uint8_t MagDECn;  /* 磁场减弱告警位，低有效，保留用于诊断。 */
    uint8_t parity;   /* 帧携带的偶校验位。 */
    uint8_t parity_ok;/* 1表示本地重算结果与帧校验位一致。 */
} SSI_Data_t;

/* ENC? 使用的 AS5145 采样链诊断快照；查询只复制 RAM，不发起 SPI 访问。 */
typedef struct
{
    uint8_t raw[4];
    SSI_Data_t parsed;
    uint32_t latched_error_code;
    uint32_t last_error_code;
    uint32_t consecutive_error_code;
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t transfer_error_count;
    uint32_t disconnected_pattern_count;
    uint32_t queue_overrun_count;
    uint32_t timeout_count;
    uint32_t parity_count;
    uint32_t ocf_count;
    uint32_t cof_count;
    uint32_t lin_count;
    uint32_t position_jump_count;
    uint32_t other_error_count;
    uint32_t last_ok_tick;
    uint32_t last_bad_tick;
    uint8_t parsed_valid;
    uint8_t consecutive_bad_count;
    uint8_t recovery_good_count;
    uint8_t fault_latched;
} AS5145DiagnosticSnapshot;

extern SPI_HandleTypeDef SSI;
extern TIM_HandleTypeDef ENCODER_TIM_HANDLE;

/**
 * @brief 启动新一轮采集前原子清空队列、统计和故障锁存，然后启动TIM1并立即触发首帧。
 *
 * 定时器启动失败直接返回HAL状态，交由编码器初始化决定是否阻止测量。
 *
 * 启动接口会清空本次上电的队列、连续帧计数和故障锁存，再启动TIM1并主动读首帧。
 * 查询接口只读RAM；WaitFirstValidSample仅用于启动线程的有限等待。
 *
 * @return 返回 HAL 外设操作状态；HAL_OK 表示启动成功，HAL_BUSY 表示外设仍被占用，HAL_TIMEOUT 或 HAL_ERROR 表示超时或硬件操作失败。
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void);
/**
 * @brief 返回 AS5145 当前锁存故障或最近一次访问错误码。
 *
 * @return 存在锁存故障时返回锁存错误码，否则返回最近一次 AS5145 访问错误码；无错误时为 NO_ERROR。
 */
uint32_t AS5145_GetLastError(void);
/**
 * @brief 读取已经达到连续确认条件并锁存的编码器错误。
 *
 * @return 已锁存时返回锁存错误码；未锁存时返回 NO_ERROR。
 */
uint32_t AS5145_GetLatchedError(void);
/**
 * @brief 判断 AS5145 是否已经取得至少一个有效角度样本。
 *
 * @return true 表示 AS5145 已经取得至少一个有效角度样本；false 表示 AS5145 尚未取得至少一个有效角度样本。
 */
bool AS5145_HasValidSample(void);
/**
 * @brief 启动线程有限等待首帧；超时后优先返回已识别的编码器错误，否则返回首帧超时。
 *
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return NO_ERROR 表示启动线程有限等待首帧；超时后优先返回已识别的编码器错误，否则返回首帧超时已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t AS5145_WaitFirstValidSample(uint32_t timeout_ms);

/**
 * @brief 有界处理 SPI5 中断投递的编码器原始事件。
 *
 * @details 调用场景：最低优先级 PendSV。
 * @note 关键约束：不打印、不调用 ErrorLog、不执行停机；每轮只处理固定数量事件。
 */
void AS5145_ProcessDeferred(void);

/**
 * @brief 判断SSI延后事件是否仍有积压。
 *
 * @return true表示仍有事件或溢出证据待处理；false表示本轮已经排空。
 */
bool AS5145_HasDeferredWork(void);

/**
 * @brief 为新的顶层正式业务过程解除编码器故障锁存。
 *
 * @details 调用场景：ProcessMeasureCmd 进入正式测量初始化之前。
 * @note 关键约束：通信恢复和业务内部重试不得调用。
 */
void AS5145_ClearLatchedFaultForNewProcess(void);

/**
 * @brief 判断 AS5145 采样链路是否仍锁存故障。
 *
 * @return true 表示 AS5145 采样链路仍锁存故障；false 表示 AS5145 采样链路已不再锁存故障。
 */
bool AS5145_IsFaultLatched(void);

/**
 * @brief 取得最近原始帧、解析状态和分项错误计数的一致 RAM 快照。
 *
 * @param snapshot 诊断快照输出；不得为 NULL。
 * @return true 表示快照已复制；false 表示输出指针无效。
 */
bool AS5145_GetDiagnosticSnapshot(AS5145DiagnosticSnapshot *snapshot);

#endif