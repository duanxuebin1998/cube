/*
 * AS5145.c
 *
 * SPI5 和 DMA 中断只保存原始事件；帧解析、连续异常确认和 RAM 更新由 PendSV 有界处理。
 */

#include "AS5145.h"
#include "encoder.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include <string.h>
#include <stdio.h>

/* SPI5 DMA每次固定接收4字节，包含18bit有效SSI载荷及尾部无效位。 */
#define SSI_FRAME_LENGTH            4U
/* ISR到PendSV的环形事件队列容量；满队列也会转化为异常证据。 */
#define SSI_EVENT_QUEUE_CAPACITY    8U
/* 单次PendSV最多消费4项，限制低优先级异常处理对其它延后服务的占用。 */
#define SSI_DEFERRED_EVENT_BUDGET   4U
/* 任意类型连续3个异常事件才锁存，正常帧会打断未锁存的连续计数。 */
#define SSI_FAULT_CONFIRM_FRAMES    3U

/* ISR投递的证据类型；PendSV统一把传输异常映射为编码器超时故障。 */
typedef enum {
    SSI_EVENT_FRAME = 0,   /* DMA收到一份完整原始帧。 */
    SSI_EVENT_SPI_ERROR,   /* HAL SPI错误回调携带的错误证据。 */
    SSI_EVENT_START_ERROR  /* 定时触发时SPI/DMA无法开始新接收。 */
} SSI_EventType;

/* 固定长度事件邮箱项；ISR只填充，不解析业务字段。 */
typedef struct {
    uint8_t raw[SSI_FRAME_LENGTH]; /* 完整原始4字节，错误事件允许为全0。 */
    uint8_t type;                  /* SSI_EventType的紧凑存储值。 */
    uint8_t hal_status;            /* HAL_OK/BUSY/ERROR的现场状态。 */
    uint16_t reserved;             /* 固定清零，为结构对齐和后续扩展预留。 */
    uint32_t hal_error;            /* SPI句柄ErrorCode快照。 */
    uint32_t sample_tick;          /* ISR投递时HAL毫秒时基。 */
    uint32_t sequence;             /* 单调事件序号，用于新流程丢弃旧队列证据。 */
} SSI_Event;

/*
 * 队列由SPI/TIM中断生产、PendSV消费；头尾和计数的复合更新均在短临界区完成。
 */
static volatile uint8_t ssi_queue_head = 0U; /* 下一项写入索引。 */
static volatile uint8_t ssi_queue_tail = 0U; /* 下一项读取索引。 */
static volatile uint8_t ssi_queue_count = 0U; /* 当前已发布且未消费的事件数。 */
static SSI_Event ssi_event_queue[SSI_EVENT_QUEUE_CAPACITY]; /* 固定环形事件存储。 */
static volatile uint32_t ssi_event_sequence = 0U; /* 每次成功入队递增的事件序号。 */
static volatile uint32_t ssi_process_start_sequence = 0U; /* 新正式流程允许处理的序号边界。 */
static volatile uint32_t ssi_frame_count = 0U; /* 本次采集收到的完整帧总数。 */
static volatile uint32_t ssi_error_count = 0U; /* 传输错误和队列溢出证据总数。 */
static volatile uint32_t ssi_queue_overrun_count = 0U; /* 因队列满而丢失的事件数。 */
static uint32_t ssi_processed_overrun_count = 0U; /* 已转换为故障证据的溢出数。 */

/* 连续异常、锁存和首帧状态只在PendSV更新，查询及清锁存路径可能在线程态读取。 */
static volatile uint8_t ssi_consecutive_bad_count = 0U; /* 尚未锁存时的连续异常数。 */
static volatile uint8_t ssi_recovery_good_count = 0U; /* 锁存后观察到的正常帧数，仅供诊断。 */
static volatile bool ssi_fault_latched = false; /* 锁存后正常帧不自动清除。 */
static volatile uint32_t ssi_latched_error_code = NO_ERROR; /* 第3个连续异常对应的锁存码。 */
static volatile uint32_t ssi_last_error_code = NO_ERROR; /* 最近一次未锁存异常或锁存码。 */
static volatile bool ssi_first_valid_sample = false; /* 本次采集是否至少收到一帧有效数据。 */
static volatile uint32_t ssi_last_ok_tick = 0U; /* 最近有效帧处理时刻，单位ms。 */
static uint8_t rxData[SSI_FRAME_LENGTH] = {0U}; /* SPI5 DMA当前接收缓冲区。 */

static uint8_t Calculate_Even_Parity(uint32_t data);
static SSI_Data_t Parse_SSI_Data(const uint8_t *raw);
static bool Check_SSI_Error_Condition(const SSI_Data_t *data);
static bool Is_SSI_DisconnectedPattern(const uint8_t *raw);
static uint32_t Get_SSI_Error_Code(const SSI_Data_t *data);
static void Recover_SSI_Bus(void);
static HAL_StatusTypeDef Start_Read_SSI_Data(void);
static void SSI_ProcessError(uint32_t error_code);
static void SSI_ProcessFrame(const uint8_t *raw);

/* 挂起最低优先级PendSV，并用屏障保证事件内容先于挂起请求对处理器可见。 */
static void SSI_PendDeferred(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

/*
 * 函数用途：从硬件中断向编码器事件邮箱投递一项原始证据。
 * 调用场景：SPI5 DMA 完成、SPI5 错误和 TIM1 启动失败路径。
 * 关键约束：只复制固定长度数据和计数，不打印、不解析、不访问 FRAM。
 */
static void SSI_EnqueueEvent(SSI_EventType type,
                             const uint8_t *raw,
                             HAL_StatusTypeDef hal_status,
                             uint32_t hal_error)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t head;
    uint8_t next;
    SSI_Event *event;

    __disable_irq();
    head = ssi_queue_head;
    next = (uint8_t)((head + 1U) % SSI_EVENT_QUEUE_CAPACITY);
    if (ssi_queue_count >= SSI_EVENT_QUEUE_CAPACITY) {
        ssi_queue_overrun_count++;
        ssi_error_count++;
        if (primask == 0U) {
            __enable_irq();
        }
        SSI_PendDeferred();
        return;
    }

    event = &ssi_event_queue[head];
    if (raw != NULL) {
        memcpy(event->raw, raw, SSI_FRAME_LENGTH);
    } else {
        memset(event->raw, 0, SSI_FRAME_LENGTH);
    }
    event->type = (uint8_t)type;
    event->hal_status = (uint8_t)hal_status;
    event->reserved = 0U;
    event->hal_error = hal_error;
    event->sample_tick = HAL_GetTick();
    event->sequence = ++ssi_event_sequence;
    if (type == SSI_EVENT_FRAME) {
        ssi_frame_count++;
    } else {
        ssi_error_count++;
    }
    __DMB();
    ssi_queue_head = next;
    ssi_queue_count++;
    if (primask == 0U) {
        __enable_irq();
    }
    SSI_PendDeferred();
}

/*
 * 在短临界区取出一项已发布事件；复制完成后才推进尾指针，避免ISR覆盖未复制内容。
 */
static bool SSI_DequeueEvent(SSI_Event *event)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t tail;

    __disable_irq();
    tail = ssi_queue_tail;
    if (ssi_queue_count == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }

    *event = ssi_event_queue[tail];
    __DMB();
    ssi_queue_tail = (uint8_t)((tail + 1U) % SSI_EVENT_QUEUE_CAPACITY);
    ssi_queue_count--;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

/* 统计17个数据/状态位中的置位数，返回AS5145帧要求的偶校验位。 */
static uint8_t Calculate_Even_Parity(uint32_t data)
{
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 17U; index++) {
        if ((data & (1UL << index)) != 0U) {
            count++;
        }
    }
    return ((count % 2U) == 0U) ? 0U : 1U;
}

/* 按AS5145 SSI位序从4字节缓冲拼出18bit载荷并解析角度、状态和校验结果。 */
static SSI_Data_t Parse_SSI_Data(const uint8_t *raw)
{
    SSI_Data_t result;
    uint32_t raw_data =
        (((uint32_t)(raw[0] & 0x7FU) << 11) |
         ((uint32_t)raw[1] << 3) |
         ((uint32_t)raw[2] >> 5));

    result.angle = (uint16_t)((raw_data >> 6) & 0x0FFFU);
    result.OCF = (uint8_t)((raw_data >> 5) & 0x01U);
    result.COF = (uint8_t)((raw_data >> 4) & 0x01U);
    result.LIN = (uint8_t)((raw_data >> 3) & 0x01U);
    result.MagINCn = (uint8_t)((raw_data >> 2) & 0x01U);
    result.MagDECn = (uint8_t)((raw_data >> 1) & 0x01U);
    result.parity = (uint8_t)(raw_data & 0x01U);
    result.parity_ok = (Calculate_Even_Parity(raw_data >> 1) == result.parity) ? 1U : 0U;
    return result;
}

/* 校验、OCF和COF属于阻止位置更新的硬异常；LIN只作为独立诊断码保留。 */
static bool Check_SSI_Error_Condition(const SSI_Data_t *data)
{
    return (data->parity_ok == 0U) || (data->OCF == 0U) || (data->COF != 0U);
}

/* 全0或全FF通常表示总线悬空/短接，优先映射为超时而不是解析为有效角度。 */
static bool Is_SSI_DisconnectedPattern(const uint8_t *raw)
{
    return ((raw[0] == 0x00U) && (raw[1] == 0x00U) &&
            (raw[2] == 0x00U) && (raw[3] == 0x00U)) ||
           ((raw[0] == 0xFFU) && (raw[1] == 0xFFU) &&
            (raw[2] == 0xFFU) && (raw[3] == 0xFFU));
}

static uint32_t Get_SSI_Error_Code(const SSI_Data_t *data)
{
    if (data == NULL) {
        return ENCODER_TIMEOUT;
    }
    if (data->parity_ok == 0U) {
        return ENCODER_PARITY_ERROR;
    }
    if (data->OCF == 0U) {
        return ENCODER_OCF_INCOMPLETE;
    }
    if (data->COF != 0U) {
        return ENCODER_CORDIC_OVERFLOW;
    }
    if (data->LIN != 0U) {
        return ENCODER_LINEARITY_WARNING;
    }
    return NO_ERROR;
}

/*
 * 函数用途：处理一项延后的编码器异常。
 * 调用场景：PendSV 消费原始帧或 SPI/DMA 错误事件。
 * 关键约束：第 3 个连续异常才锁存；只发布故障快照，不打印和不执行阻塞停机。
 */
static void SSI_ProcessError(uint32_t error_code)
{
    ssi_last_error_code = error_code;
    ssi_recovery_good_count = 0U;

    if (MotorCtrl_IsPositionSourceMotor()) {
        ssi_consecutive_bad_count = 0U;
        return;
    }

    if (ssi_fault_latched) {
        FaultManager_LatchAsyncError(ssi_latched_error_code);
        return;
    }

    if (ssi_consecutive_bad_count < UINT8_MAX) {
        ssi_consecutive_bad_count++;
    }
    if (ssi_consecutive_bad_count >= SSI_FAULT_CONFIRM_FRAMES) {
        ssi_fault_latched = true;
        ssi_latched_error_code = error_code;
        FaultManager_LatchAsyncError(error_code);
    }
}

/*
 * 处理一帧原始证据：异常帧不更新累计位置；正常帧清未锁存连续计数，
 * 但已经锁存的故障只重复发布，必须由下一条顶层正式命令清除。
 */
static void SSI_ProcessFrame(const uint8_t *raw)
{
    SSI_Data_t parsed;

    if (Is_SSI_DisconnectedPattern(raw)) {
        SSI_ProcessError(ENCODER_TIMEOUT);
        return;
    }

    parsed = Parse_SSI_Data(raw);
    if (Check_SSI_Error_Condition(&parsed)) {
        SSI_ProcessError(Get_SSI_Error_Code(&parsed));
        return;
    }

    ssi_consecutive_bad_count = 0U;
    ssi_first_valid_sample = true;
    ssi_last_ok_tick = HAL_GetTick();
    if (ssi_fault_latched && (!MotorCtrl_IsPositionSourceMotor())) {
        if (ssi_recovery_good_count < UINT8_MAX) {
            ssi_recovery_good_count++;
        }
        FaultManager_LatchAsyncError(ssi_latched_error_code);
    } else {
        ssi_recovery_good_count = 0U;
        ssi_last_error_code = NO_ERROR;
    }
    Update_Encoder_Count(parsed.angle);
}

/*
 * 函数用途：按固定预算先补记队列溢出，再顺序处理原始事件。
 * 调用场景：PendSV_Handler。
 * 关键约束：超过本轮预算时重新挂起PendSV，不在单次低优先级中断无限排空。
 */
void AS5145_ProcessDeferred(void)
{
    SSI_Event event;
    uint32_t processed = 0U;

    while ((processed < SSI_DEFERRED_EVENT_BUDGET) &&
           (ssi_processed_overrun_count != ssi_queue_overrun_count)) {
        ssi_processed_overrun_count++;
        SSI_ProcessError(ENCODER_TIMEOUT);
        processed++;
    }

    while ((processed < SSI_DEFERRED_EVENT_BUDGET) && SSI_DequeueEvent(&event)) {
        if ((int32_t)(event.sequence - ssi_process_start_sequence) <= 0) {
            processed++;
            continue;
        }
        if (event.type == (uint8_t)SSI_EVENT_FRAME) {
            SSI_ProcessFrame(event.raw);
        } else {
            SSI_ProcessError(ENCODER_TIMEOUT);
        }
        processed++;
    }

    if ((ssi_queue_count != 0U) ||
        (ssi_processed_overrun_count != ssi_queue_overrun_count)) {
        SSI_PendDeferred();
    }
}

/* 仅恢复SPI5接收硬件到可重启状态；不清业务故障锁存，也不伪造正常帧。 */
static void Recover_SSI_Bus(void)
{
    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
    if (SSI.hdmarx != NULL) {
        (void)HAL_SPI_DMAStop(&SSI);
        __HAL_DMA_DISABLE(SSI.hdmarx);
        __HAL_DMA_CLEAR_FLAG(SSI.hdmarx, DMA_FLAG_TCIF3_7);
    }
}

/*
 * 发起一次4字节SPI5 DMA接收；总线忙时先作有限硬件复位，仍不可用则投递启动异常。
 * 函数可能由TIM1 ISR调用，因此不等待、不打印、不解析。
 */
static HAL_StatusTypeDef Start_Read_SSI_Data(void)
{
    HAL_StatusTypeDef status;

    if (HAL_SPI_GetState(&SSI) != HAL_SPI_STATE_READY) {
        Recover_SSI_Bus();
        if (HAL_SPI_GetState(&SSI) != HAL_SPI_STATE_READY) {
            SSI_EnqueueEvent(SSI_EVENT_START_ERROR,
                             NULL,
                             HAL_BUSY,
                             SSI.ErrorCode);
            return HAL_BUSY;
        }
    } else if (SSI.hdmarx != NULL) {
        __HAL_DMA_DISABLE(SSI.hdmarx);
        __HAL_DMA_CLEAR_FLAG(SSI.hdmarx, DMA_FLAG_TCIF3_7);
    }

    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_Receive_DMA(&SSI, rxData, SSI_FRAME_LENGTH);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
        SSI_EnqueueEvent(SSI_EVENT_START_ERROR,
                         NULL,
                         status,
                         SSI.ErrorCode);
    }
    return status;
}

/* SPI5 DMA完成回调：先释放片选，再把完整原始帧复制到事件队列。 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &SSI) {
        HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
        SSI_EnqueueEvent(SSI_EVENT_FRAME, rxData, HAL_OK, SSI.ErrorCode);
    }
}

/* SPI5错误回调：释放片选并保存HAL状态，故障确认留给PendSV。 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &SSI) {
        HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
        SSI_EnqueueEvent(SSI_EVENT_SPI_ERROR,
                         rxData,
                         HAL_ERROR,
                         SSI.ErrorCode);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        (void)Start_Read_SSI_Data();
    }
}

uint32_t AS5145_GetLastError(void)
{
    return ssi_fault_latched ? ssi_latched_error_code : ssi_last_error_code;
}

bool AS5145_HasValidSample(void)
{
    return ssi_first_valid_sample;
}

bool AS5145_IsFaultLatched(void)
{
    return ssi_fault_latched;
}

/*
 * 清除锁存时记录当前事件/溢出序号边界，避免新正式流程重新消费清锁存前的旧证据。
 */
void AS5145_ClearLatchedFaultForNewProcess(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ssi_fault_latched = false;
    ssi_latched_error_code = NO_ERROR;
    ssi_last_error_code = NO_ERROR;
    ssi_consecutive_bad_count = 0U;
    ssi_recovery_good_count = 0U;
    ssi_process_start_sequence = ssi_event_sequence;
    ssi_processed_overrun_count = ssi_queue_overrun_count;
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 启动线程有限等待首帧；超时后优先返回已识别的编码器错误，否则返回首帧超时。 */
uint32_t AS5145_WaitFirstValidSample(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (ssi_first_valid_sample) {
            return NO_ERROR;
        }
        HAL_Delay(5U);
    }
    if (ssi_first_valid_sample) {
        return NO_ERROR;
    }
    if (AS5145_GetLastError() != NO_ERROR) {
        return AS5145_GetLastError();
    }
    return ENCODER_FIRST_SAMPLE_TIMEOUT;
}

/*
 * 启动新一轮采集前原子清空队列、统计和故障锁存，然后启动TIM1并立即触发首帧。
 * 定时器启动失败直接返回HAL状态，交由编码器初始化决定是否阻止测量。
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ssi_queue_head = 0U;
    ssi_queue_tail = 0U;
    ssi_queue_count = 0U;
    ssi_event_sequence = 0U;
    ssi_frame_count = 0U;
    ssi_error_count = 0U;
    ssi_queue_overrun_count = 0U;
    ssi_processed_overrun_count = 0U;
    ssi_consecutive_bad_count = 0U;
    ssi_recovery_good_count = 0U;
    ssi_fault_latched = false;
    ssi_latched_error_code = NO_ERROR;
    ssi_last_error_code = NO_ERROR;
    ssi_first_valid_sample = false;
    ssi_last_ok_tick = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
    status = HAL_TIM_Base_Start_IT(&ENCODER_TIM_HANDLE);
    if (status != HAL_OK) {
        printf("[编码器][初始化][失败] 定时器启动失败：%d\r\n", status);
        return status;
    }

    printf("[编码器][初始化][成功] 定时器已启动，已触发首帧读取\r\n");
    (void)Start_Read_SSI_Data();
    return status;
}