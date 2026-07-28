#include "power_monitor.h"

#include "adc.h"
#include "encoder.h"
#include "error_log.h"
#include "mb85rs2m.h"
#include "system_parameter.h"

#include <stdio.h>

/* 12bit ADC 满量程计数，用于采样合法性判断和电压换算。 */
#define POWER_MONITOR_ADC_FULL_SCALE_COUNTS       4095U
/* 默认 ADC 参考电压，单位 mV；只用于调试换算，不直接决定硬件阈值。 */
#define POWER_MONITOR_VDDA_MV                     3300U
/* 100k/10k 分压的输入/ADC 电压倍率：(100k + 10k) / 10k = 11。 */
#define POWER_MONITOR_DIVIDER_RATIO_NUMERATOR     11U
/* 首次启动或局部恢复等待 DMA 写入新样本的最大时间，单位 ms。 */
#define POWER_MONITOR_SAMPLE_TIMEOUT_MS           5U
/* 超出 12bit 范围的哨兵值，用于区分“旧样本”与“DMA 已写入新样本”。 */
#define POWER_MONITOR_INVALID_ADC_SAMPLE           0xFFFFFFFFUL

/*
 * 模块状态由启动线程、ADC回调、SysTick和主循环共同访问，因此跨上下文变量使用
 * volatile；需要多字段一致性的更新由短临界区保护。
 */
static volatile uint8_t s_power_fail_active = 0U; /* 真实低压恢复闭环是否仍在进行。 */
static volatile uint16_t s_recover_stable_ms = 0U; /* 连续高于22V阈值的1ms计数。 */
static volatile uint32_t s_last_24v_adc_sample = 0U; /* 最近一次已接受的合法采样。 */
static volatile uint32_t s_power_fail_trip_count = 0U; /* 本次上电低压边沿计数。 */
static volatile uint32_t s_last_trip_adc_sample = 0U; /* 最近低压边沿的证据采样。 */
static volatile uint32_t s_power_adc_dma_sample = POWER_MONITOR_INVALID_ADC_SAMPLE; /* DMA单元素循环目标。 */
static volatile uint8_t s_motor_power_inhibited = 1U; /* 驱动使能硬门禁，默认安全关闭。 */
static volatile uint8_t s_power_monitor_ready = 0U; /* 是否已有可用于安全判断的新采样。 */
static volatile uint8_t s_emergency_persistence_armed = 0U; /* 可信位置建立后的掉电保存布防。 */
static volatile uint8_t s_emergency_request_sent = 0U; /* 当前低压事件是否已投递一次紧急保存。 */
static volatile uint32_t s_first_fault_code = NO_ERROR; /* 当前锁存周期的首个故障原因。 */
static volatile uint32_t s_latched_fault_code = NO_ERROR; /* 按优先级保留的对外故障码。 */
static volatile uint32_t s_recovery_cause_code = NO_ERROR; /* 本轮局部恢复的原始原因。 */
static volatile uint8_t s_fault_report_pending = 0U; /* 等待主循环发布故障快照。 */
static volatile uint8_t s_recovery_attempts = 0U; /* 当前原因已执行的局部恢复次数。 */
static volatile PowerMonitorState s_monitor_state = POWER_MONITOR_STATE_STOPPED; /* ADC/DMA链路状态机。 */

/*
 * 函数用途：把电源监控切入禁止运动和禁止普通 FRAM 写入的安全状态。
 * 调用场景：启动失败、真实低压以及 ADC/DMA 运行故障。
 * 关键约束：可在中断上下文调用，只写寄存器和 RAM 标志，不打印、不等待。
 */
static void PowerMonitor_ForceSafeState(void)
{
    s_motor_power_inhibited = 1U;
    FRAM_SetNormalWriteInhibitedFromISR(true);
    DRV_ENN_GPIO_Port->BSRR = DRV_ENN_Pin;
    __DMB();
}

/*
 * 函数用途：返回电源监控故障的升级优先级。
 * 调用场景：同一上电周期出现多个原因时决定对外保留哪个故障码。
 * 关键约束：只允许升级，避免后出现的次要原因覆盖掉电保存或恢复失败。
 */
static uint8_t PowerMonitor_GetFaultPriority(uint32_t error_code)
{
    switch (error_code) {
    case POWER_LOSS_POSITION_SAVE_FAILED:
        return 6U;
    case POWER_MONITOR_RECOVERY_FAILED:
        return 5U;
    case POWER_SUPPLY_24V_UNDERVOLTAGE:
        return 4U;
    case POWER_MONITOR_ADC_OVERRUN:
    case POWER_MONITOR_DMA_STOPPED:
        return 3U;
    case POWER_MONITOR_INIT_FAILED:
        return 2U;
    default:
        return 0U;
    }
}

/*
 * 函数用途：锁存首次故障并按优先级更新当前故障码。
 * 调用场景：启动线程、SysTick、ADC回调和紧急保存报告均可调用。
 * 关键约束：只发布RAM待处理标志并硬禁止，不在中断内打印或进入故障管理器。
 */
static void PowerMonitor_LatchFault(uint32_t error_code)
{
    uint32_t primask;
    uint8_t current_priority;
    uint8_t new_priority;

    if (PowerMonitor_GetFaultPriority(error_code) == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_first_fault_code == NO_ERROR) {
        s_first_fault_code = error_code;
    }
    current_priority = PowerMonitor_GetFaultPriority(s_latched_fault_code);
    new_priority = PowerMonitor_GetFaultPriority(error_code);
    if ((s_latched_fault_code == NO_ERROR) ||
        (new_priority > current_priority)) {
        s_latched_fault_code = error_code;
        s_fault_report_pending = 1U;
    }
    PowerMonitor_ForceSafeState();
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：按当前100k/10k分压假设把ADC计数换算为24V输入毫伏值。
 * 调用场景：上电打印和PWR?查询。
 * 关键约束：换算值用于调试趋势，最终阈值仍需台架实测校准。
 */
static uint32_t PowerMonitor_AdcTo24VMillivolts(uint32_t adc_sample)
{
    uint64_t numerator;

    if (adc_sample > POWER_MONITOR_ADC_FULL_SCALE_COUNTS) {
        adc_sample = POWER_MONITOR_ADC_FULL_SCALE_COUNTS;
    }
    numerator = (uint64_t)adc_sample *
                POWER_MONITOR_VDDA_MV *
                POWER_MONITOR_DIVIDER_RATIO_NUMERATOR;
    numerator += POWER_MONITOR_ADC_FULL_SCALE_COUNTS / 2U;
    return (uint32_t)(numerator / POWER_MONITOR_ADC_FULL_SCALE_COUNTS);
}

/*
 * 函数用途：读取ADC1循环DMA最近一次写入的24V采样值。
 * 调用场景：上电确认、PWR?查询、低压恢复检查和模拟看门狗回调。
 * 关键约束：32位读取在Cortex-M4上原子，不等待DMA、不访问ADC数据寄存器。
 */
static uint32_t PowerMonitor_ReadLatestAdcSample(void)
{
    uint32_t adc_sample = s_power_adc_dma_sample;

    if (adc_sample > POWER_MONITOR_ADC_FULL_SCALE_COUNTS) {
        return s_last_24v_adc_sample;
    }
    return adc_sample;
}

/*
 * 函数用途：判断ADC循环DMA流是否仍处于硬件使能状态。
 * 调用场景：SysTick监测、恢复验证和PWR?现场诊断。
 * 关键约束：只读取DMA寄存器，不启动、停止或重配DMA。
 */
static uint8_t PowerMonitor_IsDmaRunning(void)
{
    if (hdma_adc1.Instance == NULL) {
        return 0U;
    }
    return ((hdma_adc1.Instance->CR & DMA_SxCR_EN) != 0U) ? 1U : 0U;
}

/*
 * 函数用途：判断ADC运行期是否发生过过载。
 * 调用场景：SysTick监测、恢复验证和PWR?现场诊断。
 * 关键约束：HAL错误码具有锁存语义，局部恢复成功前不能忽略。
 */
static uint8_t PowerMonitor_HasAdcOverrun(void)
{
    if ((hadc1.ErrorCode & HAL_ADC_ERROR_OVR) != 0U) {
        return 1U;
    }
    return (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != RESET) ? 1U : 0U;
}

/*
 * 函数用途：配置模拟看门狗并启动单样本循环DMA，等待一份新采样。
 * 调用场景：首次启动和主循环局部恢复。
 * 关键约束：只在线程态调用；模拟看门狗中断在采样验证完成后才开放。
 */
static bool PowerMonitor_TryStartAdcDma(void)
{
    ADC_AnalogWDGConfTypeDef watchdog = {0};
    uint32_t start_tick;

    /*
     * 先关闭模拟看门狗并停止旧DMA，确保清标志和哨兵值不会与旧转换并发。
     * 恢复过程只在线程态执行，因此允许等待有限的首样本超时。
     */
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
    (void)HAL_ADC_Stop_DMA(&hadc1);
    s_power_adc_dma_sample = POWER_MONITOR_INVALID_ADC_SAMPLE;
    hadc1.ErrorCode = HAL_ADC_ERROR_NONE;
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD | ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* 看门狗只监视24V所在的通道12，首次验证前暂不开放中断。 */
    watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    watchdog.HighThreshold = POWER_MONITOR_ADC_FULL_SCALE_COUNTS;
    watchdog.LowThreshold = POWER_MONITOR_24V_FAIL_ADC_COUNTS;
    watchdog.Channel = ADC_CHANNEL_12;
    watchdog.ITMode = DISABLE;
    if (HAL_ADC_AnalogWDGConfig(&hadc1, &watchdog) != HAL_OK) {
        return false;
    }
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)&s_power_adc_dma_sample,
                          1U) != HAL_OK) {
        return false;
    }

    /* 单样本循环模式关闭传输类中断，避免每次转换进入DMA中断。 */
    __HAL_DMA_DISABLE_IT(&hdma_adc1,
                         DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_FE);

    /*
     * 必须看到DMA覆盖哨兵值，不能把重启前遗留的“正常电压”误判为恢复成功。
     */
    start_tick = HAL_GetTick();
    while (s_power_adc_dma_sample > POWER_MONITOR_ADC_FULL_SCALE_COUNTS) {
        if ((HAL_GetTick() - start_tick) >=
            POWER_MONITOR_SAMPLE_TIMEOUT_MS) {
            (void)HAL_ADC_Stop_DMA(&hadc1);
            return false;
        }
    }

    if ((PowerMonitor_IsDmaRunning() == 0U) ||
        (PowerMonitor_HasAdcOverrun() != 0U)) {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        return false;
    }
    s_last_24v_adc_sample = s_power_adc_dma_sample;
    return true;
}

/*
 * 函数用途：把有效的新采样应用到低压状态机。
 * 调用场景：首次启动或ADC/DMA局部恢复成功后。
 * 关键约束：20V以下触发紧急保存，20V至22V之间只保持安全禁止。
 */
static void PowerMonitor_ApplyRestartSample(void)
{
    uint32_t adc_sample = PowerMonitor_ReadLatestAdcSample();

    s_last_24v_adc_sample = adc_sample;
    s_power_monitor_ready = 1U;
    s_monitor_state = POWER_MONITOR_STATE_HEALTHY;
    if (adc_sample < POWER_MONITOR_24V_FAIL_ADC_COUNTS) {
        /* 已处于真实低压区，复用ISR安全路径投递一次紧急位置保存。 */
        PowerMonitor_Handle24VWatchdogFromISR(adc_sample);
    } else if (adc_sample < POWER_MONITOR_24V_RECOVER_ADC_COUNTS) {
        /*
         * 采样位于20V至22V回差区：不重复投递保存，但保持低压、写禁止和运动禁止。
         */
        s_power_fail_active = 1U;
        s_recover_stable_ms = 0U;
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
        PowerMonitor_LatchFault(POWER_SUPPLY_24V_UNDERVOLTAGE);
    } else {
        /* 首样本已高于恢复阈值，重新开放看门狗；既有故障锁存仍由新流程清除。 */
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
        FRAM_SetNormalWriteInhibitedFromISR(false);
        if (s_latched_fault_code == NO_ERROR) {
            s_motor_power_inhibited = 0U;
        }
    }
}

/*
 * 函数用途：配置并启动24V监测，输出首次有效采样值。
 * 调用场景：CPU2业务初始化开始阶段调用一次。
 * 关键约束：失败返回独立初始化故障码，并由主循环继续三次局部恢复。
 */
uint32_t PowerMonitor_Start(void)
{
    s_power_fail_active = 0U;
    s_recover_stable_ms = 0U;
    s_last_24v_adc_sample = 0U;
    s_power_fail_trip_count = 0U;
    s_last_trip_adc_sample = 0U;
    s_power_adc_dma_sample = POWER_MONITOR_INVALID_ADC_SAMPLE;
    s_power_monitor_ready = 0U;
    s_emergency_persistence_armed = 0U;
    s_emergency_request_sent = 0U;
    s_first_fault_code = NO_ERROR;
    s_latched_fault_code = NO_ERROR;
    s_recovery_cause_code = NO_ERROR;
    s_fault_report_pending = 0U;
    s_recovery_attempts = 0U;
    s_monitor_state = POWER_MONITOR_STATE_STOPPED;
    /* 上电先保持驱动和普通写入关闭，首个新样本验证成功后再按状态开放。 */
    PowerMonitor_ForceSafeState();

    if (!PowerMonitor_TryStartAdcDma()) {
        s_monitor_state = POWER_MONITOR_STATE_RECOVERY_PENDING;
        s_recovery_cause_code = POWER_MONITOR_INIT_FAILED;
        PowerMonitor_LatchFault(POWER_MONITOR_INIT_FAILED);
        printf("POWER_BOOT 电源监测启动：结果=失败，故障码=0x%08lX，后续=主循环局部恢复\r\n",
               (unsigned long)POWER_MONITOR_INIT_FAILED);
        return POWER_MONITOR_INIT_FAILED;
    }

    PowerMonitor_ApplyRestartSample();
    printf("POWER_BOOT 电源监测启动：结果=成功，ADC=%lu，24V电压=%lu毫伏，低压阈值ADC=%u，恢复阈值ADC=%u，DMA运行=%u，电机电源锁存=%u，故障码=0x%08lX\r\n",
           (unsigned long)s_last_24v_adc_sample,
           (unsigned long)PowerMonitor_AdcTo24VMillivolts(
               s_last_24v_adc_sample),
           (unsigned int)POWER_MONITOR_24V_FAIL_ADC_COUNTS,
           (unsigned int)POWER_MONITOR_24V_RECOVER_ADC_COUNTS,
           (unsigned int)PowerMonitor_IsDmaRunning(),
           (unsigned int)s_motor_power_inhibited,
           (unsigned long)s_latched_fault_code);
    return s_latched_fault_code;
}

/*
 * 函数用途：响应24V低压并投递编码器紧急保存。
 * 调用场景：ADC_IRQHandler通过HAL回调进入。
 * 关键约束：低压期间关闭看门狗中断防止中断风暴；FRAM提交留给PendSV。
 */
void PowerMonitor_Handle24VWatchdogFromISR(uint32_t adc_sample)
{
    s_last_24v_adc_sample = adc_sample;
    if (adc_sample >= POWER_MONITOR_24V_FAIL_ADC_COUNTS) {
        return;
    }

    if (s_power_fail_active == 0U) {
        /* 只在低压边沿统计一次并保存证据，低压持续期间不重复累计。 */
        s_power_fail_active = 1U;
        s_recover_stable_ms = 0U;
        s_power_fail_trip_count++;
        s_last_trip_adc_sample = adc_sample;
    }
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
    PowerMonitor_LatchFault(POWER_SUPPLY_24V_UNDERVOLTAGE);

    /*
     * 每个低压事件最多投递一次。FRAM预约先于PendSV请求发布，防止新的普通事务
     * 在掉电窗口插队；实际A/B写入和回读均留在PendSV执行。
     */
    if ((s_emergency_persistence_armed != 0U) &&
        (s_emergency_request_sent == 0U)) {
        s_emergency_request_sent = 1U;
        FRAM_RequestEmergencyReservationFromISR();
        Encoder_RequestEmergencyPersistenceFromISR(
            ENCODER_EMERGENCY_SOURCE_ADC);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

/*
 * 函数用途：记录ADC/DMA监控链路故障并挂起线程态恢复。
 * 调用场景：SysTick或新正式过程入口发现OVR、DMA停止。
 * 关键约束：只记录首次原因、关闭看门狗并硬禁止，不在调用点重启外设。
 */
static void PowerMonitor_RequestRecovery(uint32_t error_code)
{
    if ((s_monitor_state == POWER_MONITOR_STATE_RECOVERY_PENDING) ||
        (s_monitor_state == POWER_MONITOR_STATE_RECOVERY_FAILED)) {
        return;
    }
    s_power_monitor_ready = 0U;
    s_monitor_state = POWER_MONITOR_STATE_RECOVERY_PENDING;
    s_recovery_attempts = 0U;
    s_recovery_cause_code = error_code;
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
    PowerMonitor_LatchFault(error_code);
}

/*
 * 函数用途：监控ADC/DMA、维持紧急保存投递并统计24V稳定恢复时间。
 * 调用场景：SysTick每1ms调用。
 * 关键约束：不在SysTick内重启ADC/DMA、访问FRAM或打印。
 */
void PowerMonitor_TickFromISR(void)
{
    uint32_t adc_sample;

    /*
     * 启动时A/B无效也不永久失去布防机会；回零或人工修正一旦重建可信位置，
     * SysTick会自动补布防，避免编码器模块反向依赖电源模块。
     */
    if ((s_emergency_persistence_armed == 0U) &&
        Encoder_HasTrustedPosition()) {
        PowerMonitor_ArmEmergencyPersistence();
    }
    if (Encoder_HasEmergencyPersistencePending()) {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    /* 只有链路曾经就绪且处于健康态时才把运行期 OVR/DMA停止转换为恢复请求。 */
    if ((s_power_monitor_ready != 0U) &&
        (s_monitor_state == POWER_MONITOR_STATE_HEALTHY)) {
        if (PowerMonitor_HasAdcOverrun() != 0U) {
            PowerMonitor_RequestRecovery(POWER_MONITOR_ADC_OVERRUN);
            return;
        }
        if (PowerMonitor_IsDmaRunning() == 0U) {
            PowerMonitor_RequestRecovery(POWER_MONITOR_DMA_STOPPED);
            return;
        }
    }
    if ((s_monitor_state != POWER_MONITOR_STATE_HEALTHY) ||
        (s_power_fail_active == 0U)) {
        return;
    }

    adc_sample = PowerMonitor_ReadLatestAdcSample();
    s_last_24v_adc_sample = adc_sample;
    if (adc_sample >= POWER_MONITOR_24V_RECOVER_ADC_COUNTS) {
        if (s_recover_stable_ms < POWER_MONITOR_RECOVER_STABLE_MS) {
            s_recover_stable_ms++;
        }
    } else {
        s_recover_stable_ms = 0U;
    }

    /*
     * 电压稳定还不够：必须等紧急保存结束后再释放FRAM预约，避免普通写入
     * 与尚未完成的A/B记录或掉电回执竞争同一SPI总线。
     */
    if ((s_recover_stable_ms >= POWER_MONITOR_RECOVER_STABLE_MS) &&
        (!Encoder_HasEmergencyPersistencePending())) {
        s_power_fail_active = 0U;
        s_recover_stable_ms = 0U;
        s_emergency_request_sent = 0U;
        FRAM_ReleaseEmergencyReservation();
        FRAM_SetNormalWriteInhibitedFromISR(false);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
        /*
         * 电源稳定后恢复普通FRAM写；电机和故障码继续锁存，
         * 等下一条顶层正式命令确认后再开放运动。
         */
    }
}

/*
 * 函数用途：发布待报告故障并执行一次ADC/DMA局部恢复。
 * 调用场景：App_MainLoop每轮最先调用。
 * 关键约束：恢复成功不清故障码，三次失败后保持禁止且不自动重启CPU2。
 */
uint32_t PowerMonitor_ProcessDeferred(void)
{
    uint32_t recovery_cause;
    uint32_t report_code = NO_ERROR;
    uint32_t primask;

    if (s_monitor_state == POWER_MONITOR_STATE_RECOVERY_PENDING) {
        /*
         * 每次主循环只做一次局部重启；失败后保留PENDING，下一轮再尝试，
         * 避免单轮主循环连续阻塞三个5ms等待窗口。
         */
        recovery_cause = s_recovery_cause_code;
        if (s_recovery_attempts < POWER_MONITOR_RECOVERY_RETRY_LIMIT) {
            s_recovery_attempts++;
        }
        /* 错误 阶段：错误重试 模块：系统 操作：恢复24V电源监测 原因：ErrorLog_GetReasonByCode(recovery_cause) 尝试：s_recovery_attempts/POWER_MONITOR_RECOVERY_RETRY_LIMIT 错误码：recovery_cause 错误名：ErrorLog_GetCodeName(recovery_cause) */
        ErrorLog_Retry(ERROR_LOG_MODULE_SYSTEM,
                       "恢复24V电源监测",
                       ErrorLog_GetReasonByCode(recovery_cause),
                       s_recovery_attempts,
                       POWER_MONITOR_RECOVERY_RETRY_LIMIT,
                       recovery_cause);

        if (PowerMonitor_TryStartAdcDma()) {
            PowerMonitor_ApplyRestartSample();
            /* 错误 阶段：重试成功 模块：系统 操作：恢复24V电源监测 原因：ErrorLog_GetReasonByCode(recovery_cause) 尝试：s_recovery_attempts/POWER_MONITOR_RECOVERY_RETRY_LIMIT */
            ErrorLog_Recover(ERROR_LOG_MODULE_SYSTEM,
                             "恢复24V电源监测",
                             ErrorLog_GetReasonByCode(recovery_cause),
                             s_recovery_attempts,
                             POWER_MONITOR_RECOVERY_RETRY_LIMIT);
        } else if (s_recovery_attempts >=
                   POWER_MONITOR_RECOVERY_RETRY_LIMIT) {
            s_power_monitor_ready = 0U;
            s_monitor_state = POWER_MONITOR_STATE_RECOVERY_FAILED;
            PowerMonitor_LatchFault(POWER_MONITOR_RECOVERY_FAILED);
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_fault_report_pending != 0U) {
        report_code = s_latched_fault_code;
        s_fault_report_pending = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return report_code;
}

/*
 * 函数用途：编码器可信位置建立后开放低压事件的紧急位置保存。
 * 调用场景：启动恢复、回零、人工位置修正以及SysTick自动补布防。
 * 关键约束：位置仍不可信时保持未布防，避免保存无效累计值。
 */
void PowerMonitor_ArmEmergencyPersistence(void)
{
    uint32_t primask;

    if (!Encoder_HasTrustedPosition()) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    s_emergency_persistence_armed = 1U;
    if ((s_power_monitor_ready != 0U) &&
        (s_monitor_state == POWER_MONITOR_STATE_HEALTHY) &&
        (s_power_fail_active != 0U) &&
        (s_last_24v_adc_sample < POWER_MONITOR_24V_FAIL_ADC_COUNTS) &&
        (s_emergency_request_sent == 0U)) {
        s_emergency_request_sent = 1U;
        FRAM_RequestEmergencyReservationFromISR();
        Encoder_RequestEmergencyPersistenceFromISR(
            ENCODER_EMERGENCY_SOURCE_ADC);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

bool PowerMonitor_IsPowerFailActive(void)
{
    return s_power_fail_active != 0U;
}

bool PowerMonitor_IsMotorInhibited(void)
{
    return s_motor_power_inhibited != 0U;
}

bool PowerMonitor_HasLatchedFault(void)
{
    return s_latched_fault_code != NO_ERROR;
}

uint32_t PowerMonitor_GetLatchedFaultCode(void)
{
    return s_latched_fault_code;
}

/*
 * 函数用途：检查新正式过程是否可以解除电源监控相关锁存。
 * 调用场景：ProcessMeasureCmd通过即时控制命令过滤后。
 * 关键约束：只有监控健康、DMA运行、无OVR且24V稳定恢复才清除锁存。
 */
uint32_t PowerMonitor_BeginNewProcess(void)
{
    uint32_t adc_sample;
    uint32_t primask;
    uint32_t error_code;

    if ((s_monitor_state == POWER_MONITOR_STATE_HEALTHY) &&
        (PowerMonitor_HasAdcOverrun() != 0U)) {
        PowerMonitor_RequestRecovery(POWER_MONITOR_ADC_OVERRUN);
    } else if ((s_monitor_state == POWER_MONITOR_STATE_HEALTHY) &&
               (PowerMonitor_IsDmaRunning() == 0U)) {
        PowerMonitor_RequestRecovery(POWER_MONITOR_DMA_STOPPED);
    }

    adc_sample = PowerMonitor_ReadLatestAdcSample();
    if ((s_monitor_state == POWER_MONITOR_STATE_HEALTHY) &&
        (adc_sample < POWER_MONITOR_24V_FAIL_ADC_COUNTS)) {
        PowerMonitor_Handle24VWatchdogFromISR(adc_sample);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    /*
     * 新正式流程是唯一的软件解锁点。解锁前再次现场复核监控链路、电压和恢复状态，
     * 防止命令到达恰好落在DMA停止或电压再次下降的竞态窗口。
     */
    if ((s_power_monitor_ready != 0U) &&
        (s_monitor_state == POWER_MONITOR_STATE_HEALTHY) &&
        (PowerMonitor_IsDmaRunning() != 0U) &&
        (PowerMonitor_HasAdcOverrun() == 0U) &&
        (s_power_fail_active == 0U) &&
        (adc_sample >= POWER_MONITOR_24V_RECOVER_ADC_COUNTS)) {
        s_latched_fault_code = NO_ERROR;
        s_first_fault_code = NO_ERROR;
        s_recovery_cause_code = NO_ERROR;
        s_recovery_attempts = 0U;
        s_fault_report_pending = 0U;
        s_motor_power_inhibited = 0U;
        FRAM_SetNormalWriteInhibitedFromISR(false);
        __DMB();
        error_code = NO_ERROR;
    } else {
        error_code = s_latched_fault_code;
        if (error_code == NO_ERROR) {
            error_code = POWER_MONITOR_INIT_FAILED;
        }
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return error_code;
}

uint32_t PowerMonitor_GetLast24VAdcSample(void)
{
    return PowerMonitor_ReadLatestAdcSample();
}

void PowerMonitor_ReportEmergencyPersistenceFailure(void)
{
    PowerMonitor_LatchFault(POWER_LOSS_POSITION_SAVE_FAILED);
}

/*
 * 函数用途：取得当前24V采样、低压锁存和恢复状态的一致快照。
 * 调用场景：线程态处理PWR?命令。
 * 关键约束：读取DMA最新采样后只短暂关中断复制RAM状态。
 */
bool PowerMonitor_GetDebugSnapshot(PowerMonitorDebugSnapshot *snapshot)
{
    uint32_t adc_sample;
    uint32_t primask;

    if (snapshot == NULL) {
        return false;
    }

    adc_sample = PowerMonitor_ReadLatestAdcSample();
    primask = __get_PRIMASK();
    __disable_irq();
    s_last_24v_adc_sample = adc_sample;
    snapshot->adc_sample = adc_sample;
    snapshot->trip_count = s_power_fail_trip_count;
    snapshot->last_trip_adc_sample = s_last_trip_adc_sample;
    snapshot->first_fault_code = s_first_fault_code;
    snapshot->latched_fault_code = s_latched_fault_code;
    snapshot->recovery_cause_code = s_recovery_cause_code;
    snapshot->recover_stable_ms = s_recover_stable_ms;
    snapshot->recovery_attempts = s_recovery_attempts;
    snapshot->power_fail_active = s_power_fail_active;
    snapshot->dma_running = PowerMonitor_IsDmaRunning();
    snapshot->adc_overrun = PowerMonitor_HasAdcOverrun();
    snapshot->emergency_persistence_armed =
        s_emergency_persistence_armed;
    snapshot->emergency_persistence_pending =
        Encoder_HasEmergencyPersistencePending() ? 1U : 0U;
    snapshot->motor_inhibited = s_motor_power_inhibited;
    snapshot->monitor_state = s_monitor_state;
    if (primask == 0U) {
        __enable_irq();
    }

    snapshot->millivolts_24v =
        PowerMonitor_AdcTo24VMillivolts(snapshot->adc_sample);
    return true;
}

/*
 * 函数用途：模拟一次不改变ADC状态的紧急编码器保存请求。
 * 调用场景：线程态处理PWRTEST命令。
 * 关键约束：编码器位置无效或已有紧急请求时拒绝，不直接访问FRAM。
 */
PowerMonitorTestRequestResult PowerMonitor_RequestEmergencyPersistenceForTest(void)
{
    uint32_t primask;

    if (!Encoder_HasTrustedPosition()) {
        return POWER_MONITOR_TEST_ENCODER_INVALID;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (Encoder_HasEmergencyPersistencePending()) {
        if (primask == 0U) {
            __enable_irq();
        }
        return POWER_MONITOR_TEST_BUSY;
    }

    FRAM_RequestEmergencyReservationFromISR();
    Encoder_RequestEmergencyPersistenceFromISR(
        ENCODER_EMERGENCY_SOURCE_TEST);
    __DMB();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    if (primask == 0U) {
        __enable_irq();
    }
    return POWER_MONITOR_TEST_QUEUED;
}

/*
 * HAL ADC模拟看门狗回调：只接受ADC1事件，并把12bit DR采样交给ISR安全处理路径。
 * 回调内不做FRAM事务、日志输出或外设重启。
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) {
        PowerMonitor_Handle24VWatchdogFromISR(
            ((uint32_t)hadc->Instance->DR) &
            POWER_MONITOR_ADC_FULL_SCALE_COUNTS);
    }
}