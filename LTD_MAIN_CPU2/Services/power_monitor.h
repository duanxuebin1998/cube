#ifndef SERVICES_POWER_MONITOR_H_
/* SERVICES_POWER_MONITOR_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SERVICES_POWER_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 默认阈值按 100k/10k 分压与 3.3V VDDA 计算。
 * 台架测得正常24V范围和掉电保持时间后，只需调整这里的可覆盖常量。
 */
/* 24V 下降到约 20V 时触发安全停机；允许由编译参数覆盖以适配台架校准值。 */
#ifndef POWER_MONITOR_24V_FAIL_ADC_COUNTS
/* 24 V 输入下降到约 20 V 时触发安全停机的 ADC 原始计数阈值 2256；允许由编译参数覆盖以适配实际分压和台架校准。 */
#define POWER_MONITOR_24V_FAIL_ADC_COUNTS       2256U
#endif

/*
 * 24V 回升到约 22V 后才进入恢复计时。
 * 与触发阈值之间保留约 2V 回差，避免电压在边界抖动时反复进出低压状态。
 */
#ifndef POWER_MONITOR_24V_RECOVER_ADC_COUNTS
/* 24 V 输入回升到约 22 V 后进入恢复计时的 ADC 原始计数阈值 2482；与停机阈值形成约 2 V 回差，避免边界抖动。 */
#define POWER_MONITOR_24V_RECOVER_ADC_COUNTS    2482U
#endif

/* 电压连续高于恢复阈值的确认时间，单位 ms，由 1ms SysTick 累计。 */
#define POWER_MONITOR_RECOVER_STABLE_MS         100U
/* ADC/DMA 局部重启总尝试次数；达到上限后保持运动禁止，不自动重启整机。 */
#define POWER_MONITOR_RECOVERY_RETRY_LIMIT      3U

/* PWRTEST 请求的同步受理结果；只说明是否入队，不代表 FRAM 已经提交成功。 */
typedef enum {
    POWER_MONITOR_TEST_QUEUED = 0,       /* 请求已写入 RAM 并挂起 PendSV。 */
    POWER_MONITOR_TEST_BUSY,             /* 已有紧急保存请求，拒绝覆盖现有快照。 */
    POWER_MONITOR_TEST_ENCODER_INVALID   /* 累计位置不可信，禁止保存无效位置。 */
} PowerMonitorTestRequestResult;

/* ADC/DMA 监控链路状态；真实低压活动状态由 power_fail_active 单独表达。 */
typedef enum {
    POWER_MONITOR_STATE_STOPPED = 0,     /* 尚未成功取得首个可信 ADC 样本。 */
    POWER_MONITOR_STATE_HEALTHY,         /* ADC、DMA 和采样时效均可用于安全判断。 */
    POWER_MONITOR_STATE_RECOVERY_PENDING,/* 已禁止运动，等待主循环执行局部恢复。 */
    POWER_MONITOR_STATE_RECOVERY_FAILED  /* 三次局部恢复均失败，本次上电保持禁止。 */
} PowerMonitorState;

/* PWR? 使用的只读快照；所有字段均来自 RAM 或只读寄存器，不触发外设恢复。 */
typedef struct {
    uint32_t adc_sample;                  /* DMA 最近一次 12bit 原始采样计数。 */
    uint32_t millivolts_24v;              /* 按默认分压估算的输入电压，单位 mV。 */
    uint32_t trip_count;                  /* 本次上电真实低压首次触发次数。 */
    uint32_t last_trip_adc_sample;        /* 最近一次由正常态进入低压态时的采样。 */
    uint32_t first_fault_code;            /* 本次锁存周期最先出现的原因，用于追根。 */
    uint32_t latched_fault_code;          /* 按安全优先级升级后的当前对外故障码。 */
    uint32_t recovery_cause_code;         /* 本轮 ADC/DMA 局部恢复的原始触发原因。 */
    uint16_t recover_stable_ms;           /* 已连续满足 22V 恢复条件的毫秒数。 */
    uint8_t recovery_attempts;            /* 当前恢复原因已执行的局部重启次数。 */
    uint8_t power_fail_active;            /* 1 表示真实低压尚未完成稳定恢复。 */
    uint8_t dma_running;                  /* 1 表示 DMA2 Stream0 使能位仍有效。 */
    uint8_t adc_overrun;                  /* 1 表示 ADC OVR 或 HAL OVR 锁存存在。 */
    uint8_t emergency_persistence_armed;  /* 1 表示编码器位置可信，可响应低压保存。 */
    uint8_t emergency_persistence_pending;/* 1 表示紧急 A/B 快照或回执尚未结束。 */
    uint8_t motor_inhibited;              /* 1 表示驱动使能入口必须保持关闭。 */
    PowerMonitorState monitor_state;      /* ADC/DMA 监控链路当前状态。 */
} PowerMonitorDebugSnapshot;

/**
 * @brief 启动 ADC1 通道12连续采样和24V低压模拟看门狗。
 *
 * @details 调用场景：CPU2业务初始化开始阶段调用一次。
 * @note 关键约束：返回准确故障码；失败后由主循环延后服务最多尝试三次局部恢复。
 */
uint32_t PowerMonitor_Start(void);

/**
 * @brief 锁存24V低压事件并请求编码器紧急保存。
 *
 * @details 调用场景：ADC模拟看门狗中断回调。
 * @note 关键约束：只置标志和投递PendSV，不访问FRAM、不打印、不阻塞。
 */
void PowerMonitor_Handle24VWatchdogFromISR(uint32_t adc_sample);

/**
 * @brief 检测监控链路、维持紧急保存投递并统计24V稳定恢复时间。
 *
 * @details 调用场景：SysTick中断每1ms调用。
 * @note 关键约束：只记录故障和执行硬禁止，不重启ADC/DMA、不访问FRAM、不打印。
 */
void PowerMonitor_TickFromISR(void);

/**
 * @brief 在线程态发布故障并局部恢复ADC/DMA监控链路。
 *
 * @details 调用场景：App_MainLoop每轮最先调用。
 * @note 关键约束：每轮最多尝试一次，累计三次失败后保持禁止并返回恢复失败码。
 */
uint32_t PowerMonitor_ProcessDeferred(void);

/**
 * @brief 在编码器可信位置建立后开放紧急位置持久化。
 *
 * @details 调用场景：启动恢复、回零或人工位置修正后，也可由SysTick自动补布防。
 * @note 关键约束：函数自身复核可信位置，不能把无效位置布防为可保存状态。
 */
void PowerMonitor_ArmEmergencyPersistence(void);

/**
 * @brief 判断 24 V 低压掉电状态当前是否仍然有效。
 *
 * 以下查询接口均为无阻塞 RAM/寄存器读取，可用于线程态和短中断路径。
 * IsPowerFailActive 表示电压恢复过程；IsMotorInhibited 还包含监控链路故障锁存。
 *
 * @return true 表示 24 V 低压掉电状态当前仍然有效；false 表示 24 V 低压掉电状态当前已不再有效。
 */
bool PowerMonitor_IsPowerFailActive(void);
/**
 * @brief 判断电源监控是否仍禁止电机驱动动作。
 *
 * @return true 表示电源监控仍禁止电机驱动动作；false 表示电源监控已不再禁止电机驱动动作。
 */
bool PowerMonitor_IsMotorInhibited(void);
/**
 * @brief 判断电源监控是否已经锁存整机故障码。
 *
 * @return true 表示电源监控已经锁存整机故障码；false 表示电源监控尚未锁存整机故障码。
 */
bool PowerMonitor_HasLatchedFault(void);
/**
 * @brief 返回电源监控当前锁存的整机故障码。
 *
 * @return 返回当前锁存的电源监控整机错误码；没有锁存故障时返回 NO_ERROR。
 */
uint32_t PowerMonitor_GetLatchedFaultCode(void);
/**
 * @brief 读取并返回 24 V 监控 ADC 循环 DMA 的最新原始采样值。
 *
 * @return 返回最近一次 24 V ADC 原始采样计数；调用时会先从循环 DMA 读取最新值。
 */
uint32_t PowerMonitor_GetLast24VAdcSample(void);

/**
 * @brief 在新的顶层正式过程开始时检查电源并解除可恢复锁存。
 *
 * @details 调用场景：测量命令通过即时控制命令过滤后。
 * @note 关键约束：只在监控健康、电压稳定且无恢复任务时返回NO_ERROR。
 */
uint32_t PowerMonitor_BeginNewProcess(void);

/**
 * @brief 记录真实低压紧急位置保存最终失败。
 *
 * @details 调用场景：主循环消费编码器紧急保存报告时。
 * @note 关键约束：软件测试保存失败不调用本接口，避免把测试结果伪装成现场故障。
 */
void PowerMonitor_ReportEmergencyPersistenceFailure(void);

/**
 * @brief 取得24V监测、恢复和紧急保存状态的一致调试快照。
 *
 * @details 调用场景：线程态处理PWR?查询。
 * @note 关键约束：只短暂关中断复制RAM状态，不读取FRAM、不打印。
 */
bool PowerMonitor_GetDebugSnapshot(PowerMonitorDebugSnapshot *snapshot);

/**
 * @brief 在线程态模拟一次掉电紧急保存请求。
 *
 * @details 调用场景：处理PWRTEST串口命令。
 * @note 关键约束：不伪造ADC低压、不改变恢复状态，只验证FRAM预留与紧急提交链。
 */
PowerMonitorTestRequestResult PowerMonitor_RequestEmergencyPersistenceForTest(void);

#endif