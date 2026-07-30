#ifndef CPU3_CLOCK_H_
/* CPU3_CLOCK_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define CPU3_CLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CPU3 本机 RTC 的普通二进制时间结构，供 SI 时间寄存器和 profile 时间戳使用。 */
typedef struct {
    uint16_t year;   /* 公历年份，当前实现限制为 2000~2099。 */
    uint8_t month;   /* 月份 1~12，用于 SI profile 时间戳和当前时间显示。 */
    uint8_t day;     /* 日期 1~31，实际合法性由月份和闰年共同校验。 */
    uint8_t hour;    /* 24 小时制小时，直接映射 SI 当前时间小时寄存器。 */
    uint8_t minute;  /* 分钟 0~59。 */
    uint8_t second;  /* 秒 0~59。 */
    uint8_t valid;   /* 读取结果是否合法；非法时外部协议侧应保持寄存器为 0。 */
} Cpu3DateTime;

typedef enum {
    CPU3_CLOCK_SOURCE_NONE = 0,     /* RTC 未完成初始化或外设异常。 */
    CPU3_CLOCK_SOURCE_LSE,          /* RTC 使用外部 32.768 kHz LSE。 */
    CPU3_CLOCK_SOURCE_LSI,          /* RTC 使用内部 LSI 兜底。 */
} Cpu3ClockSource;

typedef enum {
    CPU3_CLOCK_STATE_ERROR = 0,     /* RTC 初始化失败或运行期读数异常。 */
    CPU3_CLOCK_STATE_UNSET,         /* RTC 可运行，但尚未由用户校时。 */
    CPU3_CLOCK_STATE_LSE_VALID,     /* RTC 使用 LSE，且已有用户校时。 */
    CPU3_CLOCK_STATE_LSI_FALLBACK,  /* RTC 使用 LSI 兜底，且已有用户校时。 */
} Cpu3ClockState;

/**
 * @brief 初始化 CPU3 本机 RTC。
 *
 * 只服务外部协议时间显示和 profile 时间戳，不参与 CPU2 测量调度。
 *
 * 初始化 CPU3 RTC；失败时只影响协议时间显示，不阻塞主程序启动。
 */
void Cpu3Clock_Init(void);

/**
 * @brief 读取当前 RTC 时间。
 *
 * 返回 1 表示 out 中字段已通过合法性校验，返回 0 时调用方应保持协议寄存器为 0。
 *
 * 获取当前时间；返回 1 表示 out 中字段已通过合法性校验。
 *
 * @param out 当前 RTC 时间输出对象；缓存或硬件读取成功时写入经过范围校验的日期时间。
 * @return 1 表示已返回有效 RTC 时间，0 表示输出参数非法或当前 RTC 数据无效。
 */
uint8_t Cpu3Clock_GetDateTime(Cpu3DateTime *out);

/**
 * @brief 设置 CPU3 RTC 时间。
 *
 * 保存成功后写入 SET 备份标记，后续上电不会再覆盖 RTC。
 *
 * @param dt 待校验、换算或写入的 CPU3 日期时间结构。
 * @return 1 表示年月日、时分秒已通过范围校验并成功写入 RTC 日期和时间；0 表示日期时间字段越界，或 HAL_RTC_SetDate 或 HAL_RTC_SetTime 写入失败。
 */
uint8_t Cpu3Clock_SetDateTime(const Cpu3DateTime *dt);

/**
 * @brief 返回 CPU3 RTC 当前初始化与有效性状态。
 *
 * 获取 RTC 当前运行状态，供屏幕菜单显示 LSE/LSI/未校时/异常。
 *
 * @return 返回当前 RTC 获取状态枚举，用于区分未初始化、有效、无效和恢复中状态。
 */
Cpu3ClockState Cpu3Clock_GetState(void);

/**
 * @brief 返回 CPU3 RTC 当前采用的 LSE、LSI 或未建立时钟源状态。
 *
 * 获取 RTC 当前时钟源，供诊断页面区分 LSE 与 LSI 兜底。
 *
 * @return 返回当前 RTC 时钟源枚举，用于区分 LSE、LSI 和尚未建立时钟源。
 */
Cpu3ClockSource Cpu3Clock_GetSource(void);


#ifdef __cplusplus
}
#endif

#endif /* CPU3_CLOCK_H_ */
