#ifndef CPU3_CLOCK_H_
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

/* 初始化 CPU3 RTC；失败时只影响协议时间显示，不阻塞主程序启动。 */
void Cpu3Clock_Init(void);

/* 获取当前时间；返回 1 表示 out 中字段已通过合法性校验。 */
uint8_t Cpu3Clock_GetDateTime(Cpu3DateTime *out);

/* 设置 CPU3 RTC 时间；由屏幕菜单校时调用，成功后写入备份标记。 */
uint8_t Cpu3Clock_SetDateTime(const Cpu3DateTime *dt);

/* 获取 RTC 当前运行状态，供屏幕菜单显示 LSE/LSI/未校时/异常。 */
Cpu3ClockState Cpu3Clock_GetState(void);

/* 获取 RTC 当前时钟源，供诊断页面区分 LSE 与 LSI 兜底。 */
Cpu3ClockSource Cpu3Clock_GetSource(void);


#ifdef __cplusplus
}
#endif

#endif /* CPU3_CLOCK_H_ */
