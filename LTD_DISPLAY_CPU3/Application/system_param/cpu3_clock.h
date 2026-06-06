#ifndef CPU3_CLOCK_H_
#define CPU3_CLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CPU3 本机 RTC 的普通二进制时间结构，供 SI7000 时间寄存器和 profile 时间戳使用。 */
typedef struct {
    uint16_t year;   /* 公历年份，当前实现限制为 2000~2099。 */
    uint8_t month;   /* 月份 1~12，用于 SI7000 profile 时间戳和当前时间显示。 */
    uint8_t day;     /* 日期 1~31，实际合法性由月份和闰年共同校验。 */
    uint8_t hour;    /* 24 小时制小时，直接映射 SI7000 当前时间小时寄存器。 */
    uint8_t minute;  /* 分钟 0~59。 */
    uint8_t second;  /* 秒 0~59。 */
    uint8_t valid;   /* 读取结果是否合法；非法时外部协议侧应保持寄存器为 0。 */
} Cpu3DateTime;

/* 初始化 CPU3 RTC；失败时只影响协议时间显示，不阻塞主程序启动。 */
void Cpu3Clock_Init(void);

/* 获取当前时间；返回 1 表示 out 中字段已通过合法性校验。 */
uint8_t Cpu3Clock_GetDateTime(Cpu3DateTime *out);


#ifdef __cplusplus
}
#endif

#endif /* CPU3_CLOCK_H_ */
