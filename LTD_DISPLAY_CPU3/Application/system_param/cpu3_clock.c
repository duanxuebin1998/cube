#include "cpu3_clock.h"

#include "main.h"

/*
 * CPU3 本机时钟只用于外部协议展示和 profile 完成时间戳锁存。
 * 它不参与 CPU2 测量控制，也不作为测量状态机的调度依据，避免显示协议反向影响底层测量流程。
 */
#define CPU3_CLOCK_BKP_MARKER        0x43505533UL
#define CPU3_CLOCK_INIT_TIMEOUT_MS   100U
#define CPU3_CLOCK_DEFAULT_YEAR      2026U
#define CPU3_CLOCK_DEFAULT_MONTH     1U
#define CPU3_CLOCK_DEFAULT_DAY       1U

/* RTC 初始化成功后置位，读取接口用它区分“时间无效”和“时间为 0 点”。 */
static uint8_t s_cpu3_clock_ready = 0U;

/*
 * 将 RTC 寄存器中的一个 BCD 字段转成普通二进制值。
 * 仅解析单个字段，不访问硬件寄存器，便于当前时间和日期共用。
 */
static uint8_t cpu3_clock_bcd2bin(uint32_t value)
{
    return (uint8_t)((((value >> 4U) & 0x0FU) * 10U) + (value & 0x0FU));
}

/*
 * 判断年份是否闰年。
 * 当前只支持 2000~2099，但保留完整规则，避免后续扩展时埋边界问题。
 */
static uint8_t cpu3_clock_is_leap_year(uint16_t year)
{
    return (((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U))) ? 1U : 0U;
}

/*
 * 获取指定年月的最大日期。
 * 返回 0 表示月份非法，调用方据此把整组时间判为无效。
 */
static uint8_t cpu3_clock_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[] = {
        31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
    };

    if ((month < 1U) || (month > 12U)) {
        return 0U;
    }

    if ((month == 2U) && (cpu3_clock_is_leap_year(year) != 0U)) {
        return 29U;
    }

    return days[month - 1U];
}

/*
 * 校验 CPU3 时间结构体的字段范围。
 * 该检查是所有读写接口的统一入口，避免非法日期写入 RTC 或暴露给 SI7000。
 */
static uint8_t cpu3_clock_validate(const Cpu3DateTime *dt)
{
    uint8_t max_day;

    if (dt == NULL) {
        return 0U;
    }

    max_day = cpu3_clock_days_in_month(dt->year, dt->month);
    if ((dt->year < 2000U) || (dt->year > 2099U) ||
        (dt->day < 1U) || (dt->day > max_day) ||
        (dt->hour > 23U) || (dt->minute > 59U) || (dt->second > 59U)) {
        return 0U;
    }

    return 1U;
}

/*
 * 根据日期计算 RTC DR 所需的星期字段。
 * 外部协议暂不使用星期，但硬件寄存器写入时必须保持合法。
 */
static uint8_t cpu3_clock_weekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t offsets[] = {0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U};
    uint16_t y = year;
    uint8_t w;

    if (month < 3U) {
        y--;
    }

    w = (uint8_t)((y + (y / 4U) - (y / 100U) + (y / 400U) + offsets[month - 1U] + day) % 7U);
    return (w == 0U) ? 7U : w;
}

/*
 * 关闭 RTC 写保护。
 * 只能在主线程初始化/设置时间路径调用，不在中断中使用。
 */
static void cpu3_clock_disable_write_protection(void)
{
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;
}

/*
 * 恢复 RTC 写保护。
 * 与 cpu3_clock_disable_write_protection 成对使用，防止运行期误写 RTC 寄存器。
 */
static void cpu3_clock_enable_write_protection(void)
{
    RTC->WPR = 0xFFU;
}

/*
 * 进入 RTC 初始化模式。
 * 带超时保护，避免 LSI 或 RTC 异常时卡住 CPU3 启动流程。
 */
static uint8_t cpu3_clock_enter_init_mode(void)
{
    uint32_t tick_start = HAL_GetTick();

    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0U) {
        /* 初始化模式等待有超时保护，避免 LSI/RTC 异常时卡死启动流程。 */
        if ((HAL_GetTick() - tick_start) > CPU3_CLOCK_INIT_TIMEOUT_MS) {
            return 0U;
        }
    }

    return 1U;
}

/*
 * 退出 RTC 初始化模式。
 * 调用方负责在退出前完成 TR/DR/PRER 等寄存器写入。
 */
static void cpu3_clock_exit_init_mode(void)
{
    RTC->ISR &= ~RTC_ISR_INIT;
}

/*
 * 将已校验的时间写入 RTC TR/DR 原始寄存器。
 * 所有字段在这里统一拆成 BCD，避免多个写路径各自处理格式。
 */
static void cpu3_clock_write_datetime_registers(const Cpu3DateTime *dt)
{
    uint8_t year = (uint8_t)(dt->year % 100U);
    uint8_t weekday = cpu3_clock_weekday(dt->year, dt->month, dt->day);

    /* 直接写 RTC 原始寄存器，所有字段都按硬件要求拆成 BCD 个位/十位。 */
    RTC->TR = ((uint32_t)(dt->second % 10U) << 0U) |
              ((uint32_t)(dt->second / 10U) << 4U) |
              ((uint32_t)(dt->minute % 10U) << 8U) |
              ((uint32_t)(dt->minute / 10U) << 12U) |
              ((uint32_t)(dt->hour % 10U) << 16U) |
              ((uint32_t)(dt->hour / 10U) << 20U);

    RTC->DR = ((uint32_t)(dt->day % 10U) << 0U) |
              ((uint32_t)(dt->day / 10U) << 4U) |
              ((uint32_t)(dt->month % 10U) << 8U) |
              ((uint32_t)(dt->month / 10U) << 12U) |
              ((uint32_t)weekday << 13U) |
              ((uint32_t)(year % 10U) << 16U) |
              ((uint32_t)(year / 10U) << 20U);
}

/*
 * 配置并打开 RTC 时钟源。
 * 当前使用 LSI，满足协议时间展示和 profile 完成时间戳兜底需求。
 */
static uint8_t cpu3_clock_configure_rtc_clock(void)
{
    RCC_PeriphCLKInitTypeDef periph_clk = {0};

    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    /* 使用内部 LSI，不要求外部晶振；精度足够满足协议时间显示和完成时间戳。 */
    periph_clk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        return 0U;
    }

    __HAL_RCC_RTC_ENABLE();
    return 1U;
}

/*
 * 初始化 CPU3 本机 RTC。
 * 只服务外部协议时间显示和 profile 时间戳，不参与 CPU2 测量调度。
 */
void Cpu3Clock_Init(void)
{
    Cpu3DateTime default_dt = {
        .year = CPU3_CLOCK_DEFAULT_YEAR,
        .month = CPU3_CLOCK_DEFAULT_MONTH,
        .day = CPU3_CLOCK_DEFAULT_DAY,
        .hour = 0U,
        .minute = 0U,
        .second = 0U,
        .valid = 1U,
    };

    s_cpu3_clock_ready = 0U;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (cpu3_clock_configure_rtc_clock() == 0U) {
        return;
    }

    cpu3_clock_disable_write_protection();

    /*
     * BKP0R 标记用于区分“已由本程序配置过 RTC”和上电默认状态。
     * 未配置时写入保守默认日期，保证 SI7000 时间寄存器不会输出非法 BCD。
     */
    if ((RTC->BKP0R != CPU3_CLOCK_BKP_MARKER) || ((RTC->ISR & RTC_ISR_INITS) == 0U)) {
        if (cpu3_clock_enter_init_mode() == 0U) {
            cpu3_clock_enable_write_protection();
            return;
        }

        RTC->CR &= ~RTC_CR_FMT;
        RTC->PRER = (127UL << RTC_PRER_PREDIV_A_Pos) | 255UL;
        cpu3_clock_write_datetime_registers(&default_dt);
        RTC->BKP0R = CPU3_CLOCK_BKP_MARKER;
        cpu3_clock_exit_init_mode();
    }

    cpu3_clock_enable_write_protection();
    s_cpu3_clock_ready = 1U;
}

/*
 * 读取当前 RTC 时间。
 * 返回 1 表示 out 中字段已通过合法性校验，返回 0 时调用方应保持协议寄存器为 0。
 */
uint8_t Cpu3Clock_GetDateTime(Cpu3DateTime *out)
{
    uint32_t tr;
    uint32_t dr;

    if ((out == NULL) || (s_cpu3_clock_ready == 0U)) {
        return 0U;
    }

    /* 先快照 TR/DR，再做字段解析，减少跨秒读取造成的字段不一致窗口。 */
    tr = RTC->TR;
    dr = RTC->DR;

    out->second = cpu3_clock_bcd2bin(((tr >> 4U) & 0x07U) << 4U | (tr & 0x0FU));
    out->minute = cpu3_clock_bcd2bin(((tr >> 12U) & 0x07U) << 4U | ((tr >> 8U) & 0x0FU));
    out->hour = cpu3_clock_bcd2bin(((tr >> 20U) & 0x03U) << 4U | ((tr >> 16U) & 0x0FU));
    out->day = cpu3_clock_bcd2bin(((dr >> 4U) & 0x03U) << 4U | (dr & 0x0FU));
    out->month = cpu3_clock_bcd2bin(((dr >> 12U) & 0x01U) << 4U | ((dr >> 8U) & 0x0FU));
    out->year = (uint16_t)(2000U + cpu3_clock_bcd2bin(((dr >> 20U) & 0x0FU) << 4U | ((dr >> 16U) & 0x0FU)));
    out->valid = cpu3_clock_validate(out);

    return out->valid;
}

/*
 * 设置 CPU3 本机 RTC 时间。
 * 仅接受合法日期，成功写入后刷新备份标记并允许后续读取。
 */
uint8_t Cpu3Clock_SetDateTime(const Cpu3DateTime *dt)
{
    if (cpu3_clock_validate(dt) == 0U) {
        return 0U;
    }

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (cpu3_clock_configure_rtc_clock() == 0U) {
        s_cpu3_clock_ready = 0U;
        return 0U;
    }

    cpu3_clock_disable_write_protection();
    if (cpu3_clock_enter_init_mode() == 0U) {
        cpu3_clock_enable_write_protection();
        s_cpu3_clock_ready = 0U;
        return 0U;
    }

    /* 固定 24 小时制，和 SI7000 Modbus 小时寄存器的 0~23 语义一致。 */
    RTC->CR &= ~RTC_CR_FMT;
    RTC->PRER = (127UL << RTC_PRER_PREDIV_A_Pos) | 255UL;
    cpu3_clock_write_datetime_registers(dt);
    RTC->BKP0R = CPU3_CLOCK_BKP_MARKER;
    cpu3_clock_exit_init_mode();
    cpu3_clock_enable_write_protection();

    s_cpu3_clock_ready = 1U;
    return 1U;
}
