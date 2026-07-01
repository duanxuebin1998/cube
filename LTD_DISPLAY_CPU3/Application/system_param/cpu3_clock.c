#include "cpu3_clock.h"

#include "main.h"

/*
 * CPU3 本机时钟只用于外部协议展示和 profile 完成时间戳锁存。
 * 它不参与 CPU2 测量控制，也不作为测量状态机的调度依据，避免显示协议反向影响底层测量流程。
 */
#define CPU3_CLOCK_BKP_MARKER_DEFAULT 0x43505533UL
#define CPU3_CLOCK_BKP_MARKER_SET     0x43525443UL
#define CPU3_CLOCK_INIT_TIMEOUT_MS    100U
#define CPU3_CLOCK_LSE_TIMEOUT_MS     1000U
#define CPU3_CLOCK_LSI_TIMEOUT_MS     100U
#define CPU3_CLOCK_DEFAULT_YEAR       2026U
#define CPU3_CLOCK_DEFAULT_MONTH      1U
#define CPU3_CLOCK_DEFAULT_DAY        1U

/* RTC 初始化成功后置位，读取接口用它区分“时间无效”和“时间为 0 点”。 */
static uint8_t s_cpu3_clock_ready = 0U; /* 参数存储模块级变量，保存跨函数共享的业务状态。 */
static Cpu3ClockSource s_cpu3_clock_source = CPU3_CLOCK_SOURCE_NONE;
static Cpu3ClockState s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;

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
 * 该检查是所有读写接口的统一入口，避免非法日期写入 RTC 或暴露给 SI。
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
 * 从 RTC 原始寄存器读取普通二进制时间。
 * 读取路径不依赖 s_cpu3_clock_ready，初始化阶段也可用于判断备份域时间是否合法。
 */
static uint8_t cpu3_clock_read_datetime_registers(Cpu3DateTime *out)
{
    uint32_t tr;
    uint32_t dr;

    if (out == NULL) {
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

static uint8_t cpu3_clock_backup_marker_is_set(void)
{
    return (RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_SET) ? 1U : 0U;
}

static uint8_t cpu3_clock_backup_marker_is_known(void)
{
    return ((RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_DEFAULT) || (RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_SET)) ? 1U : 0U;
}

/*
 * 根据备份标记和当前时钟源刷新对外状态。
 * 是否已校时由 BKP0R 的 SET 标记决定，默认时间只算未校时。
 */
static void cpu3_clock_apply_state(uint8_t calibrated)
{
    if (calibrated == 0U) {
        s_cpu3_clock_state = CPU3_CLOCK_STATE_UNSET;
    } else if (s_cpu3_clock_source == CPU3_CLOCK_SOURCE_LSI) {
        s_cpu3_clock_state = CPU3_CLOCK_STATE_LSI_FALLBACK;
    } else if (s_cpu3_clock_source == CPU3_CLOCK_SOURCE_LSE) {
        s_cpu3_clock_state = CPU3_CLOCK_STATE_LSE_VALID;
    } else {
        s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;
    }
}

/*
 * 等待低速时钟 ready 标志，避免旧板缺 LSE 时卡住启动。
 */
static uint8_t cpu3_clock_wait_rcc_flag(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t tick_start = HAL_GetTick();

    while (__HAL_RCC_GET_FLAG(flag) == RESET) {
        if ((HAL_GetTick() - tick_start) >= timeout_ms) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t cpu3_clock_enable_lse(void)
{
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    return cpu3_clock_wait_rcc_flag(RCC_FLAG_LSERDY, CPU3_CLOCK_LSE_TIMEOUT_MS);
}

static void cpu3_clock_disable_lse(void)
{
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
}

static uint8_t cpu3_clock_enable_lsi(void)
{
    __HAL_RCC_LSI_ENABLE();
    return cpu3_clock_wait_rcc_flag(RCC_FLAG_LSIRDY, CPU3_CLOCK_LSI_TIMEOUT_MS);
}

static uint8_t cpu3_clock_select_rtc_source(uint32_t clock_source)
{
    RCC_PeriphCLKInitTypeDef periph_clk = {0};

    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    periph_clk.RTCClockSelection = clock_source;

    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        return 0U;
    }

    __HAL_RCC_RTC_ENABLE();
    return 1U;
}

/*
 * 已经使用 LSI 且存在用户校时标记时优先保留，避免切换到 LSE 时复位备份域丢时间。
 */
static uint8_t cpu3_clock_existing_lsi_should_be_preserved(void)
{
    Cpu3DateTime current_dt;

    if (cpu3_clock_backup_marker_is_set() == 0U) {
        return 0U;
    }

    if ((RTC->ISR & RTC_ISR_INITS) == 0U) {
        return 0U;
    }

    return cpu3_clock_read_datetime_registers(&current_dt);
}

/*
 * 配置并打开 RTC 时钟源。
 * 新板优先使用 LSE；LSE 不存在或启动失败时自动回退到 LSI，保证旧板能启动。
 */
static uint8_t cpu3_clock_configure_rtc_clock(void)
{
    uint32_t current_source = __HAL_RCC_GET_RTC_SOURCE();

    if (((RCC->BDCR & RCC_BDCR_RTCEN) != 0U) && (current_source == RCC_RTCCLKSOURCE_LSE)) {
        if (cpu3_clock_enable_lse() != 0U) {
            __HAL_RCC_RTC_ENABLE();
            s_cpu3_clock_source = CPU3_CLOCK_SOURCE_LSE;
            return 1U;
        }
    }

    if (((RCC->BDCR & RCC_BDCR_RTCEN) != 0U) && (current_source == RCC_RTCCLKSOURCE_LSI)) {
        if (cpu3_clock_enable_lsi() != 0U) {
            __HAL_RCC_RTC_ENABLE();
            if (cpu3_clock_existing_lsi_should_be_preserved() != 0U) {
                s_cpu3_clock_source = CPU3_CLOCK_SOURCE_LSI;
                return 1U;
            }
        }
    }

    if (cpu3_clock_enable_lse() != 0U) {
        if (cpu3_clock_select_rtc_source(RCC_RTCCLKSOURCE_LSE) != 0U) {
            s_cpu3_clock_source = CPU3_CLOCK_SOURCE_LSE;
            return 1U;
        }
    }

    if (cpu3_clock_enable_lsi() != 0U) {
        cpu3_clock_disable_lse();
        if (cpu3_clock_select_rtc_source(RCC_RTCCLKSOURCE_LSI) != 0U) {
            s_cpu3_clock_source = CPU3_CLOCK_SOURCE_LSI;
            return 1U;
        }
    }

    s_cpu3_clock_source = CPU3_CLOCK_SOURCE_NONE;
    return 0U;
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
    Cpu3DateTime current_dt;
    uint8_t calibrated;
    uint8_t need_default_time = 0U;

    s_cpu3_clock_ready = 0U;
    s_cpu3_clock_source = CPU3_CLOCK_SOURCE_NONE;
    s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (cpu3_clock_configure_rtc_clock() == 0U) {
        return;
    }

    cpu3_clock_disable_write_protection();

    /*
     * BKP0R 的 DEFAULT 标记表示程序写过保守默认值但用户未校时；
     * SET 标记表示用户通过菜单校过时。只有标记无效、RTC 未初始化或时间非法时才改写默认时间。
     */
    calibrated = cpu3_clock_backup_marker_is_set();
    if (cpu3_clock_backup_marker_is_known() == 0U) {
        need_default_time = 1U;
    }

    if ((RTC->ISR & RTC_ISR_INITS) == 0U) {
        need_default_time = 1U;
    } else if (cpu3_clock_read_datetime_registers(&current_dt) == 0U) {
        need_default_time = 1U;
    }

    if (need_default_time != 0U) {
        if (cpu3_clock_enter_init_mode() == 0U) {
            cpu3_clock_enable_write_protection();
            s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;
            return;
        }

        RTC->CR &= ~RTC_CR_FMT;
        RTC->PRER = (127UL << RTC_PRER_PREDIV_A_Pos) | 255UL;
        cpu3_clock_write_datetime_registers(&default_dt);
        RTC->BKP0R = CPU3_CLOCK_BKP_MARKER_DEFAULT;
        cpu3_clock_exit_init_mode();
        calibrated = 0U;
    }

    cpu3_clock_enable_write_protection();
    cpu3_clock_apply_state(calibrated);
    s_cpu3_clock_ready = 1U;
}

/*
 * 读取当前 RTC 时间。
 * 返回 1 表示 out 中字段已通过合法性校验，返回 0 时调用方应保持协议寄存器为 0。
 */
uint8_t Cpu3Clock_GetDateTime(Cpu3DateTime *out)
{
    if ((out == NULL) || (s_cpu3_clock_ready == 0U)) {
        return 0U;
    }

    if (cpu3_clock_read_datetime_registers(out) == 0U) {
        s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;
        return 0U;
    }

    return 1U;
}

/*
 * 设置 CPU3 RTC 时间。
 * 保存成功后写入 SET 备份标记，后续上电不会再覆盖 RTC。
 */
uint8_t Cpu3Clock_SetDateTime(const Cpu3DateTime *dt)
{
    if ((dt == NULL) || (s_cpu3_clock_ready == 0U)) {
        return 0U;
    }

    if (cpu3_clock_validate(dt) == 0U) {
        return 0U;
    }

    cpu3_clock_disable_write_protection();

    if (cpu3_clock_enter_init_mode() == 0U) {
        cpu3_clock_enable_write_protection();
        s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;
        return 0U;
    }

    RTC->CR &= ~RTC_CR_FMT;
    RTC->PRER = (127UL << RTC_PRER_PREDIV_A_Pos) | 255UL;
    cpu3_clock_write_datetime_registers(dt);
    RTC->BKP0R = CPU3_CLOCK_BKP_MARKER_SET;
    cpu3_clock_exit_init_mode();

    cpu3_clock_enable_write_protection();
    cpu3_clock_apply_state(1U);

    return 1U;
}

Cpu3ClockState Cpu3Clock_GetState(void)
{
    return s_cpu3_clock_state;
}

Cpu3ClockSource Cpu3Clock_GetSource(void)
{
    return s_cpu3_clock_source;
}
