#include "cpu3_clock.h"

#include "main.h"

/*
 * CPU3 本机时钟只用于外部协议展示和 profile 完成时间戳锁存。
 * 它不参与 CPU2 测量控制，也不作为测量状态机的调度依据，避免显示协议反向影响底层测量流程。
 */
/* RTC 备份域使用默认时钟配置时写入的标记 0x43505533（ASCII 'CPU3'）；用于兼容既有设备的初始化状态。 */
#define CPU3_CLOCK_BKP_MARKER_DEFAULT 0x43505533UL
/* RTC 备份域已完成初始化的标记值 0x43525443（ASCII 'CRTC'）；只有标记和 RTC 状态均可信时才保留已有时间。 */
#define CPU3_CLOCK_BKP_MARKER_SET     0x43525443UL
/* RTC 初始化状态等待超时 100 ms。 */
#define CPU3_CLOCK_INIT_TIMEOUT_MS    100U
/* 外部低速晶振 LSE 就绪等待超时 1000 ms；超时后转入备用时钟处理。 */
#define CPU3_CLOCK_LSE_TIMEOUT_MS     1000U
/* 内部低速振荡器 LSI 就绪等待超时 100 ms。 */
#define CPU3_CLOCK_LSI_TIMEOUT_MS     100U
/* RTC 无有效备份时间时使用的默认年份 2026。 */
#define CPU3_CLOCK_DEFAULT_YEAR       2026U
/* RTC 无有效备份时间时使用的默认月份 1 月。 */
#define CPU3_CLOCK_DEFAULT_MONTH      1U
/* RTC 无有效备份时间时使用的默认日期 1 日。 */
#define CPU3_CLOCK_DEFAULT_DAY        1U

/* RTC 初始化成功后置位，读取接口用它区分“时间无效”和“时间为 0 点”。 */
static uint8_t s_cpu3_clock_ready = 0U; /* CPU3 RTC 已完成初始化且可安全读写的标志。 */
static Cpu3ClockSource s_cpu3_clock_source = CPU3_CLOCK_SOURCE_NONE;
static Cpu3ClockState s_cpu3_clock_state = CPU3_CLOCK_STATE_ERROR;

/**
 * @brief 将 RTC 寄存器中的一个 BCD 字段转成普通二进制值。
 *
 * 仅解析单个字段，不访问硬件寄存器，便于当前时间和日期共用。
 *
 * @param value 时钟使用的输入数值。
 * @return 返回将 RTC 寄存器中的一个 BCD 字段转成普通二进制值得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint8_t cpu3_clock_bcd2bin(uint32_t value)
{
    return (uint8_t)((((value >> 4U) & 0x0FU) * 10U) + (value & 0x0FU));
}

/**
 * @brief 判断年份是否闰年。
 *
 * 当前只支持 2000~2099，但保留完整规则，避免后续扩展时埋边界问题。
 *
 * @param year 完整年份数值。
 * @return 1 表示年份能被 4 整除，且不能被 100 整除或同时能被 400 整除，属于公历闰年；0 表示其它年份。
 */
static uint8_t cpu3_clock_is_leap_year(uint16_t year)
{
    return (((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U))) ? 1U : 0U;
}

/**
 * @brief 获取指定年月的最大日期。
 *
 * 返回 0 表示月份非法，调用方据此把整组时间判为无效。
 *
 * @param year 完整年份数值。
 * @param month 月份，合法范围为 1～12。
 * @return 返回指定月份的天数；月份非法时返回 0。
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

/**
 * @brief 校验 CPU3 时间结构体的字段范围。
 *
 * 该检查是所有读写接口的统一入口，避免非法日期写入 RTC 或暴露给 SI。
 *
 * @param dt 待校验、换算或写入的 CPU3 日期时间结构。
 * @return 1 表示年、月、日、时、分、秒及星期字段均合法；任一字段越界时返回 0。
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

/**
 * @brief 根据日期计算 RTC DR 所需的星期字段。
 *
 * 外部协议暂不使用星期，但硬件寄存器写入时必须保持合法。
 *
 * @param year 完整年份数值。
 * @param month 月份，合法范围为 1～12。
 * @param day 日期中的日字段，取值范围由当前月份和年份共同限制。
 * @return 返回 RTC 使用的星期编号 1 至 7，其中 1 表示星期一、7 表示星期日。
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

/**
 * @brief 关闭 RTC 写保护。
 *
 * 只能在主线程初始化/设置时间路径调用，不在中断中使用。
 */
static void cpu3_clock_disable_write_protection(void)
{
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;
}

/**
 * @brief 恢复 RTC 写保护。
 *
 * 与 cpu3_clock_disable_write_protection 成对使用，防止运行期误写 RTC 寄存器。
 */
static void cpu3_clock_enable_write_protection(void)
{
    RTC->WPR = 0xFFU;
}

/**
 * @brief 进入 RTC 初始化模式。
 *
 * 带超时保护，避免 LSI 或 RTC 异常时卡住 CPU3 启动流程。
 *
 * @return 1 表示 RTC 已进入初始化模式；等待 INITF 超时或写保护操作失败时返回 0。
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

/**
 * @brief 退出 RTC 初始化模式。
 *
 * 调用方负责在退出前完成 TR/DR/PRER 等寄存器写入。
 */
static void cpu3_clock_exit_init_mode(void)
{
    RTC->ISR &= ~RTC_ISR_INIT;
}

/**
 * @brief 将已校验的时间写入 RTC TR/DR 原始寄存器。
 *
 * 所有字段在这里统一拆成 BCD，避免多个写路径各自处理格式。
 *
 * @param dt 待校验、换算或写入的 CPU3 日期时间结构。
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

/**
 * @brief 从 RTC 原始寄存器读取普通二进制时间。
 *
 * 读取路径不依赖 s_cpu3_clock_ready，初始化阶段也可用于判断备份域时间是否合法。
 *
 * @param out RTC 时间输出对象；原始寄存器和字段范围均有效时写入年月日、时分秒及 valid 标志。
 * @return 返回 out->valid；1 表示 RTC 原始寄存器已转换且字段范围有效，0 表示参数或时间字段非法。
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

/**
 * @brief 判断 RTC 备份域标记是否为已初始化值。
 *
 * @return 1 表示 RTC 备份域标记为已初始化值；0 表示 RTC 备份域标记不是已初始化值。
 */
static uint8_t cpu3_clock_backup_marker_is_set(void)
{
    return (RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_SET) ? 1U : 0U;
}

/**
 * @brief 判断 RTC 备份域标记是否属于受支持版本。
 *
 * @return 1 表示 RTC 备份域标记属于受支持版本；0 表示 RTC 备份域标记不属于受支持版本。
 */
static uint8_t cpu3_clock_backup_marker_is_known(void)
{
    return ((RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_DEFAULT) || (RTC->BKP0R == CPU3_CLOCK_BKP_MARKER_SET)) ? 1U : 0U;
}

/**
 * @brief 根据备份标记和当前时钟源刷新对外状态。
 *
 * 是否已校时由 BKP0R 的 SET 标记决定，默认时间只算未校时。
 *
 * @param calibrated RTC 校准状态；true 表示备份域与时间基准已经校准。
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

/**
 * @brief 等待低速时钟 ready 标志，避免旧板缺 LSE 时卡住启动。
 *
 * @param flag 等待置位的 RCC 就绪标志。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return 1 表示目标 RCC ready 标志在时限内置位；等待超时返回 0。
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

/**
 * @brief 开启 LSE 并在 1 s 上限内等待稳定。
 *
 * @return 1 表示 LSE 已在 1 s 内稳定；未就绪或超时返回 0。
 */
static uint8_t cpu3_clock_enable_lse(void)
{
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    return cpu3_clock_wait_rcc_flag(RCC_FLAG_LSERDY, CPU3_CLOCK_LSE_TIMEOUT_MS);
}

/**
 * @brief 关闭 LSE 并等待硬件就绪标志清除。
 */
static void cpu3_clock_disable_lse(void)
{
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
}

/**
 * @brief 开启 LSI 并在 100 ms 上限内等待稳定。
 *
 * @return 1 表示 LSI 已在 100 ms 内稳定；未就绪或超时返回 0。
 */
static uint8_t cpu3_clock_enable_lsi(void)
{
    __HAL_RCC_LSI_ENABLE();
    return cpu3_clock_wait_rcc_flag(RCC_FLAG_LSIRDY, CPU3_CLOCK_LSI_TIMEOUT_MS);
}

/**
 * @brief 配置并使能 RTC 外设时钟源，返回 HAL 配置结果。
 *
 * @param clock_source 时钟来源。
 * @return 1 表示 RTC 时钟源配置和使能成功；HAL RCC 配置失败时返回 0。
 */
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

/**
 * @brief 已经使用 LSI 且存在用户校时标记时优先保留，避免切换到 LSE 时复位备份域丢时间。
 *
 * @return 1 表示当前 LSI 时间有效且带有用户校时标记，应保留现有备份域；否则返回 0。
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

/**
 * @brief 配置并打开 RTC 时钟源。
 *
 * 新板优先使用 LSE；LSE 不存在或启动失败时自动回退到 LSI，保证旧板能启动。
 *
 * @return 1 表示已有或新选择的 RTC 时钟源可用；LSE、LSI 均无法建立时返回 0。
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

/**
 * @brief 初始化 CPU3 本机 RTC。
 *
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

/**
 * @brief 读取当前 RTC 时间。
 *
 * 返回 1 表示 out 中字段已通过合法性校验，返回 0 时调用方应保持协议寄存器为 0。
 *
 * @param out 当前 RTC 时间输出对象；缓存或硬件读取成功时写入经过范围校验的日期时间。
 * @return 1 表示已返回有效 RTC 时间，0 表示输出参数非法或当前 RTC 数据无效。
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

/**
 * @brief 设置 CPU3 RTC 时间。
 *
 * 保存成功后写入 SET 备份标记，后续上电不会再覆盖 RTC。
 *
 * @param dt 待校验、换算或写入的 CPU3 日期时间结构。
 * @return 1 表示年月日、时分秒已通过范围校验并成功写入 RTC 日期和时间；0 表示日期时间字段越界，或 HAL_RTC_SetDate/HAL_RTC_SetTime 写入失败。
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

/**
 * @brief 返回 CPU3 RTC 当前初始化与有效性状态。
 *
 * @return 返回当前 RTC 获取状态枚举，用于区分未初始化、有效、无效和恢复中状态。
 */
Cpu3ClockState Cpu3Clock_GetState(void)
{
    return s_cpu3_clock_state;
}

/**
 * @brief 返回 CPU3 RTC 当前采用的 LSE、LSI 或未建立时钟源状态。
 *
 * @return 返回当前 RTC 时钟源枚举，用于区分 LSE、LSI 和尚未建立时钟源。
 */
Cpu3ClockSource Cpu3Clock_GetSource(void)
{
    return s_cpu3_clock_source;
}
