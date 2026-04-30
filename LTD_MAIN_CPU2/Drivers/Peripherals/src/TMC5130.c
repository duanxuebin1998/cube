#include "TMC5130.h"

#include "assert.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"
#include "fault_manager.h"

#include "sensor.h"    // 传感器相关接口（如称重、防撞检测等）
#include "motor_ctrl.h" // 电机控制上层接口

/* 保持电流固定为同一口径，运行时修改 motor_current 只改变 IRUN。 */
#define TMC5130_MOTOR_IHOLD_VALUE       (5U)
#define TMC5130_MOTOR_IHOLDDELAY_VALUE  (7U)

#define TMC5130_BYTE(value, n)           (((value) >> ((n) << 3)) & 0xFF)
#define TMC5130_SET_IHOLD(a)             (((a) & 0x1F) << 0)
#define TMC5130_SET_IRUN(a)              (((a) & 0x1F) << 8)
#define TMC5130_SET_IHOLDDELAY(a)        (((a) & 0x0F) << 16)
#define TMC5130_IHOLD_IRUN_FIELD_MASK     (0x000F1F1FU)
#ifndef TMC5130_SPI_CS_DELAY_CYCLES
#define TMC5130_SPI_CS_DELAY_CYCLES      (64U)
#endif
#ifndef TMC5130_XACTUAL_READ_TOLERANCE_TICKS
#define TMC5130_XACTUAL_READ_TOLERANCE_TICKS  (8192L)
#endif
#ifndef TMC5130_INIT_WRITE_RETRY_MAX
#define TMC5130_INIT_WRITE_RETRY_MAX          (3U)
#endif

#define TMC5130_REQUIRE_WRITE(handle, address, value)            \
    do {                                                         \
        if (!stpr_writeInt((handle), (address), (value))) {      \
            return MOTOR_TMC_COMM_ERROR;                         \
        }                                                        \
    } while (0)

// 全局步进电机驱动句柄（默认配置）
// 注意：这里只是给出一个全局默认实例，实际项目中也可以在别处重新初始化
TMC5130TypeDef stepper = {
    .spi     = &hspi2,        // 使用的 SPI 句柄
    .cs_port = GPIOB,         // SPI 片选 GPIO 端口
    .cs_pin  = GPIO_PIN_12,   // SPI 片选引脚
    .en_port = GPIOD,         // 使能引脚 GPIO 端口
    .en_pin  = GPIO_PIN_10    // 使能引脚
};

// => SPI 底层封装（仅本文件内部使用）
static bool tmc5130_tryReadArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length, uint32_t *value);
static bool tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
static bool tmc5130_readRegisterOnce(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value);
static bool tmc5130_readXactualStable(TMC5130TypeDef *tmc5130, int32_t *value);
static bool tmc5130_isCloseInt32(int32_t a, int32_t b, int32_t tolerance);
static uint32_t tmc5130_buildCurrentSetting(uint8_t current);
static void tmc5130_logCurrentSetting(uint32_t setting);
static void tmc5130_delayCsGuard(void);
static void tmc5130_initWrite(TMC5130TypeDef *tmc5130,
                              uint8_t address,
                              int32_t value,
                              uint32_t *failed_count);
// <= SPI 底层封装

static void tmc5130_delayCsGuard(void)
{
    for (uint32_t i = 0; i < TMC5130_SPI_CS_DELAY_CYCLES; ++i) {
        __NOP();
    }
}

/************************ SPI 读写封装 ************************/

/**
 * @brief  通过 SPI 读一个寄存器（发送 length 字节，接收 5 字节）
 *         TMC5130 的寄存器读取机制：
 *         - 第一次发送地址，只是“触发”内部准备数据，不使用返回值
 *         - 第二次再发送同一地址，才能读到上一帧准备好的 32bit 数据
 *         此函数只负责发一帧并返回 RX 数据解析出来的 32bit 整数
 *
 * @param  tmc5130  驱动句柄
 * @param  data     TX 缓冲区指针（data[0] 一般为寄存器地址）
 * @param  length   发送/接收长度，一般为 5
 * @retval 组合后的 32bit 数据（rxBuff[1..4]）
 */
static bool tmc5130_tryReadArray(TMC5130TypeDef *tmc5130,
                                 uint8_t *data,
                                 size_t length,
                                 uint32_t *value)
{
    uint8_t rxBuff[5] = { 0, 0, 0, 0, 0 };
    HAL_StatusTypeDef status;

    if ((tmc5130 == NULL) || (data == NULL) || (value == NULL) ||
        (length == 0U) || (length > sizeof(rxBuff))) {
        return false;
    }

    /* TMC5130 的 SPI 帧以 CS 上升沿结束、下一次下降沿重新对齐。
     * 连续读 XACTUAL 时如果 CS 高电平保持太短，现场会出现状态字 0x21 混入数据区，
     * 例如 0x21000000、0x77210000 这类错位值。这里给 CS 建立/保持留出少量 CPU 周期。 */
    tmc5130_delayCsGuard();
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);
    tmc5130_delayCsGuard();
    status = HAL_SPI_TransmitReceive(tmc5130->spi, data, rxBuff, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);
    tmc5130_delayCsGuard();

    if (status != HAL_OK) {
        return false;
    }

    // TMC5130 数据在 rxBuff[1..4]，按大端组合为 32 位
    *value = (((uint32_t)rxBuff[1] << 24) |
              ((uint32_t)rxBuff[2] << 16) |
              ((uint32_t)rxBuff[3] << 8) |
              ((uint32_t)rxBuff[4]));
    return true;
}

/**
 * @brief  通过 SPI 写一帧数据，并返回 HAL 发送是否成功
 *
 * @param  tmc5130 驱动句柄
 * @param  data    待发送数据（data[0] 为寄存器地址 | 写标志位）
 * @param  length  数据长度，一般为 5
 */
static bool tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;

    if ((tmc5130 == NULL) || (data == NULL) ||
        (tmc5130->spi == NULL) || (tmc5130->cs_port == NULL) ||
        (length == 0U)) {
        return false;
    }

    tmc5130_delayCsGuard();
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);
    tmc5130_delayCsGuard();
    status = HAL_SPI_Transmit(tmc5130->spi, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);
    tmc5130_delayCsGuard();

    return (status == HAL_OK);
}

/************************ 寄存器级读写 ************************/

/**
 * @brief  写 4 字节（x1~x4）到指定寄存器地址
 *
 * @param  address 寄存器地址（不带写标志位）
 * @param  x1~x4   四个字节，高字节在前
 */
static bool tmc5130_writeDatagram(TMC5130TypeDef *tmc5130,
                                  uint8_t address,
                                  uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
    uint8_t data[5] = { address | TMC5130_WRITE_BIT, x1, x2, x3, x4 };
    return tmc5130_writeArray(tmc5130, &data[0], 5);
}

/**
 * @brief  向 TMC5130 指定寄存器写入 32 位整数
 *
 * @param  address 寄存器地址
 * @param  value   要写入的 32 位数
 */
bool stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value)
{
    return tmc5130_writeDatagram(
        tmc5130,
        address,
        TMC5130_BYTE(value, 3),
        TMC5130_BYTE(value, 2),
        TMC5130_BYTE(value, 1),
        TMC5130_BYTE(value, 0)
    );
}

static void tmc5130_initWrite(TMC5130TypeDef *tmc5130,
                              uint8_t address,
                              int32_t value,
                              uint32_t *failed_count)
{
    for (uint32_t attempt = 1U; attempt <= TMC5130_INIT_WRITE_RETRY_MAX; ++attempt) {
        if (stpr_writeInt(tmc5130, address, value)) {
            return;
        }

        /* 初始化阶段出现 HAL_BUSY/瞬态干扰时，短延时后重试，避免一次失败直接误判。 */
        HAL_Delay(1U);
    }

    if (failed_count != NULL) {
        (*failed_count)++;
    }
    printf("TMC5130初始化写寄存器失败 | reg=0x%02X | retry=%u\r\n",
           (unsigned int)address,
           (unsigned int)TMC5130_INIT_WRITE_RETRY_MAX);
}

/**
 * @brief  执行一次完整的 TMC5130 寄存器读取流程。
 *         TMC5130 的读操作带流水线：第一帧只提交地址，第二帧才返回上一帧准备好的数据。
 */
static bool tmc5130_readRegisterOnce(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value)
{
    uint8_t  data[5] = { 0, 0, 0, 0, 0 };
    uint32_t raw = 0U;

    if (value == NULL) {
        return false;
    }

    data[0] = address;
    if (!tmc5130_tryReadArray(tmc5130, data, 5, &raw)) { // 第一次触发读取（数据无效）
        return false;
    }

    data[0] = address;
    if (!tmc5130_tryReadArray(tmc5130, data, 5, &raw)) { // 第二次才是真正数据
        return false;
    }

    *value = (int32_t)raw;
    return true;
}

static bool tmc5130_isCloseInt32(int32_t a, int32_t b, int32_t tolerance)
{
    int64_t delta = (int64_t)a - (int64_t)b;

    if (delta < 0) {
        delta = -delta;
    }
    return (delta <= (int64_t)tolerance);
}

/**
 * @brief  稳定读取 XACTUAL。
 *
 * 现场日志中出现 0x21 状态字混入数据区的错位值，例如 0x4DF12100、0xF1210000。
 * 这类值会让上层看到瞬时位置跳变。XACTUAL 是位置核心基准，因此这里要求连续
 * 两次完整读取足够接近；若第一组不稳定，再补读第三组，用后两组稳定值作为结果。
 */
static bool tmc5130_readXactualStable(TMC5130TypeDef *tmc5130, int32_t *value)
{
    int32_t first = 0;
    int32_t second = 0;
    int32_t third = 0;

    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &first)) {
        return false;
    }
    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &second)) {
        return false;
    }

    if (tmc5130_isCloseInt32(first, second, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = second;
        return true;
    }

    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &third)) {
        return false;
    }

    if (tmc5130_isCloseInt32(second, third, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = third;
        return true;
    }
    if (tmc5130_isCloseInt32(first, third, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = third;
        return true;
    }

    printf("TMC5130 XACTUAL连续读取不稳定 | first=%ld | second=%ld | third=%ld\r\n",
           (long)first,
           (long)second,
           (long)third);
    return false;
}

/**
 * @brief  从 TMC5130 指定寄存器读取 32 位整数。
 *
 * XACTUAL 直接影响电机尺带长度和运动判断，单次错位值风险最高，因此使用稳定读取；
 * 其它寄存器保持原来的两帧流水线读取流程，避免改变状态类寄存器的实时语义。
 */
bool stpr_tryReadInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value)
{
    if (value == NULL) {
        return false;
    }

    if (address == TMC5130_XACTUAL) {
        return tmc5130_readXactualStable(tmc5130, value);
    }

    return tmc5130_readRegisterOnce(tmc5130, address, value);
}

int32_t stpr_readInt(TMC5130TypeDef *tmc5130, uint8_t address)
{
    int32_t value = 0;

    if (!stpr_tryReadInt(tmc5130, address, &value)) {
        return 0;
    }
    return value;
}

/************************ 内部速度模式工具 / 位置控制接口 ************************/

/**
 * @brief  以给定速度持续旋转（速度模式）
 *
 * @param  velocity 目标速度（正：正向，负：反向）
 */
static uint32_t tmc5130_rotate(TMC5130TypeDef *tmc5130, int32_t velocity)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, abs(velocity));
    TMC5130_REQUIRE_WRITE(tmc5130,
                          TMC5130_RAMPMODE,
                          (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG);
    return NO_ERROR;
}

/**
 * @brief  停止电机并回到安全的位置模式。
 *
 * 速度模式下只把 VMAX 写成 0 虽然能停住当前动作，但 RAMPMODE 会停留在速度模式。
 * 后续如果只是恢复速度参数并写 VMAX，驱动可能在没有新位置命令的情况下重新运动。
 * 因此停止时把当前位置同步为目标位置，再切回位置模式，保证空闲态写 VMAX 不会启动电机。
 */
uint32_t stpr_stop(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;
    int32_t xactual = 0;

    ret = tmc5130_rotate(tmc5130, 0);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, xactual);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    return NO_ERROR;
}

/**
 * @brief  以给定最大速度，运行到指定位置（位置模式）
 *
 * @param  position   目标位置（步数）
 * @param  velocityMax 最大速度（VMAX）
 */
uint32_t stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, velocityMax);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, position);
    MotorCtrl_PersistRegistersFromDriver();
    return NO_ERROR;
}

/**
 * @brief  在当前位置基础上“相对移动”一定步数
 *         ticks 传入的是相对位移，函数内部会转换成绝对位置并回写到 *ticks
 *
 * @param  ticks       [入/出] 相对位移 / 计算后的绝对目标位置
 * @param  velocityMax 最大速度
 */
uint32_t stpr_moveBy(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax)
{
    int32_t xactual;
    int64_t target;

    if ((tmc5130 == NULL) || (ticks == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    /* 相对位移必须建立在可信的 XACTUAL 上；读失败时不能把当前位置当 0。 */
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    target = (int64_t)xactual + (int64_t)(*ticks);
    if ((target > (int64_t)INT32_MAX) || (target < (int64_t)INT32_MIN)) {
        printf("stpr_moveBy溢出: XACTUAL=%ld, 增量=%ld, 目标=%lld\r\n",
               (long)xactual,
               (long)(*ticks),
               (long long)target);
        return PARAM_RANGE_ERROR;
    }

    *ticks = (int32_t)target;
    return stpr_moveTo(tmc5130, *ticks, velocityMax);
}

/************************ 使能与位置设置 ************************/

/**
 * @brief  禁止驱动（EN 引脚拉高或拉低视硬件设计，一般为高电平关闭）
 */
void stpr_disableDriver(TMC5130TypeDef *tmc5130)
{
    HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_SET);
}

/**
 * @brief  使能驱动（EN 引脚）
 */
void stpr_enableDriver(TMC5130TypeDef *tmc5130)
{
    HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_RESET);
}

/**
 * @brief  同时设置“实际位置”和“目标位置”，相当于软件重置坐标
 */
uint32_t stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, position);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XACTUAL, position);
    MotorCtrl_PersistRegistersFromDriver();
    return NO_ERROR;
}

/************************ 运动等待与故障检测 ************************/

/**
 * @brief  阻塞等待当前运动完成（RAMPSTAT.vzero == 1）
 *         在等待过程中会检测称重/碰撞与 TMC5130 内部故障，
 *         一旦出错立即返回错误码。
 *
 * @retval NO_ERROR                       正常完成
 *         MOTOR_OVERTEMPERATURE          驱动过温
 *         MOTOR_CHARGE_PUMP_UNDER_VOLTAGE 充电泵欠压
 *         MOTOR_UNKNOWN_FEEDBACK         其他未知故障
 *         以及传感器防撞返回的错误码等
 */
uint32_t stpr_waitMove(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;
    int32_t rampstat;
    int32_t gstat;

    while (1) {
        /* RAMPSTAT.vzero=1 表示斜坡发生器速度已经为 0。
         * SPI 读取失败时不能继续等待，否则兼容版 stpr_readInt() 的 0 会让这里无限循环。 */
        if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
            MotorCtrl_SlowStop();
            return MOTOR_TMC_COMM_ERROR;
        }
        if ((rampstat & 0x400) == 0x400) {
            break;
        }

        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前等待运动\r\n");
            MotorCtrl_SlowStop();
            return STATE_SWITCH;
        }

        // 在运动过程中周期性检查防撞（比如称重超限等）
        ret = CheckWeightCollision();    // 防撞检测
        CHECK_ERROR(ret);                // 若有错误直接返回

        // 检查 TMC5130 全局状态；读取失败时停止运动并把错误交给上层处理
        if (!stpr_tryReadInt(tmc5130, TMC5130_GSTAT, &gstat)) {
            MotorCtrl_SlowStop();
            return MOTOR_TMC_COMM_ERROR;
        }
        if (gstat) {
            // 逐位判断具体错误
            if (gstat & (1 << 0)) {
                printf("检测到 TMC5130 过驱或短路故障（GSTAT[0]）\n");
                if (!stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07)) { // 清除所有 GSTAT 标志位
                    return MOTOR_TMC_COMM_ERROR;
                }
                continue; // 清除后继续观察
            }

            if (gstat & (1 << 1)) {
                printf("检测到驱动芯片过温（GSTAT[1]）\n");
                RETURN_ERROR(MOTOR_OVERTEMPERATURE);
            }

            if (gstat & (1 << 2)) {
                printf("检测到 充电泵欠压（GSTAT[2]）\n");
                RETURN_ERROR(MOTOR_CHARGE_PUMP_UNDER_VOLTAGE);
            }

            // 其他错误情况
            if (!stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07)) { // 清除所有标志
                return MOTOR_TMC_COMM_ERROR;
            }
            RETURN_ERROR(MOTOR_UNKNOWN_FEEDBACK);
        }
        MotorCtrl_PollRuntimePosition();
        HAL_Delay(50);
    }
    MotorCtrl_RefreshDebugDrumState();
    return NO_ERROR; // 正常结束运动
}
/************************ 电流 / 速度 / 初始化 ************************/

/**
 * @brief  生成 IHOLD_IRUN 寄存器值。
 *
 * 运行时只改变 IRUN；IHOLD 和 IHOLDDELAY 保持统一口径，避免不同入口写出不同电流配置。
 */
static uint32_t tmc5130_buildCurrentSetting(uint8_t current)
{
    return TMC5130_SET_IHOLD(TMC5130_MOTOR_IHOLD_VALUE) |
           TMC5130_SET_IRUN(current) |
           TMC5130_SET_IHOLDDELAY(TMC5130_MOTOR_IHOLDDELAY_VALUE);
}

/**
 * @brief  记录本次下发的 IHOLD_IRUN 电流字段。
 */
static void tmc5130_logCurrentSetting(uint32_t setting)
{
    const uint32_t fields = setting & TMC5130_IHOLD_IRUN_FIELD_MASK;

    /* IHOLD_IRUN 是写配置寄存器，IFCNT 在 SPI 模式下禁用。
     * 因此 SPI 模式不能用普通回读或 IFCNT 判断本寄存器是否写入成功。 */
    printf("TMC5130电流设置已下发 | IHOLD=%u | IRUN=%u | IHOLDDELAY=%u\r\n",
           (unsigned)(fields & 0x1FU),
           (unsigned)((fields >> 8) & 0x1FU),
           (unsigned)((fields >> 16) & 0x0FU));
}
/**
 * @brief  设置驱动电流（IHOLD_IRUN 寄存器）
 *
 * @param  current  运行电流 IRUN 的编码值（0~31）
 */
uint32_t stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current)
{
    const uint32_t value = tmc5130_buildCurrentSetting(current);

    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_IHOLD_IRUN, value);
    tmc5130_logCurrentSetting(value);
    return NO_ERROR;
}

/**
 * @brief  设置最大速度（VMAX）
 */
uint32_t stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, velocity);
    return NO_ERROR;
}

/**
 * @brief  初始化 TMC5130 步进电机驱动器的寄存器参数
 *         包括斩波配置、加减速、速度、stealthChop、StallGuard 等
 *
 * @param  spi      SPI 句柄
 * @param  cs_port  片选 GPIO 端口
 * @param  cs_pin   片选 GPIO 引脚
 * @param  dir      方向配置（如需从 GCONF 中配置方向，可使用）
 * @param  current  电流设置（IRUN）
 */
uint32_t stpr_initStepper(TMC5130TypeDef *tmc5130,
                          SPI_HandleTypeDef *spi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin,
                          uint8_t dir,
                          uint8_t current)
{
    uint32_t value = 0;
    uint32_t init_write_failed_count = 0U;

#define TMC5130_INIT_WRITE(handle, address, value) \
    tmc5130_initWrite((handle), (address), (value), &init_write_failed_count)

    if ((tmc5130 == NULL) || (spi == NULL) || (cs_port == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    // 更新句柄中的硬件资源
    tmc5130->spi     = spi;
    tmc5130->cs_pin  = cs_pin;
    tmc5130->cs_port = cs_port;
    (void)dir;

    // 基础配置：GCONF
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, 0x0084);
    // 若需要根据 dir 设置方向，可改为：
    // TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, (dir << 4));

    // 空闲功耗降低时间
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TPOWERDOWN, 0x0000000A); // TPOWERDOWN = 10

    // 斩波器配置 CHOPCONF
    TMC5130_INIT_WRITE(
        tmc5130,
        TMC5130_CHOPCONF,
        (2 << 15) |   // TBL：死区时间
        (0 << 24) |   // 其他参数可按需调整
        (3 << 0)  |   // TOFF
        (4 << 7)  |   // HEND
        (0 << 4)      // HSTART
    );

    // 基本加减速、速度参数（需根据实际机械系统调优）
    TMC5130_INIT_WRITE(tmc5130, TMC5130_AMAX,    20 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_A1,      10 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_V1,     800 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_D1,      10 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_DMAX,    20 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_VSTOP,  0x0000000A);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_VSTART, 0x00000005);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TZEROWAIT, 10000);

    // 位置模式 & 初始位置
    TMC5130_INIT_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_XACTUAL,  0x00000000);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_XTARGET,  0x00000000);

    // 设置 IHOLD / IRUN / IHOLDDELAY
    value = tmc5130_buildCurrentSetting(current);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_IHOLD_IRUN, value);
    tmc5130_logCurrentSetting(value);

    // 启用 PWM 模式（stealthChop）
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, 0x00000004); // EN_PWM_MODE=1

    // 速度阈值（PWM、CoolStep 等，这里默认关闭）
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TPWMTHRS, 0);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TCOOLTHRS, 0);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_THIGH, 20);

    // PWM 配置：AUTO=1, Fclk/1024, 振幅限制=200, Grad=1 等
    TMC5130_INIT_WRITE(tmc5130, TMC5130_PWMCONF, 0x000401C8);

    // 清除 GSTAT 错误标志
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GSTAT, 0x07);

    if (init_write_failed_count != 0U) {
        printf("TMC5130初始化失败 | 写失败次数=%lu，已停止初始化流程\r\n",
               (unsigned long)init_write_failed_count);
#undef TMC5130_INIT_WRITE
        return MOTOR_TMC_COMM_ERROR;
    }

#undef TMC5130_INIT_WRITE
    return NO_ERROR;
}

