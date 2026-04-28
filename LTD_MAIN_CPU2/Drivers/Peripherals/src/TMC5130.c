#include "TMC5130.h"

#include "assert.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"
#include "fault_manager.h"

#include "sensor.h"    // 传感器相关接口（如称重、防撞检测等）
#include "motor_ctrl.h" // 电机控制上层接口

// 全局步进电机驱动句柄（默认配置）
// 注意：这里只是给出一个全局默认实例，实际项目中也可以在别处重新初始化
TMC5130TypeDef stepper = {
    .home_state      = HOME,        // 回零状态机当前状态
    .prev_home_state = HOME,        // 上一次的回零状态
    .control_type    = INTERNAL,    // 控制类型：内部控制
    .spi             = &hspi2,      // 使用的 SPI 句柄
    .drvstat         = 0,           // DRVSTATUS 寄存器缓存
    .sg_flag         = 0,           // StallGuard 触发标志
    .sg_result       = 0,           // StallGuard 结果值
    .cs_actual       = 0,           // 实际电流（编码值）
    .rampstat        = 0,           // RAMPSTAT 缓存
    .homing_done     = 0,           // 回零完成标志

    // 片选与使能脚 GPIO 端口与引脚
    .cs_port = GPIOB,               // SPI 片选使用 GPIOB
    .cs_pin  = GPIO_PIN_12,         // 片选引脚
    .en_port = GPIOD,               // 使能引脚 GPIO 端口
    .en_pin  = GPIO_PIN_10          // 使能引脚
};

// => SPI 底层封装
static bool tmc5130_tryReadArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length, uint32_t *value);
uint32_t tmc5130_readArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
void     tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
// <= SPI 底层封装

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

    // 片选拉低，开始 SPI 通信；无论收发是否成功，退出前都恢复片选高电平
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(tmc5130->spi, data, rxBuff, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);

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

uint32_t tmc5130_readArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length)
{
    uint32_t value = 0U;

    (void)tmc5130_tryReadArray(tmc5130, data, length, &value);
    return value;
}

/**
 * @brief  通过 SPI 写一帧数据（不关心返回值）
 *
 * @param  tmc5130 驱动句柄
 * @param  data    待发送数据（data[0] 为寄存器地址 | 写标志位）
 * @param  length  数据长度，一般为 5
 */
void tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length)
{
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(tmc5130->spi, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);
}

/************************ 寄存器级读写 ************************/

/**
 * @brief  写 4 字节（x1~x4）到指定寄存器地址
 *
 * @param  address 寄存器地址（不带写标志位）
 * @param  x1~x4   四个字节，高字节在前
 */
void tmc5130_writeDatagram(TMC5130TypeDef *tmc5130,
                           uint8_t address,
                           uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
    uint8_t data[5] = { address | TMC5130_WRITE_BIT, x1, x2, x3, x4 };
    tmc5130_writeArray(tmc5130, &data[0], 5);
}

/**
 * @brief  向 TMC5130 指定寄存器写入 32 位整数
 *
 * @param  address 寄存器地址
 * @param  value   要写入的 32 位数
 */
void stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value)
{
    tmc5130_writeDatagram(
        tmc5130,
        address,
        BYTE(value, 3),
        BYTE(value, 2),
        BYTE(value, 1),
        BYTE(value, 0)
    );
}

/**
 * @brief  从 TMC5130 指定寄存器读取 32 位整数
 *         注意：TMC5130 读取需要两次访问：
 *         1. 第一次发送地址，触发内部准备数据
 *         2. 第二次再发送同一地址，才能读到上一帧的寄存器内容
 *
 * @param  address 寄存器地址
 * @retval 寄存器 32bit 数据
 */
bool stpr_tryReadInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value)
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

int32_t stpr_readInt(TMC5130TypeDef *tmc5130, uint8_t address)
{
    int32_t value = 0;

    if (!stpr_tryReadInt(tmc5130, address, &value)) {
        return 0;
    }
    return value;
}

/************************ 速度 / 位置控制接口 ************************/

/**
 * @brief  以给定速度持续旋转（速度模式）
 *
 * @param  velocity 目标速度（正：正向，负：反向）
 */
void tmc5130_rotate(TMC5130TypeDef *tmc5130, int32_t velocity)
{
    // 设置绝对速度
    stpr_writeInt(tmc5130, TMC5130_VMAX, abs(velocity));
    // 设置运行模式与方向
    stpr_writeInt(
        tmc5130,
        TMC5130_RAMPMODE,
        (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG
    );
}

/**
 * @brief  以给定速度向右旋转
 */
void stpr_right(TMC5130TypeDef *tmc5130, uint32_t velocity)
{
    tmc5130_rotate(tmc5130, velocity);
}

/**
 * @brief  以给定速度向左旋转
 */
void stpr_left(TMC5130TypeDef *tmc5130, uint32_t velocity)
{
    tmc5130_rotate(tmc5130, -((int32_t)velocity));
}

/**
 * @brief  停止电机（将目标速度设置为 0）
 */
void stpr_stop(TMC5130TypeDef *tmc5130)
{
//	printf("stpr_stop\r\n");
    tmc5130_rotate(tmc5130, 0);
}

/**
 * @brief  以给定最大速度，运行到指定位置（位置模式）
 *
 * @param  position   目标位置（步数）
 * @param  velocityMax 最大速度（VMAX）
 */
void stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax)
{
    // 位置模式
    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

    // 设置位置模式的最大速度
    stpr_writeInt(tmc5130, TMC5130_VMAX, velocityMax);

    // 设置目标位置
    stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
    motorPersistRegistersFromDriver();
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
    stpr_moveTo(tmc5130, *ticks, velocityMax);
    return NO_ERROR;
}

/**
 * @brief  以“角度”形式进行移动（内部换算为步数），位置模式
 *
 * @param  angle       目标角度（单位：度，正负均可）
 * @param  velocityMax 最大速度
 * @note   这里假设 360° 对应 51200 步（即 51200/360 步/度）
 */
void stpr_moveAngle(TMC5130TypeDef *tmc5130, float angle, uint32_t velocityMax)
{
    int32_t position;

    position = (int32_t)((angle * 51200.0f) / 360.0f);

    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    stpr_writeInt(tmc5130, TMC5130_VMAX, velocityMax);
    stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
    motorPersistRegistersFromDriver();
}

/**
 * @brief  获取当前电机实际位置（XACTUAL）
 */
int32_t stpr_getPos(TMC5130TypeDef *tmc5130)
{
    int32_t XActual;
    XActual = stpr_readInt(tmc5130, TMC5130_XACTUAL);
    return XActual;
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
void stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position)
{
    stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
    stpr_writeInt(tmc5130, TMC5130_XACTUAL, position);
    motorPersistRegistersFromDriver();
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
            motorSlowStop();
            return MOTOR_TMC_COMM_ERROR;
        }
        if ((rampstat & 0x400) == 0x400) {
            break;
        }

        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前等待运动\r\n");
            motorSlowStop();
            return STATE_SWITCH;
        }

        // 在运动过程中周期性检查防撞（比如称重超限等）
        ret = CheckWeightCollision();    // 防撞检测
        CHECK_ERROR(ret);                // 若有错误直接返回

        // 检查 TMC5130 全局状态；读取失败时停止运动并把错误交给上层处理
        if (!stpr_tryReadInt(tmc5130, TMC5130_GSTAT, &gstat)) {
            motorSlowStop();
            return MOTOR_TMC_COMM_ERROR;
        }
        if (gstat) {
            // 逐位判断具体错误
            if (gstat & (1 << 0)) {
                printf("检测到 TMC5130 过驱或短路故障（GSTAT[0]）\n");
                stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07); // 清除所有 GSTAT 标志位
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
            stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07); // 清除所有标志
            RETURN_ERROR(MOTOR_UNKNOWN_FEEDBACK);
        }
        motorPollRuntimePosition();
        HAL_Delay(50);
    }
    motorRefreshDebugDrumState();
    return NO_ERROR; // 正常结束运动
}
/************************ 电流 / 速度 / 初始化 ************************/

/**
 * @brief  设置驱动电流（IHOLD_IRUN 寄存器）
 *
 * @param  current  运行电流 IRUN 的编码值（0~31）
 */
void stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current)
{
    // IHOLD_IRUN: IHOLD=6, IRUN=current, IHOLDDELAY=7
    stpr_writeInt(tmc5130, TMC5130_IHOLD_IRUN,
                  SET_IHOLD(6) | SET_IRUN(current) | SET_IHOLDDELAY(7));
}

/**
 * @brief  设置最大速度（VMAX）
 */
void stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity)
{
    stpr_writeInt(tmc5130, VMAX, velocity);
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
void stpr_initStepper(TMC5130TypeDef *tmc5130,
                      SPI_HandleTypeDef *spi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin,
                      uint8_t dir,
                      uint8_t current)
{
    uint32_t value = 0;

    // 更新句柄中的硬件资源
    tmc5130->spi        = spi;
    tmc5130->cs_pin     = cs_pin;
    tmc5130->cs_port    = cs_port;
    tmc5130->homing_done = 0;
    tmc5130->home_state  = HOME;
    tmc5130->prev_home_state = HOME;

    // 基础配置：GCONF
    stpr_writeInt(tmc5130, TMC5130_GCONF, 0x0084);
    // 若需要根据 dir 设置方向，可改为：
    // stpr_writeInt(tmc5130, TMC5130_GCONF, (dir << 4));

    // 空闲功耗降低时间
    stpr_writeInt(tmc5130, TMC5130_TPOWERDOWN, 0x0000000A); // TPOWERDOWN = 10

    // 斩波器配置 CHOPCONF
    stpr_writeInt(
        tmc5130,
        TMC5130_CHOPCONF,
        (2 << 15) |   // TBL：死区时间
        (0 << 24) |   // 其他参数可按需调整
        (3 << 0)  |   // TOFF
        (4 << 7)  |   // HEND
        (0 << 4)      // HSTART
    );

    // 基本加减速、速度参数（需根据实际机械系统调优）
    stpr_writeInt(tmc5130, TMC5130_AMAX,    20 * 32);
    stpr_writeInt(tmc5130, TMC5130_A1,      10 * 32);
    stpr_writeInt(tmc5130, TMC5130_V1,     800 * 32);
    stpr_writeInt(tmc5130, TMC5130_D1,      10 * 32);
    stpr_writeInt(tmc5130, DMAX,            20 * 32);
    stpr_writeInt(tmc5130, TMC5130_VSTOP,  0x0000000A);
    stpr_writeInt(tmc5130, TMC5130_VSTART, 0x00000005);
    stpr_writeInt(tmc5130, TMC5130_TZEROWAIT, 10000);

    // 位置模式 & 初始位置
    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    stpr_writeInt(tmc5130, TMC5130_XACTUAL,  0x00000000);
    stpr_writeInt(tmc5130, TMC5130_XTARGET,  0x00000000);

    // 设置 IHOLD / IRUN / IHOLDDELAY
    value = SET_IHOLD(5) | SET_IRUN(current) | SET_IHOLDDELAY(7);
    stpr_writeInt(tmc5130, IHOLD_IRUN, value);

    // 启用 PWM 模式（stealthChop）
    stpr_writeInt(tmc5130, READ_ACCESS, 0x00000004); // EN_PWM_MODE=1

    // 速度阈值（PWM、CoolStep 等，这里默认关闭）
    stpr_writeInt(tmc5130, TPWM_THRS, 0);
    stpr_writeInt(tmc5130, TCOOLTHRS, 0);
    stpr_writeInt(tmc5130, THIGH, 20);

    // PWM 配置：AUTO=1, Fclk/1024, 振幅限制=200, Grad=1 等
    stpr_writeInt(tmc5130, PWMCONF, 0x000401C8);

    // 清除 GSTAT 错误标志
    stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07);

}

/************************ StallGuard 回零（简单版本） ************************/

/**
 * @brief  使用 StallGuard 实现一次回零（简单版本）
 *
 * @param  homing_speed        回零速度（VMAX）
 * @param  stallguardthreshold StallGuard 灵敏度阈值（SGT）
 *                             数值越低越敏感，所需力越小
 */
void stpr_home(TMC5130TypeDef *tmc5130, uint16_t homing_speed, uint8_t stallguardthreshold)
{
    uint16_t stall_speed;

    // 根据 TMC5130 手册换算 TCOOLTHRS 对应的速度阈值
    stall_speed = 16777216 / homing_speed;
    // 可根据细分修正，这里注释掉：
    // stall_speed = stall_speed / 16;
    // 略高一点的速度阈值，留下 10% 余量
    stall_speed = (uint16_t)(stall_speed * 1.10);

    // StallGuard 回零时关闭 stealthChop，改用电流斩波以确保 SG 有效
    stpr_writeInt(tmc5130, TMC5130_GCONF, 0x1080);
    // 设置 StallGuard 阈值
    stpr_writeInt(tmc5130, TMC5130_COOLCONF, ((stallguardthreshold & 0x7F) << 16));
    // TCOOLTHRS：速度阈值，低于该速度启用 StallGuard
    stpr_writeInt(tmc5130, TMC5130_TCOOLTHRS, stall_speed);
    // SWMODE：开启 StallGuard 相关检测
    stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x400);
    // 回零时的最大加速度
    stpr_writeInt(tmc5130, TMC5130_AMAX, 1000);

    // 设置速度模式，朝端点方向运动（这里使用正向，可根据机械方向调整为 VELNEG）
    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);
    stpr_writeInt(tmc5130, TMC5130_VMAX, homing_speed);

    HAL_Delay(20);

    // 等待运动结束（RAMPSTAT.vzero == 1）；读失败时停止并退出，避免 SPI 异常导致死等。
    while (1) {
        int32_t rampstat = 0;
        if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
            motorSlowStop();
            return;
        }
        if ((rampstat & 0x400) == 0x400) {
            break;
        }
    }

    // 到达端点后：切换回位置模式，清除开关配置，位置归零
    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD);
    stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x0);
    stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    stpr_writeInt(tmc5130, TMC5130_DMAX, 0xFFFF);
    // 回到 stealthChop 以获得更安静的运行
    stpr_writeInt(tmc5130, TMC5130_GCONF, 0x1080);
    // 归零位置
    stpr_writeInt(tmc5130, TMC5130_XACTUAL, 0x0);
    stpr_writeInt(tmc5130, TMC5130_XTARGET, 0x0);
    motorPersistRegistersFromDriver();

    HAL_Delay(200);
}


