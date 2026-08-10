#ifndef LH_REGISTER_MAP_H_
/* LH_REGISTER_MAP_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define LH_REGISTER_MAP_H_

#include <stdint.h>

/* LH 线圈：连续地址 0x0000~0x0010。 */
/* LH 协议线圈命令地址：电机回零；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_BACK_ZERO                    0x0000U
/* LH 协议线圈命令地址：执行油位测量；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_FIND_OIL                     0x0001U
/* LH 协议线圈命令地址：执行水位测量；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_FIND_WATER                   0x0002U
/* LH 协议线圈命令地址：执行探底/罐底测量；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_FIND_BOTTOM                  0x0003U
/* LH 协议线圈命令地址：执行水位跟随；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_FOLLOW_WATER                 0x0004U
/* LH 协议线圈命令地址：取消当前测量流程；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CANCEL_MEASUREMENT           0x0005U
/* LH 协议线圈命令地址：进入维护模式；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_MAINTENANCE_ENTER            0x0006U
/* LH 协议线圈命令地址：退出维护模式；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_MAINTENANCE_EXIT             0x0007U
/* LH 协议线圈命令地址：清除全部继电器锁存报警；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CLEAR_ALL_RELAY_LATCHED      0x0008U
/* LH 协议线圈命令地址：执行油位标定；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CALIBRATE_OIL                0x0009U
/* LH 协议线圈命令地址：请求读取部分参数；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_READ_PART_PARAMS             0x000AU
/* LH 协议线圈命令地址：手动向上移动电机；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_MOVE_UP                      0x000BU
/* LH 协议线圈命令地址：手动向下移动电机；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_MOVE_DOWN                    0x000CU
/* LH 协议线圈命令地址：执行油位修正；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CORRECT_OIL                  0x000DU
/* LH 协议线圈命令地址：执行零点标定；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CALIBRATE_ZERO               0x000EU
/* LH 协议线圈命令地址：执行水位标定；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CALIBRATE_WATER              0x000FU
/* LH 协议线圈命令地址：执行罐高标定；主站通过 FC05 写入有效触发值后映射为一次设备命令，地址本身不代表持续运行状态。 */
#define LH_COIL_CALIBRATE_TANKHEIGHT         0x0010U
/* LH 命令线圈表的地址容量和排他上界；合法线圈地址范围为 0（含）到 LH_COIL_COUNT（不含）。 */
#define LH_COIL_COUNT                        0x0011U

/* LH 输入寄存器：连续地址 0x0000~0x001B。 */
/* LH 输入寄存器“设备状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_DEVICE_STATE                   0x0000U
/* LH 输入寄存器“当前故障码”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_ERROR_CODE                     0x0001U
/* LH 输入寄存器“油位测量值”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_OIL_LEVEL                      0x0003U
/* LH 输入寄存器“水位测量值”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_WATER_LEVEL                    0x0005U
/* LH 输入寄存器“罐底/罐高测量值”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_TANK_BOTTOM_HEIGHT             0x0007U
/* LH 输入寄存器“传感器实时位置”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_REALTIME_POSITION              0x0009U
/* LH 输入寄存器“继电器 K1 逻辑动作状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_RELAY_K1_ACTION                0x000BU
/* LH 输入寄存器“继电器 K2 逻辑动作状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_RELAY_K2_ACTION                0x000CU
/* LH 输入寄存器“继电器 K3 逻辑动作状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_RELAY_K3_ACTION                0x000DU
/* LH 输入寄存器“维护模式实际生效状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_MAINTENANCE_ACTIVE             0x000EU
/* LH 输入寄存器“传感器 X 轴倾角”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_ANGLE_X                        0x000FU
/* LH 输入寄存器“传感器 Y 轴倾角”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_ANGLE_Y                        0x0010U
/* LH 输入寄存器“传感器实时频率”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_FREQUENCY                      0x0011U
/* LH 输入寄存器“传感器实时温度”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_TEMPERATURE                    0x0013U
/* LH 输入寄存器“传感器实时幅值”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_AMPLITUDE                      0x0014U
/* LH 输入寄存器“水相电容放大 10 倍值”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_WATER_CAPACITANCE_X10          0x0015U
/* LH 输入寄存器“电机运行状态”的起始地址；该字段在线表中占 1 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_MOTOR_STATE                    0x0017U
/* LH 输入寄存器“电机实时速度”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_MOTOR_SPEED                    0x0018U
/* LH 输入寄存器“电机当前扭矩”的起始地址；该字段在线表中占 2 个 16 位寄存器，按 LH 字序打包。 */
#define LH_IR_CURRENT_TORQUE                 0x001AU
/* LH 输入寄存器镜像的容量和排他结束地址；FC04 连续读取区间必须整体小于该上界。 */
#define LH_INPUT_REGISTER_COUNT              0x001CU

/* LH 保持寄存器通用及电机参数：0x0000~0x0014。 */
/* LH 保持寄存器“罐高”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_TANK_HEIGHT                    0x0000U
/* LH 保持寄存器“油位标定值”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_CALIBRATE_OIL_LEVEL            0x0002U
/* LH 保持寄存器“水位标定值”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_CALIBRATE_WATER_LEVEL          0x0004U
/* LH 保持寄存器“罐高标定值”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_CALIBRATE_TANK_HEIGHT          0x0006U
/* LH 保持寄存器“电机命令移动距离”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_MOTOR_COMMAND_DISTANCE         0x0008U
/* LH 保持寄存器“电机运行电流”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_MOTOR_CURRENT                  0x000AU
/* LH 保持寄存器“电机最大速度”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_MAX_MOTOR_SPEED                0x000BU
/* LH 保持寄存器“测量盲区”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_BLIND_ZONE                     0x000DU
/* LH 保持寄存器“编码轮周长”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_ENCODER_WHEEL_CIRCUMFERENCE    0x000FU
/* LH 保持寄存器“液位探头到水位探头距离”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE 0x0011U
/* LH 保持寄存器“电机最大下行距离”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_MAX_DOWN_DISTANCE              0x0012U
/* LH 保持寄存器“传感器标识”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_SENSOR_ID                      0x0013U

/* LH AO 参数：0x0015~0x0026。 */
/* LH 保持寄存器“AO 工作模式”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_WORK_MODE                   0x0015U
/* LH 保持寄存器“AO 电流量程模式”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_CURRENT_MODE                0x0016U
/* LH 保持寄存器“AO 过程量来源”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_OUTPUT_SOURCE               0x0017U
/* LH 保持寄存器“AO 电流修正量，单位 0.001 mA”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_CURRENT_CORRECTION_MA_X1000 0x0018U
/* LH 保持寄存器“AO 固定输出电流，单位 0.01 mA”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_FIXED_CURRENT_MA_X100       0x001AU
/* LH 保持寄存器“AO 量程 0% 对应过程量，单位 0.01 mm”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_RANGE_0_01MM                0x001CU
/* LH 保持寄存器“AO 量程 100% 对应过程量，单位 0.01 mm”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_RANGE_100_01MM              0x001EU
/* LH 保持寄存器“AO 阻尼时间，单位 0.1 s”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_DAMPING_X10_S               0x0020U
/* LH 保持寄存器“AO 故障输出模式”的起始地址；该字段占 1 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_FAULT_MODE                  0x0022U
/* LH 保持寄存器“AO 故障输出电流，单位 0.01 mA”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_FAULT_CURRENT_MA_X100       0x0023U
/* LH 保持寄存器“AO 上电初始电流，单位 0.01 mA”的起始地址；该字段占 2 个 16 位寄存器，并映射到 CPU2 对应设备参数。 */
#define LH_HR_AO_POWER_ON_CURRENT_MA_X100    0x0025U

/* LH K1~K3 参数：每路连续占 0x12 个寄存器。 */
/* LH 保持寄存器继电器 K1 配置块的起始地址；三路块使用完全相同的字段布局，块间步长由 LH_HR_RELAY_CHANNEL_REG_COUNT 规定。 */
#define LH_HR_RELAY_K1_BASE                  0x0027U
/* LH 保持寄存器继电器 K2 配置块的起始地址；三路块使用完全相同的字段布局，块间步长由 LH_HR_RELAY_CHANNEL_REG_COUNT 规定。 */
#define LH_HR_RELAY_K2_BASE                  0x0039U
/* LH 保持寄存器继电器 K3 配置块的起始地址；三路块使用完全相同的字段布局，块间步长由 LH_HR_RELAY_CHANNEL_REG_COUNT 规定。 */
#define LH_HR_RELAY_K3_BASE                  0x004BU
/* 单路 LH 继电器配置块占用的 16 位寄存器数量 0x12；用于校验跨字段写入并计算 K1～K3 的块边界。 */
#define LH_HR_RELAY_CHANNEL_REG_COUNT        0x0012U

/* 根据继电器块基址 base 计算“工作模式”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_OPERATING_MODE(base)     ((base) + 0x0000U)
/* 根据继电器块基址 base 计算“数字量输入源”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_DIGITAL_SOURCE(base)     ((base) + 0x0001U)
/* 根据继电器块基址 base 计算“触点常开/常闭类型”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_CONTACT_TYPE(base)       ((base) + 0x0002U)
/* 根据继电器块基址 base 计算“报警比较模式”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_ALARM_MODE(base)         ((base) + 0x0003U)
/* 根据继电器块基址 base 计算“错误时替代值”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_ERROR_VALUE(base)        ((base) + 0x0004U)
/* 根据继电器块基址 base 计算“报警过程量来源”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_ALARM_SOURCE(base)       ((base) + 0x0005U)
/* 根据继电器块基址 base 计算“高高报警阈值”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_HH_ALARM_VALUE(base)     ((base) + 0x0006U)
/* 根据继电器块基址 base 计算“高报警阈值”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_H_ALARM_VALUE(base)      ((base) + 0x0008U)
/* 根据继电器块基址 base 计算“低报警阈值”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_L_ALARM_VALUE(base)      ((base) + 0x000AU)
/* 根据继电器块基址 base 计算“低低报警阈值”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_LL_ALARM_VALUE(base)     ((base) + 0x000CU)
/* 根据继电器块基址 base 计算“报警回差”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_ALARM_HYSTERESIS(base)   ((base) + 0x000EU)
/* 根据继电器块基址 base 计算“报警阻尼系数”字段的 LH 保持寄存器地址；base 必须取 K1、K2 或 K3 的合法块起始地址。 */
#define LH_HR_RELAY_DAMPING_FACTOR(base)     ((base) + 0x0010U)

/* LH 保持寄存器镜像容量和排他结束地址 0x005D；FC03/FC10 区间校验必须同时满足字段完整性和该总边界。 */
#define LH_HOLDING_REGISTER_COUNT            0x005DU

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(LH_COIL_CALIBRATE_TANKHEIGHT + 1U == LH_COIL_COUNT,
               "LH coil map must remain continuous");
_Static_assert(LH_IR_CURRENT_TORQUE + 2U == LH_INPUT_REGISTER_COUNT,
               "LH input register map must remain continuous");
_Static_assert(LH_HR_AO_POWER_ON_CURRENT_MA_X100 + 2U == LH_HR_RELAY_K1_BASE,
               "LH AO and relay maps must remain continuous");
_Static_assert(LH_HR_RELAY_K2_BASE - LH_HR_RELAY_K1_BASE == LH_HR_RELAY_CHANNEL_REG_COUNT,
               "LH K1 register span must remain continuous");
_Static_assert(LH_HR_RELAY_K3_BASE - LH_HR_RELAY_K2_BASE == LH_HR_RELAY_CHANNEL_REG_COUNT,
               "LH K2 register span must remain continuous");
_Static_assert(LH_HR_RELAY_K3_BASE + LH_HR_RELAY_CHANNEL_REG_COUNT == LH_HOLDING_REGISTER_COUNT,
               "LH K3 register span must remain continuous");
#endif

#endif /* LH_REGISTER_MAP_H_ */
