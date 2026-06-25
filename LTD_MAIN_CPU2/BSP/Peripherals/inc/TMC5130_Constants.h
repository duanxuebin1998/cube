/*
 * TMC5130_Constants.h
 *
 *  Created on: 13.06.2018
 *      Author: LK
 */

#ifndef TMC_IC_TMC5130_TMC5130_CONSTANTS_H_
#define TMC_IC_TMC5130_TMC5130_CONSTANTS_H_

#define TMC5130_REGISTER_COUNT   128 /* TMC5130 寄存器地址表容量。 */
#define TMC5130_MOTORS           1 /* 当前项目使用的 TMC5130 电机数量。 */
#define TMC5130_WRITE_BIT        0x80 /* TMC5130 SPI 写操作地址位。 */
#define TMC5130_ADDRESS_MASK     0x7F /* TMC5130 SPI 寄存器地址掩码。 */
#define TMC5130_MAX_VELOCITY     8388096 /* TMC5130 速度寄存器允许的最大值。 */
#define TMC5130_MAX_ACCELERATION 65535 /* TMC5130 加速度寄存器允许的最大值。 */

/* ramp modes (Register TMC5161_RAMPMODE) */
#define TMC5130_MODE_POSITION  0 /* TMC5130 位置模式取值。 */
#define TMC5130_MODE_VELPOS    1 /* TMC5130 正向速度模式取值。 */
#define TMC5130_MODE_VELNEG    2 /* TMC5130 反向速度模式取值。 */
#define TMC5130_MODE_HOLD      3 /* TMC5130 保持模式取值。 */

/* limit switch mode bits (Register TMC5130_SWMODE) */
#define TMC5130_SW_STOPL_ENABLE    0x0001 /* TMC5130 SWMODE 位掩码：左限位使能。 */
#define TMC5130_SW_STOPR_ENABLE    0x0002 /* TMC5130 SWMODE 位掩码：右限位使能。 */
#define TMC5130_SW_STOPL_POLARITY  0x0004 /* TMC5130 SWMODE 位掩码：左限位极性。 */
#define TMC5130_SW_STOPR_POLARITY  0x0008 /* TMC5130 SWMODE 位掩码：右限位极性。 */
#define TMC5130_SW_SWAP_LR         0x0010 /* TMC5130 SWMODE 位掩码：左右限位互换。 */
#define TMC5130_SW_LATCH_L_ACT     0x0020 /* TMC5130 SWMODE 位掩码：左限位有效沿锁存。 */
#define TMC5130_SW_LATCH_L_INACT   0x0040 /* TMC5130 SWMODE 位掩码：左限位无效沿锁存。 */
#define TMC5130_SW_LATCH_R_ACT     0x0080 /* TMC5130 SWMODE 位掩码：右限位有效沿锁存。 */
#define TMC5130_SW_LATCH_R_INACT   0x0100 /* TMC5130 SWMODE 位掩码：右限位无效沿锁存。 */
#define TMC5130_SW_LATCH_ENC       0x0200 /* TMC5130 SWMODE 位掩码：编码器锁存使能。 */
#define TMC5130_SW_SG_STOP         0x0400 /* TMC5130 SWMODE 位掩码：stallGuard 停止使能。 */
#define TMC5130_SW_SOFTSTOP        0x0800 /* TMC5130 SWMODE 位掩码：软停止使能。 */

/* Status bits (Register TMC5130_RAMPSTAT) */
#define TMC5130_RS_STOPL          0x0001 /* TMC5130 RAMPSTAT 位掩码：左限位输入状态。 */
#define TMC5130_RS_STOPR          0x0002 /* TMC5130 RAMPSTAT 位掩码：右限位输入状态。 */
#define TMC5130_RS_LATCHL         0x0004 /* TMC5130 RAMPSTAT 位掩码：左限位锁存状态。 */
#define TMC5130_RS_LATCHR         0x0008 /* TMC5130 RAMPSTAT 位掩码：右限位锁存状态。 */
#define TMC5130_RS_EV_STOPL       0x0010 /* TMC5130 RAMPSTAT 位掩码：左限位事件。 */
#define TMC5130_RS_EV_STOPR       0x0020 /* TMC5130 RAMPSTAT 位掩码：右限位事件。 */
#define TMC5130_RS_EV_STOP_SG     0x0040 /* TMC5130 RAMPSTAT 位掩码：stallGuard 停止事件。 */
#define TMC5130_RS_EV_POSREACHED  0x0080 /* TMC5130 RAMPSTAT 位掩码：到达目标位置事件。 */
#define TMC5130_RS_VELREACHED     0x0100 /* TMC5130 RAMPSTAT 位掩码：达到目标速度。 */
#define TMC5130_RS_POSREACHED     0x0200 /* TMC5130 RAMPSTAT 位掩码：当前位置已到达目标。 */
#define TMC5130_RS_VZERO          0x0400 /* TMC5130 RAMPSTAT 位掩码：速度为零。 */
#define TMC5130_RS_ZEROWAIT       0x0800 /* TMC5130 RAMPSTAT 位掩码：零速等待中。 */
#define TMC5130_RS_SECONDMOVE     0x1000 /* TMC5130 RAMPSTAT 位掩码：第二段运动状态。 */
#define TMC5130_RS_SG             0x2000 /* TMC5130 RAMPSTAT 位掩码：SG。 */

/* Encoderbits (Register TMC5130_ENCMODE) */
#define TMC5130_EM_DECIMAL     0x0400 /* TMC5130 ENCMODE 位掩码：十进制模式。 */
#define TMC5130_EM_LATCH_XACT  0x0200 /* TMC5130 ENCMODE 位掩码：锁存 XACTUAL。 */
#define TMC5130_EM_CLR_XENC    0x0100 /* TMC5130 ENCMODE 位掩码：清零 XENC。 */
#define TMC5130_EM_NEG_EDGE    0x0080 /* TMC5130 ENCMODE 位掩码：负边沿计数。 */
#define TMC5130_EM_POS_EDGE    0x0040 /* TMC5130 ENCMODE 位掩码：正边沿计数。 */
#define TMC5130_EM_CLR_ONCE    0x0020 /* TMC5130 ENCMODE 位掩码：一次性清零。 */
#define TMC5130_EM_CLR_CONT    0x0010 /* TMC5130 ENCMODE 位掩码：连续清零。 */
#define TMC5130_EM_IGNORE_AB   0x0008 /* TMC5130 ENCMODE 位掩码：忽略 AB 相。 */
#define TMC5130_EM_POL_N       0x0004 /* TMC5130 ENCMODE 位掩码：N 相极性。 */
#define TMC5130_EM_POL_B       0x0002 /* TMC5130 ENCMODE 位掩码：B 相极性。 */
#define TMC5130_EM_POL_A       0x0001 /* TMC5130 ENCMODE 位掩码：A 相极性。 */

#endif /* TMC_IC_TMC5130_TMC5130_CONSTANTS_H_ */
