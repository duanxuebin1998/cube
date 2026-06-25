/*
 * TMC5130_Register.h
 *
 *  Created on: 30.09.2016
 *      Author: ed based on te
 */

#ifndef TMC5130_REGISTER_H
#define TMC5130_REGISTER_H

/* ===== TMC5130 register set ===== */

#define TMC5130_GCONF       0x00 /* TMC5130 GCONF 全局配置寄存器地址。 */
#define TMC5130_GSTAT       0x01 /* TMC5130 GSTAT 全局状态寄存器地址。 */
#define TMC5130_IFCNT       0x02 /* TMC5130 IFCNT SPI 通信计数寄存器地址。 */
#define TMC5130_SLAVECONF   0x03 /* TMC5130 SLAVECONF 从机配置寄存器地址。 */
#define TMC5130_IOIN        0x04 /* TMC5130 IOIN 输入状态寄存器地址。 */
#define TMC5130_X_COMPARE   0x05 /* TMC5130 X_COMPARE 位置比较寄存器地址。 */

#define TMC5130_IHOLD_IRUN  0x10 /* TMC5130 IHOLD_IRUN 电流配置寄存器地址。 */
#define TMC5130_TPOWERDOWN  0x11 /* TMC5130 TPOWERDOWN 掉电延时寄存器地址。 */
#define TMC5130_TSTEP       0x12 /* TMC5130 TSTEP 步进周期寄存器地址。 */
#define TMC5130_TPWMTHRS    0x13 /* TMC5130 TPWMTHRS PWM 阈值寄存器地址。 */
#define TMC5130_TCOOLTHRS   0x14 /* TMC5130 TCOOLTHRS coolStep 阈值寄存器地址。 */
#define TMC5130_THIGH       0x15 /* TMC5130 THIGH 高速阈值寄存器地址。 */

#define TMC5130_RAMPMODE    0x20 /* TMC5130 RAMPMODE 运动模式寄存器地址。 */
#define TMC5130_XACTUAL     0x21 /* TMC5130 XACTUAL 当前实际位置寄存器地址。 */
#define TMC5130_VACTUAL     0x22 /* TMC5130 VACTUAL 当前速度寄存器地址。 */
#define TMC5130_VSTART      0x23 /* TMC5130 VSTART 起步速度寄存器地址。 */
#define TMC5130_A1          0x24 /* TMC5130 A1 第一段加速度寄存器地址。 */
#define TMC5130_V1          0x25 /* TMC5130 V1 第一段速度寄存器地址。 */
#define TMC5130_AMAX        0x26 /* TMC5130 AMAX 最大加速度寄存器地址。 */
#define TMC5130_VMAX        0x27 /* TMC5130 VMAX 最大速度寄存器地址。 */
#define TMC5130_DMAX        0x28 /* TMC5130 DMAX 最大减速度寄存器地址。 */
#define TMC5130_D1          0x2A /* TMC5130 D1 最后一段减速度寄存器地址。 */
#define TMC5130_VSTOP       0x2B /* TMC5130 VSTOP 停止速度寄存器地址。 */
#define TMC5130_TZEROWAIT   0x2C /* TMC5130 TZEROWAIT 零速等待寄存器地址。 */
#define TMC5130_XTARGET     0x2D /* TMC5130 XTARGET 目标位置寄存器地址。 */

#define TMC5130_VDCMIN      0x33 /* TMC5130 VDCMIN dcStep 最小速度寄存器地址。 */
#define TMC5130_SWMODE      0x34 /* TMC5130 SWMODE 限位开关模式寄存器地址。 */
#define TMC5130_RAMPSTAT    0x35 /* TMC5130 RAMPSTAT 运动状态寄存器地址。 */
#define TMC5130_XLATCH      0x36 /* TMC5130 XLATCH 锁存位置寄存器地址。 */

#define TMC5130_ENCMODE     0x38 /* TMC5130 ENCMODE 编码器模式寄存器地址。 */
#define TMC5130_XENC        0x39 /* TMC5130 XENC 编码器位置寄存器地址。 */
#define TMC5130_ENC_CONST   0x3A /* TMC5130 ENC_CONST 编码器换算常数寄存器地址。 */
#define TMC5130_ENC_STATUS  0x3B /* TMC5130 ENC_STATUS 编码器状态寄存器地址。 */
#define TMC5130_ENC_LATCH   0x3C /* TMC5130 ENC_LATCH 编码器锁存寄存器地址。 */

#define TMC5130_MSLUT0      0x60 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT1      0x61 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT2      0x62 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT3      0x63 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT4      0x64 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT5      0x65 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT6      0x66 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUT7      0x67 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUTSEL    0x68 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSLUTSTART  0x69 /* TMC5130 微步正弦查表相关寄存器地址。 */
#define TMC5130_MSCNT       0x6A /* TMC5130 MSCNT 微步计数寄存器地址。 */
#define TMC5130_MSCURACT    0x6B /* TMC5130 MSCURACT 微步实际电流寄存器地址。 */

#define TMC5130_CHOPCONF    0x6C /* TMC5130 CHOPCONF 斩波配置寄存器地址。 */
#define TMC5130_COOLCONF    0x6D /* TMC5130 COOLCONF coolStep 配置寄存器地址。 */
#define TMC5130_DCCTRL      0x6E /* TMC5130 DCCTRL dcStep 控制寄存器地址。 */
#define TMC5130_DRVSTATUS   0x6F /* TMC5130 DRV_STATUS 驱动状态寄存器地址。 */
#define TMC5130_PWMCONF     0x70 /* TMC5130 PWMCONF PWM 配置寄存器地址。 */
#define TMC5130_PWMSTATUS   0x71 /* TMC5130 PWMSTATUS PWM 状态寄存器地址。 */
#define TMC5130_ENCM_CTRL   0x72 /* TMC5130 ENCM_CTRL 编码器控制寄存器地址。 */
#define TMC5130_LOST_STEPS  0x73 /* TMC5130 LOST_STEPS 丢步计数寄存器地址。 */

#endif /* TMC5130_REGISTER_H */
