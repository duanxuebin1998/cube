/**
 * @file service_debug.h
 * @brief CPU2 串口维护测试命令的公共分派入口。
 */

#ifndef APPLICATION_SERVICE_DEBUG_H
#define APPLICATION_SERVICE_DEBUG_H

#include <stdint.h>

/**
 * @brief 校验并分派 CPU2 串口维护测试命令。
 *
 * 空缓冲区或未被 SerialCommandParser 严格识别为测试类的命令立即返回，避免仅凭首字符前缀误启动电机或维护动作。
 * 传感器与无线通信、蓝牙配对、模拟量输出、B/BE 电机往返等无需通用测量初始化的命令优先处理；除 A 手动运动和 X 展示模拟外，其余后续测试在分派前调用 MeasureStart。
 * 支持固定频率找液位、电机步进与重复性试验、卷筒参数拟合、位置源切换和静态 SPI 等维护入口；需要 OLED 调试状态的分支会保存并恢复原显示状态。
 * 长时间循环测试会反复检查新命令切换请求，并在需要时慢停电机后退出；这些命令会真实操作传感器、电机、模拟量输出或参数，不得在中断上下文调用。
 *
 * @param command 已完成基础收帧的可修改串口测试命令缓冲区，以 NUL 结尾。
 * @return 已识别并处理返回 1，非测试命令返回 0；空命令或没有匹配到具体测试分支时也返回 0，由上层继续处理。
 * @note 返回 1 只表示测试命令已被本函数消费，不等同于对应硬件测试成功；实际结果由打印、错误码和设备状态判断。
 */
uint8_t ServiceDebug_ProcessSerialCommand(uint8_t *command);
#endif /* APPLICATION_SERVICE_DEBUG_H */
