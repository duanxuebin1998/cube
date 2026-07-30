#ifndef _DATAANALYSIS_MODBUS_H
/* _DATAANALYSIS_MODBUS_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _DATAANALYSIS_MODBUS_H

#include <stdint.h>
#include <stdbool.h>

#define RETURN_OK 0 /* 运行正常 */
#define RETURN_EXEFAIL -1 /* 发生故障 */
#define RETURN_UNSUPPORTED -2 /* 不支持的操作 */
#define RETURN_UNDEFADDRESS -3 /* 非法数据地址 */


/* void WriteDeviceParamsToHoldingRegisters(int *p_holdregister);/ /写入保持寄存器 */
/**
 * @brief 将 g_deviceParams 写入保持寄存器数组。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * void ReadDeviceParamsFromHoldingRegisters(int *p_inputregister);/ /从输入寄存器读取设备参数。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 */
void WriteDeviceParamsToHoldingRegisters(uint16_t *HoldingRegisterArray);
/**
 * @brief 从保持寄存器数组读取数据到 g_deviceParams。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 * @note command 一般由线圈或功能码触发, 这里按照保持寄存器映射也支持读回。
 */
void ReadDeviceParamsFromHoldingRegisters(uint16_t *HoldingRegisterArray);
/**
 * @brief 将当前测量结果按 Modbus 输入寄存器布局写入缓存。
 * @param regs 输入寄存器缓存首地址。
 */
void write_measurement_result_to_InputRegisters(uint16_t *regs); /* 将测量结果写入输入寄存器数组 */













#endif



