#ifndef _DATAANALYSIS_MODBUS_H
/* _DATAANALYSIS_MODBUS_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _DATAANALYSIS_MODBUS_H

#include <stdint.h>
#include <stdbool.h>

#define RETURN_OK 0 /* 运行正常 */
#define RETURN_EXEFAIL -1 /* 发生故障 */
#define RETURN_UNSUPPORTED -2 /* 不支持的操作 */
#define RETURN_UNDEFADDRESS -3 /* 非法数据地址 */

/**
 * @brief 解析03功能码保持寄存器数据。
 */
void AnalysisHoldRegister(void); /* 解析保持寄存器数据 */

/**
 * @brief 将 g_deviceParams 写入保持寄存器数组。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 */
void WriteDeviceParamsToHoldingRegisters(uint16_t *HoldingRegisterArray); /* 写入保持寄存器 */
/**
 * @brief 从保持寄存器数组读取数据到 g_deviceParams。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 * @note command 一般由线圈或功能码触发, 这里按照保持寄存器映射也支持读回。
 */
void ReadDeviceParamsFromHoldingRegisters(uint16_t *HoldingRegisterArray); /* 从输入寄存器读取设备参数 */
/**
 * @brief 将输入寄存器数组解析回 MeasurementResult 结构体。
 *
 * 函数按 CPU2/CPU3 共享输入寄存器地址表，依次恢复设备状态、调试数据、液位、水位、罐高、单点测量、单点监测和密度分布平均值。
 * 同质的密度分布点阵统一循环解析 MAX_MEASUREMENT_POINTS 个点，每点按温度、密度、位置、标准密度、VCF20 和重量密度六个 32 位字段恢复。
 * 随后恢复无线配对与连接状态、四路继电器报警运行态、AO 固定运行块、SI Profile 生命周期、固定点代际计数器以及维护和继电器状态槽。
 * 无符号量、有符号量和 float 分别使用共享寄存器解码函数，保持 CPU2 写入端的高低字顺序和原始位模式不变。
 *
 * 将测量结果写入输入寄存器数组。
 *
 * @param regs 输入寄存器数组（uint16_t 数组）；内容是已经通过板间响应校验的 CPU2 快照，至少包含共享地址表要求的完整范围，函数从中恢复 CPU3 的
 *             g_measurement。
 * @note 调用方必须提供已完成长度、CRC、功能码和代际一致性校验的完整输入寄存器快照；本函数只做字段映射，不重新验证帧或缓冲区长度。
 */
void read_measurement_result_from_InputRegisters(uint16_t *regs); /* 从输入寄存器数组读取测量结果 */












#endif



