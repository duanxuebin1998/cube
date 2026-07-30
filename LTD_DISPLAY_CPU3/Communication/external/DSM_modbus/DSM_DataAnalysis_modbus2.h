#ifndef _DSM_DATAANALYSIS_MODBUS2_H
/* _DSM_DATAANALYSIS_MODBUS2_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _DSM_DATAANALYSIS_MODBUS2_H
#include "DSM_SlaveModbus_modbus2.h"
#include "main.h"
#include "system_parameter.h"

/* #define SOFTOFVERSION_0 1213 / / 程序版本 */
/* #define SOFTOFVERSION_1 'P' / / 程序版本 */
/* #define SOFTOFVERSION_2 'B' / / 程序版本 */
/* #define SOFTOFVERSION_3 'C' / / 程序版本 */
/* #define SOFTOFVERSION_4 'D' / / 程序版本 */
/* #define SOFTOFVERSION_5 'E' / / 程序版本 */
/* 第六位是数据定义 */
#define PARAMETER_ERROR -1
/* DSM 参数写入专用失败返回值 -2；用于区分参数落地失败与普通协议解析结果。 */
#define PARAMETER_WRITE_FAIL -2

/* DSM 数据分析返回值 0：请求解析和业务处理成功。 */
#define RETURN_OK 			0
/* DSM 数据分析返回值 -1：从站内部处理失败。 */
#define RETURN_SLAVEFAIL 		-1
/* DSM 数据分析返回值 -2：当前功能或命令不受支持。 */
#define RETURN_UNSUPPORTED 	-2
/* DSM 数据分析返回值 -3：请求地址未在已定义映射内。 */
#define RETURN_UNDEFADDRESS -3
/* DSM 数据分析返回值 -4：数据或命令次序未定义；保留历史拼写 DATEORDER 以兼容现有调用。 */
#define RETURN_UNDEFDATEORDER -4
/* DSM 数据分析返回值 -5：从站忙，调用方可在退避后重试。 */
#define RETURN_SLAVEBUSY -5

/**
  * @brief 将当前设备参数和兼容固定值刷新到 DSM 保持寄存器影子。
 */
void SystemParameterSet(void);
/**
 * @brief 将 DSM 保持寄存器写入范围映射到设备参数并同步至 CPU2。
 *
 * 函数仅解析本次写入范围完整覆盖的参数字段，并把对应寄存器值刷新到 g_deviceParams。
 * 本地参数更新完成后统一调用 DeviceParams_SyncAllToCPU2；只有 CPU2 确认全部参数同步成功，外部 Modbus 写请求才可视为成功。
 * 处理顺序固定为：先从 DSM_HoldingRegisterArray 刷新本次完整覆盖的 g_deviceParams 字段，再与参数元数据和 CPU2
 * 当前值对比，最后通过统一板间写入口下发差异参数。
 *
 * @param startadd 本次 DSM 写保持寄存器请求的起始地址。
 * @param reamount 本次 DSM 写请求连续覆盖的寄存器数量。
 * @return 0 表示本次覆盖字段已映射并获得 CPU2 同步确认；PARAMETER_WRITE_FAIL 表示 CPU2 链路不可用或同步未获确认。
 * @note 没有对应 DeviceParameters 字段的旧兼容寄存器保持忽略；CPU2 链路不可用或同步未获确认时禁止返回成功。
 */
int UpdateDeviceParamsFromLegacyRegs(int startadd, int reamount);
/**
 * @brief 写入或设置Modbus 协议中的 Input_Write 逻辑。
 */
void Input_Write(void);
#endif
