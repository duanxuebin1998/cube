/*
 * 模块职责：向密度、液位、水位、姿态和维护业务提供统一传感器服务接口。
 * 调用方向：Application调用本层，本层通过当前SensorDriverOps分派到唯一物理传感器。
 * 流程约束：公共模式稳定等待和错误归因由服务层管理，液位恢复策略留在液位业务模块。
 */
#ifndef SENSOR_SERVICE_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_SERVICE_H_

#include <stdint.h>

#include "sensor_types.h"

/*
 * 函数用途：通过无线滑环自动识别当前物理传感器。
 * 调用场景：CPU2上电初始化和维护诊断重新探测。
 * 关键约束：DM4按V4识别为物理类型14，不在普通探测中发送Safe HELLO。
 */
uint32_t SensorService_Detect(void);

/*
 * 函数用途：判断最近一次物理传感器探测是否有效。
 * 调用场景：测量命令入口检查历史FRAM身份能否用于本次运行。
 * 关键约束：只认本次启动的成功探测，不以持久化旧身份替代。
 */
uint8_t SensorService_IsDetectionValid(void);

/*
 * 函数用途：获取最近一次自动识别的完整结果码。
 * 调用场景：测量入口和诊断命令需要保留具体探测错误时调用。
 * 关键约束：读取不清零，错误码不做归一化。
 */
uint32_t SensorService_GetDetectionResult(void);

/*
 * 函数用途：获取DM4当前协议运行模式。
 * 调用场景：维护诊断和未来Safe模式切换状态查询。
 * 关键约束：非DM4返回NOT_APPLICABLE，Safe模式尚无公开切换入口。
 */
SensorDm4ProtocolMode SensorService_GetDm4ProtocolMode(void);

/*
 * 函数用途：判断当前活动驱动是否为多参数V3传感器。
 * 调用场景：部件诊断保留V3密度模式稳定等待时调用。
 * 关键约束：只查询本次识别结果，不访问UART6。
 */
uint8_t SensorService_IsMultiparamV3(void);

/*
 * 函数用途：判断当前活动驱动是否提供零点霍尔通道。
 * 调用场景：部件诊断决定是否读取DM4零点霍尔电压。
 * 关键约束：只查询能力，不切换模式或使能功能。
 */
int SensorService_SupportsMagneticZeroChannel(void);
/*
 * 函数用途：按当前活动驱动切换密度测量模式。
 * 调用场景：密度、液位恢复和部件参数读取流程。
 * 关键约束：不负责稳定等待，通信超时统一进入无线链路诊断。
 */
uint32_t SensorService_EnableDensityMode(void);

/*
 * 函数用途：按当前活动驱动切换液位测量模式。
 * 调用场景：液位测量、跟随和模式恢复流程。
 * 关键约束：先让电机受控停稳，成功后执行可被命令切换打断的稳定等待。
 */
uint32_t SensorService_EnableLevelMode(void);

/*
 * 函数用途：判断当前活动驱动是否提供水位电容通道。
 * 调用场景：水位测量、回零和部件参数读取前检查。
 * 关键约束：本次识别无效时返回不支持，不发送传感器命令。
 */
int SensorService_SupportsWaterCapChannel(void);

/*
 * 函数用途：判断当前活动驱动是否提供姿态角通道。
 * 调用场景：罐高、回零和部件参数读取前检查。
 * 关键约束：只查询能力，不隐式建立Safe会话。
 */
int SensorService_SupportsGyroChannel(void);

/*
 * 函数用途：读取一次液位模式频率。
 * 调用场景：液位状态判断和主动快照消费。
 * 关键约束：本接口不执行电机动作或模式恢复。
 */
uint32_t SensorService_ReadLevelFrequency(uint32_t *frequency_out);


/*
 * 函数用途：按当前活动驱动读取频率、密度和温度。
 * 调用场景：密度测量、液位辅助判断和诊断读取。
 * 关键约束：三个输出均必须有效，整机固定修正只在服务层执行一次。
 */
uint32_t SensorService_ReadDensity(float *frequency, float *density, float *temp);

/*
 * 函数用途：按当前活动驱动读取水位电容。
 * 调用场景：水位搜索、回零和部件参数读取。
 * 关键约束：先核对能力；DM4读取前自动使能测水且完成后保持开启。
 */
uint32_t SensorService_ReadWaterCapacitance(float *cap_out);

/*
 * 函数用途：读取当前传感器可选的供电电压诊断量。
 * 调用场景：部件参数维护读取DSM扩展数据。
 * 关键约束：不支持时不访问UART6，通信超时统一诊断无线链路。
 */
uint32_t SensorService_ReadSupplyVoltage(float *voltage_v);

/*
 * 函数用途：读取22.5度、45度周期平方均值和动态黏度。
 * 调用场景：部件参数维护读取DSM密度分析扩展数据。
 * 关键约束：三项输出必须同时有效，不支持时不访问UART6。
 */
uint32_t SensorService_ReadDensityAnalysis(float *period_22_5,
                                           float *period_45,
                                           float *dynamic_viscosity);
/*
 * 函数用途：读取DM4零点霍尔电压。
 * 调用场景：零点流程需要磁零点辅助量时调用。
 * 关键约束：读取前自动处理密度模式和功能互斥，完成后保持开启。
 */
uint32_t SensorService_ReadMagneticZeroVoltage(float *voltage_v);

/*
 * 函数用途：按当前活动驱动读取双轴姿态角。
 * 调用场景：罐高、回零和部件参数读取。
 * 关键约束：先核对动态能力；不支持时不伪造零值。
 */
uint32_t SensorService_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg);



#endif /* SENSOR_SERVICE_H_ */
