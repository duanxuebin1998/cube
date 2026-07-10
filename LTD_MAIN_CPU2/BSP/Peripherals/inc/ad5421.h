#ifndef sil__AD5421_H
#define sil__AD5421_H
#include "main.h"

#define AD5421_CS_GPIO_PORT   GPIOB /* AD5421 片选 GPIO 端口。 */
#define AD5421_CS_PIN         GPIO_PIN_6 /* AD5421 片选 GPIO 引脚。 */

/* ============================== */
/* 寄存器地址宏定义 */
/* ============================== */
#define DELAY30US  delay_us(100) /* AD5421 时序使用的短延时宏。 */
#define SPI2WRITE00 SPI2_ReadWriteByte(0x00) /* SPI2 发送 0x00 并读取返回字节的宏。 */

/* 满量程电流范围 */
#define STARTFULLSCALE 3.2 /* AD5421 起始满量程电流值，单位 mA。 */
#define STOPFULLSCALE 24.0 /* AD5421 终止满量程电流值，单位 mA。 */
/* AD5421 寄存器指令 */
#define WRITEDAC 0x01u /* AD5421 写 DAC 寄存器命令。 */
#define WRITECONTROL 0x02u /* AD5421 写控制寄存器命令。 */
#define RESETAD5421REG 0x07u /* AD5421 复位寄存器命令。 */
#define NOOPAD5421 0x09u /* AD5421 空操作命令。 */
#define READCONTROL 0x82u /* AD5421 读控制寄存器命令。 */
#define READFAULT 0x85u /* AD5421 读故障寄存器命令。 */
/* SPI 看门狗开关 */
#define CUR_SPION_COMMAND 0xC000u /* SPI 看门狗开启，4s */
#define CUR_SPIOFF_COMMAND 0x1000u /* SPI 看门狗关闭 */
#define CUR_AUTO_FAULT_READBACK_OFF_COMMAND 0x0800u /* 禁止自动 fault 回读，允许专用寄存器回读 */
#define CUR_SPIOFF_READBACK_COMMAND (CUR_SPIOFF_COMMAND | CUR_AUTO_FAULT_READBACK_OFF_COMMAND) /* AD5421 关闭 SPI 看门狗并允许专用回读的控制字。 */

#define AD5421_FAULT_FLAG_SPI_WRITE    0x00000001u /* AD5421 故障位标志：AD5421 故障 FLAG SPI 写入。 */
#define AD5421_FAULT_FLAG_SPI_READ     0x00000002u /* AD5421 故障位标志：AD5421 故障 FLAG SPI 读取。 */
#define AD5421_FAULT_FLAG_READBACK     0x00000004u /* AD5421 故障位标志：AD5421 故障 FLAG 回读。 */
#define AD5421_FAULT_FLAG_PIN          0x00000008u /* 历史 PB8 位，当前硬件未接入 */
#define AD5421_FAULT_FLAG_STATUS       0x00000010u /* AD5421 故障位标志：AD5421 故障 FLAG 状态。 */


/* 电流设置 */
#define CURRENT_INIT 				3.5 /* AD5421 初始化阶段输出电流，单位 mA。 */
#define CURRENT_UNDERAOL 			3.6 /* 模拟量低于量程报警时输出电流，单位 mA。 */
#define CURRENT_GREATERAOH 			21.0 /* 模拟量高于量程报警时输出电流，单位 mA。 */
#define CURRENT_EQUIPMENT_ERROR 	22.0 /* 设备故障时输出电流，单位 mA。 */
#define CURRENT_CHECKMODE_MIN 		3.5 /* 校验模式最小输出电流，单位 mA。 */
#define CURRENT_CHECKMODE_MAX 		24.0 /* 校验模式最大输出电流，单位 mA。 */
#define CURRENT_CONFIGMODE 			23.0 /* 配置模式输出电流，单位 mA。 */
#define CURRENT_MEAMODE 			21.5 /* 测量模式默认输出电流，单位 mA。 */


uint32_t Ad5421Init(void);
uint32_t AD5421_SetCurrent(float mA);
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100);
uint32_t AD5421_RecoverCurrentX100(uint32_t target_mA_x100);
uint32_t AD5421_PollDiagnostics(void);
uint32_t AD5421_GetFaultFlags(void);
uint32_t AD5421_GetFaultRegister(void);
uint8_t AD5421_SetTraceSuppressed(uint8_t suppress);
uint8_t AD5421_IsAccessBusy(void);

#endif

