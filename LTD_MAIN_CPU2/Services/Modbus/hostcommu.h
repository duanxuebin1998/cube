#ifndef __HOSTCOMMU_H
/* __HOSTCOMMU_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __HOSTCOMMU_H
#include "main.h"

#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_SET) /* RS485 收发控制切换为接收模式。 */
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_RESET) /* RS485 收发控制切换为发送模式。 */

#define HOSTCOMMU_SENDLENGTH 1000 /* 主机通信发送缓冲区长度。 */

/**
 * @brief 把 CPU2 主机 Modbus 从站地址初始化为默认地址 1。
 *
 * @return 固定返回 0；当前初始化只设置内存中的默认从站地址，不执行可失败的外设访问。
 */
int HostCommuInit(void);
/**
 * @brief 校验并处理 CPU2 UART5 收到的一帧主机 Modbus RTU 请求，生成响应或恢复 DMA 接收。
 *
 * 处理顺序为帧长、从站地址和 CRC 校验；合法请求解析功能码、起始地址和数量，再分派 FC03、FC04 或 FC10，并为正常或异常响应追加低字节在前的 CRC16。
 * 帧长、地址或 CRC 不合法时只记录延后日志并恢复 UART5 DMA 接收；响应 DMA 启动失败时同样立即退回接收，避免 RS485 总线停留在发送方向。
 *
 * @param rcvbuff 接收到的 Modbus 帧数据；已经由 UART5 空闲中断结束接收，有效字节范围由 rcvcount 指定。
 * @param rcvcount 接收到的数据长度，单位字节；对应 rcvbuff 中实际有效的帧范围，必须大于 3 且小于 MAXRCVLENGTH。
 */
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount);
/**
 * @brief 在主循环中输出主机通信异常日志。
 *
 * 函数依次检查帧长、地址、CRC、功能码和寄存器范围五类延后日志；每类日志在中断侧只保存最近一次详情并累计限频期间的抑制次数。
 * 复制 pending 日志时先保存 PRIMASK 并短暂关闭中断，清除 pending 后按原中断状态恢复，避免 UART5 中断同时改写共享缓存。
 * 帧长错误输出实际长度与最大长度，地址或 CRC 错误输出地址和长度，功能码或寄存器范围错误输出功能码、起始地址和寄存器数量；存在抑制记录时追加累计次数。
 * 格式化完成后通过统一通信告警接口在线程态打印，单类日志一次只消费一份快照。
 *
 * 在主循环中输出 HostCommuProcess 延后的异常日志，避免中断里直接打印。
 *
 * @note 统一故障日志会走 printf，因此必须延后到主循环，避免拉长串口中断时间。
 */
void HostCommu_ProcessDeferredLogs(void);

#endif	  

