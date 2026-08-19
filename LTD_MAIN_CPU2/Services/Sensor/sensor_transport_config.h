/*
 * 模块职责：集中声明传感器服务恢复和UART6单次事务共用的重试、超时及缓冲参数。
 * 层次边界：服务层恢复次数与协议事务重试次数含义不同，禁止合并或重复套用。
 * 修改约束：时间单位均为毫秒；调整前必须核对DSM、V3、V4和无线AT共享链路影响。
 */
#ifndef SENSOR_TRANSPORT_CONFIG_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_TRANSPORT_CONFIG_H_

/*
 * 服务层恢复参数：SENSOR_COMM_MAX_RETRY用于液位无效值恢复，模式稳定等待只在服务层执行。
 * 这些参数不得下沉到驱动操作表，否则一次业务调用会出现重复重试或重复等待。
 */
#define SENSOR_COMM_MAX_RETRY 10

/*
 * UART6单次协议事务参数：DSM、V3和V4交互请求共享三次上限，但各协议自行校验应答。
 * UART6所有权冲突不进入该重试次数，冲突应直接返回模式未就绪，由上层决定何时重试。
 */
#define UART6_COMM_MAX_RETRY 3U
/* 普通传感器事务重试前等待时间，单位ms。 */
#define SENSOR_COMM_RETRY_DELAY_MS 300
/* 收到协议错误后再次尝试前等待时间，单位ms。 */
#define SENSOR_COMM_ERROR_RETRY_DELAY_MS 300
/* 切换到液位模式后的业务稳定等待时间，单位ms。 */
#define SENSOR_LEVEL_MODE_SETTLE_MS 10000U
/* DSM单次命令最大尝试次数，沿用UART6事务上限。 */
#define DSM_MAX_RETRY UART6_COMM_MAX_RETRY
/* DSM校验或响应错误后的重试等待时间，单位ms。 */
#define DSM_BCC_DELAY SENSOR_COMM_ERROR_RETRY_DELAY_MS
/* DSM下一次发送前的保护等待时间，单位ms。 */
#define DSM_PRE_SEND_DELAY SENSOR_COMM_RETRY_DELAY_MS
/* DSM最短有效响应长度，包含数据、BCC和换行契约的最小边界。 */
#define DSM_MIN_RESP_LEN 3
/* DSM及共用固定帧事务的默认接收超时，单位ms。 */
#define DSM_CMD_TIMEOUT 1000
/* 传感器UART6通用临时接收缓冲容量，单位字节。 */
#define RX_BUF_LEN 128
/* 启用DSM协议调试编译分支。 */
#define DEBUG_DSM
/* UART6逐字节调试日志开关，0表示关闭。 */
#define DEBUG_UART6 0

#endif /* SENSOR_TRANSPORT_CONFIG_H_ */
