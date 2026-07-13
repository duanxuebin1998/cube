#ifndef ERROR_LOG_H_
#define ERROR_LOG_H_

#include <stdint.h>

#define ERROR_LOG_MODULE_MOTOR   "电机" /* 错误日志模块名文本：电机。 */
#define ERROR_LOG_MODULE_ENCODER "编码器" /* 错误日志模块名文本：编码器。 */
#define ERROR_LOG_MODULE_SENSOR  "传感器" /* 错误日志模块名文本：传感器。 */
#define ERROR_LOG_MODULE_WEIGHT  "扭力" /* 错误日志模块名文本：扭力。 */
#define ERROR_LOG_MODULE_PARAM   "参数" /* 错误日志模块名文本：参数。 */
#define ERROR_LOG_MODULE_COMM    "通信" /* 错误日志模块名文本：通信。 */
#define ERROR_LOG_MODULE_SLIPRING_COMM "滑环通信" /* 错误日志模块名文本：滑环通信。 */
#define ERROR_LOG_MODULE_MEASURE "测量" /* 错误日志模块名文本：测量。 */
#define ERROR_LOG_MODULE_AO_OUTPUT "模拟量输出" /* 错误日志模块名文本：模拟量输出。 */
#define ERROR_LOG_MODULE_SYSTEM  "系统" /* 错误日志模块名文本：系统。 */
#define ERROR_LOG_MODULE_UNKNOWN "未知" /* 错误日志模块名文本：未知。 */
#define ERROR_LOG_TEXT_UNKNOWN   "未知文本" /* 错误日志兜底文本：未知文本。 */
#define ERROR_LOG_REASON_UNKNOWN "未知原因" /* 错误日志原因文本：未知。 */
#define ERROR_LOG_ACTION_UNKNOWN "未知处理" /* 错误日志处理动作文本：未知。 */

#define ERROR_LOG_OP_ERROR_EXIT       "置错误状态" /* 错误日志操作名文本：置错误状态。 */
#define ERROR_LOG_OP_SET_ERROR_STATE  "设置错误状态" /* 错误日志操作名文本：设置错误状态。 */
#define ERROR_LOG_OP_DRIVER_CHECK     "驱动状态检查" /* 错误日志操作名文本：驱动器 检查。 */
#define ERROR_LOG_OP_READ_LEVEL       "读取液位" /* 错误日志操作名文本：读取 液位。 */
#define ERROR_LOG_OP_SWITCH_MODE      "切换模式" /* 错误日志操作名文本：切换 模式。 */
#define ERROR_LOG_OP_READ_FLOAT_PARAM "读取浮点参数" /* 错误日志操作名文本：读取 浮点 参数。 */
#define ERROR_LOG_OP_READ_INT_PARAM   "读取整数参数" /* 错误日志操作名文本：读取 整数 参数。 */
#define ERROR_LOG_OP_READ_LEVEL_FREQ  "读取液位频率" /* 错误日志操作名文本：读取 液位 频率。 */
#define ERROR_LOG_OP_FRAM_FALLBACK    "FRAM参数分区回退" /* 错误日志操作名文本：FRAM 参数分区回退。 */
#define ERROR_LOG_OP_SEARCH_OIL_LEVEL "搜索液位" /* 错误日志操作名文本：搜索液位。 */
#define ERROR_LOG_OP_FOLLOW_OIL_LEVEL "跟随液位" /* 错误日志操作名文本：跟随液位。 */
#define ERROR_LOG_OP_ENABLE_LEVEL_MODE "启用液位模式" /* 错误日志操作名文本：启用液位模式。 */
#define ERROR_LOG_OP_SEARCH_WATER_ROUGH "粗找水位" /* 错误日志操作名文本：粗找水位。 */
#define ERROR_LOG_OP_SEARCH_WATER_PRECISE "精找水位" /* 错误日志操作名文本：搜索 水位 精找。 */
#define ERROR_LOG_OP_SEARCH_BOTTOM_ROUGH "粗找罐底" /* 错误日志操作名文本：粗找罐底。 */
#define ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE "精找罐底" /* 错误日志操作名文本：搜索 罐底 精找。 */
#define ERROR_LOG_OP_SEARCH_ZERO_ROUGH "粗找零点" /* 错误日志操作名文本：粗找零点。 */
#define ERROR_LOG_OP_SEARCH_ZERO_PRECISE "精找零点" /* 错误日志操作名文本：搜索 零点 精找。 */
#define ERROR_LOG_OP_AUTO_RECOVER     "自动恢复" /* 错误日志操作名文本：自动 恢复。 */
#define ERROR_LOG_OP_HOST_FRAME       "接收主机帧" /* 错误日志操作名文本：接收主机帧。 */
#define ERROR_LOG_OP_COMM_DIAG        "通信诊断" /* 错误日志操作名文本：通信 诊断。 */
#define ERROR_LOG_OP_WIRELESS_PROBE   "蓝牙链路检查" /* 错误日志操作名文本：蓝牙链路检查。 */
#define ERROR_LOG_OP_PARAM_VALIDATE   "参数校验" /* 错误日志操作名文本：参数 校验。 */
#define ERROR_LOG_OP_WEIGHT_COLLISION "扭力碰撞检查" /* 错误日志操作名文本：扭力 碰撞。 */
#define ERROR_LOG_OP_DRIVER_INIT      "驱动初始化" /* 错误日志操作名文本：驱动器 初始化。 */
#define ERROR_LOG_OP_STEP_MOTION      "步进运动" /* 错误日志操作名文本：步进运动。 */
#define ERROR_LOG_OP_WAIT_STOP        "等待电机停止" /* 错误日志操作名文本：等待 停止。 */

#define ERROR_LOG_REASON_COMM_TIMEOUT   "通信超时" /* 错误日志原因文本：通信 超时。 */
#define ERROR_LOG_REASON_COMM_FAIL      "通信失败" /* 错误日志原因文本：通信 失败。 */
#define ERROR_LOG_REASON_DEVICE_ERROR   "设备返回错误" /* 错误日志原因文本：设备 错误。 */
#define ERROR_LOG_REASON_BCC_ERROR      "数据校验错误" /* 错误日志原因文本：BCC 校验 错误。 */
#define ERROR_LOG_REASON_FORMAT_ERROR   "响应格式错误" /* 错误日志原因文本：格式 错误。 */
#define ERROR_LOG_REASON_DRIVER_ERROR   "TMC5130驱动器错误" /* 错误日志原因文本：驱动器 错误。 */
#define ERROR_LOG_REASON_FRAM_FALLBACK  "A分区异常，使用B分区" /* 错误日志原因文本：FRAM 回退。 */
#define ERROR_LOG_REASON_FRAM_ERROR     "FRAM参数分区异常" /* 错误日志原因文本：FRAM 错误。 */
#define ERROR_LOG_REASON_PARAM_OVERFLOW "参数分区容量不足" /* 错误日志原因文本：参数 溢出。 */
#define ERROR_LOG_REASON_WEIGHT_TIMEOUT "扭力通信超时" /* 错误日志原因文本：扭力 超时。 */
#define ERROR_LOG_REASON_WEIGHT_RECOVER "扭力通信恢复" /* 错误日志原因文本：扭力 恢复。 */
#define ERROR_LOG_REASON_SEARCH_FAIL    "搜索失败" /* 错误日志原因文本：搜索 失败。 */
#define ERROR_LOG_REASON_FOLLOW_FAIL    "跟随失败" /* 错误日志原因文本：跟随 失败。 */
#define ERROR_LOG_REASON_MODE_FAIL      "模式启用失败" /* 错误日志原因文本：模式 失败。 */
#define ERROR_LOG_REASON_COMMAND_SWITCH "命令切换" /* 错误日志原因文本：命令 切换。 */
#define ERROR_LOG_REASON_AUTO_RECOVER_START "进入自动恢复" /* 错误日志原因文本：自动 恢复 启动。 */
#define ERROR_LOG_REASON_AUTO_RECOVER_FAIL "自动恢复失败" /* 错误日志原因文本：自动 恢复 失败。 */
#define ERROR_LOG_REASON_RECOVER_OK     "恢复成功" /* 错误日志原因文本：恢复 OK。 */
#define ERROR_LOG_REASON_DRIVER_OVERTEMP "驱动过温" /* 错误日志原因文本：驱动器 过温。 */
#define ERROR_LOG_REASON_DRIVER_SHORT   "驱动短路" /* 错误日志原因文本：驱动器 短路。 */
#define ERROR_LOG_REASON_DRIVER_OPEN    "驱动断线" /* 错误日志原因文本：驱动断线。 */
#define ERROR_LOG_REASON_DRIVER_ALARM   "驱动报警" /* 错误日志原因文本：驱动器 报警。 */
#define ERROR_LOG_REASON_DRIVER_UV      "电荷泵欠压" /* 错误日志原因文本：驱动器 UV。 */
#define ERROR_LOG_REASON_INIT_FAIL      "初始化失败" /* 错误日志原因文本：初始化 失败。 */
#define ERROR_LOG_REASON_WEIGHT_LIMIT   "扭力超限" /* 错误日志原因文本：扭力 限值。 */
#define ERROR_LOG_REASON_COLLISION      "碰撞检测" /* 错误日志原因文本：碰撞。 */
#define ERROR_LOG_REASON_VALIDATE_FAIL  "校验失败" /* 错误日志原因文本：校验 失败。 */
#define ERROR_LOG_REASON_ADDRESS_ERROR  "地址错误" /* 错误日志原因文本：地址 错误。 */
#define ERROR_LOG_REASON_LENGTH_ERROR   "长度错误" /* 错误日志原因文本：长度错误。 */
#define ERROR_LOG_REASON_FUNCTION_ERROR "功能码错误" /* 错误日志原因文本：功能 错误。 */
#define ERROR_LOG_REASON_REGISTER_RANGE "寄存器范围错误" /* 错误日志原因文本：寄存器 范围。 */
#define ERROR_LOG_REASON_PARAM_MAGIC    "魔术字不匹配" /* 错误日志原因文本：参数 魔术字。 */
#define ERROR_LOG_REASON_PARAM_SIZE     "结构体大小不匹配" /* 错误日志原因文本：参数 大小。 */
#define ERROR_LOG_REASON_PARAM_VERSION  "版本不匹配" /* 错误日志原因文本：参数 版本。 */
#define ERROR_LOG_REASON_PARAM_CRC      "CRC不匹配" /* 错误日志原因文本：CRC 不匹配。 */
#define ERROR_LOG_REASON_FREQ_ABNORMAL  "频率异常" /* 错误日志原因文本：频率异常。 */
#define ERROR_LOG_REASON_DENSITY_INVALID "密度值异常" /* 错误日志原因文本：密度 无效。 */
#define ERROR_LOG_REASON_DENSITY_UNSTABLE "密度值不稳定" /* 错误日志原因文本：密度值不稳定。 */
#define ERROR_LOG_REASON_TEMP_ERROR     "温度异常" /* 错误日志原因文本：温度 错误。 */
#define ERROR_LOG_REASON_VOLTAGE_ERROR  "电压异常" /* 错误日志原因文本：电压 错误。 */
#define ERROR_LOG_REASON_POSITION_ERROR "位置异常" /* 错误日志原因文本：位置 错误。 */
#define ERROR_LOG_REASON_ZERO_RANGE     "零点超限" /* 错误日志原因文本：零点 范围。 */
#define ERROR_LOG_REASON_MEASURE_TIMEOUT "测量超时" /* 错误日志原因文本：测量 超时。 */
#define ERROR_LOG_REASON_MOTOR_TIMEOUT "电机运行超时" /* 错误日志原因文本：电机 超时。 */
#define ERROR_LOG_REASON_LEVEL_OVERSPEED "液位变化过快" /* 错误日志原因文本：液位变化过快。 */
#define ERROR_LOG_REASON_DENSITY_RANGE  "密度测量范围异常" /* 错误日志原因文本：密度 范围。 */
#define ERROR_LOG_REASON_PARAM_RANGE    "参数超限" /* 错误日志原因文本：参数 范围。 */
#define ERROR_LOG_REASON_PARAM_UNINIT   "参数未初始化" /* 错误日志原因文本：参数未初始化。 */
#define ERROR_LOG_REASON_PARAM_CALL     "参数调用错误" /* 错误日志原因文本：参数 调用。 */
#define ERROR_LOG_REASON_WEIGHT_DRIFT   "扭力漂移" /* 错误日志原因文本：扭力 漂移。 */
#define ERROR_LOG_REASON_WEIGHT_SATURATION "扭力传感器饱和" /* 错误日志原因文本：扭力 饱和。 */
#define ERROR_LOG_REASON_POWER_FLUCTUATION "电源波动" /* 错误日志原因文本：电源波动。 */
#define ERROR_LOG_REASON_UNKNOWN_FAULT  "未知故障" /* 错误日志原因文本：未知 故障。 */
#define ERROR_LOG_REASON_LOST_STEP      "丢步检测" /* 错误日志原因文本：丢步检测。 */
#define ERROR_LOG_REASON_DRIVER_DISABLED "电机被禁止" /* 错误日志原因文本：驱动器 被禁止。 */
#define ERROR_LOG_REASON_SETTING_FAIL   "电机设置失败" /* 错误日志原因文本：设置 失败。 */

#define ERROR_LOG_ACTION_STOP_MEASURE "停止测量" /* 错误日志处理动作文本：停止 测量。 */
#define ERROR_LOG_ACTION_STOP_MOTOR   "停止电机" /* 错误日志处理动作文本：停止 电机。 */
#define ERROR_LOG_ACTION_CONTINUE     "继续尝试" /* 错误日志处理动作文本：继续。 */
#define ERROR_LOG_ACTION_RETRY_MEASURE "重新执行测量" /* 错误日志处理动作文本：重试 测量。 */
#define ERROR_LOG_ACTION_USE_DEFAULT_PARAM "使用默认参数" /* 错误日志处理动作文本：使用 默认值 参数。 */

/* 根据错误码返回中文错误名称。 */
const char *ErrorLog_GetCodeName(uint32_t code);
/* 根据错误码返回中文模块名。 */
const char *ErrorLog_GetModuleByCode(uint32_t code);
/* 根据错误码返回中文故障原因。 */
const char *ErrorLog_GetReasonByCode(uint32_t code);
/* 取走最近最终报错标记，用于短时间去重。 */
uint8_t ErrorLog_TakeRecentReport(uint32_t code);

/* 打印错误重试阶段日志。 */
void ErrorLog_Retry(const char *module,
                    const char *op,
                    const char *reason,
                    uint32_t attempt,
                    uint32_t max,
                    uint32_t code);
/* 打印带详情的错误重试阶段日志。 */
void ErrorLog_RetryDetail(const char *module,
                          const char *op,
                          const char *reason,
                          uint32_t attempt,
                          uint32_t max,
                          uint32_t code,
                          const char *detail);
/* 打印重试成功阶段日志。 */
void ErrorLog_Recover(const char *module,
                      const char *op,
                      const char *reason,
                      uint32_t attempt,
                      uint32_t max);
/* 打印带详情的重试成功阶段日志。 */
void ErrorLog_RecoverDetail(const char *module,
                            const char *op,
                            const char *reason,
                            uint32_t attempt,
                            uint32_t max,
                            const char *detail);
/* 打印带详情的最终报错日志。 */
void ErrorLog_ReportDetail(const char *module,
                           const char *op,
                           const char *reason,
                           uint32_t code,
                           const char *action,
                           const char *detail);
/* 打印错误报警阶段日志。 */
void ErrorLog_Warn(const char *module,
                   const char *op,
                   const char *reason,
                   const char *action);
/* 打印带详情的错误报警阶段日志。 */
void ErrorLog_WarnDetail(const char *module,
                         const char *op,
                         const char *reason,
                         const char *action,
                         const char *detail);

#endif /* ERROR_LOG_H_ */
