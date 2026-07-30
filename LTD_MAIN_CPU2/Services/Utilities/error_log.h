#ifndef ERROR_LOG_H_
/* ERROR_LOG_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
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
#define ERROR_LOG_REASON_FREQ_ABNORMAL  "震动管频率异常" /* 错误日志原因文本：震动管频率异常。 */
#define ERROR_LOG_REASON_DENSITY_INVALID "密度值异常" /* 错误日志原因文本：密度 无效。 */
#define ERROR_LOG_REASON_POSITION_ERROR "位置异常" /* 错误日志原因文本：位置 错误。 */
#define ERROR_LOG_REASON_ZERO_RANGE     "零点超限" /* 错误日志原因文本：零点 范围。 */
#define ERROR_LOG_REASON_MEASURE_TIMEOUT "测量超时" /* 错误日志原因文本：测量 超时。 */
#define ERROR_LOG_REASON_MOTOR_TIMEOUT "电机运行超时" /* 错误日志原因文本：电机 超时。 */
#define ERROR_LOG_REASON_LEVEL_OVERSPEED "液位变化过快" /* 错误日志原因文本：液位变化过快。 */
#define ERROR_LOG_REASON_PARAM_RANGE    "参数超限" /* 错误日志原因文本：参数 范围。 */
#define ERROR_LOG_REASON_PARAM_UNINIT   "参数未初始化" /* 错误日志原因文本：参数未初始化。 */
#define ERROR_LOG_REASON_PARAM_CALL     "参数调用错误" /* 错误日志原因文本：参数 调用。 */
#define ERROR_LOG_REASON_WEIGHT_DRIFT   "扭力漂移" /* 错误日志原因文本：扭力 漂移。 */
#define ERROR_LOG_REASON_WEIGHT_SATURATION "扭力传感器饱和" /* 错误日志原因文本：扭力 饱和。 */
#define ERROR_LOG_REASON_UNKNOWN_FAULT  "未知故障" /* 错误日志原因文本：未知 故障。 */
#define ERROR_LOG_REASON_LOST_STEP      "丢步检测" /* 错误日志原因文本：丢步检测。 */
#define ERROR_LOG_REASON_DRIVER_DISABLED "电机被禁止" /* 错误日志原因文本：驱动器 被禁止。 */
#define ERROR_LOG_REASON_SETTING_FAIL   "电机设置失败" /* 错误日志原因文本：设置 失败。 */

#define ERROR_LOG_ACTION_STOP_MEASURE "停止测量" /* 错误日志处理动作文本：停止 测量。 */
#define ERROR_LOG_ACTION_STOP_MOTOR   "停止电机" /* 错误日志处理动作文本：停止 电机。 */
#define ERROR_LOG_ACTION_CONTINUE     "继续尝试" /* 错误日志处理动作文本：继续。 */
#define ERROR_LOG_ACTION_RETRY_MEASURE "重新执行测量" /* 错误日志处理动作文本：重试 测量。 */
#define ERROR_LOG_ACTION_USE_DEFAULT_PARAM "使用默认参数" /* 错误日志处理动作文本：使用 默认值 参数。 */

/**
 * @brief 根据错误码返回中文错误名称。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据错误码返回中文错误名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *ErrorLog_GetCodeName(uint32_t code);
/**
 * @brief 根据逐码责任域和编号高位返回中文模块名。
 *
 * 根据错误码返回中文模块名。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据逐码责任域和编号高位返回中文模块名对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *ErrorLog_GetModuleByCode(uint32_t code);
/**
 * @brief 根据具体错误码返回中文故障原因。
 * @note 优先匹配具体错误码，未覆盖时再按错误大类兜底。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据具体错误码返回中文故障原因对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *ErrorLog_GetReasonByCode(uint32_t code);
/**
 * @brief 判断最近一次最终报错标记。
 *
 * 取走最近最终报错标记，用于短时间去重。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 1 表示指定错误码刚刚已经打印过；0 表示需要继续打印。
 */
uint8_t ErrorLog_TakeRecentReport(uint32_t code);

/**
 * @brief 打印“错误重试”阶段错误日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param attempt 当前重试序号。
 * @param max 本次操作允许的总尝试次数，用于格式化“当前次数/总次数”的重试日志。
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 */
void ErrorLog_Retry(const char *module,
                    const char *op,
                    const char *reason,
                    uint32_t attempt,
                    uint32_t max,
                    uint32_t code);
/**
 * @brief 打印带详情的“错误重试”阶段错误日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param attempt 当前重试序号。
 * @param max 本次操作允许的总尝试次数，用于带详情重试日志的次数显示。
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @param detail 错误或诊断记录使用的详细信息。
 */
void ErrorLog_RetryDetail(const char *module,
                          const char *op,
                          const char *reason,
                          uint32_t attempt,
                          uint32_t max,
                          uint32_t code,
                          const char *detail);
/**
 * @brief 打印“重试成功”阶段错误日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param attempt 当前重试序号。
 * @param max 本次操作允许的总尝试次数，用于恢复成功日志的次数显示。
 */
void ErrorLog_Recover(const char *module,
                      const char *op,
                      const char *reason,
                      uint32_t attempt,
                      uint32_t max);
/**
 * @brief 打印带详情的“重试成功”阶段错误日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param attempt 当前重试序号。
 * @param max 本次操作允许的总尝试次数，用于带详情恢复日志的次数显示。
 * @param detail 错误或诊断记录使用的详细信息。
 */
void ErrorLog_RecoverDetail(const char *module,
                            const char *op,
                            const char *reason,
                            uint32_t attempt,
                            uint32_t max,
                            const char *detail);
/**
 * @brief 打印最终报错日志，带额外详情字段。
 *
 * 打印带详情的最终报错日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @param action 故障日志中记录的后续处置或恢复动作文字。
 * @param detail 错误或诊断记录使用的详细信息。
 */
void ErrorLog_ReportDetail(const char *module,
                           const char *op,
                           const char *reason,
                           uint32_t code,
                           const char *action,
                           const char *detail);
/**
 * @brief 打印“错误报警”阶段错误日志，不带额外详情。
 *
 * 打印错误报警阶段日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param action 故障日志中记录的后续处置或恢复动作文字。
 */
void ErrorLog_Warn(const char *module,
                   const char *op,
                   const char *reason,
                   const char *action);
/**
 * @brief 打印“错误报警”阶段错误日志，带额外详情字段。
 *
 * 打印带详情的错误报警阶段日志。
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param action 故障日志中记录的后续处置或恢复动作文字。
 * @param detail 错误或诊断记录使用的详细信息。
 */
void ErrorLog_WarnDetail(const char *module,
                         const char *op,
                         const char *reason,
                         const char *action,
                         const char *detail);

#endif /* ERROR_LOG_H_ */
