#include "error_log.h"

#include "main.h"
#include "system_parameter.h"
#include <stdio.h>

#define ERROR_LOG_RECENT_REPORT_WINDOW_MS 200U /* 错误日志重复上报抑制窗口，单位 ms。 */
#define ERROR_LOG_RETRY_VERBOSE_LIMIT 3U /* 重试日志详细打印次数上限。 */

static uint8_t s_error_log_recent_report_valid = 0U; /* 错误日志故障记录，供恢复、显示或日志链路使用。 */
static uint32_t s_error_log_recent_report_code = 0U; /* 错误日志故障记录，供恢复、显示或日志链路使用。 */
static uint32_t s_error_log_recent_report_tick = 0U; /* 错误日志故障记录，供恢复、显示或日志链路使用。 */
/**
 * @brief 判断当前重试次数是否需要打印。
 * @note 短重试保留每次打印，长重试只打印首次和末次，避免现场日志刷屏。
 */
static uint8_t ErrorLog_ShouldPrintRetry(uint32_t attempt, uint32_t max)
{
    /* 先处理异常边界，避免错误日志状态机带故障继续运行。 */
    if ((max <= ERROR_LOG_RETRY_VERBOSE_LIMIT) || (attempt <= 1U) || (attempt >= max)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 标记刚刚已经输出过的最终报错。
 * @note 用于短时间内抑制同一个错误码的重复最终报错日志。
 */
static void ErrorLog_MarkRecentReport(uint32_t code)
{
    /* 先处理异常边界，避免错误日志状态机带故障继续运行。 */
    if ((code == NO_ERROR) || (code == STATE_SWITCH)) {
        s_error_log_recent_report_valid = 0U;
        return;
    }

    s_error_log_recent_report_code = code;
    s_error_log_recent_report_tick = HAL_GetTick();
    s_error_log_recent_report_valid = 1U;
}

/**
 * @brief 判断最近一次最终报错标记。
 * @return 1 表示指定错误码刚刚已经打印过；0 表示需要继续打印。
 */
uint8_t ErrorLog_TakeRecentReport(uint32_t code)
{
    /* 先处理异常边界，避免错误日志状态机带故障继续运行。 */
    if (s_error_log_recent_report_valid == 0U) {
        return 0U;
    }

    /* 先处理异常边界，避免错误日志状态机带故障继续运行。 */
    if ((s_error_log_recent_report_code == code) &&
        ((HAL_GetTick() - s_error_log_recent_report_tick) <= ERROR_LOG_RECENT_REPORT_WINDOW_MS)) {
        return 1U;
    }

    s_error_log_recent_report_valid = 0U;
    return 0U;
}

/**
 * @brief 保护普通文本字段，避免传入空指针导致 printf 异常。
 */
static const char *ErrorLog_NonNull(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_TEXT_UNKNOWN;
}

/**
 * @brief 保护模块名字段，空指针时输出“未知”模块。
 */
static const char *ErrorLog_ModuleText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_MODULE_UNKNOWN;
}

/**
 * @brief 保护原因字段，空指针时输出“未知原因”。
 */
static const char *ErrorLog_ReasonText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_REASON_UNKNOWN;
}

/**
 * @brief 保护处理动作字段，空指针时输出“未知处理”。
 */
static const char *ErrorLog_ActionText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_ACTION_UNKNOWN;
}

typedef struct {
    uint32_t code;
    const char *module;
} ErrorLogModuleOverride;

/*
 * 函数用途：登记不能仅按编号高位判断的主责任模块。
 * 调用场景：最终报错需要区分无线通信、软件内部和模拟量输出时使用。
 * 关键约束：无线专用码仍显示滑环通信模块，其余故障按新版编号责任域归类。
 */
static const ErrorLogModuleOverride s_error_log_module_overrides[] = {
    {SLIPRING_COMM_FAIL, ERROR_LOG_MODULE_SLIPRING_COMM},
    {SLIPRING_SIGNAL_WEAK, ERROR_LOG_MODULE_SLIPRING_COMM},
    {WIRELESS_HOST_COMM_TIMEOUT, ERROR_LOG_MODULE_SLIPRING_COMM},
    {WIRELESS_SLAVE_COMM_TIMEOUT, ERROR_LOG_MODULE_SLIPRING_COMM},
    {WIRELESS_RESP_FORMAT_ERROR, ERROR_LOG_MODULE_SLIPRING_COMM},
};

/**
 * @brief 根据逐码责任域和编号高位返回中文模块名。
 */
const char *ErrorLog_GetModuleByCode(uint32_t code)
{
    uint32_t index;

    for (index = 0U;
         index < (uint32_t)(sizeof(s_error_log_module_overrides) / sizeof(s_error_log_module_overrides[0]));
         index++) {
        if (s_error_log_module_overrides[index].code == code) {
            return s_error_log_module_overrides[index].module;
        }
    }

    switch (code & 0xFFFF0000UL) {
    case 0x000B0000UL:
        return ERROR_LOG_MODULE_MOTOR;
    case 0x000C0000UL:
        return ERROR_LOG_MODULE_ENCODER;
    case 0x000D0000UL:
        return ERROR_LOG_MODULE_SENSOR;
    case 0x000E0000UL:
    case 0x000F0000UL:
        return ERROR_LOG_MODULE_MEASURE;
    case 0x00110000UL:
        return ERROR_LOG_MODULE_PARAM;
    case 0x00120000UL:
        return ERROR_LOG_MODULE_AO_OUTPUT;
    case 0x00140000UL:
        return ERROR_LOG_MODULE_COMM;
    case 0x00150000UL:
        return ERROR_LOG_MODULE_WEIGHT;
    case 0x00160000UL:
        return ERROR_LOG_MODULE_SYSTEM;
    default:
        return ERROR_LOG_MODULE_UNKNOWN;
    }
}

/**
 * @brief 根据具体错误码返回中文故障原因。
 * @note 优先匹配具体错误码，未覆盖时再按错误大类兜底。
 */
const char *ErrorLog_GetReasonByCode(uint32_t code)
{
    switch (code) {
    case MOTOR_TMC_COMM_ERROR:
        return "TMC5130寄存器通信失败";
    case MOTOR_DISABLED:
        return ERROR_LOG_REASON_DRIVER_DISABLED;
    case MOTOR_UNKNOWN_FEEDBACK:
        return "电机反馈状态未知";
    case MOTOR_ALARM_TRIGGERED:
        return ERROR_LOG_REASON_DRIVER_ALARM;
    case MOTOR_STEP_ERROR:
    case ENCODER_LOST_STEP:
    case ENCODER_DIFF_EXCESS:
        return ERROR_LOG_REASON_LOST_STEP;
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
        return ERROR_LOG_REASON_DRIVER_UV;
    case MOTOR_OVERTEMPERATURE:
        return ERROR_LOG_REASON_DRIVER_OVERTEMP;
    case MOTOR_RUN_TIMEOUT:
        return ERROR_LOG_REASON_MOTOR_TIMEOUT;
    case MOTOR_TMC_CONFIG_LOST:
        return "TMC5130关键配置丢失";
    case ENCODER_TIMEOUT:
        return "编码器无响应";
    case ENCODER_PARITY_ERROR:
        return "编码器校验失败";
    case ENCODER_POWERON_FAIL:
        return ERROR_LOG_REASON_INIT_FAIL;
    case ENCODER_POWERON_CHANGE:
        return "编码器上电状态变化";
    case ENCODER_INVALID_DATA:
        return "编码器数据无效";
    case ENCODER_CORDIC_OVERFLOW:
        return "编码器CORDIC计算溢出";
    case ENCODER_LINEARITY_WARNING:
        return "编码器线性度报警";
    case ENCODER_OCF_INCOMPLETE:
        return "编码器状态未完成";
    case SENSOR_BCC_ERROR:
        return "传感器校验失败";
    case SONIC_FREQ_ABNORMAL:
        return ERROR_LOG_REASON_FREQ_ABNORMAL;
    case SENSOR_DEVICE_COMM_TIMEOUT:
        return "传感器无响应";
    case SENSOR_DEVICE_REPORTED_ERROR:
        return "传感器设备主动上报内部错误";
    case DENSITY_INVALID:
        return ERROR_LOG_REASON_DENSITY_INVALID;
    case SENSOR_RESP_FORMAT_ERROR:
        return "传感器响应格式异常";
    case MEASUREMENT_ZERO_OUT_OF_RANGE:
        return ERROR_LOG_REASON_ZERO_RANGE;
    case MEASUREMENT_POSITION_ERROR:
        return "位置反馈或到位结果异常";
    case MEASUREMENT_ZERO_REPEAT_FAIL:
    case MEASUREMENT_HEIGHT_DEVIATION:
        return ERROR_LOG_REASON_VALIDATE_FAIL;
    case MEASUREMENT_OILLEVEL_HIGH:
        return "液位测量结果超过罐高";
    case MEASUREMENT_OVERSPEED:
        return ERROR_LOG_REASON_LEVEL_OVERSPEED;
    case MEASUREMENT_TIMEOUT:
        return ERROR_LOG_REASON_MEASURE_TIMEOUT;
    case MEASUREMENT_OILLEVEL_LOW:
        return "下行测量未找到液位";
    case MEASUREMENT_OILLEVEL_NOTFOUND:
        return "上行测量未找到液位";
    case MEASUREMENT_WEIGHT_DOWN_FAIL:
        return "下行寻重失败";
    case MEASUREMENT_WEIGHT_UP_FAIL:
        return "上行寻重失败";
    case MEASUREMENT_WATERLEVEL_LOW:
        return "下行测量未找到水位";
    case MEASUREMENT_DENSITY_NO_VALID_POINT:
        return "密度分布测量无有效测点";
    case MEASUREMENT_DENSITY_SURFACE_NOTFOUND:
        return "密度分布测量未找到油面";
    case PARAM_EEPROM_FAIL:
        return ERROR_LOG_REASON_FRAM_ERROR;
    case PARAM_UNINITIALIZED:
        return ERROR_LOG_REASON_PARAM_UNINIT;
    case PARAM_RANGE_ERROR:
        return ERROR_LOG_REASON_PARAM_RANGE;
    case PARAM_CRC_ERROR:
        return ERROR_LOG_REASON_PARAM_CRC;
    case AD5421_WRITE_CURRENT_ERROR:
        return "AD5421写电流失败";
    case AD5421_INIT_ERROR:
        return "AD5421初始化失败";
    case AD5421_FAULT_STATUS_ERROR:
        return "AD5421故障寄存器报告异常";
    case AD5421_READBACK_ERROR:
        return "AD5421控制寄存器回读失败";
    case AD5421_READFAULT_ERROR:
        return "AD5421故障寄存器读取失败";
    case COMM_UART_TRANSFER_ERROR:
        return "串口或DMA传输异常";
    case SLIPRING_COMM_FAIL:
        return "滑环设备应答失败";
    case SLIPRING_SIGNAL_WEAK:
        return "滑环信号弱";
    case WIRELESS_HOST_COMM_TIMEOUT:
        return "蓝牙主机无响应";
    case WIRELESS_SLAVE_COMM_TIMEOUT:
        return "蓝牙从机未连接";
    case WIRELESS_RESP_FORMAT_ERROR:
        return "无线模块响应格式异常";
    case WEIGHT_OUT_OF_RANGE:
    case WEIGHT_UNDER_RANGE:
        return ERROR_LOG_REASON_WEIGHT_LIMIT;
    case WEIGHT_COLLISION_DETECTED:
        return ERROR_LOG_REASON_COLLISION;
    case WEIGHT_DRIFT_ERROR:
        return ERROR_LOG_REASON_WEIGHT_DRIFT;
    case WEIGHT_SENSOR_SATURATION:
        return ERROR_LOG_REASON_WEIGHT_SATURATION;
    case WEIGHT_COMM_TIMEOUT:
        return "扭力模块无响应";
    case PARAM_ADDRESS_OVERFLOW:
        return "数据地址或输出位置异常";
    case PARAM_ERROR:
        return "参数组合或内部调用条件异常";
    case OTHER_PERIPHERAL_CONFIG_ERROR:
        return ERROR_LOG_REASON_INIT_FAIL;
    default:
        break;
    }

    switch (code & 0xFFFF0000UL) {
    case 0x000B0000UL:
        return ERROR_LOG_REASON_DRIVER_ERROR;
    case 0x000C0000UL:
    case 0x000D0000UL:
    case 0x00120000UL:
    case 0x00150000UL:
        return ERROR_LOG_REASON_DEVICE_ERROR;
    case 0x000E0000UL:
    case 0x000F0000UL:
        return ERROR_LOG_REASON_SEARCH_FAIL;
    case 0x00110000UL:
        return ERROR_LOG_REASON_VALIDATE_FAIL;
    case 0x00140000UL:
        return ERROR_LOG_REASON_COMM_FAIL;
    case 0x00160000UL:
        return ERROR_LOG_REASON_UNKNOWN_FAULT;
    default:
        return ERROR_LOG_REASON_UNKNOWN;
    }
}

/**
 * @brief 根据错误码返回中文错误名称。
 */
const char *ErrorLog_GetCodeName(uint32_t code)
{
    switch (code) {
    case NO_ERROR:
        return "正常";
    case STATE_SWITCH:
        return "状态切换";
    case MOTOR_TMC_COMM_ERROR:
        return "TMC5130通信异常";
    case MOTOR_DISABLED:
        return "电机被禁止";
    case MOTOR_UNKNOWN_FEEDBACK:
        return "电机未知反馈";
    case MOTOR_ALARM_TRIGGERED:
        return "电机报警";
    case MOTOR_STEP_ERROR:
        return "电机步进数错误";
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
        return "电荷泵欠压";
    case MOTOR_OVERTEMPERATURE:
        return "电机过温";
    case MOTOR_RUN_TIMEOUT:
        return "电机运行超时";
    case MOTOR_TMC_CONFIG_LOST:
        return "电机驱动配置丢失";
    case ENCODER_TIMEOUT:
        return "编码器通信超时";
    case ENCODER_PARITY_ERROR:
        return "编码器校验失败";
    case ENCODER_LOST_STEP:
        return "编码器丢步";
    case ENCODER_POWERON_FAIL:
        return "编码器上电初始化失败";
    case ENCODER_POWERON_CHANGE:
        return "编码器上电值变化";
    case ENCODER_DIFF_EXCESS:
        return "编码器相邻差值过大";
    case ENCODER_INVALID_DATA:
        return "编码器无效数据";
    case ENCODER_CORDIC_OVERFLOW:
        return "编码器CORDIC溢出";
    case ENCODER_LINEARITY_WARNING:
        return "编码器线性度报警";
    case ENCODER_OCF_INCOMPLETE:
        return "编码器OCF未完成";
    case SENSOR_BCC_ERROR:
        return "传感器校验错误";
    case SONIC_FREQ_ABNORMAL:
        return "震动管频率异常";
    case SENSOR_DEVICE_COMM_TIMEOUT:
        return "传感器通信超时";
    case SENSOR_DEVICE_REPORTED_ERROR:
        return "传感器设备内部错误";
    case DENSITY_INVALID:
        return "密度值异常";
    case SENSOR_RESP_FORMAT_ERROR:
        return "传感器响应格式异常";
    case MEASUREMENT_ZERO_OUT_OF_RANGE:
        return "零点超限";
    case MEASUREMENT_POSITION_ERROR:
        return "定位结果异常";
    case MEASUREMENT_ZERO_REPEAT_FAIL:
        return "零点重复性差";
    case MEASUREMENT_OILLEVEL_HIGH:
        return "液位超过罐高";
    case MEASUREMENT_OVERSPEED:
        return "液位变化过快";
    case MEASUREMENT_HEIGHT_DEVIATION:
        return "实高偏差过大";
    case MEASUREMENT_TIMEOUT:
        return "测量超时";
    case MEASUREMENT_OILLEVEL_LOW:
        return "下行未找到液位";
    case MEASUREMENT_OILLEVEL_NOTFOUND:
        return "上行未找到液位";
    case MEASUREMENT_WEIGHT_DOWN_FAIL:
        return "下行寻重失败";
    case MEASUREMENT_WEIGHT_UP_FAIL:
        return "上行寻重失败";
    case MEASUREMENT_WATERLEVEL_LOW:
        return "下行未找到水位";
    case MEASUREMENT_DENSITY_NO_VALID_POINT:
        return "密度测量无有效点";
    case MEASUREMENT_DENSITY_SURFACE_NOTFOUND:
        return "密度测量未找到油面";
    case PARAM_EEPROM_FAIL:
        return "参数存储读写失败";
    case PARAM_UNINITIALIZED:
        return "参数未初始化";
    case PARAM_RANGE_ERROR:
        return "参数值超出范围";
    case PARAM_CRC_ERROR:
        return "参数CRC错误";
    case AD5421_WRITE_CURRENT_ERROR:
        return "AD5421写电流失败";
    case AD5421_INIT_ERROR:
        return "AD5421初始化失败";
    case AD5421_FAULT_STATUS_ERROR:
        return "AD5421芯片状态异常";
    case AD5421_READBACK_ERROR:
        return "AD5421控制回读失败";
    case AD5421_READFAULT_ERROR:
        return "AD5421故障寄存器读取失败";
    case COMM_UART_TRANSFER_ERROR:
        return "串口传输异常";
    case SLIPRING_COMM_FAIL:
        return "滑环通信失败";
    case SLIPRING_SIGNAL_WEAK:
        return "滑环信号弱";
    case WIRELESS_HOST_COMM_TIMEOUT:
        return "蓝牙主机通信超时";
    case WIRELESS_SLAVE_COMM_TIMEOUT:
        return "蓝牙从机未连接";
    case WIRELESS_RESP_FORMAT_ERROR:
        return "无线响应格式异常";
    case WEIGHT_OUT_OF_RANGE:
        return "扭力超上限";
    case WEIGHT_UNDER_RANGE:
        return "扭力超下限";
    case WEIGHT_COLLISION_DETECTED:
        return "检测到碰撞";
    case WEIGHT_DRIFT_ERROR:
        return "扭力漂移异常";
    case WEIGHT_SENSOR_SATURATION:
        return "扭力传感器饱和";
    case WEIGHT_COMM_TIMEOUT:
        return "扭力通信超时";
    case PARAM_ADDRESS_OVERFLOW:
        return "数据位置异常";
    case PARAM_ERROR:
        return "调用条件异常";
    case OTHER_PERIPHERAL_CONFIG_ERROR:
        return "外设配置错误";
    default:
        return ERROR_LOG_TEXT_UNKNOWN;
    }
}

/**
 * @brief 打印“错误重试”阶段错误日志。
 */
void ErrorLog_Retry(const char *module,
                    const char *op,
                    const char *reason,
                    uint32_t attempt,
                    uint32_t max,
                    uint32_t code)
{
    ErrorLog_RetryDetail(module, op, reason, attempt, max, code, NULL);
}

/**
 * @brief 打印带详情的“错误重试”阶段错误日志。
 */
void ErrorLog_RetryDetail(const char *module,
                          const char *op,
                          const char *reason,
                          uint32_t attempt,
                          uint32_t max,
                          uint32_t code,
                          const char *detail)
{
    /* 正常结果和命令切换都不是故障，避免进入“错误”打印链路。 */
    if ((code == NO_ERROR) || (code == STATE_SWITCH)) {
        return;
    }

    /* 先处理异常边界，避免错误日志状态机带故障继续运行。 */
    if (ErrorLog_ShouldPrintRetry(attempt, max) == 0U) {
        return;
    }

    if (detail != NULL) {
        printf("错误\t阶段：错误重试\t模块：%s\t操作：%s\t原因：%s\t尝试：%lu/%lu\t错误码：0x%08lX\t错误名：%s\t详情：%s\r\n",
               ErrorLog_ModuleText(module),
               ErrorLog_NonNull(op),
               ErrorLog_ReasonText(reason),
               (unsigned long)attempt,
               (unsigned long)max,
               (unsigned long)code,
               ErrorLog_GetCodeName(code),
               detail);
        return;
    }

    printf("错误\t阶段：错误重试\t模块：%s\t操作：%s\t原因：%s\t尝试：%lu/%lu\t错误码：0x%08lX\t错误名：%s\r\n",
           ErrorLog_ModuleText(module),
           ErrorLog_NonNull(op),
           ErrorLog_ReasonText(reason),
           (unsigned long)attempt,
           (unsigned long)max,
           (unsigned long)code,
           ErrorLog_GetCodeName(code));
}

/**
 * @brief 打印“重试成功”阶段错误日志。
 */
void ErrorLog_Recover(const char *module,
                      const char *op,
                      const char *reason,
                      uint32_t attempt,
                      uint32_t max)
{
    ErrorLog_RecoverDetail(module, op, reason, attempt, max, NULL);
}

/**
 * @brief 打印带详情的“重试成功”阶段错误日志。
 */
void ErrorLog_RecoverDetail(const char *module,
                            const char *op,
                            const char *reason,
                            uint32_t attempt,
                            uint32_t max,
                            const char *detail)
{
    if (detail != NULL) {
        printf("错误\t阶段：重试成功\t模块：%s\t操作：%s\t原因：%s\t尝试：%lu/%lu\t详情：%s\r\n",
               ErrorLog_ModuleText(module),
               ErrorLog_NonNull(op),
               ErrorLog_ReasonText(reason),
               (unsigned long)attempt,
               (unsigned long)max,
               detail);
        return;
    }

    printf("错误\t阶段：重试成功\t模块：%s\t操作：%s\t原因：%s\t尝试：%lu/%lu\r\n",
           ErrorLog_ModuleText(module),
           ErrorLog_NonNull(op),
           ErrorLog_ReasonText(reason),
           (unsigned long)attempt,
           (unsigned long)max);
}


/**
 * @brief 打印最终报错日志，带额外详情字段。
 */
void ErrorLog_ReportDetail(const char *module,
                           const char *op,
                           const char *reason,
                           uint32_t code,
                           const char *action,
                           const char *detail)
{
    /* 正常结果和命令切换都不是故障，避免进入“错误”打印链路。 */
    if ((code == NO_ERROR) || (code == STATE_SWITCH)) {
        return;
    }

    ErrorLog_MarkRecentReport(code);

    if (detail != NULL) {
        printf("错误\t阶段：最终报错\t模块：%s\t操作：%s\t原因：%s\t错误码：0x%08lX\t错误名：%s\t处理：%s\t详情：%s\r\n",
                   ErrorLog_ModuleText(module),
               ErrorLog_NonNull(op),
               ErrorLog_ReasonText(reason),
               (unsigned long)code,
               ErrorLog_GetCodeName(code),
               ErrorLog_ActionText(action),
               detail);
        return;
    }

    printf("错误\t阶段：最终报错\t模块：%s\t操作：%s\t原因：%s\t错误码：0x%08lX\t错误名：%s\t处理：%s\r\n",
           ErrorLog_ModuleText(module),
           ErrorLog_NonNull(op),
           ErrorLog_ReasonText(reason),
           (unsigned long)code,
           ErrorLog_GetCodeName(code),
           ErrorLog_ActionText(action));
}

/**
 * @brief 打印“错误报警”阶段错误日志，不带额外详情。
 */
void ErrorLog_Warn(const char *module,
                   const char *op,
                   const char *reason,
                   const char *action)
{
    ErrorLog_WarnDetail(module, op, reason, action, NULL);
}

/**
 * @brief 打印“错误报警”阶段错误日志，带额外详情字段。
 */
void ErrorLog_WarnDetail(const char *module,
                         const char *op,
                         const char *reason,
                         const char *action,
                         const char *detail)
{
    if (detail != NULL) {
        printf("错误\t阶段：错误报警\t模块：%s\t操作：%s\t原因：%s\t处理：%s\t详情：%s\r\n",
                   ErrorLog_ModuleText(module),
               ErrorLog_NonNull(op),
               ErrorLog_ReasonText(reason),
               ErrorLog_ActionText(action),
               detail);
        return;
    }

    printf("错误\t阶段：错误报警\t模块：%s\t操作：%s\t原因：%s\t处理：%s\r\n",
           ErrorLog_ModuleText(module),
           ErrorLog_NonNull(op),
           ErrorLog_ReasonText(reason),
           ErrorLog_ActionText(action));
}
