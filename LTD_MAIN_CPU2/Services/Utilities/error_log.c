#include "error_log.h"

#include "main.h"
#include "system_parameter.h"
#include <stdio.h>

#define ERROR_LOG_RECENT_REPORT_WINDOW_MS 200U /* 错误日志重复上报抑制窗口，单位 ms。 */
#define ERROR_LOG_RETRY_VERBOSE_LIMIT 3U /* 重试日志详细打印次数上限。 */

static uint8_t s_error_log_recent_report_valid = 0U; /* 最近一次最终报错抑制记录是否有效；仅在已登记真实故障码和时间戳后置位。 */
static uint32_t s_error_log_recent_report_code = 0U; /* 最近一次已经输出最终报错的错误码，用于在短窗口内识别上下层重复上报。 */
static uint32_t s_error_log_recent_report_tick = 0U; /* 最近一次最终报错的 HAL 毫秒时刻，与错误码共同判断 ERROR_LOG_RECENT_REPORT_WINDOW_MS 抑制窗口。 */
/**
 * @brief 判断当前重试次数是否需要打印。
 * @note 短重试保留每次打印，长重试只打印首次和末次，避免现场日志刷屏。
 *
 * @param attempt 当前重试序号。
 * @param max 本次操作计划的最大尝试次数，用于判断首轮、末轮和限频日志是否应输出。
 * @return 1 表示当前重试次数需要打印；0 表示当前重试次数不需要打印。
 */
static uint8_t ErrorLog_ShouldPrintRetry(uint32_t attempt, uint32_t max)
{
    /* 短重试打印每一次；长重试只保留首轮和末轮，在保留恢复时间线的同时限制串口日志量。 */
    if ((max <= ERROR_LOG_RETRY_VERBOSE_LIMIT) || (attempt <= 1U) || (attempt >= max)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 标记刚刚已经输出过的最终报错。
 * @note 用于短时间内抑制同一个错误码的重复最终报错日志。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 */
static void ErrorLog_MarkRecentReport(uint32_t code)
{
    /* 正常结果和命令切换会清空最近报错标记，不能进入真实故障的重复抑制窗口。 */
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
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 1 表示指定错误码刚刚已经打印过；0 表示需要继续打印。
 */
uint8_t ErrorLog_TakeRecentReport(uint32_t code)
{
    /* 尚未登记最近报错时直接判定为未重复，避免读取无效的错误码和时间戳。 */
    if (s_error_log_recent_report_valid == 0U) {
        return 0U;
    }

    /* 仅同一错误码且仍在 200 ms 抑制窗口内才视为重复；超时或换码后清除标记，允许新的最终报错输出。 */
    if ((s_error_log_recent_report_code == code) &&
        ((HAL_GetTick() - s_error_log_recent_report_tick) <= ERROR_LOG_RECENT_REPORT_WINDOW_MS)) {
        return 1U;
    }

    s_error_log_recent_report_valid = 0U;
    return 0U;
}

/**
 * @brief 保护普通文本字段，避免传入空指针导致 printf 异常。
 *
 * @param text 待规范化的 NUL 结尾诊断文字；空指针或空字符串按当前字段的默认占位文字处理。
 * @return 返回保护普通文本字段，避免传入空指针导致 printf 异常对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *ErrorLog_NonNull(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_TEXT_UNKNOWN;
}

/**
 * @brief 保护模块名字段，空指针时输出“未知”模块。
 *
 * @param text 待规范化的 NUL 结尾诊断文字；空指针或空字符串按当前字段的默认占位文字处理。
 * @return 返回保护模块名字段，空指针时输出“未知”模块对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *ErrorLog_ModuleText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_MODULE_UNKNOWN;
}

/**
 * @brief 保护原因字段，空指针时输出“未知原因”。
 *
 * @param text 待规范化的 NUL 结尾诊断文字；空指针或空字符串按当前字段的默认占位文字处理。
 * @return 返回保护原因字段，空指针时输出“未知原因”对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *ErrorLog_ReasonText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_REASON_UNKNOWN;
}

/**
 * @brief 保护处理动作字段，空指针时输出“未知处理”。
 *
 * @param text 待规范化的 NUL 结尾诊断文字；空指针或空字符串按当前字段的默认占位文字处理。
 * @return 返回保护处理动作字段，空指针时输出“未知处理”对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *ErrorLog_ActionText(const char *text)
{
    return (text != NULL) ? text : ERROR_LOG_ACTION_UNKNOWN;
}

/* 特定故障码到日志模块名称的覆盖映射；用于替代仅按故障码高字节推导出的默认模块名。 */
typedef struct {
    /* 故障码到日志模块名称的覆盖表项。 */
    uint32_t code; /* 需要覆盖默认模块归属的完整故障码。 */
    const char *module; /* 该故障码应显示和记录的中文模块名称。 */
} ErrorLogModuleOverride;

/**
 * @brief 登记不能仅按编号高位判断的主责任模块。
 *
 * @details 调用场景：最终报错需要区分无线通信、软件内部和模拟量输出时使用。
 * @note 关键约束：无线专用码仍显示滑环通信模块，其余故障按新版编号责任域归类。
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
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据逐码责任域和编号高位返回中文模块名对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
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
        return ERROR_LOG_MODULE_WEIGHT;
    case 0x00140000UL:
        return ERROR_LOG_MODULE_COMM;
    case 0x00150000UL:
        return ERROR_LOG_MODULE_AO_OUTPUT;
    case 0x00160000UL:
    case 0x00170000UL:
        return ERROR_LOG_MODULE_SYSTEM;
    default:
        return ERROR_LOG_MODULE_UNKNOWN;
    }
}

/**
 * @brief 根据具体错误码返回中文故障原因。
 * @note 优先匹配具体错误码，未覆盖时再按错误大类兜底。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据具体错误码返回中文故障原因对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *ErrorLog_GetReasonByCode(uint32_t code)
{
    switch (code) {

    case MOTOR_TMC_COMM_ERROR:
        return "TMC5130寄存器通信失败";
    case MOTOR_DISABLED:
        return "电机驱动输出未使能";
    case MOTOR_UNKNOWN_FEEDBACK:
        return "电机反馈状态无法识别";
    case MOTOR_STEP_ERROR:
    case ENCODER_LOST_STEP:
    case ENCODER_DIFF_EXCESS:
        return ERROR_LOG_REASON_LOST_STEP;
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
        return ERROR_LOG_REASON_DRIVER_UV;
    case MOTOR_OVERTEMPERATURE:
        return "电机驱动已过温关断";
    case MOTOR_RUN_TIMEOUT:
        return "电机整段运行超时";
    case MOTOR_TMC_CONFIG_LOST:
        return "TMC5130运行期复位或关键配置丢失";
    case MOTOR_STALL_ERROR:
        return "电机堵转或负载阻力过大";
    case MOTOR_PHASE_SHORT_ERROR:
        return "电机相线检测到短路";
    case MOTOR_PHASE_OPEN_ERROR:
        return "电机相线断路或接触不良";
    case MOTOR_DRIVER_OVERTEMP_WARNING:
        return "电机驱动温度接近保护值";
    case MOTOR_DRIVER_NOT_INITIALIZED:
        return "电机驱动尚未完成初始化";
    case MOTOR_STOP_WAIT_TIMEOUT:
        return "停止命令后电机未在规定时间停稳";
    case MOTOR_ARRIVAL_WAIT_TIMEOUT:
        return "电机未在规定时间到达目标位置";
    case ENCODER_TIMEOUT:
        return "编码器SPI或DMA采集接口异常";
    case ENCODER_PARITY_ERROR:
        return "编码器校验失败";
    case ENCODER_POWERON_FAIL:
        return ERROR_LOG_REASON_INIT_FAIL;
    case ENCODER_POWERON_CHANGE:
        return "编码器上电状态变化";
    case ENCODER_CORDIC_OVERFLOW:
        return "编码器内部角度运算溢出";
    case ENCODER_LINEARITY_WARNING:
        return "编码器线性度报警";
    case ENCODER_OCF_INCOMPLETE:
        return "编码器角度计算尚未完成";
    case ENCODER_FIRST_SAMPLE_TIMEOUT:
        return "启动后未收到首个有效位置";
    case ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR:
        return "编码轮周长标定结果不可信";
    case ENCODER_POSITION_JUMP:
        return "编码器运行位置变化超过物理上限";
    case SENSOR_BCC_ERROR:
        return "传感器数据校验失败";
    case SONIC_FREQ_ABNORMAL:
        return ERROR_LOG_REASON_FREQ_ABNORMAL;
    case SENSOR_DEVICE_COMM_TIMEOUT:
        return "传感器无响应";
    case SENSOR_INTERNAL_CPU_COMM_TIMEOUT:
        return "传感器内部处理单元通信超时";
    case SENSOR_GYRO_ANGLE_ERROR:
        return "传感器姿态角异常";
    case SENSOR_INTERNAL_COMM_CHECK_ERROR:
        return "传感器内部通信校验异常";
    case SENSOR_NO_RESONANCE:
        return "传感器未检测到谐振信号";
    case DENSITY_INVALID:
        return ERROR_LOG_REASON_DENSITY_INVALID;
    case SENSOR_RESP_FORMAT_ERROR:
        return "传感器响应内容不符合协议";
    case SENSOR_POWER_SUPPLY_ERROR:
        return "传感器供电电压异常";
    case SENSOR_GYRO_COMM_TIMEOUT:
        return "传感器姿态模块通信超时";
    case SENSOR_SELF_TEST_FAILED:
        return "传感器自检未通过";
    case SENSOR_IDENTITY_MISMATCH:
        return "传感器身份信息与当前设备配置不一致";
    case SENSOR_PROTOCOL_VERSION_INCOMPATIBLE:
        return "传感器通信协议版本不兼容";
    case SENSOR_MODE_NOT_READY:
        return "传感器测量模式尚未准备完成";
    case SENSOR_CONFIG_EPOCH_MISMATCH:
        return "传感器配置代次与当前会话不一致";
    case SENSOR_STREAM_STATE_ERROR:
        return "传感器周期上报状态与当前记录不一致";
    case SENSOR_DATA_STALE:
        return "传感器数据超过允许更新时间";
    case SENSOR_TEMPERATURE_RANGE_ERROR:
        return "传感器温度超出工作范围";
    case SENSOR_REMOTE_INTERNAL_ERROR:
        return "传感器远端内部故障";
    case SENSOR_ADDRESS_MISMATCH:
        return "传感器响应地址与目标地址不一致";
    case SENSOR_SESSION_INVALID:
        return "传感器通信会话已失效";
    case SENSOR_SEQUENCE_ERROR:
        return "传感器报文序号不连续或不符合预期";
    case SENSOR_HANDSHAKE_REQUIRED:
        return "传感器要求重新建立通信会话";
    case SENSOR_REPLAY_DETECTED:
        return "检测到传感器重复或过期报文";
    case SENSOR_CAPABILITY_UNSUPPORTED:
        return "传感器不支持当前设备要求的功能";
    case SENSOR_COMMAND_UNSUPPORTED:
        return "传感器不支持本次命令";
    case SENSOR_ARGUMENT_REJECTED:
        return "传感器拒绝本次命令参数";
    case SENSOR_MODE_MISMATCH:
        return "传感器返回的测量模式与请求不一致";
    case SENSOR_MODE_NOT_ALLOWED:
        return "传感器当前状态不允许执行本次操作";
    case SENSOR_TRANSACTION_PENDING:
        return "上一笔传感器通信事务尚未完成";
    case SENSOR_DEVICE_BUSY:
        return "传感器正在处理其它任务";
    case SENSOR_PARAM_CRC_ERROR:
        return "传感器参数内容校验失败";
    case SENSOR_SAMPLE_COUNTER_ERROR:
        return "传感器采样计数停滞或跳变异常";
    case SENSOR_STREAM_STOPPED:
        return "传感器周期上报意外停止";
    case SENSOR_STREAM_NOT_ACTIVE:
        return "传感器周期上报尚未启动";
    case SENSOR_STREAM_ALREADY_ACTIVE:
        return "传感器周期上报已在运行";
    case SENSOR_STREAM_EXIT_FAILED:
        return "传感器周期上报退出失败";
    case MEASUREMENT_ZERO_OUT_OF_RANGE:
        return ERROR_LOG_REASON_ZERO_RANGE;
    case MEASUREMENT_ZERO_REPEAT_FAIL:
    case MEASUREMENT_HEIGHT_DEVIATION:
        return ERROR_LOG_REASON_VALIDATE_FAIL;
    case POSITION_DATA_INVALID:
        return "当前位置数据无效";
    case POSITION_TARGET_OVERRUN:
        return "运动越过目标位置";
    case POSITION_ARRIVAL_DEVIATION:
        return "停止后实际位置与目标偏差过大";
    case POSITION_MOTOR_NOT_STOPPED:
        return "提交测量结果时电机仍未停止";
    case MEASUREMENT_OILLEVEL_HIGH:
        return "液位测量结果超过罐高";
    case MEASUREMENT_OVERSPEED:
        return ERROR_LOG_REASON_LEVEL_OVERSPEED;
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
    case MEASUREMENT_DENSITY_LEVEL_TIMEOUT:
        return "密度闭环找液位超时";
    case MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT:
        return "频率闭环找液位超时";
    case MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED:
        return "液位定时矫正失败或无法恢复有效液位";
    case MEASUREMENT_BOTTOM_RELEASE_FAIL:
        return "粗找罐底前上行后仍处于触底状态";
    case MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED:
        return "罐高标定值未设置";
    case MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED:
        return "水位标定值未设置";
    case MEASUREMENT_DENSITY_PLAN_INVALID:
        return "密度测点范围、步距或数量无法形成有效计划";
    case MEASUREMENT_TANK_HEIGHT_RESULT_INVALID:
        return "罐高测量未得到有效实高";
    case MEASUREMENT_WATER_CALC_OUT_OF_RANGE:
        return "水位标定计算结果超出允许范围";
    case PARAM_EEPROM_FAIL:
        return ERROR_LOG_REASON_FRAM_ERROR;
    case PARAM_UNINITIALIZED:
        return ERROR_LOG_REASON_PARAM_UNINIT;
    case PARAM_RANGE_ERROR:
        return ERROR_LOG_REASON_PARAM_RANGE;
    case PARAM_CRC_ERROR:
        return ERROR_LOG_REASON_PARAM_CRC;
    case PARAM_CONFIG_MISSING:
        return "执行当前功能所需配置缺失";
    case PARAM_COMBINATION_CONFLICT:
        return "多个参数组合互相冲突";
    case PARAM_FEATURE_UNSUPPORTED:
        return "当前传感器、方法或模式不支持所选功能";
    case PARAM_STORAGE_SIZE_MISMATCH:
        return "参数存储结构大小与当前程序不一致";
    case PARAM_STORAGE_VERSION_MISMATCH:
        return "参数存储版本与当前程序不一致";
    case PARAM_STORAGE_WRITE_VERIFY_FAILED:
        return "参数写入后读回内容不一致";
    case AD5421_INIT_ERROR:
        return "AD5421初始化失败且未取得更具体原因";
    case AD5421_READBACK_ERROR:
        return "AD5421控制寄存器回读不一致";
    case AD5421_INTERNAL_COMM_ERROR:
        return "AD5421内部通信异常";
    case AD5421_LOOP_CURRENT_HIGH:
        return "模拟输出环路电流过高";
    case AD5421_LOOP_CURRENT_LOW:
        return "模拟输出环路电流过低或断环";
    case AD5421_LOOP_VOLTAGE_LOW:
        return "模拟输出环路供电电压不足";
    case AD5421_SPI_TRANSFER_ERROR:
        return "AD5421与主控之间SPI传输失败";
    case AD5421_ACCESS_BUSY:
        return "AD5421访问冲突或前一操作未结束";
    case AD5421_OVERTEMP_SHUTDOWN:
        return "AD5421过温并停止输出";
    case AD5421_OVERTEMP_WARNING:
        return "AD5421温度接近保护值";
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
    case WIRELESS_SCAN_NO_DEVICE:
        return "无线扫描未发现任何候选设备";
    case WIRELESS_NAME_NOT_UNIQUE:
        return "扫描到两个及以上同名无线设备，无法确定连接目标";
    case WIRELESS_NAME_NOT_FOUND:
        return "扫描到候选无线设备，但没有找到指定名称";
    case WIRELESS_NAME_INVALID:
        return "无线设备名称为空、过长或格式无效";
    case WIRELESS_NOT_HOST_MODE:
        return "无线模块未处于主机模式";
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
    case SYSTEM_BUFFER_CAPACITY_ERROR:
        return "内部缓冲区或存储分区容量不足";
    case SYSTEM_CALL_CONDITION_ERROR:
        return "内部调用参数或前置条件异常";
    case SYSTEM_CALCULATION_ERROR:
        return "内部计算无法得到有效结果";
    case POWER_SUPPLY_24V_UNDERVOLTAGE:
        return "整机24V供电低于20V安全阈值";
    case POWER_MONITOR_ADC_OVERRUN:
        return "24V监控ADC数据溢出，采样时效无法确认";
    case POWER_MONITOR_DMA_STOPPED:
        return "24V监控DMA停止，采样值可能已经陈旧";
    case POWER_MONITOR_INIT_FAILED:
        return "24V电源监控启动或首次采样失败";
    case POWER_MONITOR_RECOVERY_FAILED:
        return "24V电源监控连续三次局部恢复失败";
    case POWER_LOSS_POSITION_SAVE_FAILED:
        return "低压期间编码器位置和回执未完成可靠提交";
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
    case 0x00170000UL:
        return "整机供电或电源监控异常";
    default:
        return ERROR_LOG_REASON_UNKNOWN;
    }
}

/**
 * @brief 根据错误码返回中文错误名称。
 *
 * @param code 待判断、转换或上报的状态码。该值使用整机模块-原因编码，函数按职责映射模块、原因、名称或记录最近一次报告。
 * @return 返回根据错误码返回中文错误名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
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
        return "电机驱动电流未建立";
    case MOTOR_UNKNOWN_FEEDBACK:
        return "电机驱动故障类型未知";
    case MOTOR_STEP_ERROR:
        return "电机无有效位移";
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
        return "电机驱动内部升压不足";
    case MOTOR_OVERTEMPERATURE:
        return "电机驱动过温关断";
    case MOTOR_RUN_TIMEOUT:
        return "电机运行超时";
    case MOTOR_TMC_CONFIG_LOST:
        return "电机驱动配置丢失";
    case MOTOR_STALL_ERROR:
        return "电机堵转";
    case MOTOR_PHASE_SHORT_ERROR:
        return "电机相线短路";
    case MOTOR_PHASE_OPEN_ERROR:
        return "电机相线断路";
    case MOTOR_DRIVER_OVERTEMP_WARNING:
        return "电机驱动过温预警";
    case MOTOR_DRIVER_NOT_INITIALIZED:
        return "电机驱动未初始化";
    case MOTOR_STOP_WAIT_TIMEOUT:
        return "电机未按时停下";
    case MOTOR_ARRIVAL_WAIT_TIMEOUT:
        return "电机未按时到位";
    case ENCODER_TIMEOUT:
        return "编码器没有有效位置数据";
    case ENCODER_PARITY_ERROR:
        return "编码器数据校验失败";
    case ENCODER_LOST_STEP:
        return "编码器记录位移不足";
    case ENCODER_POWERON_FAIL:
        return "编码器位置记录不可用";
    case ENCODER_POWERON_CHANGE:
        return "编码器上电位置跳变";
    case ENCODER_DIFF_EXCESS:
        return "编码轮周长标定值过大";
    case ENCODER_CORDIC_OVERFLOW:
        return "编码器角度计算超限";
    case ENCODER_LINEARITY_WARNING:
        return "编码器角度线性异常";
    case ENCODER_OCF_INCOMPLETE:
        return "编码器角度计算未完成";
    case ENCODER_FIRST_SAMPLE_TIMEOUT:
        return "编码器首帧超时";
    case ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR:
        return "编码轮周长标定异常";
    case ENCODER_POSITION_JUMP:
        return "编码器运行位置跳变";
    case SENSOR_BCC_ERROR:
        return "传感器校验错误";
    case SONIC_FREQ_ABNORMAL:
        return "振动管频率异常";
    case SENSOR_DEVICE_COMM_TIMEOUT:
        return "传感器通信超时";
    case SENSOR_INTERNAL_CPU_COMM_TIMEOUT:
        return "传感器内部通信超时";
    case SENSOR_GYRO_ANGLE_ERROR:
        return "传感器姿态角异常";
    case SENSOR_INTERNAL_COMM_CHECK_ERROR:
        return "传感器内部通信校验失败";
    case SENSOR_NO_RESONANCE:
        return "传感器无谐振";
    case DENSITY_INVALID:
        return "密度值超出有效范围";
    case SENSOR_RESP_FORMAT_ERROR:
        return "传感器响应格式异常";
    case SENSOR_POWER_SUPPLY_ERROR:
        return "传感器供电异常";
    case SENSOR_GYRO_COMM_TIMEOUT:
        return "姿态模块通信超时";
    case SENSOR_SELF_TEST_FAILED:
        return "传感器自检失败";
    case SENSOR_IDENTITY_MISMATCH:
        return "传感器身份不匹配";
    case SENSOR_PROTOCOL_VERSION_INCOMPATIBLE:
        return "传感器协议版本不兼容";
    case SENSOR_MODE_NOT_READY:
        return "传感器模式未就绪";
    case SENSOR_CONFIG_EPOCH_MISMATCH:
        return "传感器配置批次不一致";
    case SENSOR_STREAM_STATE_ERROR:
        return "传感器上报状态不一致";
    case SENSOR_DATA_STALE:
        return "传感器数据过期";
    case SENSOR_TEMPERATURE_RANGE_ERROR:
        return "传感器温度超限";
    case SENSOR_REMOTE_INTERNAL_ERROR:
        return "传感器内部故障";
    case SENSOR_ADDRESS_MISMATCH:
        return "传感器地址不匹配";
    case SENSOR_SESSION_INVALID:
        return "传感器连接状态失效";
    case SENSOR_SEQUENCE_ERROR:
        return "传感器应答序号不一致";
    case SENSOR_HANDSHAKE_REQUIRED:
        return "传感器需要重新确认连接";
    case SENSOR_REPLAY_DETECTED:
        return "收到重复的传感器数据";
    case SENSOR_CAPABILITY_UNSUPPORTED:
        return "传感器不支持所需功能";
    case SENSOR_COMMAND_UNSUPPORTED:
        return "传感器命令不支持";
    case SENSOR_ARGUMENT_REJECTED:
        return "传感器不接受当前参数";
    case SENSOR_MODE_MISMATCH:
        return "传感器模式不一致";
    case SENSOR_MODE_NOT_ALLOWED:
        return "传感器当前模式不允许操作";
    case SENSOR_TRANSACTION_PENDING:
        return "上一次传感器操作尚未完成";
    case SENSOR_DEVICE_BUSY:
        return "传感器设备忙";
    case SENSOR_PARAM_CRC_ERROR:
        return "传感器参数校验失败";
    case SENSOR_SAMPLE_COUNTER_ERROR:
        return "传感器采样序号异常";
    case SENSOR_STREAM_STOPPED:
        return "传感器周期上报停止";
    case SENSOR_STREAM_NOT_ACTIVE:
        return "传感器周期上报未启动";
    case SENSOR_STREAM_ALREADY_ACTIVE:
        return "传感器周期上报重复启动";
    case SENSOR_STREAM_EXIT_FAILED:
        return "传感器周期上报退出失败";
    case MEASUREMENT_ZERO_OUT_OF_RANGE:
        return "零点位置超出允许范围";
    case MEASUREMENT_ZERO_REPEAT_FAIL:
        return "陀螺仪基准不稳定";
    case POSITION_DATA_INVALID:
        return "位置数据无效";
    case POSITION_TARGET_OVERRUN:
        return "运动越过目标";
    case POSITION_ARRIVAL_DEVIATION:
        return "电机到位偏差过大";
    case POSITION_MOTOR_NOT_STOPPED:
        return "电机未停止";
    case MEASUREMENT_OILLEVEL_HIGH:
        return "上行找液位到达位置上限";
    case MEASUREMENT_OVERSPEED:
        return "液位变化过快";
    case MEASUREMENT_HEIGHT_DEVIATION:
        return "实高偏差过大";
    case MEASUREMENT_OILLEVEL_LOW:
        return "下行未找到液位";
    case MEASUREMENT_OILLEVEL_NOTFOUND:
        return "上行未找到液位";
    case MEASUREMENT_WEIGHT_DOWN_FAIL:
        return "下行未找到罐底";
    case MEASUREMENT_WEIGHT_UP_FAIL:
        return "上行未找到零点";
    case MEASUREMENT_WATERLEVEL_LOW:
        return "下行未找到水位";
    case MEASUREMENT_DENSITY_NO_VALID_POINT:
        return "密度测量无有效点";
    case MEASUREMENT_DENSITY_SURFACE_NOTFOUND:
        return "密度测量未找到油面";
    case MEASUREMENT_DENSITY_LEVEL_TIMEOUT:
        return "密度找液位超时";
    case MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT:
        return "频率找液位超时";
    case MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED:
        return "液位定时矫正失败";
    case MEASUREMENT_BOTTOM_RELEASE_FAIL:
        return "探底前未能离开罐底";
    case MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED:
        return "未设置罐高标定值";
    case MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED:
        return "未设置水位标定值";
    case MEASUREMENT_DENSITY_PLAN_INVALID:
        return "密度测点规划失败";
    case MEASUREMENT_TANK_HEIGHT_RESULT_INVALID:
        return "罐高测量结果无效";
    case MEASUREMENT_WATER_CALC_OUT_OF_RANGE:
        return "水位标定计算超限";
    case PARAM_EEPROM_FAIL:
        return "参数存储读写失败";
    case PARAM_UNINITIALIZED:
        return "参数未初始化";
    case PARAM_RANGE_ERROR:
        return "参数值超出范围";
    case PARAM_CRC_ERROR:
        return "参数完整性校验失败";
    case PARAM_CONFIG_MISSING:
        return "关键参数未设置";
    case PARAM_COMBINATION_CONFLICT:
        return "参数组合冲突";
    case PARAM_FEATURE_UNSUPPORTED:
        return "功能不支持";
    case PARAM_STORAGE_SIZE_MISMATCH:
        return "参数存储结构不匹配";
    case PARAM_STORAGE_VERSION_MISMATCH:
        return "参数存储版本不匹配";
    case PARAM_STORAGE_WRITE_VERIFY_FAILED:
        return "参数存储写入校验失败";
    case AD5421_INIT_ERROR:
        return "模拟输出初始化失败";
    case AD5421_READBACK_ERROR:
        return "模拟输出控制回读不一致";
    case AD5421_INTERNAL_COMM_ERROR:
        return "模拟输出内部通信异常";
    case AD5421_LOOP_CURRENT_HIGH:
        return "模拟输出电流过高";
    case AD5421_LOOP_CURRENT_LOW:
        return "模拟输出电流过低";
    case AD5421_LOOP_VOLTAGE_LOW:
        return "模拟输出环路电压低";
    case AD5421_SPI_TRANSFER_ERROR:
        return "主控与模拟量输出模块通信失败";
    case AD5421_ACCESS_BUSY:
        return "模拟输出访问冲突";
    case AD5421_OVERTEMP_SHUTDOWN:
        return "模拟输出过温关断";
    case AD5421_OVERTEMP_WARNING:
        return "模拟输出过温预警";
    case COMM_UART_TRANSFER_ERROR:
        return "串口传输异常";
    case SLIPRING_COMM_FAIL:
        return "无线滑环通信失败";
    case SLIPRING_SIGNAL_WEAK:
        return "无线配对信号不达标";
    case WIRELESS_HOST_COMM_TIMEOUT:
        return "蓝牙主机通信超时";
    case WIRELESS_SLAVE_COMM_TIMEOUT:
        return "蓝牙从机未连接";
    case WIRELESS_RESP_FORMAT_ERROR:
        return "无线响应格式异常";
    case WIRELESS_SCAN_NO_DEVICE:
        return "无线扫描未发现设备";
    case WIRELESS_NAME_NOT_UNIQUE:
        return "无线名称重复";
    case WIRELESS_NAME_NOT_FOUND:
        return "无线名称未找到";
    case WIRELESS_NAME_INVALID:
        return "无线名称无效";
    case WIRELESS_NOT_HOST_MODE:
        return "无线非主机模式";
    case WEIGHT_OUT_OF_RANGE:
        return "满载标定扭力超过上限";
    case WEIGHT_UNDER_RANGE:
        return "满载标定扭力低于下限";
    case WEIGHT_COLLISION_DETECTED:
        return "扭力变化超过保护值";
    case WEIGHT_DRIFT_ERROR:
        return "空载标定扭力偏离零点";
    case WEIGHT_SENSOR_SATURATION:
        return "扭力传感器饱和";
    case WEIGHT_COMM_TIMEOUT:
        return "扭力通信超时";
    case SYSTEM_BUFFER_CAPACITY_ERROR:
        return "内部数据空间不足";
    case SYSTEM_CALL_CONDITION_ERROR:
        return "系统调用条件异常";
    case SYSTEM_CALCULATION_ERROR:
        return "系统计算异常";
    case POWER_SUPPLY_24V_UNDERVOLTAGE:
        return "整机24V欠压";
    case POWER_MONITOR_ADC_OVERRUN:
        return "电源采样数据处理不及时";
    case POWER_MONITOR_DMA_STOPPED:
        return "电源采样传输意外停止";
    case POWER_MONITOR_INIT_FAILED:
        return "电源采样启动失败";
    case POWER_MONITOR_RECOVERY_FAILED:
        return "电源采样连续恢复失败";
    case POWER_LOSS_POSITION_SAVE_FAILED:
        return "掉电位置保存失败";
    default:
        return ERROR_LOG_TEXT_UNKNOWN;
    }
}

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
                    uint32_t code)
{
    ErrorLog_RetryDetail(module, op, reason, attempt, max, code, NULL);
}

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
                          const char *detail)
{
    /* 正常结果和命令切换都不是故障，避免进入“错误”打印链路。 */
    if ((code == NO_ERROR) || (code == STATE_SWITCH)) {
        return;
    }

    /* 当前尝试不在日志保留点时直接返回，重试动作本身仍由调用方继续执行。 */
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
                      uint32_t max)
{
    ErrorLog_RecoverDetail(module, op, reason, attempt, max, NULL);
}

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
 *
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签标识传感器、电机、电源或存储等来源，写入结构化日志前缀。
 * @param op 用于标识当前失败、重试或恢复操作的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param action 故障日志中记录的后续处置或恢复动作文字。
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
