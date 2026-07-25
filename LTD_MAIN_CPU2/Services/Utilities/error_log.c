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
        return ERROR_LOG_MODULE_WEIGHT;
    case 0x00140000UL:
        return ERROR_LOG_MODULE_COMM;
    case 0x00150000UL:
        return ERROR_LOG_MODULE_AO_OUTPUT;
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
        return "电机驱动未使能";
    case MOTOR_UNKNOWN_FEEDBACK:
        return "电机反馈未知";
    case MOTOR_STEP_ERROR:
        return "电机无有效位移";
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
        return "电荷泵欠压";
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
        return "电机停止等待超时";
    case MOTOR_ARRIVAL_WAIT_TIMEOUT:
        return "电机到位等待超时";
    case ENCODER_TIMEOUT:
        return "编码器采集接口异常";
    case ENCODER_PARITY_ERROR:
        return "编码器校验失败";
    case ENCODER_LOST_STEP:
        return "编码器检测到丢步";
    case ENCODER_POWERON_FAIL:
        return "编码器上电初始化失败";
    case ENCODER_POWERON_CHANGE:
        return "编码器上电值变化";
    case ENCODER_DIFF_EXCESS:
        return "编码器相邻差值过大";
    case ENCODER_CORDIC_OVERFLOW:
        return "编码器角度运算溢出";
    case ENCODER_LINEARITY_WARNING:
        return "编码器线性度报警";
    case ENCODER_OCF_INCOMPLETE:
        return "编码器角度计算未完成";
    case ENCODER_FIRST_SAMPLE_TIMEOUT:
        return "编码器首帧超时";
    case ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR:
        return "编码轮周长标定异常";
    case SENSOR_BCC_ERROR:
        return "传感器校验错误";
    case SONIC_FREQ_ABNORMAL:
        return "震动管频率异常";
    case SENSOR_DEVICE_COMM_TIMEOUT:
        return "传感器通信超时";
    case SENSOR_INTERNAL_CPU_COMM_TIMEOUT:
        return "传感器内部通信超时";
    case SENSOR_GYRO_ANGLE_ERROR:
        return "传感器姿态角异常";
    case SENSOR_INTERNAL_COMM_CHECK_ERROR:
        return "传感器内部校验异常";
    case SENSOR_NO_RESONANCE:
        return "传感器无谐振";
    case DENSITY_INVALID:
        return "密度值异常";
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
        return "传感器配置代次不一致";
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
        return "传感器会话失效";
    case SENSOR_SEQUENCE_ERROR:
        return "传感器报文序号异常";
    case SENSOR_HANDSHAKE_REQUIRED:
        return "传感器需要重新握手";
    case SENSOR_REPLAY_DETECTED:
        return "传感器重复报文";
    case SENSOR_CAPABILITY_UNSUPPORTED:
        return "传感器能力不支持";
    case SENSOR_COMMAND_UNSUPPORTED:
        return "传感器命令不支持";
    case SENSOR_ARGUMENT_REJECTED:
        return "传感器命令参数被拒绝";
    case SENSOR_MODE_MISMATCH:
        return "传感器模式不一致";
    case SENSOR_MODE_NOT_ALLOWED:
        return "传感器模式不允许";
    case SENSOR_TRANSACTION_PENDING:
        return "传感器通信未完成";
    case SENSOR_DEVICE_BUSY:
        return "传感器设备忙";
    case SENSOR_PARAM_CRC_ERROR:
        return "传感器参数校验失败";
    case SENSOR_SAMPLE_COUNTER_ERROR:
        return "传感器采样计数异常";
    case SENSOR_STREAM_STOPPED:
        return "传感器周期上报停止";
    case SENSOR_STREAM_NOT_ACTIVE:
        return "传感器周期上报未启动";
    case SENSOR_STREAM_ALREADY_ACTIVE:
        return "传感器周期上报已启动";
    case SENSOR_STREAM_EXIT_FAILED:
        return "传感器周期上报退出失败";
    case MEASUREMENT_ZERO_OUT_OF_RANGE:
        return "零点超限";
    case MEASUREMENT_ZERO_REPEAT_FAIL:
        return "零点重复性差";
    case POSITION_DATA_INVALID:
        return "位置数据无效";
    case POSITION_TARGET_OVERRUN:
        return "运动越过目标";
    case POSITION_ARRIVAL_DEVIATION:
        return "到位偏差过大";
    case POSITION_MOTOR_NOT_STOPPED:
        return "电机未停止";
    case MEASUREMENT_OILLEVEL_HIGH:
        return "液位超过罐高";
    case MEASUREMENT_OVERSPEED:
        return "液位变化过快";
    case MEASUREMENT_HEIGHT_DEVIATION:
        return "实高偏差过大";
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
    case MEASUREMENT_DENSITY_LEVEL_TIMEOUT:
        return "密度找液位超时";
    case MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT:
        return "频率找液位超时";
    case MEASUREMENT_BOTTOM_RELEASE_FAIL:
        return "探底前离底失败";
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
        return "必需配置缺失";
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
        return "模拟输出通信失败";
    case AD5421_ACCESS_BUSY:
        return "模拟输出访问冲突";
    case AD5421_OVERTEMP_SHUTDOWN:
        return "模拟输出过温关断";
    case AD5421_OVERTEMP_WARNING:
        return "模拟输出过温预警";
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
    case WIRELESS_SCAN_NO_DEVICE:
        return "无线扫描无设备";
    case WIRELESS_NAME_NOT_UNIQUE:
        return "无线名称重复";
    case WIRELESS_NAME_NOT_FOUND:
        return "无线名称未找到";
    case WIRELESS_NAME_INVALID:
        return "无线名称无效";
    case WIRELESS_NOT_HOST_MODE:
        return "无线非主机模式";
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
    case SYSTEM_BUFFER_CAPACITY_ERROR:
        return "系统容量不足";
    case SYSTEM_CALL_CONDITION_ERROR:
        return "系统调用条件异常";
    case SYSTEM_CALCULATION_ERROR:
        return "系统计算异常";
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
