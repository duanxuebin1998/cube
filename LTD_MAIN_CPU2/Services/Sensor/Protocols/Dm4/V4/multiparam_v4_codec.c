/*
 * 模块职责：完成DM4-V4线协议的组帧、校验、字段解码和主动帧规范化。
 * 纯函数边界：本文件不启动DMA、不等待UART、不打印日志，也不改变通信模式。
 * 兼容边界：主动帧既接受标准72字节，也接受无线滑环在固定偏移插入50个0的122字节帧；
 * 后者先还原为标准帧，再复用同一套字段和CRC校验。
 */
#include "multiparam_v4_internal.h"

#include "error_log.h"
#include "main.h"
#include "my_crc.h"
#include "sensor_transport_config.h"
#include "sensor_uart6_owner.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
/*
 * 函数用途：按小端字节序还原32位无符号原始值。
 * 调用场景：主动帧参数区和8字节交互应答的数据区解析。
 * 关键约束：调用方保证至少有4个可读字节；函数不进行参数业务校验。
 */
uint32_t MULTIPARAM_V4_DecodeU32Le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/*
 * 函数用途：把32位原始值按小端字节序写入线协议数据区。
 * 调用场景：生成8字节读写、模式和功能控制请求。
 * 关键约束：调用方保证目标至少有4个可写字节；函数不计算整帧校验。
 */
static void MULTIPARAM_V4_EncodeU32Le(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/*
 * 函数用途：把IEEE 754单精度原始位转换为浮点值。
 * 调用场景：解析V4.0浮点参数；调用方还需检查有限性和业务范围。
 * 关键约束：使用位复制保持原始表示，不在本函数内把NaN或无穷值改写为0。
 */
float MULTIPARAM_V4_RawToFloat(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/*
 * 函数用途：判断参数码是否属于V4.0可读区域。
 * 调用场景：交互读取入口的本地边界检查。
 * 关键约束：只检查协议地址范围，不判断当前模式下该参数是否具有有效业务值。
 */
uint8_t MULTIPARAM_V4_IsReadableParameter(uint8_t parameter)
{
    return (uint8_t)((parameter <= MULTIPARAM_V4_PARAM_MAX) ? 1U : 0U);
}

/*
 * 函数用途：按新版独立位定义解析R02运行状态。
 * 调用场景：主动快照发布以及模式、功能目标状态核对。
 * 关键约束：Bit0～Bit2互不排斥；保留位不参与模式和开关判定。
 */
void MULTIPARAM_V4_DecodeOperatingState(uint32_t status_word,
                                        multiparam_v4_operating_state_t *operating_state)
{
    operating_state->measurement_mode =
        ((status_word & (1UL << 0U)) != 0U)
            ? MULTIPARAM_V4_MEASUREMENT_LEVEL
            : MULTIPARAM_V4_MEASUREMENT_DENSITY;
    operating_state->water_enabled =
        (uint8_t)(((status_word & (1UL << 1U)) != 0U) ? 1U : 0U);
    operating_state->magnetic_zero_enabled =
        (uint8_t)(((status_word & (1UL << 2U)) != 0U) ? 1U : 0U);
    operating_state->sweep_stage = (uint8_t)((status_word >> 4U) & 0x03U);
    operating_state->drive_stage = (uint8_t)((status_word >> 6U) & 0x03U);
}

/*
 * 函数用途：把参数4的有符号协议值拆分为频率绝对值和快扫完成标志。
 * 调用场景：主动帧解包和交互R04读取共用同一套符号语义。
 * 关键约束：0表示采集失败，INT32_MIN无法安全取绝对值，两者都返回无效。
 */
uint8_t MULTIPARAM_V4_DecodeFrequency(int32_t raw_frequency,
                                      uint32_t *frequency_hz,
                                      uint8_t *fast_sweep_completed)
{
    if ((frequency_hz == NULL) || (fast_sweep_completed == NULL) ||
        (raw_frequency == 0) || (raw_frequency == INT32_MIN)) {
        return 0U;
    }

    *fast_sweep_completed = (uint8_t)((raw_frequency < 0) ? 1U : 0U);
    *frequency_hz = (raw_frequency < 0)
                        ? (uint32_t)(-raw_frequency)
                        : (uint32_t)raw_frequency;
    return 1U;
}

/*
 * 函数用途：检查指定字节区间是否全部为0。
 * 调用场景：识别无线主机半传输缺陷插入的固定50字节零填充。
 * 关键约束：空指针返回否；仅判断字节内容，不单独据此接受补零帧。
 */
uint8_t MULTIPARAM_V4_IsZeroRange(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) {
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        if (data[index] != 0U) {
            return 0U;
        }
    }
    return 1U;
}

/*
 * 函数用途：解析标准72字节主动帧，或严格还原第50字节后插入50个0的122字节无线异常帧。
 * 调用场景：常驻流解析和交互停流窗口遇到主动帧抢占时共用。
 * 关键约束：只有标准帧CRC失败、零区完整匹配且还原后的全部协议校验通过时才兼容。
 */
/*
 * 函数用途：识别标准72字节或无线补零122字节主动帧候选并还原为标准帧。
 * 调用场景：主动流解析器和停流交互窗口收到疑似主动帧时。
 * 关键约束：数据不足返回NEED_MORE；补零必须位于固定偏移且原标准帧对应区域不能全零。
 */
multiparam_v4_active_candidate_state_t MULTIPARAM_V4_DecodeActiveCandidate(
    const uint8_t *data,
    uint16_t available_length,
    multiparam_v4_snapshot_t *snapshot,
    uint8_t canonical_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
    uint16_t *consumed_length,
    uint8_t *wireless_padding_used,
    uint32_t *parse_result)
{
    uint32_t result;

    if ((data == NULL) || (snapshot == NULL) || (canonical_frame == NULL) ||
        (consumed_length == NULL) || (wireless_padding_used == NULL) ||
        (parse_result == NULL)) {
        if (parse_result != NULL) {
            *parse_result = SYSTEM_CALL_CONDITION_ERROR;
        }
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_INVALID;
    }
    *consumed_length = 0U;
    *wireless_padding_used = 0U;
    if (available_length < MULTIPARAM_V4_ACTIVE_FRAME_SIZE) {
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_NEED_MORE;
    }

    result = MULTIPARAM_V4_ParseActiveFrame(data, snapshot);
    if (result == NO_ERROR) {
        memcpy(canonical_frame, data, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
        *consumed_length = MULTIPARAM_V4_ACTIVE_FRAME_SIZE;
        *parse_result = NO_ERROR;
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID;
    }
    *parse_result = result;
    if ((result != SENSOR_BCC_ERROR) ||
        (MULTIPARAM_V4_IsZeroRange(
             data + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET,
             (uint16_t)(MULTIPARAM_V4_ACTIVE_FRAME_SIZE -
                        MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET)) == 0U)) {
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_INVALID;
    }
    if (available_length < MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE) {
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_NEED_MORE;
    }
    if (MULTIPARAM_V4_IsZeroRange(
            data + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET,
            MULTIPARAM_V4_WIRELESS_ZERO_PADDING_SIZE) == 0U) {
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_INVALID;
    }

    memcpy(canonical_frame, data, MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET);
    memcpy(canonical_frame + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET,
           data + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET +
               MULTIPARAM_V4_WIRELESS_ZERO_PADDING_SIZE,
           MULTIPARAM_V4_ACTIVE_FRAME_SIZE -
               MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET);
    result = MULTIPARAM_V4_ParseActiveFrame(canonical_frame, snapshot);
    *parse_result = result;
    if (result != NO_ERROR) {
        return MULTIPARAM_V4_ACTIVE_CANDIDATE_INVALID;
    }

    *consumed_length = MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE;
    *wireless_padding_used = 1U;
    return MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID;
}

/*
 * 函数用途：校验并解析一帧标准72字节V4主动上发数据。
 * 调用场景：候选帧完成无线补零还原后。
 * 关键约束：依次检查固定字段、地址、CRC和业务浮点有效性；失败时不发布快照。
 */
uint32_t MULTIPARAM_V4_ParseActiveFrame(const uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
                                        multiparam_v4_snapshot_t *snapshot)
{
    uint32_t index;

    if ((frame == NULL) || (snapshot == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if ((frame[0] != MULTIPARAM_V4_ACTIVE_HEADER0) ||
        (frame[1] != MULTIPARAM_V4_ACTIVE_HEADER1) ||
        (frame[2] != MULTIPARAM_V4_ACTIVE_FORMAT_VERSION) ||
        (frame[3] != MULTIPARAM_V4_ACTIVE_FRAME_TYPE) ||
        (frame[7] != MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT) ||
        (frame[8] != (uint8_t)(MULTIPARAM_V4_ACTIVE_DATA_SIZE & 0xFFU)) ||
        (frame[9] != (uint8_t)(MULTIPARAM_V4_ACTIVE_DATA_SIZE >> 8U))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if ((s_expected_address != MULTIPARAM_V4_ANY_ADDRESS) &&
        (frame[4] != s_expected_address)) {
        return SENSOR_ADDRESS_MISMATCH;
    }
    if (!SlaveCheckCRC(frame, (int)MULTIPARAM_V4_ACTIVE_FRAME_SIZE)) {
        return SENSOR_BCC_ERROR;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->address = frame[4];
    snapshot->sequence = (uint16_t)(((uint16_t)frame[6] << 8U) | frame[5]);
    for (index = 0U; index < MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT; index++) {
        snapshot->raw_parameter[index] = MULTIPARAM_V4_DecodeU32Le(frame + 10U + index * 4U);
    }
    snapshot->software_version = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[0]);
    snapshot->protocol_version = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[1]);
    snapshot->status_word = snapshot->raw_parameter[2];
    snapshot->magnetic_zero_voltage = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[3]);
    snapshot->measurement_frequency_raw = (int32_t)snapshot->raw_parameter[4];
    snapshot->measurement_frequency_valid =
        MULTIPARAM_V4_DecodeFrequency(snapshot->measurement_frequency_raw,
                                      &snapshot->measurement_frequency_hz,
                                      &snapshot->fast_sweep_completed);
    snapshot->water_capacitance_pf = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[5]);
    snapshot->temperature_c = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[6]);
    snapshot->density_kg_m3 = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[7]);
    snapshot->dynamic_viscosity_cp = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[8]);
    snapshot->kinematic_viscosity_cst = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[9]);
    snapshot->supply_voltage_v = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[10]);
    snapshot->angle_x_deg = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[11]);
    snapshot->angle_y_deg = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[12]);
    snapshot->sweep_period_square_mean_45 =
        MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[13]);
    snapshot->sweep_period_square_mean_22_5 =
        MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[14]);
    {
        multiparam_v4_operating_state_t operating_state;

        MULTIPARAM_V4_DecodeOperatingState(snapshot->status_word, &operating_state);
        snapshot->measurement_mode = operating_state.measurement_mode;
        snapshot->water_enabled = operating_state.water_enabled;
        snapshot->magnetic_zero_enabled = operating_state.magnetic_zero_enabled;
        snapshot->sweep_stage = operating_state.sweep_stage;
        snapshot->drive_stage = operating_state.drive_stage;
    }
    snapshot->magnetic_zero_valid =
        (uint8_t)(((snapshot->magnetic_zero_enabled != 0U) &&
                   isfinite(snapshot->magnetic_zero_voltage)) ? 1U : 0U);
    snapshot->water_capacitance_valid =
        (uint8_t)(((snapshot->water_enabled != 0U) &&
                   isfinite(snapshot->water_capacitance_pf)) ? 1U : 0U);

    /* 功能关闭或结果尚未生成时，测量字段允许为NaN；帧级只校验身份和协议版本。 */
    if ((!isfinite(snapshot->software_version)) ||
        (!isfinite(snapshot->protocol_version))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (fabsf(snapshot->protocol_version - MULTIPARAM_V4_PROTOCOL_VERSION_VALUE) > 0.01f) {
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    return NO_ERROR;
}

/*
 * 函数用途：计算V4八字节交互帧前七字节的累加校验值。
 * 调用场景：请求组帧和应答校验。
 * 关键约束：纯计算函数，调用方必须提供完整八字节缓冲，不访问UART6。
 */
uint8_t MULTIPARAM_V4_CalculateChecksum(const uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    uint32_t sum = 0U;

    if (frame == NULL) {
        return 0U;
    }
    for (uint32_t index = 0U; index < 7U; index++) {
        sum += frame[index];
    }
    return (uint8_t)(sum & 0xFFU);
}

/*
 * 函数用途：按地址、功能码、32位原始值和参数号生成V4八字节请求。
 * 调用场景：所有V4交互读写、模式和功能控制事务发送前。
 * 关键约束：数据按小端写入，最后一字节统一重算校验；函数不发送数据。
 */
void MULTIPARAM_V4_BuildRequestFrame(uint8_t address,
                                     uint8_t function,
                                     uint32_t raw_value,
                                     uint8_t parameter,
                                     uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    if (frame == NULL) {
        return;
    }
    frame[0] = address;
    frame[1] = function;
    MULTIPARAM_V4_EncodeU32Le(raw_value, frame + 2U);
    frame[6] = parameter;
    frame[7] = MULTIPARAM_V4_CalculateChecksum(frame);
}

/*
 * 函数用途：校验8字节交互应答并返回准确的失败阶段。
 * 调用场景：公开V4校验接口、普通事务和V3/V4公共R01探测共用。
 * 关键约束：先校验BCC、功能码和参数码，再按请求是否为广播决定地址校验。
 */
uint32_t MULTIPARAM_V4_ValidateReplyInternal(
    const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
    multiparam_v4_reply_address_policy_t address_policy,
    const char **stage)
{
    if (stage != NULL) {
        *stage = "未确定";
    }
    if ((request == NULL) || (reply == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_CalculateChecksum(reply) != reply[7]) {
        if (stage != NULL) {
            *stage = "BCC校验";
        }
        return SENSOR_BCC_ERROR;
    }
    if (reply[1] != (uint8_t)(request[1] | 0x80U)) {
        if (stage != NULL) {
            *stage = "功能码校验";
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (reply[6] == 0xFFU) {
        if (stage != NULL) {
            *stage = "远端错误";
        }
        return SENSOR_REMOTE_INTERNAL_ERROR;
    }
    if (reply[6] != request[6]) {
        if (stage != NULL) {
            *stage = "参数码校验";
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* 临时广播事务允许设备返回自身地址；关闭临时开关后仅公共R01保留该规则。 */
    if ((reply[0] != request[0]) &&
        !((request[0] == MULTIPARAM_V4_BROADCAST_ADDRESS) &&
          ((address_policy == MULTIPARAM_V4_REPLY_ADDRESS_PROTOCOL_PROBE) ||
           (MULTIPARAM_V4_TEMP_FORCE_BROADCAST_TX != 0U)))) {
        if (stage != NULL) {
            *stage = "V4地址校验";
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if ((request[1] == MULTIPARAM_V4_FUNCTION_WRITE) &&
        (memcmp(request + 2U, reply + 2U, 4U) != 0)) {
        if (stage != NULL) {
            *stage = "写入回显校验";
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (stage != NULL) {
        *stage = "校验通过";
    }
    return NO_ERROR;
}

/*
 * 函数用途：按严格地址策略校验普通V4八字节应答。
 * 调用场景：生产交互事务收到完整应答后。
 * 关键约束：复用内部校验器的STRICT策略；广播探测的地址例外只能走专用探测入口。
 */
uint32_t MULTIPARAM_V4_ValidateReply(const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
                                    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    return MULTIPARAM_V4_ValidateReplyInternal(
        request, reply, MULTIPARAM_V4_REPLY_ADDRESS_STRICT, NULL);
}
