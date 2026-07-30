#include "hostcommu_modbus.h"
#include "crc.h"
#include "usart.h"
#include "dataanalysis_modbus.h"
#include "stateformodbus.h"
#include "hostcommu.h"
#include <string.h>
#include <ctype.h>
#include "system_parameter.h"
#include "AoOutput/ao_output.h"
#include "Relay/relay_alarm_config.h"
#include "fault_recovery.h"

/* 从机地址 */
int SlaveAddress = 1; /* 主板通信地址配置，影响协议寻址或硬件访问。 */
/* 功能码 */
static const int readholdingregisterfuncode = 0x03; /* 读保持寄存器功能码 */
static const int readinputregisterfuncode = 0x04; /* 读输入寄存器功能码 */
static const int presetmultipleregisterfuncode = 0x10; /* 写多个寄存器功能码 */
/* 异常码 */
static const int illegalfunction = 0x01; /* 非法功能 */
static const int illegaldataaddress = 0x02; /* 非法数据地址 */
static const int illegaldatavalue = 0x03; /* 非法数据值 */
/* static const int slavedevicefailure = 0x04; / /从设备故障 */
static const int slavedevicebusy = 0x06; /* 从设备忙 */
/* 现行Modbus地址直接作为数组索引，不再经过内部紧凑地址转换。 */
static uint16_t HoldingRegisterArray[HOLDREGISTER_AMOUNT] = { 0 };
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };
/* 发送区暂存数组 */
static int SlaveTempBuffer[HOSTCOMMU_SENDLENGTH];
/* 接收到的命令包数据暂存变量 */
static int RCV_functioncode = 0; /* 当前正在解析的 Modbus 功能码。 */
static int RCV_startaddress = 0; /* 主板通信地址配置，影响协议寻址或硬件访问。 */
static int RCV_registercnt = 0; /* 当前 Modbus 请求声明的寄存器数量。 */
/* 静态函数 */
static bool JudgeFunctioncode(void);
static bool JudgeStartAddress(void);
static void ReadRegister(bool registertype, int *registervalue); /* 读寄存器 */
static void PresetRegister(bool registertype, int const *registervalue); /* 写寄存器 */
static int Compose03Package(uint8_t  *revframe, uint8_t  *sendframe);
static int Compose04Package(uint8_t  *revframe, uint8_t  *sendframe);
static int Compose10Package(uint8_t  const *revframe, uint8_t  *sendframe);
static bool IsPersistentDeviceParamWrite(uint16_t startAddr, uint16_t regCount);
static bool PersistentParamWriteRuntimeAllowed(void);
/**
 * @brief 接收到的数据包进行地址检查。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param framelen 参与地址或 CRC 校验的完整帧长度，单位字节。
 * @return true 表示请求地址等于本机 SlaveAddress 或为广播地址 0；false 表示请求发往其他从站。
 */
bool SlaveCheckAddress(uint8_t  const *revframe, int framelen) {
    if (revframe[0] != SlaveAddress && revframe[0] != 0) {
        return false;
    } else {
        return true;
    }
}

/**
 * @brief 设置 CPU2 主机 Modbus 接收地址匹配值。
 *
 * @param address 准备保存的 CPU2 Modbus 本机地址；后续地址校验同时接受广播地址 0。
 * @note 保存的从站地址用于后续请求帧地址一致性判断。
 */
void SetSlaveaddress(int address) {
	SlaveAddress = address;
#if DEBUG_HOSTCOMMU_MODBUS
    printf("SlaveAddress = %d\n",SlaveAddress);
    #endif
}


/**
 * @brief 只有当前可写参数块触发FRAM保存；命令和AO仿真开关保持非持久化。
 *
 * @param startAddr 起始位置地址。
 * @param regCount 数量。
 * @return true 表示写区间命中至少一个需要保存到 FRAM 的 DeviceParameters 字段；false 表示区间无效，或仅覆盖命令、AO 仿真及清锁存瞬时写槽。
 */
static bool IsPersistentDeviceParamWrite(uint16_t startAddr, uint16_t regCount)
{
    return LtdModbus_HoldingWriteTouchesPersistent(startAddr, regCount);
}

/**
 * @brief 判断CPU2当前运行上下文是否允许写入持久参数。
 *
 * @details 调用场景：FC10地址和值解析完成前的最终权限门禁。
 * @note 关键约束：普通完成态必须无错误；错误态只在自动恢复和命令队列均空闲时放行。
 *
 * @return true 表示CPU2当前运行上下文允许写入持久参数；false 表示CPU2当前运行上下文不允许写入持久参数。
 */
static bool PersistentParamWriteRuntimeAllowed(void)
{
    return DeviceContext_AllowsPersistentParamWrite(
        g_measurement.device_status.device_state,
        g_measurement.device_status.current_command,
        g_deviceParams.command,
        g_measurement.device_status.error_code,
        FaultRecovery_IsActive());
}

/**
 * @brief 判断功能码是否正确。
 *
 * @return true 表示收到的功能码为读保持寄存器、读输入寄存器或写多个保持寄存器，属于当前主机 Modbus 实现支持的集合；false 表示其它功能码。
 */
static bool JudgeFunctioncode(void) {
	if ((RCV_functioncode != readholdingregisterfuncode) && (RCV_functioncode != readinputregisterfuncode)
			&& (RCV_functioncode != presetmultipleregisterfuncode)) {
		return false;
	} else {
		return true;
	}
}
/**
 * @brief 按标准Modbus数量限制和当前直接地址数组边界验证请求。
 *
 * @return true 表示当前功能码的寄存器数量未超 Modbus 上限，且完整区间位于对应输入、保持或可写保持寄存器范围；false 表示功能码不匹配、数量非法、区间越界或 FC10 包含不可写地址。
 */
static bool JudgeStartAddress(void) {
    uint16_t start;
    uint16_t count;

    if ((RCV_startaddress < 0) || (RCV_registercnt <= 0)) {
        return false;
    }
    start = (uint16_t)RCV_startaddress;
    count = (uint16_t)RCV_registercnt;
    if (RCV_functioncode == FUNCTIONCODE_READ_INPUTREGISTER) {
        return (count <= LTD_MODBUS_MAX_READ_REGISTERS) &&
               LtdModbus_RangeWithin(start, count, INPUTREGISTER_AMOUNT);
    }
    if (RCV_functioncode == FUNCTIONCODE_READ_HOLDREGISTER) {
        return (count <= LTD_MODBUS_MAX_READ_REGISTERS) &&
               LtdModbus_RangeWithin(start, count, HOLDREGISTER_AMOUNT);
    }
    if (RCV_functioncode == FUNCTIONCODE_WRITE_MULREGISTER) {
        return (count <= LTD_MODBUS_MAX_WRITE_REGISTERS) &&
               LtdModbus_HoldingWriteRangeIsValid(start, count);
    }
    return false;
}
/**
 * @brief 按当前请求起始地址和数量，从输入或保持寄存器镜像复制连续值到输出数组。
 *
 * @param registertype 寄存器区选择标志；true 读取输入寄存器镜像，false 读取保持寄存器镜像。
 * @param registervalue 连续寄存器值输出数组，容量至少为 RCV_registercnt 个 int 元素。
 * @note registertype 为 true 时读取输入寄存器，为 false 时读取保持寄存器；调用前必须已经校验 RCV_startaddress、RCV_registercnt
 *       及输出数组容量。
 */
static void ReadRegister(bool registertype, int *registervalue) {
    const uint16_t *registers = registertype ? InputRegisterArray : HoldingRegisterArray;
    int index;

    for (index = 0; index < RCV_registercnt; index++) {
        registervalue[index] = (int)registers[RCV_startaddress + index];
    }
}

/**
 * @brief 检查功能码，若错误则组织违法功能码响应包。
 *
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @param framelength 用于返回已构造 Modbus 响应帧的有效长度，单位字节。
 * @return true 表示功能码为当前支持的 FC03、FC04 或 FC10，可继续正常处理；false 表示功能码非法，函数已构造异常功能码响应头并写回 framelength=3。
 */
bool FunctionCheckIllPack(uint8_t  *sendframe, int *framelength) {
	if (JudgeFunctioncode() == false) {
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + RCV_functioncode;
		sendframe[2] = illegalfunction;
		*framelength = 3;
		return false;
	} else {
		return true;
	}
}
/**
 * @brief 检查数据起始地址和寄存器数量,若错误则组织违法数据响应包。
 *
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @param framelength 用于返回已构造 Modbus 响应帧的有效长度，单位字节。
 * @return true 表示起始地址和寄存器数量合法，可继续访问；false 表示请求区间非法，函数已构造非法数据地址异常响应头并写回 framelength=3。
 */
bool IllegalDataAddressPack(uint8_t  *sendframe, int *framelength) {
	if (JudgeStartAddress() == false) {
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + RCV_functioncode;
		sendframe[2] = illegaldataaddress;
		*framelength = 3;
		return false;
	} else {
		return true;
	}
}

/**
 * @brief 更新功能码\起始地址\寄存器数量。
 *
 * @param funccode 本次 Modbus 请求的功能码。
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 */
void UpdateRcvPara(int funccode, int startadd, int registercnt) {
	RCV_functioncode = funccode;
	RCV_startaddress = startadd;
	RCV_registercnt = registercnt;
#if DEBUG_HOSTCOMMU
    printf("功能码\t0x%02x\r\n",RCV_functioncode);
    printf("起始地址\t0x%02x\r\n",RCV_startaddress);
    printf("寄存器数量\t%d\r\n",RCV_registercnt);
    #endif
}

/**
 * @brief 处理03功能码命令包 并组织响应包。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回 CPU2 FC03 响应帧总长度，单位字节；请求非法时返回已构造异常帧的长度。
 */
int Response03Process(uint8_t  *revframe, uint8_t  *sendframe) {
	int length;
	/* 重置保持寄存器 */
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	/* 组包 */
	length = Compose03Package(revframe, sendframe);
	return length;
}

/**
 * @brief 在命令包格式正确的情况下,组织03响应包。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回 FC03 响应 PDU 组包长度 3 + 2×寄存器数量，单位字节，尚不包含 CRC16。
 */
static int Compose03Package(uint8_t  *revframe, uint8_t  *sendframe) {
	int i;
	int j;
	int sendlength;
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = RCV_registercnt * 2;
	/* 读保持寄存器 */
	ReadRegister(false, SlaveTempBuffer);
	for (i = 0, j = 3; i < RCV_registercnt; i++, j = j + 2) {
		sendframe[j] = (SlaveTempBuffer[i] >> 8) & 0xff;
		sendframe[j + 1] = SlaveTempBuffer[i] & 0xff;
	}
	sendlength = 3 + RCV_registercnt * 2;
	return sendlength;
}
/**
 * @brief 处理04功能码命令包 并组织响应包。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回 CPU2 FC04 响应帧总长度，单位字节；请求非法时返回已构造异常帧的长度。
 */
int Response04Process(uint8_t  *revframe, uint8_t  *sendframe) {
	int length;
	/* 重置输入寄存器 */
	write_measurement_result_to_InputRegisters(InputRegisterArray);
	/* 组包 */
	length = Compose04Package(revframe, sendframe);
	return length;
}
/**
 * @brief 在命令包格式正确的情况下,组织04响应包。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回 FC04 响应 PDU 组包长度 3 + 2×寄存器数量，单位字节，尚不包含 CRC16。
 */
static int Compose04Package(uint8_t  *revframe, uint8_t  *sendframe) {
	int i;
	int j;
	int sendlength;
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = RCV_registercnt * 2;
	/* 读输入寄存器 */
	ReadRegister(true, SlaveTempBuffer);
	for (i = 0, j = 3; i < RCV_registercnt; i++, j = j + 2) {
		sendframe[j] = (SlaveTempBuffer[i] >> 8) & 0xff;
		sendframe[j + 1] = SlaveTempBuffer[i] & 0xff;
	}
	sendlength = 3 + RCV_registercnt * 2;
	return sendlength;
}


/**
 * @brief 功能码 0x10：写多个保持寄存器处理。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回待追加 CRC 的响应长度：合法 FC10 ACK 为 6 字节，标准异常响应为 3 字节。
 */
int Response10Process(uint8_t const *revframe, uint8_t *sendframe)
{
    DeviceParameters previous_params;
    DeviceParameters candidate_params;
    int length;
    uint16_t startAddr;
    uint16_t regCount;
    uint32_t command_value = 0U;
    uint32_t previous_tank_height;
    uint32_t previous_simulation_enabled;
    uint32_t candidate_simulation_enabled;
    uint32_t candidate_simulation_raw;
    int64_t sensor_position_01mm;
    int need_save = 0;
    int persist_write = 0;
    int command_arguments_only = 0;

    startAddr = ((uint16_t)revframe[2] << 8) | revframe[3];
    regCount  = ((uint16_t)revframe[4] << 8) | revframe[5];
    if (LtdModbus_RangeContains(startAddr, regCount,
                                HOLDREGISTER_DEVICEPARAM_COMMAND, REG_STRIDE)) {
        command_value = ((uint32_t)revframe[7] << 24) |
                        ((uint32_t)revframe[8] << 16) |
                        ((uint32_t)revframe[9] << 8) |
                        (uint32_t)revframe[10];
        if ((startAddr != HOLDREGISTER_DEVICEPARAM_COMMAND) ||
            (regCount != REG_STRIDE) ||
            !LtdModbus_CommandIsImplemented(command_value)) {
            sendframe[0] = (uint8_t)SlaveAddress;
            sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
            sendframe[2] = (uint8_t)illegaldatavalue;
            return 3;
        }
    }
    /* 重复不可自中断命令只返回ACK，不得覆盖此前已确认的其他待执行命令。 */
    if ((startAddr == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
        (regCount == REG_STRIDE) &&
        (g_measurement.device_status.current_command != CMD_NONE) &&
        ((CommandType)command_value ==
         g_measurement.device_status.current_command) &&
        !IsSelfInterruptibleCommand(
            g_measurement.device_status.current_command)) {
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)presetmultipleregisterfuncode;
        sendframe[2] = revframe[2];
        sendframe[3] = revframe[3];
        sendframe[4] = revframe[4];
        sendframe[5] = revframe[5];
        return 6;
    }
    persist_write = IsPersistentDeviceParamWrite(startAddr, regCount) ? 1 : 0;
    command_arguments_only =
        LtdModbus_HoldingWriteIsCommandArgumentOnly(startAddr, regCount) ? 1 : 0;
    /* 恢复出厂从命令ACK到整套默认值保存完成期间，七字段写入必须明确返回忙。 */
    if ((command_arguments_only != 0) &&
        ((g_deviceParams.command == CMD_RESTORE_FACTORY) ||
         (g_measurement.device_status.current_command == CMD_RESTORE_FACTORY) ||
         DeviceParams_IsFactoryRestoreInProgress())) {
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
        sendframe[2] = (uint8_t)slavedevicebusy;
        return 3;
    }
    if ((persist_write != 0) &&
        (command_arguments_only == 0) &&
        !PersistentParamWriteRuntimeAllowed()) {
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
        sendframe[2] = (uint8_t)slavedevicebusy;
        return 3;
    }
    previous_params = g_deviceParams;
    previous_tank_height = previous_params.tankHeight;
    previous_simulation_enabled = AoOutput_IsSimulationEnabled();

    /* 先发布当前镜像，再叠加主站本次写入。 */
    WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
    length = Compose10Package(revframe, sendframe);
    candidate_simulation_raw =
            ((uint32_t)HoldingRegisterArray[HOLDREGISTER_AO_SIMULATION_ENABLE] << 16) |
            (uint32_t)HoldingRegisterArray[HOLDREGISTER_AO_SIMULATION_ENABLE + 1U];
    if (candidate_simulation_raw > 1U) {
        /* 仿真开关仅接受0/1，解析运行态前拒绝非法值并恢复原寄存器镜像。 */
        WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
        sendframe[2] = (uint8_t)illegaldatavalue;
        return 3;
    }
    ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
    candidate_params = g_deviceParams;
    candidate_simulation_enabled = AoOutput_IsSimulationEnabled();

    /* AO配置必须严格校验；源切换或当前源上限变化时成对加载默认量程。 */
    if (prepare_ao_params_for_write(&previous_params,
                                    &candidate_params,
                                    startAddr,
                                    regCount) != 0) {
        g_deviceParams = previous_params;
        AoOutput_SetSimulationEnabled(previous_simulation_enabled);
        WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
        sendframe[2] = (uint8_t)illegaldatavalue;
        return 3;
    }

    /* 继电器候选按实际写字段校验；禁用通道可逐项配置，启用后仍保持完整约束。 */
    if (!RelayAlarmConfig_WriteCandidateIsValid(
            &candidate_params, startAddr, regCount)) {
        g_deviceParams = previous_params;
        AoOutput_SetSimulationEnabled(previous_simulation_enabled);
        WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
        sendframe[0] = (uint8_t)SlaveAddress;
        sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
        sendframe[2] = (uint8_t)illegaldatavalue;
        return 3;
    }

    /* AO禁用时拒绝开启仿真；由输出模式切到禁用时清除潜伏的非持久化仿真状态。 */
    if ((candidate_params.ao_output.work_mode == AO_WORK_MODE_DISABLED) &&
        (candidate_simulation_enabled != 0U)) {
        if (LtdModbus_RangeContains(startAddr, regCount,
                                    HOLDREGISTER_AO_SIMULATION_ENABLE, REG_STRIDE)) {
            g_deviceParams = previous_params;
            AoOutput_SetSimulationEnabled(previous_simulation_enabled);
            WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
            sendframe[0] = (uint8_t)SlaveAddress;
            sendframe[1] = (uint8_t)(presetmultipleregisterfuncode | 0x80);
            sendframe[2] = (uint8_t)illegaldatavalue;
            return 3;
        }
        candidate_simulation_enabled = 0U;
    }
    if (persist_write != 0) {
        DeviceParams_CaptureWriteSnapshot(&previous_params);
    }
    g_deviceParams = candidate_params;
    DeviceCommandArguments_RecordWrite(startAddr, regCount);
    if (LtdModbus_RangeContains(startAddr, regCount,
                                HOLDREGISTER_DEVICEPARAM_COMMAND, REG_STRIDE)) {
        DeviceCommandArguments_CapturePending(g_deviceParams.command);
    }
    AoOutput_SetSimulationEnabled(candidate_simulation_enabled);
    WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);

    if (g_deviceParams.tankHeight != previous_tank_height) {
        /* 当前函数位于 UART5 中断，只重算缓存位置，不访问编码器或电机外设。 */
        sensor_position_01mm = (int64_t)g_deviceParams.tankHeight -
                               (int64_t)g_measurement.debug_data.cable_length;
        if (sensor_position_01mm > (int64_t)INT32_MAX) {
            sensor_position_01mm = (int64_t)INT32_MAX;
        } else if (sensor_position_01mm < (int64_t)INT32_MIN) {
            sensor_position_01mm = (int64_t)INT32_MIN;
        }
        g_measurement.debug_data.sensor_position = (int32_t)sensor_position_01mm;
    }

    if (persist_write != 0) {
        need_save = 1;
    }
    if (need_save != 0) {
        request_device_params_save();
    }
    return length;
}

/**
 * @brief 更新保持寄存器,组织10响应包。
 *
 * @param revframe CPU2 主机 Modbus RTU 请求帧只读缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe CPU2 主机 Modbus RTU 响应输出缓冲区；函数按请求功能码写入从站地址、功能码、数据或异常字段，CRC 由统一流程追加。
 * @return 返回 FC10 正常回显响应的固定 PDU 长度 6 字节，尚不包含 CRC16。
 */
static int Compose10Package(uint8_t  const *revframe, uint8_t  *sendframe) {
	int i, j;
	int length;
	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
	for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
		SlaveTempBuffer[i] = (revframe[j + 7] << 8) + revframe[j + 8];
	}
	/* 写保持寄存器 */
	PresetRegister(false, SlaveTempBuffer);
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = revframe[2];
	sendframe[3] = revframe[3];
	sendframe[4] = revframe[4];
	sendframe[5] = revframe[5];
	length = 6;
	return length;
}

/**
 * @brief 把当前主机写请求中的连续寄存器值写入 CPU2 保持寄存器镜像。
 *
 * registertype --> false - 保持寄存器。
 * > true - 输入寄存器。
 *
 * @param registertype > false - 保持寄存器。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 */
static void PresetRegister(bool registertype, int const *registervalue) {
    int index;

    if (registertype) {
        return;
    }
    for (index = 0; index < RCV_registercnt; index++) {
        HoldingRegisterArray[RCV_startaddress + index] = (uint16_t)registervalue[index];
    }
}
