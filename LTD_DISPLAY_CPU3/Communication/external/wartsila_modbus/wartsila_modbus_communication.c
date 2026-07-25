/*
 * wartsila_modbus_communication.c
 *
 *  Created on: Nov 15, 2025
 *      Author: Duan Xuebin
 */


#include "wartsila_modbus_communication.h"
#include "wartsila_modbus_data_analysis.h"
#include "my_crc.h"
#include <string.h>
#include "address.h"
#include "cpu2_communicate.h"
#include "cpu3_debug_log.h"
#include "../external_read_freshness.h"

static bool modbus_on_holding_written(uint16_t start, uint16_t qty);
/**
 * @brief 将瓦锡兰保持寄存器中的参数整理后下发到 CPU2。
 * @note 只在外部协议写入参数后调用，保持 CPU3 对外缓存和 CPU2 参数一致。
 */
static bool ForwardParamsToLowerDevice(void);
/* ================= 寄存器池 ================= */
uint16_t g_holding_regs[HOLDREG_COUNT] = {0};   /* 应用层可定期把你的参数写进/读出这个数组 */

/* ================ 内部工具 ================ */
static inline uint16_t be16(const uint8_t* p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}
/* 按 Modbus 大端字节序写入 16 位寄存器值。 */
static inline void wr_be16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

/* 判断 [start, start+qty-1] 是否覆盖了某个地址。 */
static inline int range_contains(uint16_t start, uint16_t qty, uint16_t addr)
{
    return (addr >= start) && (addr <= (uint16_t)(start + qty - 1U));
}

/* 只有命令和四个分布参数地址需要桥接到 CPU2。 */
static bool wartsila_write_targets_cpu2(uint16_t start, uint16_t qty)
{
    return (range_contains(start, qty, 0x0006U) != 0) ||
           (range_contains(start, qty, REG_SPREAD_LOWEST_POINT) != 0) ||
           (range_contains(start, qty, REG_SPREAD_HIGHEST_POINT) != 0) ||
           (range_contains(start, qty, REG_SPREAD_INTERVAL) != 0) ||
           (range_contains(start, qty, REG_SPREAD_DIST_TO_SURFACE) != 0);
}

/* ============== 异常应答（功能码|0x80, 异常码）============== */
static uint8_t build_exception(uint8_t addr, uint8_t func, uint8_t ex_code,
                               uint8_t* tx, uint16_t* tx_len)
{
    tx[0] = addr;
    tx[1] = func | 0x80;
    tx[2] = ex_code; /* 01:非法功能, 02:非法地址, 03:非法数据值, 04:从站故障 */
    uint16_t crc = CRC16_Calculate(tx, 3);
    tx[3] = (uint8_t)(crc & 0xFF);
    tx[4] = (uint8_t)(crc >> 8);
    *tx_len = 5;
    return 1;
}

/* ============== 处理 0x03 读取保持寄存器 =============== */
static uint8_t handle_0x03(uint8_t addr, const uint8_t* pdu, uint16_t pdu_len,
                           uint8_t* tx, uint16_t* tx_len)
{
    /* pdu: [func(1)=0x03][startHi][startLo][qtyHi][qtyLo] */
    if (pdu_len != 5) return build_exception(addr, 0x03, 0x03, tx, tx_len); /* 长度异常 -> 数据值非法 */

    uint16_t start = be16(&pdu[1]);
    uint16_t qty   = be16(&pdu[3]);

    if (qty < 1 || qty > 0x007D) /* Modbus建议单次最多125寄存器 */
        return build_exception(addr, 0x03, 0x03, tx, tx_len); /* ILLEGAL DATA VALUE */

    /* 如果遇到特殊包，返回定制的数据 */
    if (CPU3_ExternalWartsilaIsSignatureAddress(start) != 0U) {  /* 特定地址处理 */
        if (CPU3_ExternalWartsilaSignatureQuantityIsValid(qty) == 0U)
            return build_exception(addr, 0x03, 0x03, tx, tx_len);

        /* 填充响应数据 */
        tx[0] = addr;  /* 从站地址 */
        tx[1] = 0x03;  /* 功能码 */
        tx[2] = 0x28;  /* 数据字节数（40字节） */

        /* 填充定制的响应数据 */
        memcpy(&tx[3], "REV. 4.1  28/07/2016REV. 4.00 10/11/2017 CC", 40);  /* 定制返回的数据 */
        /* 计算 CRC 校验，包括地址、功能码、字节数和数据内容 */
        uint16_t frame_len_wo_crc = 3 + 40;  /* 3 是地址、功能码、字节数，40 是数据内容 */
        uint16_t crc = CRC16_Calculate(tx, frame_len_wo_crc);  /* 计算 CRC */
        tx[frame_len_wo_crc]     = (uint8_t)(crc & 0xFF);  /* CRC 低字节 */
        tx[frame_len_wo_crc + 1] = (uint8_t)(crc >> 8);    /* CRC 高字节 */

        *tx_len = frame_len_wo_crc + 2;  /* 更新数据包长度（包括 CRC 校验的 2 字节） */

        return 1;  /* 返回成功 */
    }

    /* 地址越界判断 */
    if (start < HOLDREG_START_ADDR || (start + qty - 1) > HOLDREG_END_ADDR)
        return build_exception(addr, 0x03, 0x02, tx, tx_len); /* ILLEGAL DATA ADDRESS */

    uint8_t requirements = CPU3_ExternalWartsilaReadRequirements(start, qty);
    if (((requirements & (uint8_t)CPU3_EXTERNAL_READ_RUNTIME) != 0U) &&
        !CPU2_CommHasRuntimeSnapshot())
        return build_exception(addr, 0x03, 0x06, tx, tx_len);
    if (((requirements & (uint8_t)CPU3_EXTERNAL_READ_PARAMETERS) != 0U) &&
        !CPU2_CommIsAvailable())
        return build_exception(addr, 0x03, 0x06, tx, tx_len);
    if (((requirements & (uint8_t)CPU3_EXTERNAL_READ_FIXED_POINT) != 0U) &&
        !CPU2_CommHasFixedPointSnapshot())
        return build_exception(addr, 0x03, 0x06, tx, tx_len);

    /* 静态白名单始终本地重建；其余字段只在对应快照门禁通过后投影。 */
    Wartsila_StoreLocalStaticRegisters(g_holding_regs);
    if (requirements != (uint8_t)CPU3_EXTERNAL_READ_LOCAL)
        DeviceParams_StoreToRegisters(g_holding_regs);

    /* 构建应答 */
    tx[0] = addr;
    tx[1] = 0x03;
    tx[2] = (uint8_t)(qty * 2); /* 字节数 */
    /* 拷贝寄存器到数据区（高字节在前） */
    uint16_t base = start - HOLDREG_START_ADDR;
    for (uint16_t i = 0; i < qty; ++i) {
        wr_be16(&tx[3 + 2*i], g_holding_regs[base + i]);
    }
    uint16_t frame_len_wo_crc = 3 + 2*qty;
    uint16_t crc = CRC16_Calculate(tx, frame_len_wo_crc);
    tx[frame_len_wo_crc]     = (uint8_t)(crc & 0xFF);
    tx[frame_len_wo_crc + 1] = (uint8_t)(crc >> 8);
    *tx_len = frame_len_wo_crc + 2;
    return 1;
}

/* ============== 处理 0x10 写多个保持寄存器 =============== */
static uint8_t handle_0x10(uint8_t addr, const uint8_t* pdu, uint16_t pdu_len,
                           uint8_t* tx, uint16_t* tx_len, bool *write_applied)
{
    if (write_applied == NULL) return 0U;
    *write_applied = false;
    /* pdu: [func(1)=0x10][startHi][startLo][qtyHi][qtyLo][byteCount][data...] */
    if (pdu_len < 6) return build_exception(addr, 0x10, 0x03, tx, tx_len); /* 长度不足 */

    uint16_t start = be16(&pdu[1]);
    uint16_t qty   = be16(&pdu[3]);
    uint8_t  bytes = pdu[5]; /* */

    if (CPU3_ExternalWartsilaWriteShapeIsValid(qty, bytes, pdu_len) == 0U)
    {
        return build_exception(addr, 0x10, 0x03, tx, tx_len);
    }

    if (start < HOLDREG_START_ADDR || (start + qty - 1) > HOLDREG_END_ADDR)
        return build_exception(addr, 0x10, 0x02, tx, tx_len);

    if (range_contains(start, qty, REG_DOWN_COMMAND) != 0)
    {
        uint16_t command_index = (uint16_t)(REG_DOWN_COMMAND - start);
        uint16_t requested_command = be16(&pdu[6U + (2U * command_index)]);

        if (CPU3_ExternalWartsilaCommandIsSupported(requested_command) == 0U)
            return build_exception(addr, 0x10, 0x03, tx, tx_len);
    }

    if (wartsila_write_targets_cpu2(start, qty) && !CPU2_CommIsAvailable())
        return build_exception(addr, 0x10, 0x06, tx, tx_len);

    /* 写入寄存器池 */
    uint16_t base = start - HOLDREG_START_ADDR;
    const uint8_t* pdata = &pdu[6];
    for (uint16_t i = 0; i < qty; ++i) {
        g_holding_regs[base + i] = be16(&pdata[2*i]);
    }
    *write_applied = true;

    /* 正常应答（回显起始地址与数量） */
    tx[0] = addr;
    tx[1] = 0x10;
    wr_be16(&tx[2], start);
    wr_be16(&tx[4], qty);
    uint16_t crc = CRC16_Calculate(tx, 6);
    tx[6] = (uint8_t)(crc & 0xFF);
    tx[7] = (uint8_t)(crc >> 8);
    *tx_len = 8;
    return 1;
}

/* ============== 顶层处理 ============== */
ModbusResult modbus_rtu_process(const uint8_t* rx, uint16_t rx_len,
                                uint8_t* tx, uint16_t* tx_len)
{
    *tx_len = 0;
    if (rx_len < 4) return MODBUS_ERR_BADLEN;

    uint8_t addr = rx[0];
    uint8_t func = rx[1];

    if (addr != SlaveAddress) return MODBUS_ERR_ADDR_MISMATCH;

/* uint16_t rx_crc = (uint16_t)(rx[rx_len - 2] | (rx[rx_len - 1] << 8)); */
	if (SlaveCheckCRC(rx, rx_len) == false) {
		return MODBUS_ERR_CRC;
	}
/* uint16_t cal_crc = modbus_crc16(rx, rx_len - 2); */
/* if (rx_crc != cal_crc) return MODBUS_ERR_CRC; */

    const uint8_t* pdu = &rx[1];
    uint16_t pdu_len = rx_len - 3;

    switch (func) {
        case 0x03:
            if (handle_0x03(addr, pdu, pdu_len, tx, tx_len)) return MODBUS_OK;
            break;

        case 0x10: {
            /* 从 PDU 中解析出起始地址和数量 */
            if (pdu_len < 6) {
                build_exception(addr, func, 0x03, tx, tx_len);
                return MODBUS_ERR_BADLEN;
            }
            uint16_t start = (uint16_t)((pdu[1] << 8) | pdu[2]);
            uint16_t qty   = (uint16_t)((pdu[3] << 8) | pdu[4]);
            bool write_applied = false;

            if (handle_0x10(addr, pdu, pdu_len, tx, tx_len, &write_applied)) {
                if (write_applied && !modbus_on_holding_written(start, qty)) {
                    /* CPU2 未确认时覆盖成功回显，明确要求外部主机重试。 */
                    build_exception(addr, func, 0x06, tx, tx_len);
                }
                return MODBUS_OK;
            }
            break;
        }

        default:
            build_exception(addr, func, 0x01, tx, tx_len);
            return MODBUS_ERR_FUNC_UNSUPPORT;
    }

    return MODBUS_OK;
}
/* 命令影子只允许消费一次；清零后同步刷新 Wartsila 寄存器池。 */
static void Wartsila_ClearCommandShadow(void)
{
	g_deviceParams.command = CMD_NONE;
	DeviceParams_StoreToRegisters(g_holding_regs);
}

/* 把写入的寄存器转成设备参数并“往下发” */
static bool modbus_on_holding_written(uint16_t start, uint16_t qty)
{
    uint32_t confirmed_upper_density_limit = g_deviceParams.wartsila_upper_density_limit;
    uint32_t confirmed_lower_density_limit = g_deviceParams.wartsila_lower_density_limit;
    uint32_t confirmed_density_interval = g_deviceParams.wartsila_density_interval;
    uint32_t confirmed_max_height_above_surface = g_deviceParams.wartsila_max_height_above_surface;
    bool command_write = range_contains(start, qty, 0x0006U) != 0;
    bool parameter_write =
        (range_contains(start, qty, REG_SPREAD_LOWEST_POINT) != 0) ||
        (range_contains(start, qty, REG_SPREAD_HIGHEST_POINT) != 0) ||
        (range_contains(start, qty, REG_SPREAD_INTERVAL) != 0) ||
        (range_contains(start, qty, REG_SPREAD_DIST_TO_SURFACE) != 0);

    if (!command_write && !parameter_write) {
        return true;
    }

    /* 1）先从 g_holding_regs 解析到 g_deviceParams */
    DeviceParams_LoadFromRegisters(g_holding_regs);

	/* 同一帧跨命令和参数区时先同步参数，避免测量命令使用旧参数启动。 */
	if (parameter_write && !ForwardParamsToLowerDevice())
	{
		/* 底层已关闭参数门禁并请求补读；本地先恢复最后确认镜像，避免暴露伪成功值。 */
		g_deviceParams.wartsila_upper_density_limit = confirmed_upper_density_limit;
		g_deviceParams.wartsila_lower_density_limit = confirmed_lower_density_limit;
		g_deviceParams.wartsila_density_interval = confirmed_density_interval;
		g_deviceParams.wartsila_max_height_above_surface = confirmed_max_height_above_surface;
		Wartsila_ClearCommandShadow();
		return false;
	}

	if (command_write)
	{
		bool command_sent = true;
		if (g_deviceParams.command != CMD_NONE)
		{
			uint32_t cmd32 = (uint32_t)g_deviceParams.command;
			CPU3_LOG_INFO("WARTSILA",
						  "收到外部命令并准备下发CPU2 命令=%u",
						  (unsigned int)g_deviceParams.command);
			/* 将 CMD_xxx 写入 HOLDREGISTER_DEVICEPARAM_COMMAND（2 个保持寄存器） */
			command_sent = CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
												 HOLDREGISTER_DEVICEPARAM_COMMAND,
												 2,
												 &cmd32);
		}
		Wartsila_ClearCommandShadow();
		if (!command_sent)
		{
			return false;
		}
	}

	return true;
}
/**
 * @brief 将瓦锡兰外部协议参数同步下发到 CPU2 参数区。
 * @note 由外部 Modbus 写保持寄存器后触发，避免 CPU3 缓存和 CPU2 参数脱节。
 */
static bool ForwardParamsToLowerDevice(void)
{
	uint32_t upper_density_limit = g_deviceParams.wartsila_upper_density_limit;
	uint32_t lower_density_limit = g_deviceParams.wartsila_lower_density_limit;
	uint32_t density_interval = g_deviceParams.wartsila_density_interval;
	uint32_t max_height_above_surface = g_deviceParams.wartsila_max_height_above_surface;
/* / / 示例：根据 command 不同，打不同的下发帧 */
/* if (g_deviceParams.command != CMD_NONE) { */
/* uint32_t cmd32 = (uint32_t)g_deviceParams.command; / / 如果原来是 uint16_t，也没问题 */
/* printf("接收到下发指令：%d\r\n", g_deviceParams.command); */
/* / * 将 CMD_xxx 写入 HOLDREGISTER_DEVICEPARAM_COMMAND（2 个保持寄存器） * / */
/* CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, 2, &cmd32 ); */
/* g_deviceParams.command = CMD_NONE; */
/* DeviceParams_StoreToRegisters(g_holding_regs); */
/* } */
	return CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									  HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,
									  2,
									  &upper_density_limit) &&
		   CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									  HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,
									  2,
									  &lower_density_limit) &&
		   CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									  HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,
									  2,
									  &density_interval) &&
		   CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									  HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,
									  2,
									  &max_height_above_surface);
}


