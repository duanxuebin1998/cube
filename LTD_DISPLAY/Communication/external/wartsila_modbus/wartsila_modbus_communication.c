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
#include "communicate.h"

static void modbus_on_holding_written(uint16_t start, uint16_t qty);
// ================= 寄存器池 =================
uint16_t g_holding_regs[HOLDREG_COUNT] = {0};   // 应用层可定期把你的参数写进/读出这个数组

// ================ 内部工具 ================
static inline uint16_t be16(const uint8_t* p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}
static inline void wr_be16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

// ============== 异常应答（功能码|0x80, 异常码）==============
static uint8_t build_exception(uint8_t addr, uint8_t func, uint8_t ex_code,
                               uint8_t* tx, uint16_t* tx_len)
{
    tx[0] = addr;
    tx[1] = func | 0x80;
    tx[2] = ex_code; // 01:非法功能, 02:非法地址, 03:非法数据值, 04:从站故障
    uint16_t crc = CRC16_Calculate(tx, 3);
    tx[3] = (uint8_t)(crc & 0xFF);
    tx[4] = (uint8_t)(crc >> 8);
    *tx_len = 5;
    return 1;
}

// ============== 处理 0x03 读取保持寄存器 ===============
static uint8_t handle_0x03(uint8_t addr, const uint8_t* pdu, uint16_t pdu_len,
                           uint8_t* tx, uint16_t* tx_len)
{
    // pdu: [func(1)=0x03][startHi][startLo][qtyHi][qtyLo]
    if (pdu_len != 5) return build_exception(addr, 0x03, 0x03, tx, tx_len); // 长度异常 -> 数据值非法

    uint16_t start = be16(&pdu[1]);
    uint16_t qty   = be16(&pdu[3]);

    if (qty < 1 || qty > 0x007D) // Modbus建议单次最多125寄存器
        return build_exception(addr, 0x03, 0x03, tx, tx_len); // ILLEGAL DATA VALUE

    // 地址越界判断
    if (start < HOLDREG_START_ADDR || (start + qty - 1) > HOLDREG_END_ADDR)
        return build_exception(addr, 0x03, 0x02, tx, tx_len); // ILLEGAL DATA ADDRESS

    // 构建应答
    tx[0] = addr;
    tx[1] = 0x03;
    tx[2] = (uint8_t)(qty * 2); // 字节数
    // 拷贝寄存器到数据区（高字节在前）
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

// ============== 处理 0x10 写多个保持寄存器 ===============
static uint8_t handle_0x10(uint8_t addr, const uint8_t* pdu, uint16_t pdu_len,
                           uint8_t* tx, uint16_t* tx_len)
{
    // pdu: [func(1)=0x10][startHi][startLo][qtyHi][qtyLo][byteCount][data...]
    if (pdu_len < 6) return build_exception(addr, 0x10, 0x03, tx, tx_len); // 长度不足

    uint16_t start = be16(&pdu[1]);
    uint16_t qty   = be16(&pdu[3]);
    uint8_t  bytes = pdu[5];//

    if (qty < 1 || qty > 0x007B) // 推荐单次最多123寄存器（字节数<=246）
    {
    	printf("qty=%d\r\n", qty);
        return build_exception(addr, 0x10, 0x03, tx, tx_len);
    }
    if (bytes != (uint8_t)(qty)){
    	printf("bytes=%d, qty*2=%d\r\n", bytes, qty*2);
        return build_exception(addr, 0x10, 0x03, tx, tx_len);
    }
    if (pdu_len != (uint16_t)(6 + 2*bytes)){
    	printf("pdu_len=%d, 6+bytes=%d\r\n", pdu_len, 6+2*bytes);
        return build_exception(addr, 0x10, 0x03, tx, tx_len);
    }

    if (start < HOLDREG_START_ADDR || (start + qty - 1) > HOLDREG_END_ADDR)
        return build_exception(addr, 0x10, 0x02, tx, tx_len);

    // 写入寄存器池
    uint16_t base = start - HOLDREG_START_ADDR;
    const uint8_t* pdata = &pdu[6];
    for (uint16_t i = 0; i < qty; ++i) {
        g_holding_regs[base + i] = be16(&pdata[2*i]);
    }

    // 正常应答（回显起始地址与数量）
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

// ============== 顶层处理 ==============
ModbusResult modbus_rtu_process(const uint8_t* rx, uint16_t rx_len,
                                uint8_t* tx, uint16_t* tx_len)
{
    *tx_len = 0;
    if (rx_len < 4) return MODBUS_ERR_BADLEN;

    uint8_t addr = rx[0];
    uint8_t func = rx[1];

    if (addr != SlaveAddress) return MODBUS_ERR_ADDR_MISMATCH;

//    uint16_t rx_crc  = (uint16_t)(rx[rx_len - 2] | (rx[rx_len - 1] << 8));
	if (SlaveCheckCRC(rx, rx_len) == false) {
		return MODBUS_ERR_CRC;
	}
//    uint16_t cal_crc = modbus_crc16(rx, rx_len - 2);
//    if (rx_crc != cal_crc) return MODBUS_ERR_CRC;

    const uint8_t* pdu = &rx[1];
    uint16_t pdu_len = rx_len - 3;

    switch (func) {
        case 0x03:
            // 读之前把设备测量值刷到寄存器
//        	printf("分布测量起始点：%d\r\n",g_deviceParams..measurement_points);
            DeviceParams_StoreToRegisters(g_holding_regs);
            if (handle_0x03(addr, pdu, pdu_len, tx, tx_len)) return MODBUS_OK;
            break;

        case 0x10: {
            // 从 PDU 中解析出起始地址和数量
            if (pdu_len < 6) {
                build_exception(addr, func, 0x03, tx, tx_len);
                return MODBUS_ERR_BADLEN;
            }
            uint16_t start = (uint16_t)((pdu[1] << 8) | pdu[2]);
            uint16_t qty   = (uint16_t)((pdu[3] << 8) | pdu[4]);

            if (handle_0x10(addr, pdu, pdu_len, tx, tx_len)) {
                // 写寄存器成功后，触发回调
                modbus_on_holding_written(start, qty);
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
// 判断 [start, start+qty-1] 是否覆盖了某个地址
static inline int range_contains(uint16_t start, uint16_t qty, uint16_t addr)
{
    return (addr >= start) && (addr <= (uint16_t)(start + qty - 1));
}

// 把写入的寄存器转成设备参数并“往下发”
static void modbus_on_holding_written(uint16_t start, uint16_t qty)
{
    int need_forward = 0;

    // 只关心 0x0006、0x005A~0x005D 这几个写寄存器
    if (range_contains(start, qty, 0x0006)  ||
        range_contains(start, qty, 0x005A) ||
        range_contains(start, qty, 0x005B) ||
        range_contains(start, qty, 0x005C) ||
        range_contains(start, qty, 0x005D)) {
        need_forward = 1;
    }

    if (!need_forward) {
        return; // 其它地址的写操作，直接忽略
    }

    // 1）先从 g_holding_regs 解析到 g_deviceParams
    DeviceParams_LoadFromRegisters(g_holding_regs);

    // 2）再把这些参数按下级设备的协议发送出去
    ForwardParamsToLowerDevice();
}
void ForwardParamsToLowerDevice(void)
{
    // 示例：根据 command 不同，打不同的下发帧
	if (g_deviceParams.command != CMD_NONE) {
		uint32_t cmd32 = (uint32_t)g_deviceParams.command;   // 如果原来是 uint16_t，也没问题
		printf("接收到下发指令：%d\r\n", g_deviceParams.command);
		/* 将 CMD_xxx 写入 HOLDREGISTER_DEVICEPARAM_COMMAND（2 个保持寄存器） */
		CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, 2, &cmd32 );
	}

}


