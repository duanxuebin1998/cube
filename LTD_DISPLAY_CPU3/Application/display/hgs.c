#include "hgs.h"
#include "stdlib.h"
#include "math.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "display.h"
static int OLED_charwidth = 7; /* OLED 字模的逻辑列宽，当前为 7；16×16 汉字按其四倍字节数取模，8×16 ASCII 按其两倍字节数取模。 */
static uint8_t data_4byte[4]; /* 单个 8 位二值字模展开为四个 4 灰度 OLED 数据字节的临时缓冲；write_4_byte 每次转换后立即顺序发送。 */
static uint8_t oled_fill_buf[64]; /* OLED 批量填充和分块清零共用的 64 字节发送缓冲；调用流程先写满目标字节，再按块同步发送并更新影子显存。 */

/* OLED 单次 SPI 传输等待超时 20 ms；超时后返回错误并交由恢复流程处理。 */
#define OLED_SPI_TIMEOUT_MS 20U
/* OLED 软件影子缓冲区容量 5120 字节；覆盖控制器完整显示 RAM，用于 CRC 检查和局部更新。 */
#define OLED_SHADOW_SIZE 5120U
/* OLED 影子数据 CRC32 的初始余数 0xFFFFFFFF。 */
#define OLED_CRC32_INIT 0xFFFFFFFFU
/* OLED 影子数据 CRC32 使用的反射多项式 0xEDB88320。 */
#define OLED_CRC32_POLY 0xEDB88320U
/* OLED 控制器允许写入的最大对比度值 0x7F；亮度档位换算结果必须钳位到该上限。 */
#define OLED_CONTRAST_MAX 0x7FU

static volatile uint32_t oled_spi_error_count = 0U; /* 本次上电以来 OLED SPI HAL 操作失败的累计次数；底层每次非 HAL_OK 结果递增，显示恢复逻辑通过计数变化识别新故障。 */
static uint8_t oled_shadow_buffer[OLED_SHADOW_SIZE]; /* 覆盖 SSD1322 完整 5120 字节显示 RAM 的软件影子显存；仅在 SPI 数据发送成功后同步更新，用于帧 CRC 和一致性诊断。 */
static uint32_t oled_shadow_crc = 0U; /* 最近一帧无新增 SPI 错误且完成提交时计算的完整 OLED 影子显存 CRC32，供显示一致性诊断读取。 */
static uint32_t oled_refresh_seq = 0U; /* 无新增 SPI 错误并完成提交的 OLED 逻辑帧累计序号；每次 OLED_MarkFrameComplete 成功记录一帧时递增。 */
static uint8_t oled_configured_brightness = OLED_BRIGHTNESS_LEVEL_LOW; /* 当前归一化后的 OLED 亮度档位；取值限制在 OLED_BRIGHTNESS_LEVEL_COUNT 内，并用于重新计算控制器对比度。 */
static uint8_t oled_configured_contrast = 0x20U; /* 最近一次应用到 SSD1322 的对比度寄存器值；OLED 重新初始化和恢复时复用该值，避免恢复后回到默认亮度。 */
static uint8_t oled_window_col_start = 0U; /* OLED 影子显存当前写窗口的起始列地址，写光标到达行尾后回绕到该列。 */
static uint8_t oled_window_col_end = 0x3FU; /* OLED 影子显存当前写窗口的结束列地址，写入该列后切换到下一行或窗口起始位置。 */
static uint8_t oled_window_row_start = 0x0CU; /* OLED 影子显存当前写窗口的起始行地址；SSD1322 可见区从 0x0C 开始，窗口回绕时恢复到该行。 */
static uint8_t oled_window_row_end = 0x4BU; /* OLED 影子显存当前写窗口的结束行地址，写光标到达该行末尾后回绕到窗口起点。 */
static uint8_t oled_cursor_col = 0U; /* 下一字节在 OLED 影子显存中的目标列地址；每次成功同步数据后按当前窗口自动推进。 */
static uint8_t oled_cursor_row = 0x0CU; /* 下一字节在 OLED 影子显存中的目标行地址；写入前限制在 SSD1322 可见行 0x0C～0x4B 范围内。 */

static HAL_StatusTypeDef WriteCommand(uint8_t Cmd);
static HAL_StatusTypeDef WriteSingleData(uint8_t Data);
static HAL_StatusTypeDef WriteDataBuffer(uint8_t *data, uint16_t len);
static void OLED_RecordSpiStatus(HAL_StatusTypeDef status);
static HAL_StatusTypeDef OLED_WriteCommandSequence(const uint8_t *commands, uint16_t len);
static HAL_StatusTypeDef OLED_WriteDisplayOnCommands(void);
static HAL_StatusTypeDef OLED_WriteInitCommands(uint8_t contrast);
static HAL_StatusTypeDef OLED_WriteContrastCommand(uint8_t contrast);
static void OLED_ResetPin(uint32_t delay_ms);
static uint8_t OLED_NormalizeBrightnessLevel(uint8_t level);
static uint8_t OLED_ConfiguredContrast(void);
static uint32_t OLED_CalcCrc32(const uint8_t *data, uint32_t len);
static void OLED_ShadowSetWindow(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end);
static void OLED_ShadowWriteByte(uint8_t data);
static void OLED_ShadowWriteBuffer(const uint8_t *data, uint16_t len);
static HAL_StatusTypeDef OLED_SetFullWindow(void);
static HAL_StatusTypeDef OLED_SetWindow(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end);
static HAL_StatusTypeDef OLED_WriteFillData(uint8_t data);
static HAL_StatusTypeDef write_4_byte(uint8_t DATA);
static HAL_StatusTypeDef wirte_1616(uint8_t x, uint8_t y, const uint8_t *buf, uint8_t coder, uint8_t select);

/**
 * @brief 记录 OLED SPI 操作结果并累计错误次数。
 *
 * @param status 本次 OLED SPI HAL 操作的返回状态。
 */
static void OLED_RecordSpiStatus(HAL_StatusTypeDef status)
{
	if (status != HAL_OK) {
		oled_spi_error_count++;
	}
}

/**
 * @brief 按顺序向 SSD1322 发送一组初始化或控制命令。
 *
 * @param commands 指向 len 个 SSD1322 初始化或控制命令字节的只读数组；函数按原顺序逐字节切换命令通道并发送。
 * @param len 数据长度。该值是本轮 OLED 命令、影子缓冲或 SPI 数据块中的有效字节数，函数只处理前 len 个字节。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_WriteCommandSequence(const uint8_t *commands, uint16_t len)
{
	HAL_StatusTypeDef status;
	uint16_t i;

	for (i = 0; i < len; i++) {
		status = WriteCommand(commands[i]);
		if (status != HAL_OK) {
			return status;
		}
	}

	return HAL_OK;
}

/**
 * @brief 把亮度挡位限制到 OLED 支持的有效范围。
 *
 * @param level 待归一化的 OLED 亮度挡位；有效范围为 OLED_BRIGHTNESS_LEVEL_LOW 至 OLED_BRIGHTNESS_LEVEL_HIGH。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint8_t OLED_NormalizeBrightnessLevel(uint8_t level)
{
	if (level >= OLED_BRIGHTNESS_LEVEL_COUNT) {
		return OLED_BRIGHTNESS_LEVEL_LOW;
	}

	return level;
}

/**
 * @brief 把亮度挡位转换为 SSD1322 对比度寄存器值。
 *
 * @param level OLED 亮度挡位；函数先钳位到有效范围，再查表得到 SSD1322 对比度寄存器值。
 * @return 返回亮度挡位归一化后在对比度表中对应的 SSD1322 寄存器值。
 */
static uint8_t OLED_BrightnessLevelToContrast(uint8_t level)
{
	static const uint8_t contrast_map[OLED_BRIGHTNESS_LEVEL_COUNT] = {
		0x10U,
		0x20U,
		0x30U,
		0x40U,
		0x50U,
	};

	return contrast_map[OLED_NormalizeBrightnessLevel(level)];
}

/**
 * @brief 读取当前配置对应的 OLED 对比度值。
 * @return 返回最近一次由屏幕亮度配置计算并保存的 SSD1322 对比度寄存器值。
 */
static uint8_t OLED_ConfiguredContrast(void)
{
	return oled_configured_contrast;
}

/**
 * @brief 按指定电平控制 OLED 硬件复位引脚。
 *
 * @param delay_ms 本次硬件时序或流程等待使用的延时时长，单位 ms。
 */
static void OLED_ResetPin(uint32_t delay_ms)
{
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_RESET);
	HAL_Delay(delay_ms);
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_SET);
}

/**
 * @brief 发送 SSD1322 显示开启所需的完整命令序列。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_WriteDisplayOnCommands(void)
{
	static const uint8_t display_on_commands[] = {
		0xAF,
		0xAD, 0x02,
	};

	return OLED_WriteCommandSequence(display_on_commands, (uint16_t)sizeof(display_on_commands));
}

/**
 * @brief 按当前亮度配置发送 OLED 初始化指令序列。
 *
 * @param contrast 对比度。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_WriteInitCommands(uint8_t contrast)
{
	static const uint8_t init_prefix_commands[] = {
		0x86,
		0xA0, 0x52,
		0xA1, 0x0C,
		0xA2, 0x4C,
		0xA4,
		0xA8, 0x3F,
		0xB1, 0x04, (uint8_t)(0x06 << 4),
		0xB2, 0x46,
		0xB3, 0x01, (uint8_t)(0x04 << 4),
		0xBC, 0x00,
		0xBE, 0x00,
		0xBF, 0x0E,
		0xB8, 0x07, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x72,
	};
	HAL_StatusTypeDef status;

	status = OLED_SetFullWindow();
	if (status != HAL_OK) {
		return status;
	}
	status = OLED_WriteContrastCommand(contrast);
	if (status != HAL_OK) {
		return status;
	}
	return OLED_WriteCommandSequence(init_prefix_commands, (uint16_t)sizeof(init_prefix_commands));
}

/**
 * @brief 返回 OLED SPI 累计传输错误次数。
 * @return 返回本次上电以来累计的 OLED SPI 传输错误次数。
 */
uint32_t OLED_GetSpiErrorCount(void)
{
	return oled_spi_error_count;
}

/**
 * @brief 获取最近一次完整刷新后的 OLED 影子缓冲区 CRC。
 * @return 影子缓冲区 CRC32，用于软件一致性检查。
 */
uint32_t OLED_GetShadowCrc(void)
{
	return oled_shadow_crc;
}

/**
 * @brief 获取无 SPI 错误的完整刷新序号。
 * @return 完整刷新累计次数，用于刷新活性检查。
 */
uint32_t OLED_GetRefreshSeq(void)
{
	return oled_refresh_seq;
}

/**
 * @brief 记录一帧 OLED 刷新已完成并推进帧代际。
 */
void OLED_MarkFrameComplete(void)
{
	oled_shadow_crc = OLED_CalcCrc32(oled_shadow_buffer, OLED_SHADOW_SIZE);
	oled_refresh_seq++;
}

/**
 * @brief 计算 OLED 影子缓冲区的 CRC32。
 *
 * @param data 待校验的 OLED 影子缓冲区首地址；len 大于 0 时必须至少包含 len 个可读字节。
 * @param len 参与 CRC32 计算的连续字节数；允许为 0。
 * @return 返回 data[0..len-1] 以 OLED_CRC32_INIT 为初值、按 OLED_CRC32_POLY 逐位更新并最终按位取反得到的 CRC32。
 */
static uint32_t OLED_CalcCrc32(const uint8_t *data, uint32_t len)
{
	uint32_t crc = OLED_CRC32_INIT;
	uint32_t i;
	uint8_t bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8U; bit++) {
			if ((crc & 1U) != 0U) {
				crc = (crc >> 1U) ^ OLED_CRC32_POLY;
			} else {
				crc >>= 1U;
			}
		}
	}

	return ~crc;
}

/**
 * @brief 在阴影状态中记录 OLED 当前写入窗口。
 *
 * @param col_start 列起始位置。
 * @param col_end 列结束位置。
 * @param row_start 起始位置。
 * @param row_end 结束位置。
 */
static void OLED_ShadowSetWindow(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end)
{
	oled_window_col_start = col_start;
	oled_window_col_end = col_end;
	oled_window_row_start = row_start;
	oled_window_row_end = row_end;
	oled_cursor_col = col_start;
	oled_cursor_row = row_start;
}

/**
 * @brief 按当前窗口坐标写入影子缓冲，并推进写指针。
 *
 * @param data 数据缓冲区。
 */
static void OLED_ShadowWriteByte(uint8_t data)
{
	uint32_t row_index;
	uint32_t index;

	if ((oled_cursor_row < 0x0CU) || (oled_cursor_row > 0x4BU) || (oled_cursor_col > 0x3FU)) {
		return;
	}

	row_index = (uint32_t)(oled_cursor_row - 0x0CU);
	index = (row_index * 64U) + oled_cursor_col;
	if (index < OLED_SHADOW_SIZE) {
		oled_shadow_buffer[index] = data;
	}

	if (oled_cursor_col >= oled_window_col_end) {
		oled_cursor_col = oled_window_col_start;
		if (oled_cursor_row >= oled_window_row_end) {
			oled_cursor_row = oled_window_row_start;
		} else {
			oled_cursor_row++;
		}
	} else {
		oled_cursor_col++;
	}
}

/**
 * @brief 将待发送数据同步写入 OLED 阴影缓冲。
 *
 * @param data 待同步到 OLED 影子显存的 len 个源字节；函数从当前影子写光标开始顺序写入，并按已设置窗口自动换列和换行。
 * @param len 数据长度。该值是本轮 OLED 命令、影子缓冲或 SPI 数据块中的有效字节数，函数只处理前 len 个字节。
 */
static void OLED_ShadowWriteBuffer(const uint8_t *data, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++) {
		OLED_ShadowWriteByte(data[i]);
	}
}

/**
 * @brief 通过 SPI 向 OLED 写入单字节命令。
 * @param Cmd OLED 控制命令。
 * @return HAL 状态码，用于判断 SPI 发送是否成功。
 */
static HAL_StatusTypeDef WriteCommand(u8 Cmd) /* 写指令函数 */
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); /* dc=0 */
/* HAL_Delay(1);/ /延时1ms */
	status = HAL_SPI_Transmit(&hspi1, &Cmd, 1, OLED_SPI_TIMEOUT_MS); /* 发送指令 */
	OLED_RecordSpiStatus(status);
	return status;
}
/**
 * @brief 通过 SPI 向 OLED 写入单字节显示数据并同步影子缓存。
 * @param Data 显示数据字节。
 * @return HAL 状态码，用于判断 SPI 发送是否成功。
 */
static HAL_StatusTypeDef WriteSingleData(u8 Data) /* 写单个数据函数 */
{
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); /* dc=1 */
/* HAL_Delay(1); */
	status = HAL_SPI_Transmit(&hspi1, &Data, 1, OLED_SPI_TIMEOUT_MS); /* 发送数据 */
	OLED_RecordSpiStatus(status);
	if (status == HAL_OK) {
		OLED_ShadowWriteByte(Data);
	}
	return status;
}

/**
 * @brief 通过 SPI 向 OLED 连续写入显示数据缓冲区。
 *
 * @param data 待通过 SPI 连续发送的 len 个可读显示数据字节；只有 HAL 发送成功后，同一数据才写入 OLED 影子显存。
 * @param len 数据长度。该值是本轮 OLED 命令、影子缓冲或 SPI 数据块中的有效字节数，函数只处理前 len 个字节。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef WriteDataBuffer(uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); /* dc=1 */
	status = HAL_SPI_Transmit(&hspi1, data, len, OLED_SPI_TIMEOUT_MS);
	OLED_RecordSpiStatus(status);
	if (status == HAL_OK) {
		OLED_ShadowWriteBuffer(data, len);
	}
	return status;
}

/**
 * @brief 设置 OLED 列、行写入窗口，并同步影子窗口边界。
 *
 * @param col_start 列起始位置。
 * @param col_end 列结束位置。
 * @param row_start 起始位置。
 * @param row_end 结束位置。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_SetWindow(uint8_t col_start, uint8_t col_end, uint8_t row_start, uint8_t row_end)
{
	HAL_StatusTypeDef status;

	status = WriteCommand(0x15);
	if (status != HAL_OK) {
		return status;
	}
	status = WriteCommand(col_start);
	if (status != HAL_OK) {
		return status;
	}
	status = WriteCommand(col_end);
	if (status != HAL_OK) {
		return status;
	}
	status = WriteCommand(0x75);
	if (status != HAL_OK) {
		return status;
	}
	status = WriteCommand(row_start);
	if (status != HAL_OK) {
		return status;
	}
	status = WriteCommand(row_end);
	if (status != HAL_OK) {
		return status;
	}

	OLED_ShadowSetWindow(col_start, col_end, row_start, row_end);
	return HAL_OK;
}

/**
 * @brief 把 SSD1322 显存写窗口设置为全屏范围。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_SetFullWindow(void)
{
	return OLED_SetWindow(0x00, 0x3F, 0x0C, 0x4B);
}

/**
 * @brief 向当前 OLED 显存窗口写入重复填充值。
 *
 * @param data 数据缓冲区。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_WriteFillData(uint8_t data)
{
	HAL_StatusTypeDef status;
	uint32_t j;
	uint32_t i;

	for (i = 0; i < sizeof(oled_fill_buf); i++) {
		oled_fill_buf[i] = data;
	}

	for (j = 0; j < 80; j++) {
		status = WriteDataBuffer(oled_fill_buf, (uint16_t)sizeof(oled_fill_buf));
		if (status != HAL_OK) {
			return status;
		}
	}

	return HAL_OK;
}

/**
 * @brief 清空 OLED 显存并复位软件绘制缓存。
 */
void OLED_Clear(void)
{
	if (OLED_SetFullWindow() != HAL_OK) {
		return;
	}
	OLED_WriteFillData(0x00);
}

/**
 * @brief 校验逻辑坐标后清空指定 OLED 矩形区域。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param width_cols 待清除区域的 OLED 8 像素列数量。
 * @param height_rows 高度。
 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width_cols, uint8_t height_rows)
{
	uint32_t total_bytes;
	uint32_t offset;
	uint16_t col_end;
	uint16_t row_end;
	uint16_t chunk_len;

	if ((width_cols == 0U) || (height_rows == 0U) || (x > 0x3FU) || (y > 0x3FU)) {
		return;
	}

	col_end = (uint16_t)x + (uint16_t)width_cols - 1U;
	row_end = (uint16_t)y + (uint16_t)height_rows - 1U;
	if (col_end > 0x3FU) {
		width_cols = (uint8_t)(0x40U - x);
		col_end = 0x3FU;
	}
	if (row_end > 0x3FU) {
		height_rows = (uint8_t)(0x40U - y);
		row_end = 0x3FU;
	}

	if (OLED_SetWindow(x, (uint8_t)col_end, (uint8_t)(0x0CU + y), (uint8_t)(0x0CU + row_end)) != HAL_OK) {
		return;
	}

	for (offset = 0U; offset < sizeof(oled_fill_buf); offset++) {
		oled_fill_buf[offset] = 0x00U;
	}

	total_bytes = (uint32_t)width_cols * (uint32_t)height_rows;
	offset = 0U;
	while (offset < total_bytes) {
		chunk_len = (uint16_t)(total_bytes - offset);
		if (chunk_len > sizeof(oled_fill_buf)) {
			chunk_len = (uint16_t)sizeof(oled_fill_buf);
		}
		if (WriteDataBuffer(oled_fill_buf, chunk_len) != HAL_OK) {
			return;
		}
		offset += chunk_len;
	}
}

/**
 * @brief 发送 SSD1322 显示开启命令。
 */
void OLED_DisplayOn(void)
{
	WriteCommand(0xAF);
}

/**
 * @brief 发送 SSD1322 显示关闭命令。
 */
void OLED_DisplayOff(void)
{
	WriteCommand(0xAE);
}

/**
 * @brief 向 SSD1322 写入对比度命令和参数。
 *
 * @param contrast 对比度。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_WriteContrastCommand(uint8_t contrast)
{
	HAL_StatusTypeDef status;

	if (contrast > OLED_CONTRAST_MAX) {
		contrast = OLED_CONTRAST_MAX;
	}

	status = WriteCommand(0x81);
	if (status != HAL_OK) {
		return status;
	}
	return WriteCommand(contrast);
}

/**
 * @brief 设置 SSD1322 对比度寄存器。
 *
 * @param contrast 对比度。
 */
void OLED_SetContrast(uint8_t contrast)
{
	if (contrast > OLED_CONTRAST_MAX) {
		contrast = OLED_CONTRAST_MAX;
	}

	oled_configured_contrast = contrast;
	(void)OLED_WriteContrastCommand(contrast);
}

/**
 * @brief 按亮度挡位更新 OLED 对比度并保存运行值。
 *
 * @param level 准备应用的 OLED 亮度挡位；越界值先钳位，随后写入 SSD1322 对比度寄存器并更新运行时亮度缓存。
 */
void OLED_SetBrightnessLevel(uint8_t level)
{
	oled_configured_brightness = OLED_NormalizeBrightnessLevel(level);
	OLED_SetContrast(OLED_BrightnessLevelToContrast(oled_configured_brightness));
}

/**
 * @brief 复位 OLED 通信和显示状态后清屏恢复。
 */
void OLED_RecoverAndClear(void)
{
	all_screen(0x00);
}
/**
 * @brief 写字符最初级。
 *
 * @param DATA 待通过 OLED SPI 接口连续写出的 32 位数据。
 * @return HAL_OK 表示HAL 操作成功。
 */
static HAL_StatusTypeDef write_4_byte(u8 DATA) {
	HAL_StatusTypeDef status;
	u8 k;
	u8 kk, kkk;
	kk = DATA;
	for (k = 0; k < 4; k++) {
		kkk = kk & 0xc0;     /* ?K=0? ?D7,D6? ?K=1? ?D5,D4? */

		switch (kkk) {
		case 0x00:
			data_4byte[k] = 0x00;
			break;
		case 0x40:
			data_4byte[k] = 0x0f;
			break;
		case 0x80:
			data_4byte[k] = 0xf0;
			break;
		case 0xc0:
			data_4byte[k] = 0xff;
			break;
		default:
			break;
		}
		kk = kk << 2;                                /* 左移两位，处理下一组点阵 */
		status = WriteSingleData(data_4byte[k]); /* 8 column a nibble of command is a dot */
		if (status != HAL_OK) {
			return status;
		}
	}
	return HAL_OK;
}
/**
 * @brief 写入汉字最第二级。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param buf 包含连续 16×16 汉字字模数据的只读字节表。
 * @param coder 待绘制的字模数据表。
 * @param select 字模绘制选择标志，用于控制正常或反显模式。
 * @return HAL_OK 表示HAL 操作成功。
 */
static HAL_StatusTypeDef wirte_1616(u8 x, u8 y, const u8 *buf, u8 coder, u8 select) {
	HAL_StatusTypeDef status;
	u8 i;
	static u8 wordlen;

	wordlen = OLED_charwidth * 4;
	status = OLED_SetWindow(x, x + 7, 0x0c + y, 0x0c + y + 15);
	if (status != HAL_OK) {
		return status;
	}
	if (!select) {
		for (i = 0; i < wordlen; i++) /* 2*8 column , a nibble of command is a dot */
		{
			status = write_4_byte(buf[wordlen * coder + i]);
			if (status != HAL_OK) {
				return status;
			}
		}
	} else {
		for (i = 0; i < wordlen; i++) /* 2*8 column , a nibble of command is a dot */
		{
			status = write_4_byte(~buf[wordlen * coder + i]);
			if (status != HAL_OK) {
				return status;
			}
		}
	}
	return HAL_OK;
}
/**
 * @brief 写入汉字最上级。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param buf 包含待连续绘制汉字字模数据的只读字节表。
 * @param m 待绘制 16×16 汉字字模的起始索引。
 * @param endm 字模数组的结束索引，配合起始索引限定连续绘制范围。
 * @param select 字模绘制选择标志，用于控制正常或反显模式。
 */
void write_hanzi16(u8 x, u8 y, const u8 *buf, u8 m, u8 endm, u8 select) {
	u8 i;

	for (i = m; i < endm; i++) {
		if (wirte_1616(x, y, buf, i, select) != HAL_OK) {
			return;
		}
		x = x + OLED_charwidth;            /* 8*2=16间隔一个汉字 */
	}
}
/**
 * @brief 重新初始化 OLED 地址窗口，并用指定字节填充整块显存后开启显示。
 *
 * @param m 写入 OLED 整块显存的填充值；0x00 清空像素，其他值按位形成全屏填充图案。
 */
void all_screen(uint8_t m) {
	OLED_ResetPin(10U);
	if (OLED_WriteInitCommands(OLED_ConfiguredContrast()) != HAL_OK) {
		return;
	}
	if (OLED_WriteFillData(m) != HAL_OK) {
		return;
	}
	(void)OLED_WriteDisplayOnCommands();

}
/* / * 清屏 * / */
/* void all_screen(uint8_t m) */
/* { */
/* uint32_t j,i; */
/* */
/* / /Column Address */
/* WriteCommand(0x15); / * Set Column Address * / */
/* WriteCommand(0x00); / * Start = 0 * / */
/* WriteCommand(0x3F); / * End = 127 * / */
/* / / Row Address */
/* WriteCommand(0x75); / * Set Row Address * / */
/* WriteCommand(0x00); / * Start = 0 * / */
/* WriteCommand(0x50); / * End = 80 * / */
/* for (j=0;j<80;j++) / * 80 row * / */
/* { */
/* for (i=0;i<64;i++) / * 64*2=128 column a nibble of command is a dot* / */
/* { */
/* WriteSingleData(m); */
/* } */
/* } */
/* */
/* } */
/**
 * @brief 写入一个8*16的字母。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param buf 包含待绘制 8×16 ASCII 字模数据的只读字节表。
 * @param coder 待绘制的字模数据表。
 * @param en 待绘制的 8×16 ASCII 字符编码。
 */
void write_816(u8 x, u8 y, const u8 *buf, u8 coder, u8 en) {
	u8 j;
	static int charlen;
	HAL_StatusTypeDef status;

	charlen = OLED_charwidth * 2;
	for (j = 0; j < charlen; j++) {
		status = OLED_SetWindow(x, x + 3, 0x0c + y + j, 0x0c + y + 15);
		if (status != HAL_OK) {
			return;
		}
		if (en) {
			status = write_4_byte(~buf[charlen * coder + j]);
		} else {
			status = write_4_byte(buf[charlen * coder + j]);
		}
		if (status != HAL_OK) {
			return;
		}
	}
}

/**
 * @brief 硬件复位 OLED，写入控制器初始化命令，清空显存并开启显示。
 */
void OLED_Init(void) {
	OLED_ResetPin(200U);
	if (OLED_WriteInitCommands(OLED_ConfiguredContrast()) != HAL_OK) {
		return;
	}
	if (OLED_WriteFillData(0x00) != HAL_OK) {
		return;
	}
	(void)OLED_WriteDisplayOnCommands();

}
