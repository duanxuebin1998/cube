#include "hgs.h"
#include "stdlib.h"
#include "math.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "display.h"
static int OLED_charwidth = 7; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t data_4byte[4]; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_fill_buf[64]; /* 屏幕显示数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */

#define OLED_SPI_TIMEOUT_MS 20U
#define OLED_SHADOW_SIZE 5120U
#define OLED_CRC32_INIT 0xFFFFFFFFU
#define OLED_CRC32_POLY 0xEDB88320U
#define OLED_CONTRAST_MAX 0x7FU

static volatile uint32_t oled_spi_error_count = 0U; /* 屏幕显示计数值，用于节拍、统计或协议数量控制。 */
static uint8_t oled_shadow_buffer[OLED_SHADOW_SIZE]; /* 屏幕显示数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */
static uint32_t oled_shadow_crc = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint32_t oled_refresh_seq = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_configured_brightness = OLED_BRIGHTNESS_LEVEL_LOW; /* 屏幕显示参数缓存，写入前后需保持与系统参数结构一致。 */
static uint8_t oled_configured_contrast = 0x20U; /* 屏幕显示参数缓存，写入前后需保持与系统参数结构一致。 */
static uint8_t oled_window_col_start = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_window_col_end = 0x3FU; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_window_row_start = 0x0CU; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_window_row_end = 0x4BU; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_cursor_col = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static uint8_t oled_cursor_row = 0x0CU; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */

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
static HAL_StatusTypeDef wirte_1616(uint8_t x, uint8_t y, uint8_t *buf, uint8_t coder, uint8_t select);

/**
 * @brief 执行屏幕显示中的 OLED_RecordSpiStatus 逻辑。
 *
 * @param status 状态值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void OLED_RecordSpiStatus(HAL_StatusTypeDef status)
{
	if (status != HAL_OK) {
		oled_spi_error_count++;
	}
}

/**
 * @brief 写入或设置屏幕显示中的 OLED_WriteCommandSequence 逻辑。
 *
 * @param commands 命令值。
 * @param len 数据长度。
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
 * @brief 执行屏幕显示中的 OLED_NormalizeBrightnessLevel 逻辑。
 *
 * @param level 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t OLED_NormalizeBrightnessLevel(uint8_t level)
{
	if (level >= OLED_BRIGHTNESS_LEVEL_COUNT) {
		return OLED_BRIGHTNESS_LEVEL_LOW;
	}

	return level;
}

/**
 * @brief 执行屏幕显示中的 OLED_BrightnessLevelToContrast 逻辑。
 *
 * @param level 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 执行屏幕显示中的 OLED_ConfiguredContrast 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t OLED_ConfiguredContrast(void)
{
	return oled_configured_contrast;
}

/**
 * @brief 清除或复位屏幕显示中的 OLED_ResetPin 逻辑。
 *
 * @param delay_ms 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void OLED_ResetPin(uint32_t delay_ms)
{
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_RESET);
	HAL_Delay(delay_ms);
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_SET);
}

/**
 * @brief 写入或设置屏幕显示中的 OLED_WriteDisplayOnCommands 逻辑。
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
 * @brief 写入或设置屏幕显示中的 OLED_WriteInitCommands 逻辑。
 *
 * @param contrast 业务参数。
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
 * @brief 执行屏幕显示中的 OLED_GetSpiErrorCount 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t OLED_GetSpiErrorCount(void)
{
	return oled_spi_error_count;
}

/**
 * @brief 计算校验屏幕显示中的 OLED_GetShadowCrc 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t OLED_GetShadowCrc(void)
{
	return oled_shadow_crc;
}

/**
 * @brief 更新屏幕显示中的 OLED_GetRefreshSeq 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t OLED_GetRefreshSeq(void)
{
	return oled_refresh_seq;
}

/**
 * @brief 执行屏幕显示中的 OLED_MarkFrameComplete 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_MarkFrameComplete(void)
{
	oled_shadow_crc = OLED_CalcCrc32(oled_shadow_buffer, OLED_SHADOW_SIZE);
	oled_refresh_seq++;
}

/**
 * @brief 计算屏幕显示中的 OLED_CalcCrc32 逻辑。
 *
 * @param data 数据缓冲区。
 * @param len 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 执行屏幕显示中的 OLED_ShadowSetWindow 逻辑。
 *
 * @param col_start 业务参数。
 * @param col_end 业务参数。
 * @param row_start 业务参数。
 * @param row_end 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕显示中的 OLED_ShadowWriteByte 逻辑。
 *
 * @param data 数据缓冲区。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕显示中的 OLED_ShadowWriteBuffer 逻辑。
 *
 * @param data 数据缓冲区。
 * @param len 数据长度。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 写入或设置屏幕显示中的 WriteDataBuffer 逻辑。
 *
 * @param data 数据缓冲区。
 * @param len 数据长度。
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
 * @brief 执行屏幕显示中的 OLED_SetWindow 逻辑。
 *
 * @param col_start 业务参数。
 * @param col_end 业务参数。
 * @param row_start 业务参数。
 * @param row_end 业务参数。
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
 * @brief 执行屏幕显示中的 OLED_SetFullWindow 逻辑。
 * @return HAL 状态码，用于判断底层外设访问是否成功。
 */
static HAL_StatusTypeDef OLED_SetFullWindow(void)
{
	return OLED_SetWindow(0x00, 0x3F, 0x0C, 0x4B);
}

/**
 * @brief 写入或设置屏幕显示中的 OLED_WriteFillData 逻辑。
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
 * @brief 清除或复位屏幕显示中的 OLED_Clear 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_Clear(void)
{
	if (OLED_SetFullWindow() != HAL_OK) {
		return;
	}
	OLED_WriteFillData(0x00);
}

/**
 * @brief 清除或复位屏幕显示中的 OLED_ClearArea 逻辑。
 *
 * @param x 业务参数。
 * @param y 业务参数。
 * @param width_cols 业务参数。
 * @param height_rows 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 显示或打印屏幕显示中的 OLED_DisplayOn 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_DisplayOn(void)
{
	WriteCommand(0xAF);
}

/**
 * @brief 显示或打印屏幕显示中的 OLED_DisplayOff 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_DisplayOff(void)
{
	WriteCommand(0xAE);
}

/**
 * @brief 写入或设置屏幕显示中的 OLED_WriteContrastCommand 逻辑。
 *
 * @param contrast 业务参数。
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
 * @brief 执行屏幕显示中的 OLED_SetContrast 逻辑。
 *
 * @param contrast 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕显示中的 OLED_SetBrightnessLevel 逻辑。
 *
 * @param level 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_SetBrightnessLevel(uint8_t level)
{
	oled_configured_brightness = OLED_NormalizeBrightnessLevel(level);
	OLED_SetContrast(OLED_BrightnessLevelToContrast(oled_configured_brightness));
}

/**
 * @brief 清除或复位屏幕显示中的 OLED_RecoverAndClear 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void OLED_RecoverAndClear(void)
{
	all_screen(0x00);
}
/******************************************
 写字符最初级
 ******************************************/
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
/******************************************
 写入汉字最第二级
 ******************************************/
static HAL_StatusTypeDef wirte_1616(u8 x, u8 y, u8 *buf, u8 coder, u8 select) {
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
/******************************************
 写入汉字最上级
 ******************************************/
void write_hanzi16(u8 x, u8 y, u8 *buf, u8 m, u8 endm, u8 select) {
	u8 i;

	for (i = m; i < endm; i++) {
		if (wirte_1616(x, y, buf, i, select) != HAL_OK) {
			return;
		}
		x = x + OLED_charwidth;            /* 8*2=16间隔一个汉字 */
	}
}
/* 清屏 */
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
/*********************************
 写入一个8*16的字母
 *********************************/
void write_816(u8 x, u8 y, u8 *buf, u8 coder, u8 en) {
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

/******************************************
 初始化
 ******************************************/
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
