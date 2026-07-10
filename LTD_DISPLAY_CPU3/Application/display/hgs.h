#ifndef __HGS_H
#define __HGS_H 
#include "main.h"

#define oled_clear() OLED_Clear()
#define DIS_MAXROWBYTE 18 /* 屏幕一行最多能显示的汉字为9个,共占18个字节 */


/**
 * @brief 初始化屏幕显示中的 OLED_Init 逻辑。
 */
void OLED_Init(void);
/**
 * @brief 清除或复位屏幕显示中的 OLED_Clear 逻辑。
 */
void OLED_Clear(void);
/**
 * @brief 清除或复位屏幕显示中的 OLED_ClearArea 逻辑。
 *
 * @param x 业务参数。
 * @param y 业务参数。
 * @param width_cols 业务参数。
 * @param height_rows 业务参数。
 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width_cols, uint8_t height_rows);
/**
 * @brief 显示或打印屏幕显示中的 OLED_DisplayOn 逻辑。
 */
void OLED_DisplayOn(void);
/**
 * @brief 显示或打印屏幕显示中的 OLED_DisplayOff 逻辑。
 */
void OLED_DisplayOff(void);
/**
 * @brief 执行屏幕显示中的 OLED_SetContrast 逻辑。
 *
 * @param contrast 业务参数。
 */
void OLED_SetContrast(uint8_t contrast);
/**
 * @brief 执行屏幕显示中的 OLED_SetBrightnessLevel 逻辑。
 *
 * @param level 业务参数。
 */
void OLED_SetBrightnessLevel(uint8_t level);
/**
 * @brief 清除或复位屏幕显示中的 OLED_RecoverAndClear 逻辑。
 */
void OLED_RecoverAndClear(void);
/**
 * @brief 执行屏幕显示中的 OLED_MarkFrameComplete 逻辑。
 */
void OLED_MarkFrameComplete(void);
/**
 * @brief 执行屏幕显示中的 OLED_GetSpiErrorCount 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t OLED_GetSpiErrorCount(void);
/**
 * @brief 获取最近一次完整刷新后的 OLED 影子缓冲区 CRC。
 * @return 影子缓冲区 CRC32，用于软件一致性检查。
 */
uint32_t OLED_GetShadowCrc(void);
/**
 * @brief 获取无 SPI 错误的完整刷新序号。
 * @return 完整刷新累计次数，用于刷新活性检查。
 */
uint32_t OLED_GetRefreshSeq(void);
/**
 * @brief 执行屏幕显示中的 all_screen 逻辑。
 *
 * @param m 业务参数。
 */
void all_screen(uint8_t m);
/**
 * @brief 写入或设置屏幕显示中的 write_hanzi16 逻辑。
 *
 * @param x 业务参数。
 * @param y 业务参数。
 * @param buf 数据缓冲区。
 * @param m 业务参数。
 * @param endm 业务参数。
 * @param select 业务参数。
 */
void write_hanzi16(uint8_t x,uint8_t y,uint8_t *buf,uint8_t m,uint8_t endm,uint8_t select);
/**
 * @brief 写入或设置屏幕显示中的 write_816 逻辑。
 *
 * @param x 业务参数。
 * @param y 业务参数。
 * @param buf 数据缓冲区。
 * @param coder 业务参数。
 * @param en 业务参数。
 */
void write_816(uint8_t x,uint8_t y, uint8_t *buf,uint8_t coder,uint8_t en);




#endif



