#ifndef __HGS_H
/* __HGS_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __HGS_H 
#include "main.h"

/* 旧版小写 oled_clear 调用的兼容封装；统一转发到 OLED_Clear，新代码应优先使用规范接口名。 */
#define oled_clear() OLED_Clear()
#define DIS_MAXROWBYTE 18 /* 屏幕一行最多能显示的汉字为9个,共占18个字节 */


/**
 * @brief 初始化屏幕显示中的 OLED_Init 逻辑。
 */
void OLED_Init(void);
/**
 * @brief 清空 OLED 显存并复位软件绘制缓存。
 */
void OLED_Clear(void);
/**
 * @brief 校验逻辑坐标后清空指定 OLED 矩形区域。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param width_cols 待清除区域的 OLED 8 像素列数量。
 * @param height_rows 高度。
 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width_cols, uint8_t height_rows);
/**
 * @brief 发送 SSD1322 显示开启命令。
 */
void OLED_DisplayOn(void);
/**
 * @brief 发送 SSD1322 显示关闭命令。
 */
void OLED_DisplayOff(void);
/**
 * @brief 设置 SSD1322 对比度寄存器。
 *
 * @param contrast 对比度。
 */
void OLED_SetContrast(uint8_t contrast);
/**
 * @brief 按亮度挡位更新 OLED 对比度并保存运行值。
 *
 * @param level 待判断或显示的级别值。该值是 OLED 亮度档位，函数先限制到支持范围，再映射为 SSD1305 对比度寄存器值。
 */
void OLED_SetBrightnessLevel(uint8_t level);
/**
 * @brief 复位 OLED 通信和显示状态后清屏恢复。
 */
void OLED_RecoverAndClear(void);
/**
 * @brief 记录一帧 OLED 刷新已完成并推进帧代际。
 */
void OLED_MarkFrameComplete(void);
/**
 * @brief 返回 OLED SPI 累计传输错误次数。
 * @return 返回本次上电以来累计的 OLED SPI 传输错误次数。
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
 * @brief 重新初始化 OLED 地址窗口，并用指定字节填充整块显存后开启显示。
 *
 * @param m 写入 OLED 整块显存的填充值；0x00 清空像素，其他值按位形成全屏填充图案。
 */
void all_screen(uint8_t m);
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
void write_hanzi16(uint8_t x,uint8_t y,const uint8_t *buf,uint8_t m,uint8_t endm,uint8_t select);
/**
 * @brief 写入一个8*16的字母。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param buf 包含待绘制 8×16 ASCII 字模数据的只读字节表。
 * @param coder 待绘制的字模数据表。
 * @param en 待绘制的 8×16 ASCII 字符编码。
 */
void write_816(uint8_t x,uint8_t y,const uint8_t *buf,uint8_t coder,uint8_t en);




#endif



