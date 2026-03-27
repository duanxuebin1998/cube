#include "hgs.h"
#include "stdlib.h"
#include "math.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
static int OLED_charwidth = 7;
static uint8_t data_4byte[4];

static void WriteCommand(uint8_t Cmd);
static void WriteSingleData(uint8_t Data);
static void write_4_byte(uint8_t DATA);
static void wirte_1616(uint8_t x, uint8_t y, uint8_t *buf, uint8_t coder, uint8_t select);

static void WriteCommand(u8 Cmd) //写指令函数
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //dc=0
//	HAL_Delay(1);//延时1ms
	HAL_SPI_Transmit(&hspi1, &Cmd, 1, HAL_MAX_DELAY); //发送指令
}
static void WriteSingleData(u8 Data) //写单个数据函数
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //dc=1
//	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &Data, 1, HAL_MAX_DELAY); //发送数据
}
/******************************************
 写字符最初级
 ******************************************/
static void write_4_byte(u8 DATA) {
	u8 k;
	u8 kk, kkk;
	kk = DATA;
	for (k = 0; k < 4; k++) {
		kkk = kk & 0xc0;     //?K=0? ?D7,D6? ?K=1? ?D5,D4?

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
		kk = kk << 2;                                //????
		WriteSingleData(data_4byte[k]); /* 8 column  a nibble of command is a dot*/
	}
}
/******************************************
 写入汉字最第二级
 ******************************************/
static void wirte_1616(u8 x, u8 y, u8 *buf, u8 coder, u8 select) {
	u8 i;
	static u8 wordlen;

	wordlen = OLED_charwidth * 4;
	WriteCommand(0x75); /* Set Row Address */
	WriteCommand(0x0c + y); /* Start = Row */
	WriteCommand(0x0c + y + 15); /* End = Row+16 */
	WriteCommand(0x15); /* Set Column Address */
	WriteCommand(x); /* Start = Column */
	WriteCommand(x + 7); /* End = Column+16 */
	if (!select) {
		for (i = 0; i < wordlen; i++) /* 2*8 column , a nibble of command is a dot*/
		{
			write_4_byte(buf[wordlen * coder + i]);
		}
	} else {
		for (i = 0; i < wordlen; i++) /* 2*8 column , a nibble of command is a dot*/
		{
			write_4_byte(~buf[wordlen * coder + i]);
		}
	}
}
/******************************************
 写入汉字最上级
 ******************************************/
void write_hanzi16(u8 x, u8 y, u8 *buf, u8 m, u8 endm, u8 select) {
	u8 i;

	for (i = m; i < endm; i++) {
		wirte_1616(x, y, buf, i, select);
		x = x + OLED_charwidth;            // 8*2=16间隔一个汉字
	}
}
/* 清屏 */
void all_screen(uint8_t m) {
	uint32_t j, i;
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_RESET); //复位引脚拉低
	HAL_Delay(10); //延时100ms
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_SET); //复位引脚拉高
	// 设置列窗口
	WriteCommand(0x15);
	WriteCommand(0x00); /* 左边界 0 */
	WriteCommand(0x3F); /* 右边界 127*/
	// 设置行窗口
	WriteCommand(0x75);
	WriteCommand(0x0C); /* 上边界 0*/
	WriteCommand(0x4b); /* 下边界 63*/
	// 设置 SEG 电流等级；
	WriteCommand(0x81);
	WriteCommand(0x20); /* 共 128 级 */
	// 设置 SEG 电流范围；
	WriteCommand(0x86);
	/* 84H=1/4，85H=1/2，86H=1 */
	// 设置逆转地图；
	WriteCommand(0xA0);
	WriteCommand(0x52);
	/* BIT-0=1 列窗口反向 ,BIT-1=1 高半字节在
	 前,BIT-2=1 COM 反向,BIT-6=0/1 EVEN/ODD */
	// 设置显示起始行;
	WriteCommand(0xA1);
	WriteCommand(0x0C); /* 0*/
	// 设置显示分支
	WriteCommand(0xA2);
	WriteCommand(0x4C); /* 0*/
	// 设置显示模式;
	WriteCommand(0xA4);
	/* A4H=正常显示,A5H=全显,A6H=关显示,A7H=反 显 */
	// 设置扫描行;
	WriteCommand(0xA8);
	WriteCommand(0x3F); /* 64*/
	// 设置 P1 P2 ;
	WriteCommand(0xB1);
	WriteCommand(0x04); /* 4 */
	WriteCommand(0x06 << 4); /* 6 */
	// 设置行周期;
	WriteCommand(0xB2);
	WriteCommand(0X46); /* 70*/
	// 设置 D 和 Fosc;
	WriteCommand(0xB3);
	WriteCommand(0x01);
	WriteCommand(0x04 << 4); /* Fosc=4 D=2 */
	// 设置 Vp2
	WriteCommand(0xBC);
	WriteCommand(0x00); /* 0.51 */
	// 设置 Vcomh
	WriteCommand(0xBE);
	WriteCommand(0x00); /* 0.51 */
	// 设置 Vsl
	WriteCommand(0xBF);
	WriteCommand(0x0E); /* 连接电容到 VSS */
	// 设置灰度;
	WriteCommand(0xB8);
	WriteCommand(0x07); /* L1[2:1] */
	WriteCommand(0x33);
	/* L3[6:4], L2[2:0] 0001 0001*/
	WriteCommand(0x33);
	/* L5[6:4], L4[2:0] 0010 0010*/
	WriteCommand(0x33);
	/* L7[6:4], L6[2:0] 0011 1011*/
	WriteCommand(0x33);
	/* L9[6:4], L8[2:0] 0100 0100*/
	WriteCommand(0x33);
	/* LB[6:4], LA[2:0] 0101 0101*/
	WriteCommand(0x33);
	/* LD[6:4], LC[2:0] 0110 0110*/
	WriteCommand(0x72);

	for (j = 0; j < 80; j++) /* 80 row */
	{
		for (i = 0; i < 64; i++) /* 64*2=128 column  a nibble of command is a dot*/
		{
			WriteSingleData(m);
		}
	}
	// 显示开关;
	WriteCommand(0xAF); /* AF=ON, AE=Sleep Mode */
	// 设置 Vcc 来源;
	WriteCommand(0xAD);
	WriteCommand(0x02); /* 03=内部 02=外部 */

}
///* 清屏 */
//void all_screen(uint8_t m)
//{
//    uint32_t j,i;
//
//    //Column Address
//    WriteCommand(0x15);     /* Set Column Address */
//    WriteCommand(0x00);     /* Start = 0 */
//    WriteCommand(0x3F);     /* End = 127 */
//    // Row Address
//    WriteCommand(0x75);     /* Set Row Address */
//    WriteCommand(0x00);     /* Start = 0 */
//    WriteCommand(0x50);     /* End = 80 */
//    for (j=0;j<80;j++)      /* 80 row */
//    {
//        for (i=0;i<64;i++)  /* 64*2=128 column  a nibble of command is a dot*/
//        {
//            WriteSingleData(m);
//        }
//    }
//
//}
/* 清屏 - 局部范围内 */
void CLS_scope(uint8_t row_start, uint8_t row_end, uint8_t column_start, uint8_t column_end) {
	uint32_t j, i;

	//Column Address
	WriteCommand(0x15); /* Set Column Address */
	WriteCommand(column_start); /* Start = 0 */
	WriteCommand(column_end); /* End = 127 */
	// Row Address
	WriteCommand(0x75); /* Set Row Address */
	WriteCommand(row_start); /* Start = 0 */
	WriteCommand(row_end); /* End = 80 */
	for (j = row_start; j < row_end; j++) /* 80 row */
	{
		for (i = column_start; i < column_end; i++) /* 64*2=128 column  a nibble of command is a dot*/
		{
			WriteSingleData(0);
		}
	}
}
/*********************************
 写入一个8*16的字母
 *********************************/
void write_816(u8 x, u8 y, u8 *buf, u8 coder, u8 en) {
	u8 j;
	static int charlen;

	charlen = OLED_charwidth * 2;
	for (j = 0; j < charlen; j++) {
		WriteCommand(0x75); /* Set Row Address */
		WriteCommand(0x0c + y + j); /* Start = Row */
		WriteCommand(0x0c + y + 15); /* End = Row+8 */
		WriteCommand(0x15); /* Set Column Address */
		WriteCommand(x); /* Start = Column */
		WriteCommand(x + 3); /* End = Column+8 */
		if (en)
			write_4_byte(~buf[charlen * coder + j]);
		else
			write_4_byte(buf[charlen * coder + j]);
	}
}

/******************************************
 初始化
 ******************************************/
void OLED_Init(void) {
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_RESET); //复位引脚拉低
	HAL_Delay(200); //延时100ms
	HAL_GPIO_WritePin(OLED_NREST_GPIO_Port, OLED_NREST_Pin, GPIO_PIN_SET); //复位引脚拉高
	// 设置列窗口
	WriteCommand(0x15);
	WriteCommand(0x00); /* 左边界 0 */
	WriteCommand(0x3F); /* 右边界 127*/
	// 设置行窗口
	WriteCommand(0x75);
	WriteCommand(0x0C); /* 上边界 0*/
	WriteCommand(0x4b); /* 下边界 63*/
	// 设置 SEG 电流等级；
	WriteCommand(0x81);
	WriteCommand(0x20); /* 共 128 级 */
	// 设置 SEG 电流范围；
	WriteCommand(0x86);
	/* 84H=1/4，85H=1/2，86H=1 */
	// 设置逆转地图；
	WriteCommand(0xA0);
	WriteCommand(0x52);
	/* BIT-0=1 列窗口反向 ,BIT-1=1 高半字节在
	 前,BIT-2=1 COM 反向,BIT-6=0/1 EVEN/ODD */
	// 设置显示起始行;
	WriteCommand(0xA1);
	WriteCommand(0x0C); /* 0*/
	// 设置显示分支
	WriteCommand(0xA2);
	WriteCommand(0x4C); /* 0*/
	// 设置显示模式;
	WriteCommand(0xA4);
	/* A4H=正常显示,A5H=全显,A6H=关显示,A7H=反 显 */
	// 设置扫描行;
	WriteCommand(0xA8);
	WriteCommand(0x3F); /* 64*/
	// 设置 P1 P2 ;
	WriteCommand(0xB1);
	WriteCommand(0x04); /* 4 */
	WriteCommand(0x06 << 4); /* 6 */
	// 设置行周期;
	WriteCommand(0xB2);
	WriteCommand(0X46); /* 70*/
	// 设置 D 和 Fosc;
	WriteCommand(0xB3);
	WriteCommand(0x01);
	WriteCommand(0x04 << 4); /* Fosc=4 D=2 */
	// 设置 Vp2
	WriteCommand(0xBC);
	WriteCommand(0x00); /* 0.51 */
	// 设置 Vcomh
	WriteCommand(0xBE);
	WriteCommand(0x00); /* 0.51 */
	// 设置 Vsl
	WriteCommand(0xBF);
	WriteCommand(0x0E); /* 连接电容到 VSS */
	// 设置灰度;
	WriteCommand(0xB8);
	WriteCommand(0x07); /* L1[2:1] */
	WriteCommand(0x33);
	/* L3[6:4], L2[2:0] 0001 0001*/
	WriteCommand(0x33);
	/* L5[6:4], L4[2:0] 0010 0010*/
	WriteCommand(0x33);
	/* L7[6:4], L6[2:0] 0011 1011*/
	WriteCommand(0x33);
	/* L9[6:4], L8[2:0] 0100 0100*/
	WriteCommand(0x33);
	/* LB[6:4], LA[2:0] 0101 0101*/
	WriteCommand(0x33);
	/* LD[6:4], LC[2:0] 0110 0110*/
	WriteCommand(0x72);
	/* LF[6:4], LE[2:0] 1000 0111*/
	all_screen(0x00);
	// 显示开关;
	WriteCommand(0xAF); /* AF=ON, AE=Sleep Mode */
	// 设置 Vcc 来源;
	WriteCommand(0xAD);
	WriteCommand(0x02); /* 03=内部 02=外部 */

}

/* 显示 128 * 64 单色位图 */
void oled_map_128_64(uint8_t *map) {
	uint16_t j, i;
	int x_start, x_stop;    //横向
	int y_start, y_stop;    //纵向

	int b_x = 128;
	int b_y = 64;

	x_start = 0;
	x_stop = b_x / 2;

	y_start = 0;
	y_stop = b_y;

	//设置列
	WriteCommand(0x75); /* 范围 0x0C ~ 0x4B */
	WriteCommand(0x0c + y_start); /* Start*/
	WriteCommand(0x0c + y_stop - 1); /* End */
	//设置行
	WriteCommand(0x15); /* 范围 0x00 ~ 0x3F */
	WriteCommand(0x00 + x_start); /* Start*/
	WriteCommand(0x00 + x_stop - 1); /* End*/

	for (j = 0; j < (y_stop - y_start); j++) {
		for (i = 0; i < 16; i++) /* 16*8 column  a nibble of command is a dot*/
		{
			write_4_byte(map[(j << 4) + i]);        // 取16个字节后换行
		}
	}
}

