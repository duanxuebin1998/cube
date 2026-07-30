#ifndef sil__MB85RS16_H
/* sil__MB85RS16_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define sil__MB85RS16_H
#include "main.h"

/* CPU3 FRAM 片选信号使用的 GPIO 引脚 PE11。 */
#define FRAM_CS_Pin GPIO_PIN_11
/* CPU3 FRAM 片选信号所在 GPIO 端口 GPIOE。 */
#define FRAM_CS_GPIO_Port GPIOE
/* CPU3 FRAM 驱动使用的 SPI4 HAL 句柄别名；所有读写必须遵守该总线的片选和互斥时序。 */
#define FRAM_SPI hspi4

#define MB_WRITEENABLE 0x06  /* Write enable */
#define MB_WRITEDATA  0x02 /* Write data */
#define MB_READDATA  0x03  /* Read data */

extern SPI_HandleTypeDef FRAM_SPI;

#define FRAM_ANGLE_ADDRESS 0x1000  /* 定义 FRAM 存储角度值的地址 */
#define FRAM_ENCODER_ADDRESS 0x1004  /* 定义 FRAM 存储角度值的地址 */

/**
 * @brief 从 24 位 FRAM 地址连续阻塞写入数据；当前接口不校验范围和 HAL 返回值。
 *
 * @param p_array 待连续写入 FRAM 的源字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length); /* 写入多字节数据到 FRAM */
/**
 * @brief 从 24 位 FRAM 地址连续阻塞读取数据；当前接口不校验范围和 HAL 返回值。
 *
 * @param p_array 用于接收 FRAM 连续读取结果的目标字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length); /* 从 FRAM 读取多字节数据 */

/**
 * @brief 从 FRAM 读取数据。
 *
 * @param address CPU3 参数 FRAM 的绝对字节地址；多字节读写从该地址开始并按连续地址递增。
 * @return 返回从指定 FRAM 绝对地址按大端顺序组合的 32 位原始值。
 */
uint32_t ReadSingleData(uint32_t address); /* 从 FRAM 读取单个数据 */
/**
 * @brief 写入或设置参数存储中的 WriteSingleData 逻辑。
 *
 * @param data 数据缓冲区。
 * @param address 地址参数。
 */
void WriteSingleData(uint32_t data, uint32_t address); /* 写入单个数据到 FRAM */
/**
 * @brief 写入两个数据。
 *
 * @param steps 与卷绕圈数一同保存的编码器累计步数。
 * @param circle 与编码器位置一同保存的卷绕圈数。
 * @param address CPU3 参数 FRAM 的绝对字节地址；多字节读写从该地址开始并按连续地址递增。
 */
void WriteTwoData(int steps, int circle, int address); /* 写入两个数据到 FRAM */
/**
 * @brief 在固定地址执行破坏性 FRAM 单字和多字节读写校验，并逐项打印比对结果。
 *
 * 单字测试依次在逻辑地址 50、511、100 和 200 写入普通模式、全零及全一数据，延时后回读 32 位值并打印通过或失败。
 * 多字节测试从字节地址 0x0100 写入固定八字节序列，再逐字节回读比较并输出每个地址的结果。
 *
 * @note 该测试会覆盖上述固定 FRAM 地址且不会恢复原内容，只能在确认这些地址允许被破坏的维护环境执行，禁止在生产流程或有效参数镜像上调用。
 */
void Test_FRAM_ReadWrite(void); /* 测试 FRAM 数据读写功能，包括单字节和多字节的读写 */

#endif

