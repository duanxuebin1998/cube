#include <mb85rs2m.h>
#include "AS5145.h"
#include "main.h"
#include "stdio.h"
#include "encoder.h"

// 状态监控结构体，记录SSI通信状态
typedef struct {
	uint8_t retry_count;  // 失败重试次数
	bool fatal_error;     // 是否发生致命错误
	TIM_HandleTypeDef *htim; // 绑定的定时器句柄
} SSI_State;

// 全局状态变量，初始化
static SSI_State ssi_state = { .retry_count = 0, .fatal_error = false, .htim = &ENCODER_TIM_HANDLE };
// SPI接收缓冲区（存放4字节原始数据）
static uint8_t rxData[10] = { 0 };

// 函数声明
static uint8_t Calculate_Even_Parity(uint32_t data);
static SSI_Data_t Parse_SSI_Data(const uint8_t *rxData);
static void Print_SSI_Error(const SSI_Data_t *data);
static bool Check_SSI_Error_Condition(SSI_Data_t *data);
static void __attribute__((unused)) Handle_SSI_Error(SSI_Data_t *data);
static void Report_Fatal_Error(SSI_Data_t *err_data);
static void Stop_Encoder_Collection_TIM(void);
static void Start_Read_SSI_Data(void);
/**
 * @brief  计算18位数据的偶校验位
 * @param  data  需要计算的18位数据
 * @retval 返回计算出的校验位（0或1）
 */
static uint8_t Calculate_Even_Parity(uint32_t data) {
	uint8_t count = 0;
	for (uint8_t i = 0; i < 17; i++) {
		if (data & (1 << i))
			count++;
	}
	return (count % 2) == 0 ? 0 : 1; // 偶数个1返回0，否则返回1
}

/**
 * @brief  解析SSI传感器返回的数据
 * @param  rxData  SPI接收到的4字节数据
 * @retval 解析后的SSI数据结构体
 */
static SSI_Data_t Parse_SSI_Data(const uint8_t *rxData) {
	SSI_Data_t result;
	// 组合18位原始数据
	uint32_t raw_data = (((rxData[0] & 0x7F) << 11) | (rxData[1] << 3) | (rxData[2] >> 5));

	// 解析具体字段
	result.angle = (raw_data >> 6) & 0x0FFF; // 提取12位角度数据
	result.OCF = (raw_data >> 5) & 0x01;    // 输出校验标志
	result.COF = (raw_data >> 4) & 0x01;    // CORDIC溢出位
	result.LIN = (raw_data >> 3) & 0x01;    // 线性误差位
//	result.LIN = 0;    // 屏蔽线性误差位
	result.MagINCn = (raw_data >> 2) & 0x01; // 磁场增大指示
	result.MagDECn = (raw_data >> 1) & 0x01; // 磁场减小指示
	result.parity = raw_data & 0x01;         // 提取奇偶校验位

	// 计算并检查奇偶校验
	uint8_t calculated_parity = Calculate_Even_Parity(raw_data >> 1);
	result.parity_ok = (calculated_parity == result.parity);

	return result;
}

/**
 * @brief  SSI接收数据处理及错误检查
 * @param  rx_data: SPI接收缓冲区指针
 * @param  cs_port: 片选信号GPIO端口（如SSI_CSN_PORT）
 * @param  cs_pin: 片选信号GPIO引脚（如SSI_CSN_PIN）
 * @retval SSI_Data_t 解析后的数据结构体
 */
static SSI_Data_t Process_SSI_Frame(uint8_t *rx_data, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
	// 关闭片选信号
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	// 解析原始数据
	SSI_Data_t parsed_data = Parse_SSI_Data(rx_data);

	// 错误检测与处理
	if (Check_SSI_Error_Condition(&parsed_data)) {
//		Handle_SSI_Error(&parsed_data);  // 带重试逻辑的处理
		Print_SSI_Error(&parsed_data);
//		Update_Encoder_Count(parsed_data.angle);  // 有效数据更新
	}
//	if(parsed_data.parity_ok == 0)
//	{
//		//校验错误
//	}
	else {
		ssi_state.retry_count = 0;  // 成功时重置计数器

		Update_Encoder_Count(parsed_data.angle);  // 有效数据更新
	}
	return parsed_data;
}

/**
 * @brief  SSI错误条件判断
 * @param  data: 解析后的数据结构体指针
 * @retval bool 错误状态（true=存在错误）
 */
static bool Check_SSI_Error_Condition(SSI_Data_t *data) {
	return (!data->parity_ok     // 奇偶校验失败
	|| !data->OCF          // 输出校验标志异常
			|| data->COF            // 计数器溢出
//			|| data->LIN            // 线性误差
	);
}
/**
 * @brief 处理SSI数据错误
 * @param data 出现错误的数据结构体
 */
static void __attribute__((unused)) Handle_SSI_Error(SSI_Data_t *data) {
	if (ssi_state.fatal_error)
		return;

	if (++ssi_state.retry_count <= 3) {
		// 延迟确保总线稳定
		HAL_Delay(1);
		// 发起重试
		Start_Read_SSI_Data();
	} else {
		// 进入故障模式
		ssi_state.fatal_error = true;
		Stop_Encoder_Collection_TIM();  //关闭定时器
		Report_Fatal_Error(data);
	}
}
/**
 * @brief SPI接收完成回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &SSI)  // SPI interface used for AS5145 communication
	{
		Process_SSI_Frame(rxData, SSI_CSN_PORT, SSI_CSN_PIN);
	}
}

/**
 * @brief 启动SSI数据读取流程（使用DMA）
 */
//static void Start_Read_SSI_Data(void)
//{
//	if (HAL_SPI_GetState(&SSI) == HAL_SPI_STATE_READY)
//	{
//		HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_RESET);
//		printf("SPI5 State: %d\n", HAL_SPI_GetState(&SSI));
//		printf("SPI5 ErrorCode: 0x%08lX\n", SSI.ErrorCode);
//		printf("SPI5 hdmarx: %p\n", SSI.hdmarx);
//		if (SSI.hdmarx != NULL) {
//		    printf("DMA Stream: %p\n", SSI.hdmarx->Instance);
//		}
//
//		HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&SSI, rxData, 4);
//		if (status != HAL_OK)
//		{
//			printf("SPI ErrorCode: 0x%08lX\n", SSI.ErrorCode);
//			printf("SPI DMA 启动失败, 错误码: %d\n", status);
//			HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
//		}
//	}
//}
static void Start_Read_SSI_Data(void) {
	if (HAL_SPI_GetState(&SSI) == HAL_SPI_STATE_READY) {
		__HAL_DMA_DISABLE(SSI.hdmarx);   // 禁用 DMA 防止残留
		__HAL_DMA_CLEAR_FLAG(SSI.hdmarx, DMA_FLAG_TCIF3_7); // 根据 stream 选择

		HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_RESET);
//        printf("SPI5 State: %d\n", HAL_SPI_GetState(&SSI));
//        printf("SPI5 ErrorCode: 0x%08lX\n", SSI.ErrorCode);
//        printf("SPI5 hdmarx: %p\n", SSI.hdmarx);
//        if (SSI.hdmarx != NULL) {
//            printf("DMA Stream: %p\n", SSI.hdmarx->Instance);
//        }

		HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&SSI, rxData, 4);
		if (status != HAL_OK) {
			printf("SPI ErrorCode: 0x%08lX\n", SSI.ErrorCode);
			printf("SPI DMA 启动失败, 错误码: %d\n", status);
			HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET);
		}
	} else {
		printf("SPI not ready, current state: %d\n", HAL_SPI_GetState(&SSI));
	}
}
/**
 * @brief 定时器中断回调函数（用于周期性触发SSI读取）
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) { // 判断是否是目标定时器
		Start_Read_SSI_Data(); // 开始一次DMA读取
	}
}
/**
 * @brief 记录并报告致命错误
 * @param err_data 发生错误的数据结构体
 */
static void Report_Fatal_Error(SSI_Data_t *err_data) {
	//置全局状态标志
	Print_SSI_Error(err_data);
}
/**
 * @brief 打印SSI错误信息
 * @param data 出错的数据结构体
 */
static void Print_SSI_Error(const SSI_Data_t *data) {
//	printf("==== 数据错误 ====\n");
//	if (!data->parity_ok) {
//		printf("错误: 偶校验失败\n");
//	}
//	if (!data->OCF) {
//		printf("错误: OCF 未完成，数据无效\n");
//	}
//	if (data->COF) {
//		printf("错误: CORDIC 溢出，数据无效\n");
//	}
//	if (data->LIN) {
//		printf("错误: 线性度报警，数据可能无效\n");
//	}
//	printf("==================\n");
}
/**
 * @brief 启动编码器数据采集的定时器中断
 * @retval HAL_StatusTypeDef 启动状态（HAL_OK=成功）
 *
 * @note 该函数会启用TIM1定时器的中断，并自动开启计数器
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void) {
// 初始化状态结构体成员
	ssi_state.retry_count = 0;
	ssi_state.fatal_error = false;
	ssi_state.htim = &ENCODER_TIM_HANDLE;

// 启动定时器中断
	HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(ssi_state.htim);
// 错误处理
	if (status != HAL_OK) {
		printf("error: encoder timer fail (code: %d)\n", status);
		// 这里可以添加更详细的错误处理（如系统复位等）
	} else {
		printf("encoder timer start\n");
	}
	return status;
}

/**
 * @brief 停止编码器数据采集定时器
 */
static void Stop_Encoder_Collection_TIM(void) {
	if (ssi_state.htim != NULL) {
		HAL_TIM_Base_Stop_IT(ssi_state.htim);
		printf("encoder timer stop\n");
	}
}
