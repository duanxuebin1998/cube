#include "ad5421.h"
#include "spi.h"
#include "stdio.h"
#include "main.h"
//static int32_t SetCurrent(double current);

static inline void AD5421_CS_LOW(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_RESET);
}

static inline void AD5421_CS_HIGH(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_SET);
}
// 向 AD5421 发送 24 位数据
void AD5421_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t txData[3];
    txData[0] = reg & 0x7F;    // bit7=0 表示写
    txData[1] = (value >> 8) & 0xFF;
    txData[2] = value & 0xFF;

    AD5421_CS_LOW();
    HAL_SPI_Transmit(&hspi3, txData, 3, HAL_MAX_DELAY);
    AD5421_CS_HIGH();
}

// 从 AD5421 读寄存器
uint16_t AD5421_ReadReg(uint8_t reg)
{
    uint8_t txData[3];
    uint8_t rxData[3];

    txData[0] = reg | 0x80;  // bit7=1 表示读
    txData[1] = 0x00;
    txData[2] = 0x00;

    AD5421_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
    AD5421_CS_HIGH();
    return ((uint16_t)rxData[1] << 8) | rxData[2];
}
// 设置 DAC 原始值
void AD5421_SetDacRaw(uint16_t value)
{
    AD5421_WriteReg(WRITEDAC, value);
}

// 设置输出电流 (单位: mA)
void AD5421_SetCurrent(float mA)
{
    // 假设范围为 4–20 mA
    if (mA < 4.0f) mA = 4.0f;
    if (mA > 20.0f) mA = 20.0f;

    float scale = (mA - 3.2f) / (24.0f-3.2f); // 转换到 0–1
    uint16_t dacValue = (uint16_t)(scale * 65535);

    AD5421_SetDacRaw(dacValue);
}

//==============================
// 初始化函数
//==============================
void AD5421_Init(void)
{

}

//==============================
// 软件复位
//==============================
void AD5421_Reset(void)
{
	AD5421_WriteReg(RESETAD5421REG, 0x0000);
    HAL_Delay(10);
}

//==============================
// 读取状态寄存器
//==============================
uint16_t AD5421_GetStatus(void)
{
    return AD5421_ReadReg(READFAULT);
}

//==============================
// 读取实际输出电流 (诊断值, 单位 mA)
//==============================
float AD5421_ReadCurrent(void)
{
    uint16_t raw = AD5421_ReadReg(READFAULT);
    // 16bit ADC 转换成 mA (假设范围 0–24mA)
    float current = (raw / 65535.0f) * 20.8f+3.2f; // 转换到 3.2–24mA 范围;
    printf("读取电流\t%.3f\r\n",current);
    return current;
}


void ResetAD5421(void)
{
	AD5421_WriteReg(RESETAD5421REG, 0x0000);
}
/*******************************************************
* Name    ReadControlRegister
* brief
* param   readbackcontrol:
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/
//
//static void ReadControlRegister(uint16_t * readbackcontrol)
//{
//	uint32_t i;
//	uint16_t  readback;
//
//	for(i=0u;i<5u;i++)
//	{
//		SYNC = 0;
//		DELAY30US;
//		(void)SPI2_ReadWriteByte(READCONTROL);
//		readback = SPI2WRITE00;
//		readback <<= 8;
//		readback += SPI2WRITE00;
//		DELAY30US;
//		SYNC = 1;
//		DELAY30US;
//
//		if((readback!=0u)&&(readback!=0xFFFFu))
//		{
//			break;
//		}
//	}
//	*readbackcontrol = readback;
//}


/*******************************************************
* Name    WriteControlRegister
* brief
* param   controldata:
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/

static uint32_t WriteControlRegister(uint16_t  controldata)
{
	uint16_t  readback;
	AD5421_WriteReg(WRITECONTROL, controldata);
	//验证写入是否成功
//	ReadControlRegister(&readback);
//	if(readback != controldata)
//	{
//		(void)printf("AD5421WriteControlRegister\treadback\t0X%X\t!=\tcontroldata\t0X%X\r\n",readback,controldata);
//		return 1;//返回错误代码
//	}
	return NO_ERROR;
}

/*******************************************************
* Name    CalculateCurrentCRC
* brief
* param   data:
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/

static uint16_t  CalculateCurrentCRC(uint32_t data)
{
	uint32_t divisor = 0x107u;
	uint32_t res;
	uint32_t bitcount = 23u;
	uint32_t fir;

	for(;bitcount!=0u;bitcount--)
	{
		fir = ((data>>bitcount)&0x100u);
		if(fir!=0u)
		{
			break;
		}
	}

	res = (data>>bitcount)^divisor;
	for(;bitcount!=0u;bitcount--)
	{
		res = (res<<1u)+((data>>(bitcount-1u))&0x01u);
		if((res&0x100u)!=0u)
		{
			res ^= divisor;
		}
	}

	return (uint16_t )res;
}

void CurrentStateJudgeAndSend(void)
{
//    result.CurrentValue = ReadCurrentCorspdDeviceStatus();
//    SetCurrent(4.0);
}

/*******************************************************
* Name    Ad5421Init
* brief
* param   None
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/

uint32_t Ad5421Init(void)
{
	uint32_t ret;
	ResetAD5421();//软件复位

	(void)WriteControlRegister(CUR_SPIOFF_COMMAND);

	(void)AD5421_SetCurrent(CURRENT_INIT);

//	Timer3Start();//开始定时设置电流

//	ret = CurrentSelfTest();
	return ret;
}

/*******************************************************
* Name    CurrentSelfTest
* brief
* param   None
* retval  ERRORCODE
* author  AUBON
* Data    2023-06-08
******************************************************/
//
//static uint32_t CurrentSelfTest(void)
//{
//	uint32_t ret;
//	if(FAULT_5421!=0u)
//	{
//		ReadFaultRegister();/*read fault register*/
//	}
//
//	if(FAULT_5421!=0u)
//	{
//		ret = 1;
//		(void)printf("AD5421 alarm!\r\n");
//	}
//	else
//	{
//		ret = NO_ERROR;
//	}
//
//	return ret;
//}




