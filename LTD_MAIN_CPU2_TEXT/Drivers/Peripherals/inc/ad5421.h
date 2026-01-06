#ifndef sil__AD5421_H
#define sil__AD5421_H
#include "main.h"

#define AD5421_CS_GPIO_PORT   GPIOB
#define AD5421_CS_PIN         GPIO_PIN_6

//==============================
// ¼Ä´æÆ÷µØÖ·ºê¶¨Òå
//==============================
#define DELAY30US  delay_us(100)
#define SPI2WRITE00 SPI2_ReadWriteByte(0x00)

/*Full-scale current value*/
#define STARTFULLSCALE 3.2
#define STOPFULLSCALE 24.0
/*AD5421 register instruction*/
#define WRITEDAC 0x01u
#define WRITECONTROL 0x02u
#define RESETAD5421REG 0x07u
#define READCONTROL 0x82u
#define READFAULT 0x85u
/*SPI watchdog switch*/
#define CUR_SPION_COMMAND 0xC000u //SPI watchdog on, 4s
#define CUR_SPIOFF_COMMAND 0x1000u //SPI watchdog off



/*Current setting*/
#define CURRENT_INIT 				3.5
#define CURRENT_UNDERAOL 			3.6
#define CURRENT_GREATERAOH 			21.0
#define CURRENT_EQUIPMENT_ERROR 	22.0
#define CURRENT_CHECKMODE_MIN 		3.5
#define CURRENT_CHECKMODE_MAX 		24.0
#define CURRENT_CONFIGMODE 			23.0
#define CURRENT_MEAMODE 			21.5



/*Current setting*/
#define CURRENT_CHECKMODE_MIN 3.5
#define CURRENT_CHECKMODE_MAX 24.0



//void CurrentStateJudgeAndSend(void);
uint32_t Ad5421Init(void);
void AD5421_SetCurrent(float mA);

#endif

