#include "exit.h"
#include "display_tankopera.h"
#include "display.h"



bool FlagofTankOpera = false;//罐上操作标志位

static uint32_t PressTimeMs = 700;//长按开机时间-单位ms



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    SetScreenBright();
    //延时消抖
    HAL_Delay(5);
    switch(GPIO_Pin)
    {
	case KEY_BACK_Pin:
        {
            if (HAL_GPIO_ReadPin(KEY_BACK_GPIO_Port, KEY_BACK_Pin) == GPIO_PIN_RESET)
            {
                printf("b");
                if(FlagofTankOpera == true)
                    KeyProcess(USE_KEY_BACK);
            }
            break;
        }
        case KEY_UP_Pin:
        {
            if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
            {
                printf("u");
                if(FlagofTankOpera == true)
                    KeyProcess(USE_KEY_UP);
            }
            break;
        }
        case KEY_DOWN_Pin:
        {
            if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
            {
                printf("d");
                if(FlagofTankOpera == true)
                    KeyProcess(USE_KEY_DOWN);
            }
            break;
        }
        case KEY_SURE_Pin:
        {
            if (HAL_GPIO_ReadPin(KEY_SURE_GPIO_Port, KEY_SURE_Pin) == GPIO_PIN_RESET)
            {
                printf("s%d ",FlagofTankOpera);
                if(FlagofTankOpera == true)
                    KeyProcess(USE_KEY_SURE);
                else
                {
//                    Timer7Startms(PressTimeMs);
                }
            }
            break;
        }
        default : break;
    }
}















