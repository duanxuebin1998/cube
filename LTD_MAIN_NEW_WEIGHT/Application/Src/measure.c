/*
 * measure.c
 *
 *  Created on: Mar 20, 2025
 *      Author: duan
 */

#include "measure.h"
#include "system_parameter.h"
#include "measure_zero.h"
#include "measure_tank_height.h"

static int MeasureStart(void)
{
	fault_info_init();//故障初始化清零
	g_measurement.device_status.error_code = NO_ERROR;//故障代码清零
	return NO_ERROR;
}

//罐底零点主函数
void MeasureZero(void)
{
	int ret = 0;
	g_measurement.device_status.device_state = STATE_BACKZEROING;
    ret = MeasureStart();
    if(ret != NO_ERROR)
    {
        return;
    }
	//开始回零点
	ret = SearchZero();
	if (ret != NO_ERROR)
	{
		return;
	}

	g_measurement.device_status.device_state = STATE_STANDBY;
	return;
}


//罐底测量主函数
void MeasureBottom(void)
{
	int ret = 0;
	g_measurement.device_status.device_state = STATE_FINDBOTTOM;
    ret = MeasureStart();
    if(ret != NO_ERROR)
    {
        return;
    }
	//开始测量罐高
	ret = SearchBottom();
	if (ret != NO_ERROR)
	{
		return;
	}

	g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
	return;
}
