/*
 * app_main.h
 *
 *  Created on: Mar 13, 2025
 *      Author: 1
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "AS5145.h"
#include <stdio.h>
#include <string.h>
#include "weight.h"
#include "TMC5130.h"
#include "fault_manager.h"
// 主程序初始化
void App_Init(void);

// 主循环任务
void App_MainLoop(void);

#endif
