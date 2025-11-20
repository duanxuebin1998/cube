#ifndef _DSM_SYSTEM_PARAMETER_H
#define _DSM_SYSTEM_PARAMETER_H
#include "main.h"
#include "system_parameter.h"
//extern int LevelCorrection;
//extern int TankHigh_LevelCorrection;
//
//extern char SoftOfversion6;
//#define SOFTOFVERSION_0 1213 // 程序版本
//#define SOFTOFVERSION_1 'P'	 // 程序版本
//#define SOFTOFVERSION_2 'B'	 // 程序版本
//#define SOFTOFVERSION_3 'C'	 // 程序版本
//#define SOFTOFVERSION_4 'D'	 // 程序版本
//#define SOFTOFVERSION_5 'E'	 // 程序版本
// 第六位是数据定义
#define PARAMETER_ERROR -1
#define PARAMETER_WRITE_FAIL -2

void SystemParameterSet(void);
int SystemParameterSave(int startadd, int reamount);

#endif
