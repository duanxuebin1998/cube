#ifndef _DSM_COMM_H__
#define _DSM_COMM_H__

#include "DSM_stateformodbus2.h"
#include "DSM_SlaveModbus_modbus2.h"
#include "main.h"
#include "my_crc.h"


/*****************************命令*****************************************/
#define COMMAND_NONE 0
#define COMMAND_BACKZERO_START 1					// 开始回零点
#define COMMAND_BACKZERO_STOP 2						// 停止回零点
#define COMMAND_FINDZERO_START 3					// 开始找零点
#define COMMAND_FINDZERO_STOP 4						// 停止找零点
#define COMMAND_SINGLEPOINT_START 5					// 开始单点测量
#define COMMAND_SINGLEPOINT_STOP 6					// 停止单点测量
#define COMMAND_SPTEST_START 7						// 开始单点监测
#define COMMAND_SPTEST_STOP 8						// 停止单点监测
#define COMMAND_SPREADPOINTS_START 9				// 开始分布测量（带液位高度）
#define COMMAND_SPREADPOINTS_STOP 10				// 停止分布测量（带液位高度）
#define COMMAND_SPREADPOINTSAI_START 11				// 开始自动分布测量
#define COMMAND_SPREADPOINTSAI_STOP 12				// 停止自动分布测量
#define COMMAND_CALOIL_START 13						// 开始标定液位
#define COMMAND_CALOIL_STOP 14						// 停止标定液位
#define COMMAND_SETWORKPATTER 15					// 设置工作模式
#define COMMAND_READPARAMETER 16					// 读取参数
#define COMMAND_RUNUP 17							// 上行
#define COMMAND_RUNDOWN 18							// 下行
#define COMMAND_SETZEROCIRCLE 19					// 设置零点圈数
#define COMMAND_SETZEROANGLE 20						// 设置零点圈数
#define COMMAND_RESTOREFACTORYSETTING 21			// 恢复出厂设置
#define COMMAND_BACKUPFILE 22						// 备份配置文件
#define COMMAND_RESTORYFILE 23						// 恢复配置文件
#define COMMAND_FINDOIL_START 24					// 开始寻找液位
#define COMMAND_FINDOIL_STOP 25						// 停止寻找液位
#define COMMAND_FINDWATER_START 26					// 开始寻找水位
#define COMMAND_FINDWATER_STOP 27					// 停止寻找水位
#define COMMAND_FINDBOTTOM_START 28					// 开始寻找罐底
#define COMMAND_FINDBOTTOM_STOP 29					// 停止寻找罐底
#define COMMAND_FORCEZERO_START 30					// 开始设置零点
#define COMMAND_FORCEZERO_STOP 31					// 停止设置零点
#define COMMAND_ONTANKOPERATION_BACKZERO 32			// 罐上操作回零点中
#define COMMAND_ONTANKOPERATION_RUN_BACKZEROOK 33	// 罐上操作完成回零点
#define COMMAND_ONTANKOPERATION_RUN_BEGAINUP 34		// 罐上操作上行
#define COMMAND_ONTANKOPERATION_RUN_UPCOMPLATE 35	// 罐上操作上行完成
#define COMMAND_ONTANKOPERATION_RUN_BEGAINDOWN 36	// 罐上操作下行
#define COMMAND_ONTANKOPERATION_RUN_DOWNCOMPLATE 37 // 罐上操作下行完成
#define COMMAND_SYNTHETIC_START 38					// 综合指令开始synthetic instruct 2019.08.07V1.105
#define COMMAND_SYNTHETIC_STOP 39					// 综合指令停止synthetic instruct 2019.08.07V1.105
#define COMMAND_METERDENSITY_START 40				// 开始密度每米测量V1.116 dq2020.4.2
#define COMMAND_METERDENSITY_STOP 41				// 停止密度每米测量V1.116 dq2020.4.2
#define COMMAND_INTERVALDENSITY_START 42			// 开始液位区间测量V1.116 dq2020.4.2
#define COMMAND_INTERVALDENSITY_STOP 43				// 停止液位区间测量V1.116 dq2020.4.2

#define COMMAND_UNKNOW 44 // 未知指令

#define DSM_STATE_SWITCH 0x00000002 // 状态切换



#endif
