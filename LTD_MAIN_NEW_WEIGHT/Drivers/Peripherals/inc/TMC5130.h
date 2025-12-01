
#ifndef TMC_IC_TMC5130_H_
#define TMC_IC_TMC5130_H_

#include "main.h"

#include "TMC5130_Constants.h"
#include "TMC5130_Pins.h"
#include "TMC5130_Register.h"

#define BYTE(value, n)    (((value) >> ((n) << 3)) & 0xFF)
// 定义控制类型的枚举类型，用于选择不同的控制方式
typedef enum
{
	STEPDIR = 0,    // 使用步进-方向信号控制电机
	INTERNAL,       // 使用驱动器的内部信号控制电机
} TMC5130ControlType; // 控制类型枚举定义


// 定义归位状态的枚举类型，用于归位过程中的状态管理
typedef enum
{
	HOME = 0,       // 初始状态，表示开始归位
	RETRACT,        // 退回状态，通常用于从错误位置退回到安全位置
	UNRETRACT,      // 从退回状态解除，继续归位
	WAIT_MOVE,      // 等待移动完成的状态
} HomingState_e;   // 归位状态枚举定义

// 定义与TMC5130电机驱动器相关的结构体，用于管理驱动器的各种参数和控制状态
typedef struct
{
	HomingState_e			home_state;      // 当前归位状态
	HomingState_e			prev_home_state; // 上一个归位状态，用于状态切换判断
	TMC5130ControlType 		control_type;    // 当前控制类型，决定电机控制方式
	SPI_HandleTypeDef	 	*spi;            // SPI总线的句柄，用于与TMC5130通信
	int32_t					drvstat;         // 驱动器状态信息，可能包括错误或故障标志
	int32_t 				sg_flag;        // 步进信号标志，用于检测负载或电机状态
	int32_t 				sg_result;      // 步进信号的结果值，用于判断当前步进状态
	int32_t 				cs_actual;      // 当前的电流控制器值，可能用于监控电流消耗
	int32_t 				rampstat;       // 电机的加速/减速状态，用于判断运动过程中的状态
	uint8_t 				homing_done;    // 归位完成标志，为1时表示归位成功

	// INITIALIZATION
	GPIO_TypeDef	*cs_port;         // 片选（CS）引脚的GPIO端口，用于驱动器的SPI通信片选
	uint16_t		cs_pin;          // 片选引脚的编号

	GPIO_TypeDef	*en_port;        // 使能（EN）引脚的GPIO端口，用于启用/禁用驱动器
	uint16_t		en_pin;          // 使能引脚的编号

} TMC5130TypeDef; // TMC5130电机驱动器类型定义
extern TMC5130TypeDef stepper;

#define GCONF       0x00 			  //全局配置标志
#define X_COMPARE   0x05 		    //位置比较寄存器
#define IHOLD_IRUN  0x10 	      //驱动电流控制
#define TPOWERDOWN  0x11        //设置在静止 (stst) 后的延迟时间。电机电流断电。时间范围约为0至4秒。0..。(2 ^ 8)-1) * 2 ^ 18 t
#define TSTEP       0X12
#define TPWM_THRS   0x13        //这是stealthChop电压PWM模式的上限速度。如果已配置，则启用stealChop模式，dcStep禁用
#define TCOOLTHRS   0x14 		    //这是用于启动智能能源coolStep和stallGuard功能的较低阈值速度。
#define THIGH       0X15        //此速度设置允许速度相关切换不同的斩波器模式和全步进以最大化扭矩。（无符号）失速检测功能将关闭2-3每当通过THIGH阈值时的电气周期补偿切换模式的影响。
#define RAMPMODE    0x20 		    //驱动模式（速度，定位，保持）
#define XACTUAL     0x21 		    //电机实际位置  ----用于读取实际位置
#define VACTUAL     0x22 		    //读取斜坡发生器的实际电机速度和方向？
#define VSTART      0x23 		    //电机启动速度
#define A_1         0x24 			  // VSTART和V1之间的第一次加速
#define V_1         0x25     	  //第一加速/减速阶段目标速度
#define AMAX        0x26 			  // V1和VMAX之间的第二次加速
#define VMAX        0x27 			  //这是速度模式下的目标速度。它可以在动作中随时更改。
#define DMAX        0x28			  // VMAX和V1之间的减速度
#define D_1         0x2A 			  // V1和VSTOP之间的减速度 //注意：即使V1 = 0，也不要在定位模式下设置0！
#define VSTOP       0x2B 			  //电机停止速度（无符号） //注意：设置VSTOP> VSTART！ //注意：不要在定位模式下设置0，最小10推荐！
#define TZEROWAIT   0x2C 		    //定义减速后的等待时间 //在下次移动之前使速度为零或 //方向反转可以开始。时间范围约为0到2秒。
#define XTARGET     0x2D 		    //斜坡模式的目标位置
#define VDCMIN      0x33        //自动换向
#define SW_MODE     0x34 		    //限位开关模式配置
#define RAMP_STAT   0x35 		    //读取斜坡状态和限位开关事件状态
#define XLATCH      0x36 		    //在可编程开关事件时锁存XACTUAL
#define CHOPCONF    0x6C 		    // Chopper和驱动程序配置
#define COOLCONF    0x6D 		    // coolStep智能电流控制寄存器和stallGuard2配置
#define DRV_STATUS  0x6F 	      // stallGuard2值和驱动程序错误标志
#define PWMCONF     0x70        //电压PWM模式斩波器-查看单独的表格


#define READ_ACCESS       0x80 			 //读取spi通信的访问权限

//参考开关的极性
#define REF_SW_HIGH_ACTIV 0x00 		    //非反相，高电平有效：REFL上的高电平会停止电机
#define REF_SW_LOW_ACTIV  0x0C 		    //反转，低电平有效：REFL低电平会停止电机

//电机方向
#define NORMAL_MOTOR_DIRECTION  0x00 	//电机正常方向
#define INVERSE_MOTOR_DIRECTION 0x10 	//反转电机方向

// RAMPMODE寄存器的模式
#define POSITIONING_MODE  0x00 				// 定位模式--使用所有A，D和V参数）
#define VELOCITY_MODE_POS 0x01 				// 速度模式为正VMAX (使用AMX加速)positiv VMAX，使用AMAX加速
#define VELOCITY_MODE_NEG 0x02 				// 速度模式为负VMAX (使用AMX加速)negativ VMAX，使用AMAX加速
#define HOLD_MODE			    0x03		    // 保持模式 (速度保持不变，除非发生停止时间)velocity remains unchanged, unless stop event occurs

#define VZERO 0x400 						      // RAMP_STAT中的标志，1：表示实际速度为0。


#define SET_IHOLD(a)		  ((a & 0x1F)<<0)
#define SET_IRUN(a)		  	((a & 0x1F)<<8)
#define SET_IHOLDDELAY(a)	((a & 0xF)<<16)



#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,4000,100.47095761381482}  // default steps per unit for Prusa   Prusa的每单位默认步数
#define HOMING_FEEDRATE {4000, 4000, 4000, 0}                          // set the homing speeds (mm/min)    设定回原点速度（mm / min）





/*-------------------
堵转检测stallGuard2TM
电流动态调coolStepTM
静音技术spreadCycleTM Chopper
精密微步microPlyer
-------------------*/
/*******************************************************************
*                      Current configuration
*******************************************************************/
// only change if necessary  只在必要时改变    注：工作电流的大小确定了负载能力和噪声的大小，根据用途适当设置。
#define X0_CURRENT_RUN	    5   // Motor run current (0=1/32 31=32/32)   //电机运行电流
#define X0_CURRENT_HOLD	    1	  // Standstill current (0=1/32 31=32/32)  //静止不动是的电流

#define X1_CURRENT_RUN	    8	  // Motor run current (0=1/32 31=32/32)
#define X1_CURRENT_HOLD	    2	  // Standstill current (0=1/32 31=32/32)

#define J_CURRENT_RUN	      20	// Motor run current (0=1/32 31=32/32)
#define J_CURRENT_HOLD	    2	  // Standstill current (0=1/32 31=32/32)

#define Barn_CURRENT_RUN	  8	// Motor run current (0=1/32 31=32/32)
#define Barn_CURRENT_HOLD	  2	  // Standstill current (0=1/32 31=32/32)

#define Heat_CURRENT_RUN	  15	// Motor run current (0=1/32 31=32/32)
#define Heat_CURRENT_HOLD	  15	  // Standstill current (0=1/32 31=32/32)


/*------------------------------------------------------------------
*             Reference switch configuration  原点参考开关配置
------------------------------------------------------------------*/
// x-axis  x轴
// switch position, select right or left reference switch (not both) 开关位置，选择右或左参考开关（不是两者）原点位置？
#define SWITCH_POSITION_X0	0x21		// left
//#define SWITCH_POSITION_X0	0x11		// right

// switch polarity, select high or low activ (not both)  切换极性，选择高或低激活（不是两者）有效极性
#define SWITCH_POLARITY_X0	    REF_SW_LOW_ACTIV		// low activ
//#define SWITCH_POLARITY_X0	REF_SW_HIGH_ACTIV		// high activ
//==================================================================

// y-axis  X1轴
// switch position, select right or left reference switch (not both)
#define SWITCH_POSITION_X1	0x21		// left
//#define SWITCH_POSITION_X1	0x11		// right

// switch polarity, select high or low activ (not both)
#define SWITCH_POLARITY_X1	   REF_SW_LOW_ACTIV		// low activ
//#define SWITCH_POLARITY_X1	REF_SW_HIGH_ACTIV		// high activ
//==================================================================

// z-axis  z轴
// switch position, select right or left reference switch (not both)
#define SWITCH_POSITION_J	0x21		// left
//#define SWITCH_POSITION_J	0x11		// right

// switch polarity, select high or low activ (not both)
#define SWITCH_POLARITY_J	    REF_SW_LOW_ACTIV		// low activ
//#define SWITCH_POLARITY_J	REF_SW_HIGH_ACTIV		// high activ
//==================================================================

// Barn-axis  Barn轴
// switch position, select right or left reference switch (not both)
#define SWITCH_POSITION_Barn	0x21		// left
//#define SWITCH_POSITION_Barn	0x11		// right

// switch polarity, select high or low activ (not both)
#define SWITCH_POLARITY_Barn	    REF_SW_LOW_ACTIV		// low activ
//#define SWITCH_POLARITY_Barn	REF_SW_HIGH_ACTIV		// high activ
//==================================================================

// Heat-axis  z轴
// switch position, select right or left reference switch (not both)
#define SWITCH_POSITION_Heat	0x21		// left
//#define SWITCH_POSITION_Heat	0x11		// right

// switch polarity, select high or low activ (not both)
#define SWITCH_POLARITY_Heat	    REF_SW_LOW_ACTIV		// low activ
//#define SWITCH_POLARITY_Heat	REF_SW_HIGH_ACTIV		// high activ
//==================================================================


/*------------------------------------------------------------------
*               Stepper direction  步进方向
------------------------------------------------------------------*/
// x-axis
#define STEPPER_DIRECTION_X0		INVERSE_MOTOR_DIRECTION   //顺时针方向
//#define STEPPER_DIRECTION_X0		  NORMAL_MOTOR_DIRECTION   //顺时针方向
//==================================================================

// y-axis
//#define STEPPER_DIRECTION_X1		INVERSE_MOTOR_DIRECTION
#define STEPPER_DIRECTION_X1		  NORMAL_MOTOR_DIRECTION
//==================================================================

// z-axis
//#define STEPPER_DIRECTION_J		INVERSE_MOTOR_DIRECTION
#define STEPPER_DIRECTION_J		  NORMAL_MOTOR_DIRECTION
//==================================================================

// Barn-axis
#define STEPPER_DIRECTION_Barn	INVERSE_MOTOR_DIRECTION
//#define STEPPER_DIRECTION_Barn	  NORMAL_MOTOR_DIRECTION
//==================================================================

// Barn-axis
//#define STEPPER_DIRECTION_Barn	INVERSE_MOTOR_DIRECTION
#define STEPPER_DIRECTION_Heat	  NORMAL_MOTOR_DIRECTION
//==================================================================

/*******************************************************************
*               STALLGUARD  堵转技术？
*******************************************************************/
#define STALLGUARDTHRESHOLD   0x02
// x-axis
#define STALLGUARD_X	// if selected, stallguard is active  如果选择，则挡板处于活动状态
#define STALLGUARDTHRESHOLD_X	0x08      // range 0x00..0x7F   StallGuardThreshold/堵转门槛？

// y-axis
#define STALLGUARD_X1	// if selected, stallguard is active
#define STALLGUARDTHRESHOLD_X1	0x08      // range 0x00..0x7F

// z-axis
#define STALLGUARD_J	// if selected, stallguard is active
#define STALLGUARDTHRESHOLD_J	0x08      // range 0x00..0x7F

// z-axis
#define STALLGUARD_Barn	// if selected, stallguard is active
#define STALLGUARDTHRESHOLD_Barn	0x08  // range 0x00..0x7F

// z-axis
#define STALLGUARD_Heat	// if selected, stallguard is active
#define STALLGUARDTHRESHOLD_Heat	0x08  // range 0x00..0x7F




// API Functions
// All functions act on one IC identified by the TMC5130TypeDef pointer

/*
 * PUBLIC FUNCTIONS
 */
void 		stpr_initStepper	(TMC5130TypeDef *tmc5130, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t dir, uint8_t current);
//uint8_t	 	stpr_home			(TMC5130TypeDef *tmc5130, uint16_t velocity, uint8_t stallguard);   // home stepper motor - stallguard = sgt value
void 		stpr_home			(TMC5130TypeDef *tmc5130, uint16_t homing_speed, uint8_t stallguardthreshold);
void 		stpr_disableDriver	(TMC5130TypeDef *tmc5130);
void 		stpr_enableDriver	(TMC5130TypeDef *tmc5130);
void 		stpr_writeInt		(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value);
int32_t 	stpr_readInt		(TMC5130TypeDef *tmc5130, uint8_t address);
void 		stpr_right			(TMC5130TypeDef *tmc5130, uint32_t velocity);
void 		stpr_left			(TMC5130TypeDef *tmc5130, uint32_t velocity);
void 		stpr_stop			(TMC5130TypeDef *tmc5130);
void 		stpr_moveTo			(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax);
void 		stpr_moveBy			(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax);
void 		stpr_moveAngle		(TMC5130TypeDef *tmc5130, float angle, uint32_t velocityMax);
void 		stpr_setPos			(TMC5130TypeDef *tmc5130, int32_t position);
int32_t 	stpr_getPos			(TMC5130TypeDef *tmc5130);
uint32_t 		stpr_waitMove		(TMC5130TypeDef *tmc5130);
void 		stpr_setCurrent		(TMC5130TypeDef *tmc5130, uint8_t current);
void 		stpr_setVelocity	(TMC5130TypeDef *tmc5130, uint32_t velocity);
void homing_test(TMC5130TypeDef *tmc5130, uint16_t homing_speed, uint8_t stallguardthreshold);
/*
 * PRIVATE FUNCTIONS
 */
void 		tmc5130_writeDatagram	(TMC5130TypeDef *tmc5130, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
void 		tmc5130_rotate			(TMC5130TypeDef *tmc5130, int32_t velocity);


#endif /* TMC_IC_TMC5130_H_ */
