#include "TMC5130.h"

#include "assert.h"
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"
#include "fault_manager.h"

#include "sensor.h"//测试用
#include "motor_ctrl.h" //测试用
TMC5130TypeDef stepper = { .home_state = HOME, // 假设 HOMING_IDLE 是归位状态的某个枚举�??
		.prev_home_state = HOME,     // 上一个归位状�??
		.control_type = INTERNAL, // 假设控制类型为位置控�??
		.spi = &hspi2,                      // SPI 句柄
		.drvstat = 0,                       // 驱动器状�??
		.sg_flag = 0,                       // 步进信号标志
		.sg_result = 0,                     // 步进信号结果
		.cs_actual = 0,                     // 当前电流控制�??
		.rampstat = 0,                      // 电机加�??/减�?�?
		.homing_done = 0,                   // 归位是否完成标志

		// 初始化 GPIO 端口和引�??
		.cs_port = GPIOB,                   // 假设 SPI 片�?�引脚在 GPIOB
		.cs_pin = GPIO_PIN_12,              // 片�?�引脚编�??
		.en_port = GPIOD,                   // 假设使能引脚�?? GPIOA �??
		.en_pin = GPIO_PIN_10                // 使能引脚编号
		};

// => SPI wrapper.
uint32_t tmc5130_readArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
void tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
// <= SPI wrapper

// => SPI wrappers

uint32_t tmc5130_readArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length) {
	uint8_t rxBuff[5] = { 0, 0, 0, 0, 0 };

	HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	HAL_SPI_TransmitReceive(tmc5130->spi, data, rxBuff, length, HAL_MAX_DELAY);

	HAL_Delay(10);

	HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);

	return ((rxBuff[1] << 24) | (rxBuff[2] << 16) | (rxBuff[3] << 8) | rxBuff[4]);

}

void tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length) {

	HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	HAL_SPI_Transmit(tmc5130->spi, data, length, HAL_MAX_DELAY);

	HAL_Delay(10);

	HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);

}

// Writes (x1 << 24) | (x2 << 16) | (x3 << 8) | x4 to the given address
void tmc5130_writeDatagram(TMC5130TypeDef *tmc5130, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {
	uint8_t data[5] = { address | TMC5130_WRITE_BIT, x1, x2, x3, x4 };
	tmc5130_writeArray(tmc5130, &data[0], 5);

}

// Write an integer to the given address
void stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value) {
	tmc5130_writeDatagram(tmc5130, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

// Read an integer from the given address
int32_t stpr_readInt(TMC5130TypeDef *tmc5130, uint8_t address) {
	/*address = TMC_ADDRESS(address);

	 // register not readable -> shadow register copy
	 if(!TMC_IS_READABLE(tmc5130->registerAccess[address]))
	 return tmc5130->config->shadowRegister[address];*/

	uint8_t data[5] = { 0, 0, 0, 0, 0 };
	uint32_t dummy = 0;

	data[0] = address;
	dummy = tmc5130_readArray(tmc5130, data, 5);

	data[0] = address;
	dummy = tmc5130_readArray(tmc5130, data, 5);

	return dummy;
}

// Rotate with a given velocity (to the right)
void tmc5130_rotate(TMC5130TypeDef *tmc5130, int32_t velocity) {
	// Set absolute velocity
	stpr_writeInt(tmc5130, TMC5130_VMAX, abs(velocity));
	// Set direction
	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG);
}

// Rotate to the right
void stpr_right(TMC5130TypeDef *tmc5130, uint32_t velocity) {
	tmc5130_rotate(tmc5130, velocity);
}

// Rotate to the left
void stpr_left(TMC5130TypeDef *tmc5130, uint32_t velocity) {
	tmc5130_rotate(tmc5130, -velocity);
}

// Stop moving
void stpr_stop(TMC5130TypeDef *tmc5130) {
	tmc5130_rotate(tmc5130, 0);
}

// Move to a specified position with a given velocity
void stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax) {
	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

	// VMAX also holds the target velocity in velocity mode.
	// Re-write the position mode maximum velocity here.
	stpr_writeInt(tmc5130, TMC5130_VMAX, velocityMax);

	stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
}

// Move by a given amount with a given velocity
// This function will write the absolute target position to *ticks
void stpr_moveBy(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax) {
	// determine actual position and add numbers of ticks to move
	*ticks += stpr_readInt(tmc5130, TMC5130_XACTUAL);

	stpr_moveTo(tmc5130, *ticks, velocityMax);
}

// Translate angle to steps with a resolution of 0.01 degrees
void stpr_moveAngle(TMC5130TypeDef *tmc5130, float angle, uint32_t velocityMax) {

	int32_t position;

	position = (int32_t) ((angle * 51200) / 360);

	//stpr_writeInt(tmc5130, TMC5130_RAMPMODE, (angle >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG)

	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
	// VMAX also holds the target velocity in velocity mode.
	// Re-write the position mode maximum velocity here.
	stpr_writeInt(tmc5130, TMC5130_VMAX, velocityMax);

	stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
}

// Get current stepper position
int32_t stpr_getPos(TMC5130TypeDef *tmc5130) {
	int32_t XActual;
	XActual = stpr_readInt(tmc5130, TMC5130_XACTUAL);

	return XActual;
}

void stpr_disableDriver(TMC5130TypeDef *tmc5130) {
	HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_SET);
}

void stpr_enableDriver(TMC5130TypeDef *tmc5130) {
	HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_RESET);
}

void stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position) {
	stpr_writeInt(tmc5130, TMC5130_XTARGET, position);
	stpr_writeInt(tmc5130, TMC5130_XACTUAL, position);
}

uint32_t stpr_waitMove(TMC5130TypeDef *tmc5130) {
	uint32_t ret;
	while ((stpr_readInt(tmc5130, TMC5130_RAMPSTAT) & 0x400) != 0x400) {
//		printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
		ret = CheckWeightCollision();//检测碰撞
		CHECK_ERROR(ret);
		uint32_t gstat = stpr_readInt(tmc5130, TMC5130_GSTAT);
		if (gstat) {
			// 打印故障信息
			if (gstat & (1 << 0)) {
				printf("芯片自上次读取 GSTAT 以来发生了复位\n");
				stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07); // 清除所有状态位
				continue;
			}

			if (gstat & (1 << 1)) {
				printf("驱动器因过热或短路检测被关闭\n");
				CHECK_ERROR(MOTOR_OVERTEMPERATURE);
			}

			if (gstat & (1 << 2)) {
				printf("charge pump 欠压\n");
				CHECK_ERROR(MOTOR_CHARGE_PUMP_UNDER_VOLTAGE);
			}

			// 可以选择清除故障（可选）
			stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07); // 清除所有状态位
			CHECK_ERROR(MOTOR_UNKNOWN_FEEDBACK);
		}
		HAL_Delay(50);
	}
	return NO_ERROR; // 返回无错误状态
}

void stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current) {
	stpr_writeInt(tmc5130, TMC5130_IHOLD_IRUN, (current << 8) | 0x0007006); //IHOLD_IRUN: IHOLD=6, IRUN=19 (max.current), IHOLDDELAY=6
}
void stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity) {
	stpr_writeInt(tmc5130, VMAX, velocity); //DMAX = 700 Deceleration above V1//10*32  	0.2S
}

void stpr_initStepper(TMC5130TypeDef *tmc5130, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t dir, uint8_t current) {
	uint32_t value = 0;
	tmc5130->spi = spi;
	tmc5130->cs_pin = cs_pin;
	tmc5130->cs_port = cs_port;
	tmc5130->homing_done = 0;
	tmc5130->home_state = HOME;
	tmc5130->prev_home_state = HOME;

	stpr_writeInt(tmc5130, TMC5130_GCONF, 0x0084); 	// GCONF
//	 stpr_writeInt(tmc5130, TMC5130_GCONF, (dir << 4)); 	// GCONF

	stpr_writeInt(tmc5130, TMC5130_TPOWERDOWN, 0x0000000A); 	//TPOWERDOWN=10

//	 stpr_writeInt(tmc5130, TMC5130_IHOLD_IRUN,	(current << 8) | 0x0007006); 	//IHOLD_IRUN: IHOLD=6, IRUN=19 (max.current), IHOLDDELAY=6
//
	// Chopconf settings
	stpr_writeInt(tmc5130, TMC5130_CHOPCONF, 2 << 15 | 		// TBL
			0 << 24 |   	//
			3 << 0 |		// TOFF
			4 << 7 |   	// HEND
			0 << 4);  	// HSTART

	stpr_writeInt(tmc5130, TMC5130_AMAX, 10 * 32); // AMAX for stallGuard homing shall be significantly lower than AMAX for printing
	stpr_writeInt(tmc5130, TMC5130_A1, 20 * 32);	// 0x000003E8);    //A1=1000
	stpr_writeInt(tmc5130, TMC5130_V1, 800 * 32);	// 0x000186A0);    //V1=100000
	stpr_writeInt(tmc5130, TMC5130_D1, 20 * 32);	// must be != 0 !!!!!!!//20*32
	stpr_writeInt(tmc5130, DMAX, 10 * 32); //DMAX = 700 Deceleration above V1//10*32  	0.2S
	stpr_writeInt(tmc5130, TMC5130_VSTOP, 0x00000000A);   // VSTOP=20
	stpr_writeInt(tmc5130, TMC5130_VSTART, 0x000000005);   // VSTART=5
	//	   stpr_writeInt(tmc5130, VMAX, 51200);     //最高速度/目标速度--10000=10hz------VMAX = 200 000    //
	stpr_writeInt(tmc5130, TMC5130_TZEROWAIT, 10000);   //

	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION); //RAMPMODE=0
	stpr_writeInt(tmc5130, TMC5130_XACTUAL, 0x00000000);     //XACTUAL=0
	stpr_writeInt(tmc5130, TMC5130_XTARGET, 0x00000000);     //XTARGET=0
	value = SET_IHOLD(5) | SET_IRUN(current) | SET_IHOLDDELAY(7);      //
	stpr_writeInt(tmc5130, IHOLD_IRUN, value); //初始化驱动电流设置  IHOLDDELAY=6 ，IRUN=31 (max. current)，IHOLD=10
//	   stpr_writeInt(tmc5130, TPOWERDOWN,  0x00000020);   // TPOWERDOWN=10:  ---设置在静止 (stst) 后的延迟时间。电机电流断电  64=1000ms？？ 注在设置回原点时时间？
	stpr_writeInt(tmc5130, READ_ACCESS, 0x00000004); //EN_PWM_MODE=1 e nables stealthChop (with default PWM_CONF)  --全局配置  -启用 stealthChop
//	   stpr_writeInt(tmc5130, TPWM_THRS,   0x000005e8);   // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM  TPWM_THRS = 500产生约35000 =约的切换速度。30RPM
	stpr_writeInt(tmc5130, TPWM_THRS, 0);
	stpr_writeInt(tmc5130, TCOOLTHRS, 0);
	stpr_writeInt(tmc5130, THIGH, 20);
	stpr_writeInt(tmc5130, PWMCONF, 0x000401C8); // PWM_CONF: AUTO=1, 1/1024 Fclk, Switch amplitude limit=200, Grad=1   PWM_CONF：AUTO = 1,1 / 1024 Fclk，开关幅度限制= 200，Grad = 1

//	   stpr_writeInt(tmc5130, RAMPMODE, 0);             //RAMPMODE = 0 (Target position move)    目标位置移动
//	   stpr_writeInt(tmc5130, GCONF, 0x1084 | NORMAL_MOTOR_DIRECTION);	//General Configuration  //常规配置     --运动方向
	stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07); // 清除所有状态位
	stpr_waitMove(&stepper); //2025/9/13 消除第一次上电电机报警
}

/*
 * Working homing routine using stallguard. Optimal values as tested:
 * Current: 23 (can be initialised in stpr_init() of stpr_setCurrent())
 * Stallguardtreshold: 4
 * Homing_speed: 30000
 */

void stpr_home(TMC5130TypeDef *tmc5130, uint16_t homing_speed, uint8_t stallguardthreshold) {
//	int16_t homing_retract = -5000;
	uint16_t stall_speed;
	stall_speed = 16777216 / homing_speed;
	//stall_speed = stall_speed / 16;  // match homing speed to actual microstep speed (at 1/16 microstep)
	stall_speed = stall_speed * 1.10; // Activate stallGuard sligthly below desired homing velocity (provide 10% tolerance)

	stpr_writeInt(tmc5130, TMC5130_GCONF, 0x1080); //stealthchop off for stallguard homing
	stpr_writeInt(tmc5130, TMC5130_COOLCONF, ((stallguardthreshold & 0x7F) << 16)); //sgt <-- Entry the value determined for SGT: lower value=higher sensitivity (lower force for stall detection)
	stpr_writeInt(tmc5130, TMC5130_TCOOLTHRS, stall_speed);	//TCOOLTHRS
	stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x400);	//SWITCH REGISTER
	stpr_writeInt(tmc5130, TMC5130_AMAX, 1000);	//AMAX for stallGuard homing shall be significantly lower than AMAX for printing

	// Set velocity mode in direction to the endstop
	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);	//VELOCITY MODE, direction to the endstop
	stpr_writeInt(tmc5130, TMC5130_VMAX, homing_speed);	//Homing Speed in VMAX

	HAL_Delay(20);

	//While motor is still moving (vzero != 1)
	while ((stpr_readInt(tmc5130, TMC5130_RAMPSTAT) & 0x400) != 0x400)
		;

	// Endstop reached. Reset and retract
	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD);	//HOLD Mode
	stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x0);			//SWITCH REGISTER
	stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);			//Position MODE
	stpr_writeInt(tmc5130, TMC5130_DMAX, 0xFFFF);				//DMAX
	stpr_writeInt(tmc5130, TMC5130_GCONF, 0x1080);	//Turn on stealthchop again
	stpr_writeInt(tmc5130, TMC5130_XACTUAL, 0x0);	//XACTUAL = 0
	stpr_writeInt(tmc5130, TMC5130_XTARGET, 0x0);	//XTARGET = 0

	/*
	 * This code is used for "retract and move to home again"
	 */

	//stpr_writeInt(tmc5130, TMC5130_XTARGET, homing_retract);	//XTARGET = homing_retract
	/*
	 HAL_Delay(20);

	 //While motor is still moving (vzero != 1)
	 while((stpr_readInt(tmc5130, TMC5130_RAMPSTAT) & 0x400) != 0x400);

	 // Endstop reached. Reset and retract
	 stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x0);	//SWITCH REGISTER
	 stpr_writeInt(tmc5130, TMC5130_RAMPMODE, 0x3);	//HOLD Mode
	 stpr_writeInt(tmc5130, TMC5130_GCONF, 0x1080);//Turn on stealthchop again
	 stpr_writeInt(tmc5130, TMC5130_XACTUAL, 0x0);	//XACTUAL = 0
	 stpr_writeInt(tmc5130, TMC5130_XTARGET, 0x0);	//XTARGET = 0
	 stpr_writeInt(tmc5130, TMC5130_RAMPMODE, 0x0);	//Position MODE
	 */

	HAL_Delay(200);
}

// Currently unused homing routine because of the problems with stallguard values.

/*
 * Optimal stallguard value is influenced by velocity and stepper current.
 */

//uint8_t stpr_home(TMC5130TypeDef *tmc5130, uint16_t velocity, uint8_t stallguard)
//{
//
//	/*
//	 * rotate home indefinitely until stallguard triggers
//	 */
//	switch(tmc5130->home_state)
//	{
//
//		case HOME:
//		{
//			//stpr_enableDriver(tmc5130);
//
//			//stpr_writeInt(tmc5130, TMC5130_SWMODE, 0x00);
//
//			stpr_writeInt(tmc5130, TMC5130_COOLCONF, 	((stallguard& 0x7F)<<16));	// sgt <-- Entry the value determined for SGT: lower value=higher sensitivity (lower force for stall detection)
//			stpr_writeInt(tmc5130, TMC5130_TCOOLTHRS,	0xFFFF);				// TCOOLTHRS
//			stpr_writeInt(tmc5130, TMC5130_SWMODE, 		0x400);						// enable SG
//			stpr_writeInt(tmc5130, TMC5130_AMAX, 		1000);						// AMAX for stallGuard homing shall be significantly lower than AMAX for printing
//
//			stpr_readInt(tmc5130, TMC5130_RAMPSTAT);
//
//			stpr_right(tmc5130, velocity);
//
//			HAL_Delay(50);
//
//			tmc5130->prev_home_state = HOME;
//			tmc5130->home_state = WAIT_MOVE;
//
//			break;
//		}
//
//		case RETRACT:
//		{
//			// Endstop reached. Reset and retract
//			//stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD);		//HOLD Mode
//			stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);	//Position MODE
//			stpr_writeInt(tmc5130, TMC5130_XACTUAL, 	0x0);					//XACTUAL = 0
//			stpr_writeInt(tmc5130, TMC5130_SWMODE, 	0x0);					//SWITCH REGISTER
//			stpr_writeInt(tmc5130, TMC5130_VMAX, 	velocity);				//Homing Speed in VMAX
//			stpr_writeInt(tmc5130, TMC5130_DMAX, 	0xFFFF);				//DMAX
//			stpr_writeInt(tmc5130, TMC5130_XTARGET, 	0);					//XTARGET = homing_retract
//
//			tmc5130->prev_home_state = RETRACT;
//			tmc5130->home_state = WAIT_MOVE;
//
//			break;
//
//		}
//		case UNRETRACT:
//		{
//			// Endstop reached. Reset and retract
//			//stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD);		//HOLD Mode
//			stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);	//Position MODE
//			stpr_writeInt(tmc5130, TMC5130_SWMODE, 	0x0);					//SWITCH REGISTER
//			stpr_writeInt(tmc5130, TMC5130_VMAX, 	velocity);				//Homing Speed in VMAX
//			stpr_writeInt(tmc5130, TMC5130_DMAX, 	0xFFFF);				//DMAX
//			stpr_writeInt(tmc5130, TMC5130_XTARGET, 	0);					//XTARGET = homing_retract
//
//
//			tmc5130->prev_home_state = UNRETRACT;
//			tmc5130->home_state = WAIT_MOVE;
//
//			break;
//		}
//
//		case WAIT_MOVE:
//		{
//
//			tmc5130->drvstat = (stpr_readInt(tmc5130, TMC5130_DRVSTATUS) & 0x11F03FF);
//			tmc5130->sg_flag = (tmc5130->drvstat & 0x1000000) >> 24;
//			tmc5130->sg_result = tmc5130->drvstat & 0x3FF;
//			tmc5130->cs_actual = tmc5130->drvstat & 0x1F0000;
//
//			if((stpr_readInt(tmc5130, TMC5130_RAMPSTAT) & 0x400) == 0x400)
//			//if((stpr_readInt(tmc5130, TMC5130_DRVSTATUS) & 0x1000000) >> 24)
//			{
//				stpr_stop(tmc5130);
//				stpr_writeInt(tmc5130, TMC5130_XTARGET, 	0);
//				if(tmc5130->prev_home_state == HOME)
//				{
//					tmc5130->home_state = RETRACT;
//				}
//				else if(tmc5130->prev_home_state == RETRACT)
//				{
//					tmc5130->home_state = UNRETRACT;
//				}
//				else if(tmc5130->prev_home_state == UNRETRACT)
//				{
//					stpr_writeInt(tmc5130, TMC5130_XACTUAL, 	0x0);					//XACTUAL = 0
//					tmc5130->homing_done = 1;
//				}
//			}
//			break;
//		}
//
//	}
//
//	return tmc5130->homing_done;
//}
//
