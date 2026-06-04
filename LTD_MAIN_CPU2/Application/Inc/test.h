/*
 * test.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include <stdint.h>


void motor_step_up_text(void); // 电机小步进上行测试
void motor_step_down_text(void); // 电机小步进下行测试
void motor_step_text(void); // 电机步进测试
void Test_Params_Storage(void); // 测试参数存储
void Test_ParamEncoder_AB_Backup(void); // A/B双备份回退测试（参数+编码值）
void DSM_V2_Test_AllParams(void) ;// DSM V2 演示函数
void SensorWireless_CommTest(void); // 传感器与无线通信综合测试
void Test_main(void) ; // 测试主函数
void motor_text_manual_stop(void); // A指令低检测停止
void motor_text_manual_once(float run_distance_mm, int dir); // A指令低检测单段运动
void motor_text(float run_distance_mm, uint8_t enable_sensor_comm); // 电机测试
void motor_text_encoder(float run_distance_mm, uint8_t enable_sensor_comm, uint32_t speed_x100, uint32_t accel_multiplier); // encoder-based mm motor test
void Test_TMC5130_SPI_Static(void); // TMC5130静态SPI通信测试
void Demo_SinglePointDisplayMock(void); // 单点测量展示（虚拟数据）
#endif /* INC_TEST_H_ */
