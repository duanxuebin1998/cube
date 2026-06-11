/*
 * test.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include <stdint.h>


/**
 * @brief 执行本模块中的 motor_step_up_text 逻辑。
 */
void motor_step_up_text(void); /* 电机小步进上行测试 */
/**
 * @brief 执行本模块中的 motor_step_down_text 逻辑。
 */
void motor_step_down_text(void); /* 电机小步进下行测试 */
/**
 * @brief 执行本模块中的 motor_step_text 逻辑。
 */
void motor_step_text(void); /* 电机步进测试 */
/**
 * @brief 执行本模块中的 Test_Params_Storage 逻辑。
 */
void Test_Params_Storage(void); /* 测试参数存储 */
/**
 * @brief 执行本模块中的 Test_ParamEncoder_AB_Backup 逻辑。
 */
void Test_ParamEncoder_AB_Backup(void); /* A/B双备份回退测试（参数+编码值） */
/**
 * @brief 执行本模块中的 DSM_V2_Test_AllParams 逻辑。
 */
void DSM_V2_Test_AllParams(void) ; /* DSM V2 演示函数 */
/**
 * @brief 执行本模块中的 SensorWireless_CommTest 逻辑。
 */
void SensorWireless_CommTest(void); /* 传感器与无线通信综合测试 */
/**
 * @brief 执行本模块中的 Test_main 逻辑。
 */
void Test_main(void) ; /* 测试主函数 */
/**
 * @brief 执行本模块中的 motor_text_manual_stop 逻辑。
 */
void motor_text_manual_stop(void); /* A指令低检测停止 */
/**
 * @brief 执行本模块中的 motor_text_manual_once 逻辑。
 *
 * @param run_distance_mm 业务参数。
 * @param dir 业务参数。
 */
void motor_text_manual_once(float run_distance_mm, int dir); /* A指令低检测单段运动 */
/**
 * @brief 执行本模块中的 motor_jog_text 逻辑。
 *
 * @param run_distance_mm 业务参数。
 * @param dir 业务参数。
 * @param speed_x100 业务参数。
 */
void motor_jog_text(float run_distance_mm, int dir, uint32_t speed_x100); /* BJ指令点动相对运动测试 */
/**
 * @brief 执行本模块中的 motor_jog_to_position_text 逻辑。
 *
 * @param target_mm 业务参数。
 * @param speed_x100 业务参数。
 */
void motor_jog_to_position_text(float target_mm, uint32_t speed_x100); /* BJP指令点动绝对位置测试 */
/**
 * @brief 执行本模块中的 motor_text 逻辑。
 *
 * @param run_distance_mm 业务参数。
 * @param enable_sensor_comm 业务参数。
 */
void motor_text(float run_distance_mm, uint8_t enable_sensor_comm); /* 电机测试 */
/**
 * @brief 执行本模块中的 motor_text_encoder 逻辑。
 *
 * @param run_distance_mm 业务参数。
 * @param enable_sensor_comm 业务参数。
 * @param speed_x100 业务参数。
 * @param accel_multiplier 业务参数。
 */
void motor_text_encoder(float run_distance_mm, uint8_t enable_sensor_comm, uint32_t speed_x100, uint32_t accel_multiplier); /* encoder-based mm motor test */
/**
 * @brief 执行本模块中的 Test_TMC5130_SPI_Static 逻辑。
 */
void Test_TMC5130_SPI_Static(void); /* TMC5130静态SPI通信测试 */
/**
 * @brief 显示或打印本模块中的 Demo_SinglePointDisplayMock 逻辑。
 */
void Demo_SinglePointDisplayMock(void); /* 单点测量展示（虚拟数据） */
/**
 * @brief 处理本模块中的 Test_ProcessSerialCommand 逻辑。
 *
 * @param command 命令值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t Test_ProcessSerialCommand(uint8_t *command);
#endif /* INC_TEST_H_ */
