/*
 * test.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_


void motor_step_up_text(void); // 电机小步进上行测试
void motor_step_down_text(void); // 电机小步进下行测试
void motor_step_text(void); // 电机步进测试
void Test_Params_Storage(void); // 测试参数存储
void Test_ParamEncoder_AB_Backup(void); // A/B双备份回退测试（参数+编码值）
void DSM_V2_Test_AllParams(void) ;// DSM V2 演示函数
void Test_main(void) ; // 测试主函数
void motor_text(void); // 电机测试
#endif /* INC_TEST_H_ */
