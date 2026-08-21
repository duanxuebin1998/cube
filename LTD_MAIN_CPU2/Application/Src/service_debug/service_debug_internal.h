/**
 * @file service_debug_internal.h
 * @brief test 目录内部维护测试接口，不属于 Application 公共契约。
 */

#ifndef APPLICATION_SERVICE_DEBUG_INTERNAL_H
#define APPLICATION_SERVICE_DEBUG_INTERNAL_H

#include <stdint.h>

/* 判断维护测试是否收到有效命令切换；检测到时慢停电机并返回 1。 */
uint8_t Test_ShouldAbortForCommandSwitch(void);

/* 执行一次不改变正式错误态的传感器通信检查并打印结果。 */
void Test_SensorCommCheckAndPrintOnly(const char *tag);

/** 处理 V3 传感器维护命令；识别并消费命令时返回 1，否则返回 0。 */
uint8_t TestCommand_HandleV3(const uint8_t *command);

/** 处理 V4 传感器维护命令；识别并消费命令时返回 1，否则返回 0。 */
uint8_t TestCommand_HandleV4(const uint8_t *command);

/** 执行水位探头上下行电容剖面维护测试，返回整机错误码。 */
uint32_t WaterSensorCapacitanceProfileTest(void);

/**
 * @brief 以 4×32 微步为单次增量执行长时间上行测试，并周期打印位置参考与扭力采样。
 *
 * 启用步进驱动后，每轮向上移动 -4×32 微步，等待两秒，再依次打印循环序号、传感器位置、电机位置参考和三次扭力读数。
 *
 * @note 该维护测试最多循环 24000 次并真实驱动电机；命令切换会立即返回，当前提前返回路径不会执行末尾的驱动关闭。
 */
void motor_step_up_text(void); /* 电机小步进上行测试 */
/**
 * @brief 以 4×32 微步为单次增量执行长时间下行测试，并周期打印位置参考与扭力采样。
 *
 * 启用步进驱动后，每轮向下移动 4×32 微步，等待两秒，再依次打印循环序号、传感器位置、电机位置参考和三次扭力读数。
 *
 * @note 该维护测试最多循环 24000 次并真实驱动电机；命令切换会立即返回，当前提前返回路径不会执行末尾的驱动关闭。
 */
void motor_step_down_text(void); /* 电机小步进下行测试 */
/**
 * @brief 依次按 4、8 和 40 个整步的细分脉冲执行往返耐久测试，并持续打印编码器与扭力数据。
 *
 * 每种步长先按正脉冲方向运行固定总行程，再按负脉冲方向返回；步长增大时相应减少循环次数，使三个阶段的累计脉冲量一致。
 * 每次动作前后检查命令切换，动作后等待两秒并分三次输出循环序号、编码器计数和当前扭力，全部阶段完成后关闭步进驱动。
 *
 * @note 该维护测试会真实、长时间驱动电机；每次移动后等待并连续采集编码器与扭力。收到命令切换时会立即返回，当前提前返回路径不会执行函数末尾的驱动关闭。
 */
void motor_step_text(void); /* 电机步进测试 */
/**
 * @brief 写入测试罐高并回读校验设备参数存储，最后恢复原始参数。
 *
 * @note 该维护测试会真实写入 FRAM 参数区两次；仅允许在受控测试环境执行，中途掉电可能使测试值保留到下次启动。
 */
void Test_Params_Storage(void); /* 测试参数存储 */
/**
 * @brief 验证参数区和编码器位置区的 FRAM A/B 双备份回退与全损坏报错逻辑。
 *
 * 测试开始时完整备份参数 A/B 分区、编码器 A/B 测试分区以及当前 g_deviceParams，随后通过破坏 magic 字段模拟单分区和双分区损坏。
 * 参数区分别验证 A 损坏时能回退到 B，以及 A/B 均损坏时必须返回 PARAM_UNINITIALIZED；编码器区分别验证 A 损坏时能从 B 恢复，以及 A/B 均损坏时必须发布
 * ENCODER_POWERON_FAIL。
 * 参数测试后恢复两个原始分区并重新保存当前参数，编码器测试后恢复两个原始分区并重新初始化编码器；每个用例只通过打印报告结果，不返回汇总状态。
 *
 * @note 该测试会真实改写 FRAM；若测试在恢复步骤前掉电或被复位，参数区或编码器位置区可能保持故意制造的损坏状态，只能在可恢复的维护环境执行。
 */
void Test_ParamEncoder_AB_Backup(void); /* A/B双备份回退测试（参数+编码值） */
/**
 * @brief  测试多参数传感器通信协议 V3.0通讯与关键参数读取
 * @note   可在初始化完成后调用，例如 main() 或 sensor init 后
 */
void MULTIPARAM_V3_Test_AllParams(void) ; /* 多参数传感器通信协议 V3.0 演示函数 */
/**
 * @brief  传感器与蓝牙链路综合通信测试
 * @note   手动调试入口，建议在系统初始化完成后临时调用；函数会执行传感器识别，
 *         并刷新 g_deviceParams.sensorType/sensorID，正式流程中不要周期性调用。
 */
void SensorWireless_CommTest(void); /* 传感器与无线通信综合测试 */
/**
 * @brief 依次执行 FRAM 读写、设备参数存储和硬件 CRC32 调试测试。
 */
void Test_main(void) ; /* 测试主函数 */
/**
 * @brief A指令测试专用：只下发停止寄存器，不判断扭力、编码器或驱动错误。
 * @note  只在任务上下文调用；用于串口低检测调试，退出时恢复进入前错误状态。
 */
void motor_text_manual_stop(void); /* A指令低检测停止 */
/**
 * @brief A指令测试专用：按指定方向执行一段低检测运动。
 * @note  运动期间只响应命令切换；不读取扭力、编码器错误或全局错误退出。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 */
void motor_text_manual_once(float run_distance_mm, int dir); /* A指令低检测单段运动 */
/**
 * @brief BJ 指令测试专用：初始化电机后直接调用点动相对运动正式接口。
 * @note  该函数不绕过 MotorCtrl_JogMoveAndWait 内部检测，用于现场验证新长距离点动控制方案。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 */
void motor_jog_text(float run_distance_mm, int dir, uint32_t speed_x100); /* BJ指令点动相对运动测试 */
/**
 * @brief BJP 指令测试专用：初始化电机后直接调用点动绝对位置正式接口。
 * @note  用于验证目标位置、提前降速、越界保护和命令切换等正式 API 行为。
 *
 * @param target_mm 目标位置，单位 mm。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 */
void motor_jog_to_position_text(float target_mm, uint32_t speed_x100); /* BJP指令点动绝对位置测试 */
/**
 * @brief 按指定距离连续执行下行与回零往返电机测试，并可在每个行程后检查传感器通信。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 */
void motor_text(float run_distance_mm, uint8_t enable_sensor_comm); /* 电机测试 */
/**
 * @brief 以固定编码器原点和下行目标连续往返测试，支持运动中传感器通信、超时或提前停稳重启及命令切换恢复。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @param accel_multiplier BE 调试使用的加速度倍率，非法值会限制到允许档位。
 */
void motor_text_encoder(float run_distance_mm, uint8_t enable_sensor_comm, uint32_t speed_x100, uint32_t accel_multiplier); /* encoder-based mm motor test */
/**
 * @brief TMC5130 静态 SPI 通信测试。
 *
 * 不启动电机，只重复读取 GSTAT/DRV_STATUS/IOIN/IFCNT，用于判断静止状态下
 * SPI 是否仍有 0xFFFFFFFF、0x00FFFFFF、0x00000100 等非法读数。
 */
void Test_TMC5130_SPI_Static(void); /* TMC5130静态SPI通信测试 */
/**
  * @brief 不驱动电机也不读取传感器；先模拟接近测量点，再持续发布带小幅波动的单点显示数据，直至新命令打断。
 */
void Demo_SinglePointDisplayMock(void); /* 单点测量展示（虚拟数据） */
#endif /* APPLICATION_SERVICE_DEBUG_INTERNAL_H */
