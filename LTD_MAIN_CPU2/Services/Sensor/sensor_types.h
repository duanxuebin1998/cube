/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Services\Sensor\sensor_types.h
 * @Description  :
 * @Author       : Aubon
 * @Date         : 2026-08-18 13:59:57
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-08-19 08:49:09
 * Copyright 2026 Aubon, All Rights Reserved.
 * 2026-08-18 13:59:57
 */
/*
 * 模块职责：声明统一传感器服务对外可见、且不依赖具体协议头文件的领域类型。
 * 类型边界：DSM、V3、DM4是互斥物理设备；V4和Safe是DM4内部协议状态。
 * 存储约束：持久化sensorType数值由system_parameter.h定义，本文件不重复定义。
 */
#ifndef SENSOR_TYPES_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_TYPES_H_

/*
 * 模块职责：只声明传感器服务层对外可见的领域类型，避免业务层依赖具体协议头文件。
 * 类型边界：DSM、V3、DM4是互斥的物理传感器；V4和Safe是同一只DM4内部的协议状态。
 * 存储约束：持久化sensorType及其数值仍由system_parameter.h维护，本文件不得重复定义。
 */
typedef enum {
    SENSOR_DM4_PROTOCOL_NOT_APPLICABLE = 0, /* 当前物理传感器不是DM4，协议状态不适用。 */
    SENSOR_DM4_PROTOCOL_V4 = 1, /* DM4当前使用普通V4主动或交互协议。 */
    SENSOR_DM4_PROTOCOL_SAFE_RESERVED = 2 /* 预留的DM4 Safe会话状态，尚未冻结进入流程。 */
} SensorDm4ProtocolMode;

#endif /* SENSOR_TYPES_H_ */
