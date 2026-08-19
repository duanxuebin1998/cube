/*
 * 模块职责：声明UART6在无线AT、DSM、V3、DM4-V4和DM4-Safe之间的互斥所有权。
 * 并发边界：同一所有者可计数式重入，不同所有者不能抢占当前传输。
 * 接口约束：所有权只保护软件访问资格，不替代DMA停止、线路空闲和协议收尾检查。
 */
#ifndef SENSOR_UART6_OWNER_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_UART6_OWNER_H_

#include <stdint.h>

typedef enum {
    SENSOR_UART6_OWNER_NONE = 0, /* UART6当前没有传感器或无线维护所有者。 */
    SENSOR_UART6_OWNER_CH9141_AT, /* CH9141无线模块AT命令占用UART6。 */
    SENSOR_UART6_OWNER_DSM, /* DSM文本协议事务占用UART6。 */
    SENSOR_UART6_OWNER_MULTIPARAM_V3, /* 多参数V3固定帧事务占用UART6。 */
    SENSOR_UART6_OWNER_DM4_V4_ACTIVE, /* DM4-V4主动上报DMA占用UART6。 */
    SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE, /* DM4-V4交互事务占用UART6。 */
    SENSOR_UART6_OWNER_DM4_SAFE /* DM4-Safe控制或快报事务占用UART6。 */
} SensorUart6Owner;

/*
 * 函数用途：以不可抢占方式申请UART6所有权。
 * 调用场景：无线AT、各传感器事务或主动接收操作UART6之前。
 * 关键约束：同一所有者允许嵌套；不同所有者冲突时立即失败，不阻塞也不打印。
 */
uint8_t SensorUart6Owner_Acquire(SensorUart6Owner owner);

/*
 * 函数用途：释放一次与申请配对的UART6所有权。
 * 调用场景：协议事务或DMA停止完成后。
 * 关键约束：所有者不匹配时保持现状；嵌套深度归零才真正释放。
 */
uint8_t SensorUart6Owner_Release(SensorUart6Owner owner);

/*
 * 函数用途：在两个协议所有者之间原子交接UART6。
 * 调用场景：V4主动流和交互窗口无空档切换。
 * 关键约束：只允许最外层深度为1时切换，不执行DMA操作。
 */
uint8_t SensorUart6Owner_Transition(SensorUart6Owner current_owner,
                                    SensorUart6Owner next_owner);

/*
 * 函数用途：读取UART6当前所有者。
 * 调用场景：回调过滤和诊断查询。
 * 关键约束：瞬时快照不能替代Acquire授权。
 */
SensorUart6Owner SensorUart6Owner_Get(void);

/*
 * 函数用途：判断UART6是否由指定协议持有。
 * 调用场景：DMA回调、停止收尾和模式检查。
 * 关键约束：不验证DMA状态和嵌套深度。
 */
uint8_t SensorUart6Owner_Is(SensorUart6Owner owner);

#endif /* SENSOR_UART6_OWNER_H_ */
