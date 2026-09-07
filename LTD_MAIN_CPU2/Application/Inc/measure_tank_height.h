/**
 * @file measure_tank_height.h
 * @brief 罐底搜索、陀螺仪基准和最近罐底尺带参考的 Application 公共接口。
 *
 * 罐底检测枚举、陀螺仪采样结构和搜索阶段函数均属于实现细节，不在公共头文件公开。
 */

#ifndef INC_MEASURE_TANK_HEIGHT_H_
#define INC_MEASURE_TANK_HEIGHT_H_

#include <stdint.h>

/* SearchBottom 与 SI 复用同一个称重探底释放余量，单位 0.1 mm。 */
#define BOTTOM_WEIGHT_RELEASE_MARGIN_01MM 2000
/* 角度探底成功后的历史上提距离；SI 按相同模式恢复释放位置。 */
#define BOTTOM_GYRO_RELEASE_MARGIN_01MM 1000

/** 设备参数 bottom_detect_mode 使用的罐底检测依据。 */
typedef enum {
    BOTTOM_DET_BY_WEIGHT = 0, /* 使用扭力变化判定探头到达罐底。 */
    BOTTOM_DET_BY_GYRO = 1    /* 使用陀螺仪角度变化判定探头到达罐底。 */
} BottomDetectMode;

/**
 * @brief 执行粗找和两次精找组成的完整罐底搜索流程。
 * @return NO_ERROR 表示罐底位置和罐高结果均已更新；命令切换或硬件错误原样返回。
 */
uint32_t SearchBottom(void);

/**
 * @brief 采集并保存探底流程使用的陀螺仪零位参考。
 * @return NO_ERROR 表示稳定参考已保存；离散度超限或传感器错误返回对应错误码。
 */
uint32_t Bottom_SaveGyroZeroRef(void);

/**
 * @brief 读取最近一次可信罐底对应的尺带长度。
 * @return 罐底尺带长度，单位 0.1 mm；尚未建立时返回负的内部无效值。
 */
int32_t TankHeight_GetBottomCableLength01mm(void);

/**
 * @brief 更新罐底尺带参考并同步给电机丢步检测模块。
 * @param cable_length_01mm 罐底对应的尺带长度，单位 0.1 mm。
 */
void TankHeight_SetBottomCableLength01mm(int32_t cable_length_01mm);

#endif /* INC_MEASURE_TANK_HEIGHT_H_ */
