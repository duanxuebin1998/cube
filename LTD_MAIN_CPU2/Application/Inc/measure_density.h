/*
 * measure_density.h
 *
 *  Created on: Nov 8, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_DENSITY_H_
/* INC_MEASURE_DENSITY_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_DENSITY_H_

#include "app_main.h"
/* ===================== 配置宏 ===================== */

/* 单次测量最多点数 */
#ifndef MAX_MEASUREMENT_POINTS
#define MAX_MEASUREMENT_POINTS  200 /* 密度分布测量最大点数。 */
#endif

/* 国标分段阈值（单位：0.1mm） */
#ifndef Spread_Gb_Onepiont_Posion
/* 密度分布流程的第一参考位置 3.000 m，线值单位为 0.01 mm；保留历史拼写以兼容现有代码。 */
#define Spread_Gb_Onepiont_Posion   (30000u)  /* 3m */
#endif

#ifndef Spread_Gb_Twopiont_Posion
/* 密度分布流程的第二参考位置 4.500 m，线值单位为 0.01 mm；保留历史拼写以兼容现有代码。 */
#define Spread_Gb_Twopiont_Posion   (45000u)  /* 4.5m */
#endif

/* 每米测步距（单位：0.1mm） */
#ifndef METER_STEP_01MM
/* 密度测量默认移动步距 1.000 m，线值单位为 0.01 mm。 */
#define METER_STEP_01MM             (10000u)  /* 1m */
#endif



typedef enum {
    DENS_MODE_SPREAD  = 1, /* 普通分布测 */
    DENS_MODE_GB      = 3, /* 国标测 */
    DENS_MODE_METER   = 4, /* 每米测 */
    DENS_MODE_INTERVAL= 5, /* 区间测 */
} DensitySpreadModeId;

/* ===================== 对外接口：四种模式入口 ===================== */
/**
 * @brief 执行普通密度分布测量，并仅在完整成功后打印和发布新的标准点阵。
 *
 * 开始时清除上一轮完成锁存并将设备状态置为普通分布测量中，再调用 Density_MeasureByMode_Exact 生成临时结果。
 * 测量完整成功后打印结果，通过 PROFILE_SOURCE_STANDARD 发布整份点阵及新代际，最后发布普通分布测量完成状态。
 *
 * @note STATE_SWITCH 是新命令触发的正常退出，不发布候选点阵也不进入错误报警；其他失败通过 SET_ERROR 进入统一处理。
 */
void CMD_MeasureDensitySpread_Spread(void);

/**
 * @brief 执行国标密度分布测量；仅在完整成功后发布点阵和完成态。
 *
 * 国标测。
 */
void CMD_MeasureDensitySpread_GB(void);

/**
 * @brief 执行每米密度分布测量；仅在完整成功后发布点阵和完成态。
 *
 * 每米测。
 */
void CMD_MeasureDensitySpread_Meter(void);

/**
 * @brief 执行区间密度分布测量；仅在完整成功后发布点阵和完成态。
 *
 * 区间测。
 */
void CMD_MeasureDensitySpread_Interval(void);
/**
 * @brief 执行 SI 独立 profile 测量。
 *
 * 调用场景：CPU3 SI Profile 线圈或自动调度下发 CMD_SI_PROFILE 后由命令分发调用。
 * 函数清空上一轮 SI 候选结果，复位进度并进入准备阶段，从独立 SI 参数区取得首点、点距、驻留时间和探底周期；这些参数不复用普通密度分布测量配置。
 * 到达探底周期时先执行 SearchBottom；命令切换立即取消本轮，其他探底错误只记录诊断，并允许点阵生成按已有罐底参考或当前位置继续。
 * 点阵生成成功后逐点移动、驻留并采集到局部候选结构；生成或测量失败会清除候选、发布错误并退出，未完成的候选不会成为对外最终结果。
 * 采点收尾与命令排队共用关中断临界区：若已有新命令或非回液位命令，本轮候选立即取消；否则锁存候选、更新进度阶段，并在命令槽为空时排队 CMD_FIND_OIL。
 * 函数结束时只进入“等待回液位”阶段；真正的 SI 最终完成快照必须等回到稳定液位且电机停止后由后续流程发布。
 *
 * SI 独立 Profile。
 *
 * @note 关键约束：不复用普通分布测 profile 参数；失败不锁存完成态，命令切换直接退出。
 * @note 候选锁存和回液位命令排队期间以新到命令为最高优先级，不得覆盖用户已经送达的其他命令。
 */
void CMD_SiProfile(void);

/**
 * @brief 按当前 SI Profile 阶段处理显式取消或命令切换。
 *
 * @details 调用场景：取消命令入口，以及 SI Profile 相关流程返回 STATE_SWITCH 时。
 *
 * SI Profile取消、失败和回液位候选提交钩子。
 *
 * @note 关键约束：Point0 前保留旧结果；Point0 后清除不完整载荷且不增加完成计数。
 */
void SiProfile_HandleCancel(void);
/**
 * @brief 按当前 SI Profile 阶段处理真实测量失败。
 *
 * @details 调用场景：SI 采点错误，以及回液位流程的真实错误出口。
 * @note 关键约束：Point0 前保留旧结果；Point0 后清除不完整载荷并保留原错误上报链路。
 */
void SiProfile_HandleFailure(void);
/**
 * @brief 回到稳定液位后一次性提交 SI Profile 候选结果。
 *
 * @details 调用场景：找液位成功且完成可选位置源切换后的二次定位，在进入液位跟随前。
 * @note 关键约束：完整载荷、Complete 和阶段先写完，内存屏障后最后递增完成计数。
 *
 * @return NO_ERROR 表示回到稳定液位后一次性提交 SI Profile 候选结果已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t SiProfile_CompleteAfterReturnToLevel(void);

/* ===================== 公共工具接口（本文件 .c 内实现，可能被其他模块复用） ===================== */
/**
 * @brief 打印密度分布结果的测点数、汇总值以及逐点位置、密度和温度。
 *
 * 打印分布测量结果（含点表与平均值）。
 *
 * @param dist 待打印的密度分布结果；包含测点数、液位或罐高、汇总密度温度和最多 MAX_MEASUREMENT_POINTS 个单点数据，传入 NULL 时只打印空结果提示。
 */
void Print_DensitySpreadResult(const DensityDistribution *dist);

/**
 * @brief 读取单点密度、温度和频率并写入测量结果。
 *
 * 单点读数：内部完成稳定判定并写入 result。
 *
 * @param result 单点测量输出记录；读取成功时写入频率、密度、温度及当前位置等字段。
 * @return NO_ERROR 表示读取单点密度、温度和频率并写入测量结果已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result);

/**
 * @brief 把完整的单点测量候选交给固定点原子发布器，并递增测量完成代际。
 *
 * @details 调用场景：真实稳定测量和显式串口虚拟展示取得完整六字段后。
 *
 * 函数用途：发布完整单点测量六字段，并在同一临界区递增测量完成代际。
 * 返回值：1 表示发布成功，0 表示参数无效或命令切换期间拒绝发布。
 *
 * @note 关键约束：命令切换期间拒绝发布，调用方必须处理返回值。
 *
 * @param candidate 待校验或比较的候选值。该只读单点测量包含频率、密度、温度和位置，只有稳定性与有效性检查通过后才发布。
 * @return 1 表示单点测量候选已原子发布且测量完成代际已递增；候选非法或发布失败时返回 0。
 */
uint8_t SinglePoint_PublishMeasurementResult(const DensityMeasurement *candidate);

/**
 * @brief 把完整的固定点监测候选交给固定点原子发布器，并递增监测样本代际。
 *
 * @details 调用场景：真实稳定监测和显式串口虚拟展示取得完整六字段后。
 *
 * 函数用途：发布完整固定点监测六字段，并在同一临界区递增监测样本代际。
 * 返回值：1 表示发布成功，0 表示参数无效或命令切换期间拒绝发布。
 *
 * @note 关键约束：命令切换期间拒绝发布，调用方必须处理返回值。
 *
 * @param candidate 待校验或比较的候选值。该只读单点测量包含频率、密度、温度和位置，只有稳定性与有效性检查通过后才发布。
 * @return 1 表示固定点监测候选已原子发布且监测样本代际已递增；候选非法或发布失败时返回 0。
 */
uint8_t SinglePoint_PublishMonitoringResult(const DensityMeasurement *candidate);

/**
 * @brief 开始一轮分布点阵测量并关闭上一轮CPU2完成锁存。
 *
 * @details 调用场景：普通、国标、每米、区间、瓦锡兰和综合测量进入测量态前。
 *
 * 函数用途：开始一轮分布点阵测量，关闭上一轮完成锁存但保留完成计数。
 * 关键约束：CPU3继续保留上一份已确认快照，新结果完整发布后才开放新代际。
 *
 * @note 关键约束：完成计数保持不变，CPU3继续使用上一份已确认快照。
 */
void DensityProfile_Begin(void);

/**
 * @brief 把完整分布点阵、来源和完成锁存按同一代际发布。
 *
 * @details 调用场景：六种分布类测量取得完整临时候选结果后。
 *
 * 函数用途：统一发布普通、国标、每米、区间、瓦锡兰和综合测量点阵。
 * 关键约束：完整点阵、来源和完成锁存先写入，完成计数在内存屏障后最后递增。
 *
 * @note 关键约束：候选结构写完并执行屏障后才递增完成计数，CPU3以计数沿启动整阵复核。
 *
 * @param candidate 待校验或比较的候选值。该可写对象承载本轮完整密度分布、测点数和关联结果，校验通过后才发布到共享测量快照。
 * @param source 本轮密度剖面结果来源枚举；用于标记标准、国标、密度计或其它测量流程，供发布快照和后续显示选择。
 */
void DensityProfile_PublishResult(DensityDistribution *candidate, ProfileSource source);

/**
 * @brief 按指定分布测量模式执行密度测量并输出完整点表。
 * @param mode 分布测量模式。
 * @param out_dist 测量结果输出结构。
 * @return NO_ERROR 表示测量完成，其他值表示测量或传感器链路异常。
 */
uint32_t Density_MeasureByMode_Exact(DensitySpreadModeId mode, DensityDistribution *out_dist);
/**
 * @brief 检查单点测量目标位置是否在当前罐高和安全范围内。
 * @param scene 调用场景名称，用于打印现场诊断信息。
 * @param target_01mm 目标位置，单位 0.1mm。
 * @return NO_ERROR 表示目标有效，其他值表示参数或位置越界。
 */
uint32_t SinglePoint_CheckTargetPosition(const char *scene, uint32_t target_01mm);
/**
 * @brief 单点测量命令：移动到指定高度 -> 单点稳定读取。
 */
void CMD_SinglePointMeasurement();
/**
 * @brief 单点监测命令：移动到监测高度 -> 循环单点稳定读取（直到命令切换）。
 */
void CMD_SinglePointMonitoring();

#endif /* INC_MEASURE_DENSITY_H_ */
