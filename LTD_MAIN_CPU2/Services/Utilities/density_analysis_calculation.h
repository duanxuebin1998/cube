#ifndef DEN_H__
/* DEN_H__ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define DEN_H__
#include "main.h"
#define DENSITY_AREA1 1 /* 密度计算区间 1 编号。 */
#define DENSITY_AREA2 2 /* 密度计算区间 2 编号。 */
#define DENSITY_AREA3 3 /* 密度计算区间 3 编号。 */
#define DENSITY_AB	  4 /* 密度计算 A/B 区间编号。 */
enum OilCategoryofDensity20
{
	/* 20 ℃ 标准密度换算采用的油品类别。 */
	CRUDE = 0, /* 原油，使用原油的 20 ℃ 密度换算系数。 */
	PETROLEUMPRODUCTS = 1, /* 石油产品，使用成品油的 20 ℃ 密度换算系数。 */
	LUBRICATINGOIL = 2 /* 润滑油，使用润滑油的 20 ℃ 密度换算系数。 */
};

#define	ERROR_DENSITY		-2.0 /* 密度计算失败返回值：密度异常。 */
#define	ERROR_TEMPERATURE	-3.0 /* 密度计算失败返回值：温度异常。 */
#define	ERROR_NORESULT		-4.0 /* 密度计算失败返回值：无计算结果。 */


/**
 * @brief 根据指定位数进行四舍五入。
 *
 * @param value 待按指定小数位执行四舍五入的浮点数。
 * @param digits 需要保留的非负小数位数。
 * @return 返回保留 digits 位小数的结果；中间值先乘 10^digits，正数加 0.5、负数减 0.5 后截为 int64_t，因此中点按远离 0 的方向舍入。
 */
double_t roundd(double_t value,int32_t  digits);
/**
 * @brief 以两点插值方式计算 20 度标准密度。
 *
 * @param rhot 经过密度计热胀修正的测量温度视密度，单位 kg/m3。
 * @param temperature rhot 对应的测量温度，单位 ℃。
 * @return 返回以相邻 2 kg/m3 参考点计算的线性插值 20 ℃ 标准密度，单位 kg/m3；函数自身不单独校验下层错误哨兵。
 */
double_t get_standdensity(double_t	rhot,double_t temperature);
/**
 * @brief 计算 20 度的 VCF20。
 *
 * RHO20 20 度密度。
 *
 * @param oilcategory 石油产品类别枚举，决定密度与体积修正公式。
 * @param RHO20 20 ℃ 标准密度，单位 kg/m3。
 * @param temperature 需要计算体积修正系数的目标温度，单位 ℃。
 * @return 成功时返回由 20 ℃ 标准密度和目标温度计算的无量纲 VCF20；输入范围或迭代失败时返回 ERROR_DENSITY、ERROR_TEMPERATURE 或
 *         ERROR_NORESULT。
 */
double_t GetVCF20ofPMP3(int32_t  oilcategory,double_t RHO20,double_t temperature);
/**
 * @brief 根据 VCF20 和 20 度密度计算增量。
 *
 * density20 20 度密度 放大十倍。
 *
 * @param VCF20 放大 10000 倍保存的无量纲 VCF20 体积修正系数。
 * @param density20 放大 10 倍保存的 20 ℃ 标准密度。
 * @return density20/10 小于 1.1 时返回 0.0；否则返回 (density20/10 - 1.1) × (VCF20/10000) 得到的密度增量。
 */
double_t DensityT_Get(uint32_t  VCF20,uint32_t  density20);
/**
 * @brief 将 RHOT 反算成未修正前的密度。
 *
 * @param density 密度计热胀修正后的 RHOT，单位 kg/m3。
 * @param temperature RHOT 对应的测量温度，单位 ℃。
 * @return 输入有效时返回反向消除密度计热胀修正并保留两位小数的密度，单位 kg/m3；密度或温度越界时分别返回 ERROR_DENSITY 或 ERROR_TEMPERATURE。
 */
double_t RHOTtoRHO(double_t density,double_t temperature);

#endif
