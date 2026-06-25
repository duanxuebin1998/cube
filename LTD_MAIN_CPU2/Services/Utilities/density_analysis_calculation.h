#ifndef DEN_H__
#define DEN_H__
#include "main.h"
#define DENSITY_AREA1 1 /* 密度计算区间 1 编号。 */
#define DENSITY_AREA2 2 /* 密度计算区间 2 编号。 */
#define DENSITY_AREA3 3 /* 密度计算区间 3 编号。 */
#define DENSITY_AB	  4 /* 密度计算 A/B 区间编号。 */
enum OilCategoryofDensity20
{
	CRUDE = 0,
	PETROLEUMPRODUCTS = 1,
	LUBRICATINGOIL = 2
};

#define	ERROR_DENSITY		-2.0 /* 密度计算失败返回值：密度异常。 */
#define	ERROR_TEMPERATURE	-3.0 /* 密度计算失败返回值：温度异常。 */
#define	ERROR_NORESULT		-4.0 /* 密度计算失败返回值：无计算结果。 */


/**
 * @brief 执行密度换算中的 roundd 逻辑。
 *
 * @param value 待处理数值。
 * @param digits 业务参数。
 * @return 计算后的业务数值。
 */
double_t roundd(double_t value,int32_t  digits);
/**
 * @brief 读取密度换算中的 get_standdensity 逻辑。
 *
 * @param rhot 业务参数。
 * @param temperature 业务参数。
 * @return 计算后的业务数值。
 */
double_t get_standdensity(double_t	rhot,double_t temperature);
/**
 * @brief 读取密度换算中的 GetVCF20ofPMP3 逻辑。
 *
 * @param oilcategory 业务参数。
 * @param RHO20 业务参数。
 * @param temperature 业务参数。
 * @return 计算后的业务数值。
 */
double_t GetVCF20ofPMP3(int32_t  oilcategory,double_t RHO20,double_t temperature);
/**
 * @brief 执行密度换算中的 DensityT_Get 逻辑。
 *
 * @param VCF20 业务参数。
 * @param density20 业务参数。
 * @return 计算后的业务数值。
 */
double_t DensityT_Get(uint32_t  VCF20,uint32_t  density20);
/**
 * @brief 执行密度换算中的 RHOTtoRHO 逻辑。
 *
 * @param density 业务参数。
 * @param temperature 业务参数。
 * @return 计算后的业务数值。
 */
double_t RHOTtoRHO(double_t density,double_t temperature);

#endif
