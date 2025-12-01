#ifndef DEN_H__
#define DEN_H__
#include "main.h"
#define DENSITY_AREA1 1
#define DENSITY_AREA2 2
#define DENSITY_AREA3 3
#define DENSITY_AB	  4
enum OilCategoryofDensity20
{
	CRUDE = 0,
	PETROLEUMPRODUCTS = 1,
	LUBRICATINGOIL = 2
};

#define	ERROR_DENSITY		-2.0
#define	ERROR_TEMPERATURE	-3.0
#define	ERROR_NORESULT		-4.0


double_t roundd(double_t value,int32_t  digits);
double_t get_standdensity(double_t	rhot,double_t temperature);
double_t GetVCF20ofPMP3(int32_t  oilcategory,double_t RHO20,double_t temperature);
double_t DensityT_Get(uint32_t  VCF20,uint32_t  density20);
double_t RHOTtoRHO(double_t density,double_t temperature);

#endif
