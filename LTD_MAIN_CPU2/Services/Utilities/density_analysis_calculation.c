/*************************************************************
* 文件名: Density_analysis_calculation.c
* 功能: 進行油品密度相關的計算, 包括:
*       1. 修正密度 RHOT
*       2. 15 度標準密度 RHO15
*       3. 20 度標準密度 RHO20
*       4. 體積修正系數 VCF
*       5. 插值與溫度修正計算
*
* 說明:
*   本文件只做計算, 不涉及硬件操作。
*   所有輸入密度單位為 kg/m3。
*   所有溫度單位為 攝氏度。
*   代碼基於國際油品標準演算法。
*************************************************************/

#include <math.h>
#include "density_analysis_calculation.h"

#define double_t double

static double_t truncd(double_t value, int32_t digits);
static double_t pow1(double_t x, int32_t y);
static double_t roundd5(double_t value, int32_t digits);
static double_t GetRHOT(int32_t oilcategory, double_t density, double_t temperature);
static double_t GetRHO15(int32_t oilcategory, double_t RHOT, double_t temperature);
static double_t GetVcf20(int32_t oilcategory, double_t RHO15, double_t T);
static double_t GetDensity20ofPMP3(int32_t oilcategory, double_t density, double_t temperature);
static double_t GetRHOTOmitHydrometer(int32_t oilcategory, double_t density, double_t temperature);
static double_t GetVCF(int32_t oilcategory, double_t RHO15, double_t T);

/*******************************************************
* Name    truncd
* Brief   按指定位數截斷小數, 不進行四捨五入
* Param   value    輸入數值
*         digits   保留的小數位數
* Return  截斷後的結果
*******************************************************/
static double_t truncd(double_t value, int32_t digits)
{
    int64_t intvalue;
    double_t multiple;
    double_t retvalue;

    multiple = pow1(10.0, digits);
    intvalue = (int64_t)(value * multiple);
    retvalue = (double_t)intvalue * pow1(0.1, digits);

    return retvalue;
}

/*******************************************************
* Name    pow1
* Brief   計算 x 的 y 次方, 只支持非負整數 y
* Param   x   底數
*         y   指數
* Return  x 的 y 次方
*******************************************************/
static double_t pow1(double_t x, int32_t y)
{
    int32_t i;
    double_t z = 1.0;

    for (i = 0; i < y; i++) {
        z *= x;
    }
    return z;
}

/*******************************************************
* Name    roundd
* Brief   根據指定位數進行四捨五入
* Param   value    輸入值
*         digits   保留的小數位數
* Return  四捨五入後的結果
*******************************************************/
double_t roundd(double_t value, int32_t digits)
{
    int64_t intvalue;
    double_t multiple;
    double_t retvalue;

    multiple = pow1(10.0, digits);

    if (value >= 0.0)
        intvalue = (int64_t)(value * multiple + 0.5);
    else
        intvalue = (int64_t)(value * multiple - 0.5);

    retvalue = (double_t)intvalue * pow1(0.1, digits);
    return retvalue;
}

/*******************************************************
* Name    roundd5
* Brief   按 0 和 5 的規則進行四捨五入
* Param   value    輸入值
*         digits   保留的小數位數
* Return  結果為最接近 0 或 5 的小數
*******************************************************/
static double_t roundd5(double_t value, int32_t digits)
{
    int32_t valueint;
    int32_t unit;
    double_t valueroundd;
    double_t multiple;
    double_t retvalue;

    valueroundd = roundd(value, digits);
    multiple = pow1(10.0, digits);
    valueint = (int32_t)(fabs(valueroundd) * multiple);

    unit = valueint % 10;

    if (unit <= 2)
        valueint += (0 - unit);
    else if (unit <= 7)
        valueint += (5 - unit);
    else
        valueint += (10 - unit);

    if (value < 0.0)
        retvalue = -valueint * pow1(0.1, digits);
    else
        retvalue = valueint * pow1(0.1, digits);

    return retvalue;
}

/*******************************************************
* Name    GetRHOT
* Brief   計算 20 度下的密度計修正密度 RHOT
* Param   oilcategory   油品類型
*         density       當前密度
*         temperature   當前溫度
* Return  RHOT 或錯誤碼
*******************************************************/
static double_t GetRHOT(int32_t oilcategory, double_t density, double_t temperature)
{
    int32_t densitymul;
    int32_t temperaturemul;

    double_t TERM1;
    double_t TERM2;
    double_t RHO, T, DELTA20, HYC, RHOT;

    densitymul = (int32_t)(density * 10.0);
    temperaturemul = (int32_t)(temperature * 100.0);

    if (densitymul < 6530 || densitymul > 10750)
        return ERROR_DENSITY;

    if (densitymul < 7780) {
        if (temperaturemul < -10000 || temperaturemul > 9500)
            return ERROR_TEMPERATURE;
    } else if (densitymul < 8240) {
        if (temperaturemul < -10000 || temperaturemul > 12500)
            return ERROR_TEMPERATURE;
    } else {
        if (temperaturemul < -10000 || temperaturemul > 15000)
            return ERROR_TEMPERATURE;
    }

    RHO = roundd5(density, 1);
    T = roundd5(temperature, 2);

    DELTA20 = T - 20.0;

    TERM1 = roundd(0.000023 * DELTA20, 9);
    TERM2 = roundd(0.00000002 * DELTA20 * DELTA20, 9);

    HYC = 1.0 - TERM1 - TERM2;
    RHOT = roundd(RHO * HYC, 2);

    return RHOT;
}

/*******************************************************
* Name    GetRHO15
* Brief   根據 RHOT 和溫度計算 15 度標準密度 RHO15
* Param   oilcategory   油品類型
*         RHOT          修正後密度
*         temperature   當前溫度
* Return  RHO15 或錯誤碼
*******************************************************/
static double_t GetRHO15(int32_t oilcategory, double_t RHOT, double_t temperature)
{
    int32_t Iteratecount;
    int32_t AreaofDensity;
    double_t RH15;
    double_t ALPHA15;
    double_t VCF15;
    double_t A = 0.0;
    double_t B = 0.0;
    double_t K0;
    double_t K1;
    double_t TERM1;
    double_t TERM2;
    double_t TERM3;
    double_t TERM4;
    double_t T;
    double_t DELTA15;
    double_t RHO15;

    RHO15 = RHOT;
    T = roundd5(temperature, 2);
    DELTA15 = T - 15.0;

    Iteratecount = 0;

    while (1) {
        do {
            Iteratecount++;
            if (Iteratecount >= 1000)
                return ERROR_NORESULT;

            RH15 = RHO15;
            RHO15 = roundd(RHO15, 2);

            if (oilcategory == CRUDE) {
                AreaofDensity = DENSITY_AREA1;
                K0 = 613.9723;
                K1 = 0.0;
            } else if (oilcategory == LUBRICATINGOIL) {
                AreaofDensity = DENSITY_AREA1;
                K0 = 0.0;
                K1 = 0.6278;
            } else {
                if (RHO15 >= 770.5 && RHO15 <= 787.5) {
                    AreaofDensity = DENSITY_AB;
                    A = -0.00336312;
                    B = 2680.3206;
                } else if (RHO15 >= 653.0 && RHO15 < 770.5) {
                    AreaofDensity = DENSITY_AREA1;
                    K0 = 346.4228;
                    K1 = 0.4388;
                } else if (RHO15 > 787.5 && RHO15 <= 839.5) {
                    AreaofDensity = DENSITY_AREA2;
                    K0 = 594.5418;
                    K1 = 0.0;
                } else {
                    AreaofDensity = DENSITY_AREA3;
                    K0 = 186.9696;
                    K1 = 0.4862;
                }
            }

            if (AreaofDensity == DENSITY_AB) {
                TERM1 = truncd(B / RHO15, 6);
                TERM2 = roundd(TERM1 / RHO15, 8);
                ALPHA15 = roundd(A + TERM2, 7);
            } else {
                TERM1 = truncd(K0 / RHO15, 8);
                TERM2 = truncd(TERM1 / RHO15, 10);
                TERM3 = truncd(K1 / RHO15, 10);
                ALPHA15 = roundd(TERM2 + TERM3, 7);
            }

            TERM1 = truncd(ALPHA15 * DELTA15, 9);
            TERM2 = truncd(0.8 * TERM1, 9);
            TERM3 = roundd(TERM1 * TERM2, 9);
            TERM4 = truncd(0.0 - TERM1 - TERM3, 8);

            VCF15 = roundd(exp(TERM4), 6);
            RHO15 = truncd(RHOT / VCF15, 3);

        } while (fabs(RH15 - RHO15) >= 0.05);

        break;
    }

    return roundd(RHO15, 2);
}

/*******************************************************
* Name    GetVcf20
* Brief   根據 RHO15 和溫度計算 VCF20
* Param   oilcategory   油品類型
*         RHO15         15 度標準密度
*         T             當前溫度
* Return  VCF20
*******************************************************/
static double_t GetVcf20(int32_t oilcategory, double_t RHO15, double_t T)
{
    double_t RHO15_temp;
    double_t T_temp;
    double_t TERM1;
    double_t TERM2;
    double_t TERM3;
    double_t TERM4;
    double_t TERM5;
    double_t TERM6;

    double_t ALPHA15;
    double_t A;
    double_t B;
    double_t K0;
    double_t K1;
    double_t DELTA20;
    double_t VCF20;

    RHO15_temp = roundd(RHO15, 2);
    T_temp = roundd5(T, 2);

    if (RHO15_temp >= 770.5 && RHO15_temp <= 787.5) {
        A = -0.00336312;
        B = 2680.3206;
        TERM1 = truncd(B / RHO15_temp, 6);
        TERM2 = truncd(TERM1 / RHO15_temp, 8);
        ALPHA15 = roundd(A + TERM2, 7);
    } else {
        if (RHO15_temp >= 653.0 && RHO15_temp < 770.5) {
            K0 = 346.4228;
            K1 = 0.4388;
        } else if (RHO15_temp > 787.5 && RHO15_temp <= 839.5) {
            K0 = 594.5418;
            K1 = 0.0;
        } else {
            K0 = 186.9696;
            K1 = 0.4862;
        }
        TERM1 = truncd(K0 / RHO15_temp, 8);
        TERM2 = truncd(TERM1 / RHO15_temp, 10);
        TERM3 = truncd(K1 / RHO15_temp, 10);
        ALPHA15 = roundd(TERM2 + TERM3, 7);
    }

    DELTA20 = T_temp - 20.0;

    TERM1 = truncd(ALPHA15 * DELTA20, 9);
    TERM2 = truncd(ALPHA15 * TERM1, 9);
    TERM3 = roundd(8.0 * TERM2, 9);
    TERM4 = truncd(0.8 * TERM1, 9);
    TERM5 = roundd(TERM1 * TERM4, 9);
    TERM6 = truncd(0.0 - TERM1 - TERM3 - TERM5, 8);

    VCF20 = roundd(exp(TERM6), 6);

    return VCF20;
}

/*******************************************************
* Name    GetDensity20ofPMP3
* Brief   將任意溫度下的密度換算為 20 度標準密度
* Param   oilcategory   油品類型
*         density       當前密度
*         temperature   當前溫度
* Return  RHO20 或錯誤碼
*******************************************************/
static double_t GetDensity20ofPMP3(int32_t oilcategory, double_t density, double_t temperature)
{
    double_t RHOT;
    double_t RHO15;
    double_t VCF20;
    double_t RHO20;

    RHOT = GetRHOT(oilcategory, density, temperature);
    if (RHOT < 0.0)
        return RHOT;

    RHO15 = GetRHO15(oilcategory, RHOT, temperature);
    if (RHO15 == ERROR_NORESULT)
        return RHO15;

    VCF20 = GetVcf20(oilcategory, RHO15, temperature);

    RHO20 = truncd(RHOT / VCF20, 3);
    return roundd(RHO20, 1);
}

/*******************************************************
* Name    get_standdensity
* Brief   以兩點插值方式計算 20 度標準密度
* Param   rhot         當前溫度下修正後密度
*         temperature  當前溫度
* Return  20 度標準密度
*******************************************************/
double_t get_standdensity(double_t rhot, double_t temperature)
{
    double_t rhot20;
    double_t ref;
    double_t ref_left;
    double_t ref_right;
    double_t ref_delt;
    double_t rhot_delt;

    ref = 601.0;
    while (rhot > ref)
        ref += 2.0;

    ref_left = GetDensity20ofPMP3(1, ref - 2.0, temperature);
    ref_right = GetDensity20ofPMP3(1, ref, temperature);

    ref_delt = ref_right - ref_left;
    rhot_delt = rhot + 2.0 - ref;

    rhot20 = ref_left + ref_delt * rhot_delt * 0.5;
    return rhot20;
}

/*******************************************************
* Name    GetRHOTOmitHydrometer
* Brief   不考慮密度計修正時的 RHOT 計算
* Param   oilcategory   油品類型
*         density       密度
*         temperature   溫度
* Return  RHOT 或錯誤碼
*******************************************************/
static double_t GetRHOTOmitHydrometer(int32_t oilcategory, double_t density, double_t temperature)
{
    int32_t densitymul;
    int32_t temperaturemul;
    double_t RHOT;

    densitymul = (int32_t)(density * 10.0);
    temperaturemul = (int32_t)(temperature * 100.0);

    if (densitymul < 6530 || densitymul > 10750)
        return ERROR_DENSITY;

    if (densitymul < 7780) {
        if (temperaturemul < -10000 || temperaturemul > 9500)
            return ERROR_TEMPERATURE;
    } else if (densitymul < 8240) {
        if (temperaturemul < -10000 || temperaturemul > 12500)
            return ERROR_TEMPERATURE;
    } else {
        if (temperaturemul < -10000 || temperaturemul > 15000)
            return ERROR_TEMPERATURE;
    }

    RHOT = roundd5(density, 1);
    return RHOT;
}

/*******************************************************
* Name    GetVCF
* Brief   計算體積修正系數 VCF
* Param   oilcategory   油品類型
*         RHO15         15 度標準密度
*         T             當前溫度
* Return  VCF20
*******************************************************/
static double_t GetVCF(int32_t oilcategory, double_t RHO15, double_t T)
{
    int32_t AreaofDensity;
    double_t RHO15_temp;
    double_t T_temp;
    double_t TERM1;
    double_t TERM2;
    double_t TERM3;
    double_t TERM4;
    double_t TERM5;
    double_t TERM6;

    double_t ALPHA15;
    double_t A = 0.0;
    double_t B = 0.0;
    double_t K0;
    double_t K1;
    double_t DELTA20;
    double_t VCF20;

    RHO15_temp = roundd(RHO15, 2);
    T_temp = roundd5(T, 2);

    if (RHO15_temp >= 770.0 && RHO15_temp <= 788.0) {
        AreaofDensity = DENSITY_AB;
        A = -0.00336312;
        B = 2680.3206;
    } else {
        if (RHO15_temp >= 653.0 && RHO15_temp < 770.0) {
            AreaofDensity = DENSITY_AREA1;
            K0 = 346.4228;
            K1 = 0.4388;
        } else if (RHO15_temp > 788.0 && RHO15_temp <= 839.0) {
            AreaofDensity = DENSITY_AREA2;
            K0 = 594.5418;
            K1 = 0.0;
        } else {
            AreaofDensity = DENSITY_AREA3;
            K0 = 186.9696;
            K1 = 0.4862;
        }
    }

    if (AreaofDensity == DENSITY_AB) {
        TERM1 = truncd(B / RHO15_temp, 6);
        TERM2 = roundd(TERM1 / RHO15_temp, 8);
        ALPHA15 = roundd(A + TERM2, 7);
    } else {
        TERM1 = truncd(K0 / RHO15_temp, 8);
        TERM2 = truncd(TERM1 / RHO15_temp, 10);
        TERM3 = truncd(K1 / RHO15_temp, 10);
        ALPHA15 = roundd(TERM2 + TERM3, 7);
    }

    DELTA20 = T_temp - 20.0;

    TERM1 = truncd(ALPHA15 * DELTA20, 9);
    TERM2 = truncd(ALPHA15 * TERM1, 9);
    TERM3 = roundd(8.0 * TERM2, 9);
    TERM4 = truncd(0.8 * TERM1, 9);
    TERM5 = roundd(TERM1 * TERM4, 9);
    TERM6 = truncd(0.0 - TERM1 - TERM3 - TERM5, 8);

    VCF20 = exp(TERM6);

    if (VCF20 < 1.0)
        VCF20 = roundd(VCF20, 5);
    else
        VCF20 = roundd(VCF20, 4);

    return roundd(VCF20, 4);
}

/*******************************************************
* Name    GetVCF20ofPMP3
* Brief   計算 20 度的 VCF20
* Param   oilcategory   油品類型
*         RHO20         20 度密度
*         temperature   當前溫度
* Return  VCF20 或錯誤碼
*******************************************************/
double_t GetVCF20ofPMP3(int32_t oilcategory, double_t RHO20, double_t temperature)
{
    double_t RHOT;
    double_t RHO15;
    double_t VCF20;

    RHOT = GetRHOTOmitHydrometer(oilcategory, RHO20, temperature);
    if (RHOT < 0.0)
        return RHOT;

    RHO15 = GetRHO15(oilcategory, RHOT, 20.0);
    if (RHO15 == ERROR_NORESULT)
        return RHO15;

    RHO15 = roundd(RHO15, 1);
    VCF20 = GetVCF(oilcategory, RHO15, temperature);

    return VCF20;
}

/*******************************************************
* Name    DensityT_Get
* Brief   根據 VCF20 和 20 度密度計算增量
* Param   VCF20      體積修正系數 放大一萬倍
*         density20  20 度密度 放大十倍
* Return  Dt         溫度修正後的密度增量
*******************************************************/
double_t DensityT_Get(uint32_t VCF20, uint32_t density20)
{
    double_t vcf20;
    double_t D20;
    double_t Dt;

    vcf20 = (double_t)VCF20 / 10000.0;
    D20 = (double_t)density20 / 10.0;

    if (D20 < 1.1)
        return 0.0;

    Dt = (D20 - 1.1) * vcf20;
    return Dt;
}

/*******************************************************
* Name    RHOTtoRHO
* Brief   將 RHOT 反算成未修正前的密度
* Param   density       RHOT 修正後密度
*         temperature   當前溫度
* Return  RHO 或錯誤碼
*******************************************************/
double_t RHOTtoRHO(double_t density, double_t temperature)
{
    int32_t densitymul;
    int32_t temperaturemul;

    double_t TERM1;
    double_t TERM2;
    double_t RHO, T, DELTA20, HYC, RHOT;

    densitymul = (int32_t)(density * 10.0);
    temperaturemul = (int32_t)(temperature * 100);

    if (densitymul < 6530 || densitymul > 10750)
        return ERROR_DENSITY;

    if (densitymul < 7780) {
        if (temperaturemul < -10000 || temperaturemul > 9500)
            return ERROR_TEMPERATURE;
    } else if (densitymul < 8240) {
        if (temperaturemul < -10000 || temperaturemul > 12500)
            return ERROR_TEMPERATURE;
    } else {
        if (temperaturemul < -10000 || temperaturemul > 15000)
            return ERROR_TEMPERATURE;
    }

    RHOT = roundd(density, 2);
    T = roundd5(temperature, 2);

    DELTA20 = T - 20.0;

    TERM1 = roundd(0.000023 * DELTA20, 9);
    TERM2 = roundd(0.00000002 * DELTA20 * DELTA20, 9);

    HYC = 1.0 - TERM1 - TERM2;

    RHO = roundd(RHOT / HYC, 2);
    return RHO;
}
