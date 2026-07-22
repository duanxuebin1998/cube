#ifndef LH_REGISTER_MAP_H_
#define LH_REGISTER_MAP_H_

#include <stdint.h>

/* LH 线圈：连续地址 0x0000~0x0010。 */
#define LH_COIL_BACK_ZERO                    0x0000U
#define LH_COIL_FIND_OIL                     0x0001U
#define LH_COIL_FIND_WATER                   0x0002U
#define LH_COIL_FIND_BOTTOM                  0x0003U
#define LH_COIL_FOLLOW_WATER                 0x0004U
#define LH_COIL_CANCEL_MEASUREMENT           0x0005U
#define LH_COIL_MAINTENANCE_ENTER            0x0006U
#define LH_COIL_MAINTENANCE_EXIT             0x0007U
#define LH_COIL_CLEAR_ALL_RELAY_LATCHED      0x0008U
#define LH_COIL_CALIBRATE_OIL                0x0009U
#define LH_COIL_READ_PART_PARAMS             0x000AU
#define LH_COIL_MOVE_UP                      0x000BU
#define LH_COIL_MOVE_DOWN                    0x000CU
#define LH_COIL_CORRECT_OIL                  0x000DU
#define LH_COIL_CALIBRATE_ZERO               0x000EU
#define LH_COIL_CALIBRATE_WATER              0x000FU
#define LH_COIL_CALIBRATE_TANKHEIGHT         0x0010U
#define LH_COIL_COUNT                        0x0011U

/* LH 输入寄存器：连续地址 0x0000~0x001B。 */
#define LH_IR_DEVICE_STATE                   0x0000U
#define LH_IR_ERROR_CODE                     0x0001U
#define LH_IR_OIL_LEVEL                      0x0003U
#define LH_IR_WATER_LEVEL                    0x0005U
#define LH_IR_TANK_BOTTOM_HEIGHT             0x0007U
#define LH_IR_REALTIME_POSITION              0x0009U
#define LH_IR_RELAY_K1_ACTION                0x000BU
#define LH_IR_RELAY_K2_ACTION                0x000CU
#define LH_IR_RELAY_K3_ACTION                0x000DU
#define LH_IR_MAINTENANCE_ACTIVE             0x000EU
#define LH_IR_ANGLE_X                        0x000FU
#define LH_IR_ANGLE_Y                        0x0010U
#define LH_IR_FREQUENCY                      0x0011U
#define LH_IR_TEMPERATURE                    0x0013U
#define LH_IR_AMPLITUDE                      0x0014U
#define LH_IR_WATER_CAPACITANCE_X10          0x0015U
#define LH_IR_MOTOR_STATE                    0x0017U
#define LH_IR_MOTOR_SPEED                    0x0018U
#define LH_IR_CURRENT_TORQUE                 0x001AU
#define LH_INPUT_REGISTER_COUNT              0x001CU

/* LH 保持寄存器通用及电机参数：0x0000~0x0014。 */
#define LH_HR_TANK_HEIGHT                    0x0000U
#define LH_HR_CALIBRATE_OIL_LEVEL            0x0002U
#define LH_HR_CALIBRATE_WATER_LEVEL          0x0004U
#define LH_HR_CALIBRATE_TANK_HEIGHT          0x0006U
#define LH_HR_MOTOR_COMMAND_DISTANCE         0x0008U
#define LH_HR_MOTOR_CURRENT                  0x000AU
#define LH_HR_MAX_MOTOR_SPEED                0x000BU
#define LH_HR_BLIND_ZONE                     0x000DU
#define LH_HR_ENCODER_WHEEL_CIRCUMFERENCE    0x000FU
#define LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE 0x0011U
#define LH_HR_MAX_DOWN_DISTANCE              0x0012U
#define LH_HR_SENSOR_ID                      0x0013U

/* LH AO 参数：0x0015~0x0026。 */
#define LH_HR_AO_WORK_MODE                   0x0015U
#define LH_HR_AO_CURRENT_MODE                0x0016U
#define LH_HR_AO_OUTPUT_SOURCE               0x0017U
#define LH_HR_AO_CURRENT_CORRECTION_MA_X100  0x0018U
#define LH_HR_AO_FIXED_CURRENT_MA_X100       0x001AU
#define LH_HR_AO_RANGE_0_01MM                0x001CU
#define LH_HR_AO_RANGE_100_01MM              0x001EU
#define LH_HR_AO_DAMPING_X10_S               0x0020U
#define LH_HR_AO_FAULT_MODE                  0x0022U
#define LH_HR_AO_FAULT_CURRENT_MA_X100       0x0023U
#define LH_HR_AO_POWER_ON_CURRENT_MA_X100    0x0025U

/* LH K1~K3 参数：每路连续占 0x12 个寄存器。 */
#define LH_HR_RELAY_K1_BASE                  0x0027U
#define LH_HR_RELAY_K2_BASE                  0x0039U
#define LH_HR_RELAY_K3_BASE                  0x004BU
#define LH_HR_RELAY_CHANNEL_REG_COUNT        0x0012U

#define LH_HR_RELAY_OPERATING_MODE(base)     ((base) + 0x0000U)
#define LH_HR_RELAY_DIGITAL_SOURCE(base)     ((base) + 0x0001U)
#define LH_HR_RELAY_CONTACT_TYPE(base)       ((base) + 0x0002U)
#define LH_HR_RELAY_ALARM_MODE(base)         ((base) + 0x0003U)
#define LH_HR_RELAY_ERROR_VALUE(base)        ((base) + 0x0004U)
#define LH_HR_RELAY_ALARM_SOURCE(base)       ((base) + 0x0005U)
#define LH_HR_RELAY_HH_ALARM_VALUE(base)     ((base) + 0x0006U)
#define LH_HR_RELAY_H_ALARM_VALUE(base)      ((base) + 0x0008U)
#define LH_HR_RELAY_L_ALARM_VALUE(base)      ((base) + 0x000AU)
#define LH_HR_RELAY_LL_ALARM_VALUE(base)     ((base) + 0x000CU)
#define LH_HR_RELAY_ALARM_HYSTERESIS(base)   ((base) + 0x000EU)
#define LH_HR_RELAY_DAMPING_FACTOR(base)     ((base) + 0x0010U)

#define LH_HOLDING_REGISTER_COUNT            0x005DU

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(LH_COIL_CALIBRATE_TANKHEIGHT + 1U == LH_COIL_COUNT,
               "LH coil map must remain continuous");
_Static_assert(LH_IR_CURRENT_TORQUE + 2U == LH_INPUT_REGISTER_COUNT,
               "LH input register map must remain continuous");
_Static_assert(LH_HR_AO_POWER_ON_CURRENT_MA_X100 + 2U == LH_HR_RELAY_K1_BASE,
               "LH AO and relay maps must remain continuous");
_Static_assert(LH_HR_RELAY_K2_BASE - LH_HR_RELAY_K1_BASE == LH_HR_RELAY_CHANNEL_REG_COUNT,
               "LH K1 register span must remain continuous");
_Static_assert(LH_HR_RELAY_K3_BASE - LH_HR_RELAY_K2_BASE == LH_HR_RELAY_CHANNEL_REG_COUNT,
               "LH K2 register span must remain continuous");
_Static_assert(LH_HR_RELAY_K3_BASE + LH_HR_RELAY_CHANNEL_REG_COUNT == LH_HOLDING_REGISTER_COUNT,
               "LH K3 register span must remain continuous");
#endif

#endif /* LH_REGISTER_MAP_H_ */
