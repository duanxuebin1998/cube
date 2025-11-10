/*
 * MeasureDensity.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */

#include "measure_density.h"
#include "motor_ctrl.h"
uint32_t SinglePointMeasurement() {
	uint32_t ret = 0;
	ret = motorMoveToPositionOneShot((float) g_deviceParams.singlePointMeasurementPosition / 10.0);
//	CHECK_ERROR(ret);
	g_measurement.device_status.device_state = STATE_SPTESTING;
	while (1) {
		Sensor_Test(); // 传感器测试
		CHECK_COMMAND_SWITCH(ret);
	}

}
uint32_t SinglePointMonitoring() {
	uint32_t ret = 0;
	ret = motorMoveToPositionOneShot((float) g_deviceParams.singlePointMeasurementPosition / 10.0);
//	CHECK_ERROR(ret);
	g_measurement.device_status.device_state = STATE_SPTESTING;
	while (1) {
		Sensor_Test(); // 传感器测试
		CHECK_COMMAND_SWITCH(ret);
	}
}
