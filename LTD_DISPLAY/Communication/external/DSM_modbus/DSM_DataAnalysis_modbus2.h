#ifndef _DSM_DATAANALYSIS_MODBUS2_H
#define _DSM_DATAANALYSIS_MODBUS2_H
//#include <stm32f10x_type.h>
//#include <stm32f10x.h>
#include "DSM_SlaveModbus_modbus2.h"
//#include "comm.h"

#define RETURN_OK 			0
#define RETURN_SLAVEFAIL 		-1
#define RETURN_UNSUPPORTED 	-2
#define RETURN_UNDEFADDRESS -3
#define RETURN_UNDEFDATEORDER -4
#define RETURN_SLAVEBUSY -5

int ResetCoil(void);
void Parameter_Init(void);
void Input_Write(void);
#endif
