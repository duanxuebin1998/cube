#ifndef RELAY_OUTPUT_H_
#define RELAY_OUTPUT_H_

#include <stdint.h>
#include "system_parameter.h"

typedef enum {
    RELAY_OUTPUT_1 = 0U,
    RELAY_OUTPUT_2,
    RELAY_OUTPUT_3,
    RELAY_OUTPUT_COUNT = RELAY_ALARM_CHANNEL_COUNT
} RelayOutputChannel;

void RelayOutput_Init(void);

/* IRQ-safe request hook. It only sets a flag and does not read parameters or GPIO. */
void RelayOutput_RequestUpdate(void);

/* Run pending relay calculation in the main context. */
void RelayOutput_ProcessPending(void);

/* Full relay calculation. Keep this in the main context unless a caller owns the timing risk. */
void RelayOutput_Update(void);

void RelayOutput_SetChannel(RelayOutputChannel channel, uint8_t active);

uint8_t RelayOutput_GetStateMask(void);

const volatile RelayAlarmRuntimeState *RelayOutput_GetRuntimeState(uint32_t channel);

#endif /* RELAY_OUTPUT_H_ */