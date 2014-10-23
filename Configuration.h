// Configuration.h

#ifndef _CONFIGURATION_h
#define _CONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <EEPROMex.h>

extern uint8_t CalibratedCoolantLevelMaxDistance;			// Save these to EEPROM
extern uint8_t CalibratedCoolantLevelMinDistance;
extern uint16_t CalibratedFlowRateMin;
extern uint16_t CalibratedFlowRateMax;

void read_config();
void save_config();

#endif

