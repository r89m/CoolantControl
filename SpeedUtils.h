// SpeedUtils.h

#ifndef _SPEEDUTILS_h
#define _SPEEDUTILS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

uint16_t convert_period_to_rpm(uint32_t speed_period);
uint32_t convert_rpm_to_period(uint16_t speed_rpm);
uint32_t convert_speed_units(uint32_t input);
uint32_t get_spindle_period();
uint16_t get_spindle_rpm();
void spindleInterrupt();

#endif