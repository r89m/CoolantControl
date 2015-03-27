// 
// 
// 

#include "SpeedUtils.h"

const uint8_t spindle_pulses_per_revolution = 8;
volatile uint32_t spindle_pulse_last_timestamp;
volatile uint32_t spindle_pulse_last_period;

uint16_t convert_period_to_rpm(uint32_t speed_period){
	
	return convert_speed_units(speed_period);
}

uint32_t convert_rpm_to_period(uint16_t speed_rpm){
	
	return convert_speed_units(speed_rpm);
}

uint32_t convert_speed_units(uint32_t input){
	
	if(input == 0){
		return 0;
		} else {
		return 60000 / input;
	}
}

uint32_t get_spindle_period(){
	
	return max(spindle_pulse_last_period, millis() - spindle_pulse_last_timestamp) * spindle_pulses_per_revolution;
}

uint16_t get_spindle_rpm(){
	
	return convert_period_to_rpm(get_spindle_period());
}

void spindleInterrupt(){
	
	uint32_t timestamp = millis();
	spindle_pulse_last_period = timestamp - spindle_pulse_last_timestamp;
	spindle_pulse_last_timestamp = timestamp;
}

