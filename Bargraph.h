// Bargraph.h

#ifndef _BARGRAPH_h
#define _BARGRAPH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

void set_bargraph(uint8_t value);
void set_bargraph(uint8_t value, boolean fill_up);
void set_bargraph(uint16_t value);
uint16_t create_bargraph_value(uint8_t value, boolean fill_up);
void set_bargraph_flashing(boolean flashing);
void set_bargraph_flashing(boolean flashing, uint16_t delay);
void update_bargraph_output();
void write_bargraph_output(uint16_t value);

#endif