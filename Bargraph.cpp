// 
// 
// 

#include "Bargraph.h"
#include <Tlc5940.h>


uint16_t bargraph_current_value = 0;
boolean bargraph_flashing = false;
boolean bargraph_lit = false;

uint32_t bargraph_flash_last_timestamp = 0;
uint16_t bargraph_flast_delay = 0;

void set_bargraph(uint16_t value){

	// Only bother re-outputting the value if it's different
	if(bargraph_current_value != value){
		bargraph_current_value = value;
		write_bargraph_output(value);
	}
}

void set_bargraph(uint8_t value, boolean fill_up){

	set_bargraph(create_bargraph_value(value, fill_up));
}

void set_bargraph(uint8_t value){

	set_bargraph(value, true);
}

uint16_t create_bargraph_value(uint8_t value, boolean fill_up){

	uint16_t graph_value = (1 << value);
	
	if(fill_up){
		graph_value <<= 1;
		graph_value--;
	}
	
	return graph_value;
}

void set_bargraph_flashing(boolean flashing){
	
	bargraph_flashing = flashing;
}

void set_bargraph_flashing(boolean flashing, uint16_t delay){
	
	bargraph_flast_delay = delay;
	set_bargraph_flashing(flashing);
}

void update_bargraph_output(){
	
	if(bargraph_flashing){
		if(millis() - bargraph_flash_last_timestamp > bargraph_flast_delay){
			if(bargraph_lit){
				write_bargraph_output(0);
			} else {
				write_bargraph_output(bargraph_current_value);
			}
			bargraph_lit = !bargraph_lit;
			bargraph_flash_last_timestamp = millis();
		}
	}	
}

void write_bargraph_output(uint16_t value){
	
	// Clear all outputs
	Tlc.setAll(0);
	
	uint16_t mask = 1;
	
	// Check all 10 bits and decide which ones to set
	for(uint8_t i = 1; i <= 10; i++){
		mask <<= 1;
		
		if(value & mask){
			Tlc.set(i, 4095);
		}
	}
	
	// Update the output
	Tlc.update();
}
