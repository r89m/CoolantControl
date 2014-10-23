#include "Configuration.h"
#include <Bounce2.h>
#include <DistanceGP2Y0A41SK.h>
#include <Tlc5940.h>
#include "SpeedUtils.h"
#include "Bargraph.h"
#include <EEPROMex.h>

const uint8_t PIN_FLOWRATE = A1;
const uint8_t PIN_COOLANT_LEVEL_DISTANCE = A3;

const uint8_t PIN_COOLANT_CUTOFF = 6;
const uint8_t PIN_PUMP_DRIVER = 5;
const uint8_t PIN_ON_OFF_SWITCH = 4;
const uint8_t PIN_CONTROL_MODE = 12;
const uint8_t PIN_SPINDLE_SPEED = 2;
const uint8_t PIN_CALIBRATE_LEVEL_MAX = 8;
const uint8_t PIN_CALIBRATE_LEVEL_MIN = 7;
const uint8_t PIN_CALIBRATE_FLOW_RATE_MAX = A0;
const uint8_t PIN_CALIBRATE_FLOW_RATE_MIN = A2;

enum ControlMode {MANUAL, AUTO};

Bounce SW_ON_OFF = Bounce();
Bounce SW_CONTROL_MODE = Bounce();
Bounce SW_CALIBRATE_COOLANT_MAX = Bounce();
Bounce SW_CALIBRATE_COOLANT_MIN = Bounce();
Bounce SW_CALIBRATE_FLOW_RATE_MAX = Bounce();
Bounce SW_CALIBRATE_FLOW_RATE_MIN = Bounce();

DistanceGP2Y0A41SK coolant_level_distance_sensor;

const uint8_t startup_animation_frame_count = 20;
uint16_t startup_animation[startup_animation_frame_count];
uint8_t startup_animation_frame;
uint32_t startup_animation_frame_previous_time;
const uint16_t startup_animation_frame_delay = 100;

boolean power_on = false;
boolean just_powered_on = false;
boolean startup_complete = false;

uint8_t coolant_flow_rate = 0;
uint8_t coolant_level = 0;
boolean coolant_level_too_low = false;
uint8_t coolant_graph_value = 0;
ControlMode control_mode = MANUAL;
uint32_t spindle_speed_period_moving;

uint32_t last_coolant_level_check_time;
uint16_t coolant_level_check_period = 1000;

boolean coolant_level_was_lower = true;
uint8_t coolant_cutoff_level = 5;
uint8_t coolant_level_hysteresis = 1;

uint32_t lastDebugPrintTime;
uint16_t debugPrintPeriod = 500;

void setup(){
	
	Serial.begin(115200);
	
	// Setup pins
	// Switches
	pinMode(PIN_ON_OFF_SWITCH, INPUT);
	pinMode(PIN_CONTROL_MODE, INPUT);
	pinMode(PIN_CALIBRATE_LEVEL_MAX, INPUT);
	pinMode(PIN_CALIBRATE_LEVEL_MIN, INPUT);
	pinMode(PIN_SPINDLE_SPEED, INPUT);
	
	// Potentiometers
	pinMode(PIN_FLOWRATE, INPUT);
	
	// Outputs
	// Pump
	pinMode(PIN_PUMP_DRIVER, OUTPUT);
	
	// Coolant Level Distance Sensor
	coolant_level_distance_sensor.begin(A3);
	coolant_level_distance_sensor.setAveraging(10);
	
	// Initialise the Bargraph Driver
	Tlc.init(0);
	
	// Attach Bounce'd pins
	SW_ON_OFF.attach(PIN_ON_OFF_SWITCH);
	SW_CONTROL_MODE.attach(PIN_CONTROL_MODE);
	SW_CALIBRATE_COOLANT_MIN.attach(PIN_CALIBRATE_LEVEL_MIN);
	SW_CALIBRATE_COOLANT_MAX.attach(PIN_CALIBRATE_LEVEL_MAX);
	SW_CALIBRATE_FLOW_RATE_MIN.attach(PIN_CALIBRATE_FLOW_RATE_MIN);
	SW_CALIBRATE_FLOW_RATE_MAX.attach(PIN_CALIBRATE_FLOW_RATE_MAX);
	
	// Attach spindle speed interrupt
	attachInterrupt(0, spindleInterrupt, RISING);
	
	// Create startup animation
	uint8_t startup_animation_frame = 0;
	for(uint8_t i = 0; i <= 10; i++){
		startup_animation[startup_animation_frame] = create_bargraph_value(i, false);
		startup_animation_frame++;
	}
	
	for(uint8_t i = 0; i <= 10; i++){
		startup_animation[startup_animation_frame] = create_bargraph_value(10 - i, true);
		startup_animation_frame++;
	}

	// Make startup animation run only when the system turns on for the first time
	startup_complete = false;
	
	// Set what the minimum speed to be considered moving is
	spindle_speed_period_moving = convert_rpm_to_period(5);
	
	// Read in the current configuration
	read_config();
}

void loop(){
	
	update_all_inputs();
	update_bargraph_output();

	if(power_on) {
		if(just_powered_on && flow_rate_cal_buttons_pressed()){
			// Do the flow rate calibration
			
			// Make the pump speed match the flow rate selected
			while(true){
				uint16_t flow_rate_selected = analogRead(PIN_FLOWRATE);
				analogWrite(PIN_PUMP_DRIVER, map(flow_rate_selected, 0, 1023, 0, 255));
				
				boolean flow_rate_cal_max_changed = SW_CALIBRATE_FLOW_RATE_MAX.update();
				boolean flow_rate_cal_min_changed = SW_CALIBRATE_FLOW_RATE_MIN.update();
				
				// If the switch states change, ie it someone lets go of it
				if(flow_rate_cal_max_changed || flow_rate_cal_min_changed){
					// Check which switch it was
					if(flow_rate_cal_max_changed){
						CalibratedFlowRateMax = flow_rate_selected;
						} else if(flow_rate_cal_min_changed){
						CalibratedFlowRateMin = flow_rate_selected;
					}
				
					// Turn off the pump
					analogWrite(PIN_PUMP_DRIVER, 0);
					
					save_configuration();
					
					// Update the buttons
					update_all_inputs();
					
					// And only quit back to the main loop once it has been powered down
					if(!power_on){
						break;
					}
				}
			}
			
		}
		
		if(startup_complete) {	
			
			// Check if coolant level calibration buttons are pressed
			if(SW_CALIBRATE_COOLANT_MIN.read() || SW_CALIBRATE_COOLANT_MAX.read()){
				if(SW_CALIBRATE_COOLANT_MIN.read()){
					// Calibrate Coolant Min Level
					CalibratedCoolantLevelMinDistance = coolant_level_distance_sensor.getDistanceCentimeter();
					
				} else if(SW_CALIBRATE_COOLANT_MAX.read()){
					// Calibrate Coolant Max Level
					CalibratedCoolantLevelMaxDistance = coolant_level_distance_sensor.getDistanceCentimeter();
				}
				
				save_configuration();
			}	
			
			// Display coolant level on bargraph
			coolant_graph_value = min((coolant_level + 10) / 10, 10);
			set_bargraph(coolant_graph_value);
			set_bargraph_flashing((coolant_graph_value <= 3), 300);	// Make the graph flash for very low levels
			
			
			// Control coolant flow					
			if(has_enough_coolant() && coolant_should_flow()) {
				pump_coolant(coolant_flow_rate);
			} else {
				pump_coolant(0);
			}
		}
		else {
			// Play the startup animation at the given frame rate
			if(millis() - startup_animation_frame_previous_time > startup_animation_frame_delay){
				set_bargraph(startup_animation[startup_animation_frame]);
				startup_animation_frame++;
				startup_animation_frame_previous_time = millis();
			}
			// If we're at the last frame, show that the startup animation is complete
			if(startup_animation_frame == startup_animation_frame_count){
				startup_complete = true;
			}
		}
	} else {
		// Turn off the pump
		pump_coolant(0);
		// Turn off the bargraph
		set_bargraph((uint8_t) 0);
		// Go back to the beginning of the start up animation - will only play again if it did not complete previously
		startup_animation_frame = 0;
	}
	
	if(millis() - lastDebugPrintTime > debugPrintPeriod){
		Serial.println();
		Serial.println();
		Serial.println(millis());
		Serial.println();
		
	
		//Serial.print("Animation frame: ");
		//Serial.println(startup_animation_frame, DEC);
		//
		//Serial.print("Animation frame value: ");
		//Serial.println(startup_animation[startup_animation_frame-1]);
		//
		//Serial.print("Coolant Level: ");
		//Serial.println(coolant_level, DEC);
		//
		//Serial.print("Coolant Graph Level: ");
		//Serial.println(coolant_graph_value, DEC);
		//
		//Serial.print("Flow rate: ");
		//Serial.println(coolant_flow_rate, DEC);
		//
		//Serial.print("Control Mode: ");
		//Serial.println(control_mode);
		//
		//Serial.print("Power On: ");
		//Serial.println(power_on);
		//
		Serial.print("Spindle Period: ");
		Serial.println(get_spindle_period(), DEC);
		//
		Serial.print("Spindle RPM: ");
		Serial.println(get_spindle_rpm(), DEC);
		//
		//Serial.print("Has enough:        ");
		//Serial.println(has_enough_coolant(), DEC);
		//
		//Serial.print("Coolant was lower: ");
		//Serial.println(coolant_level_was_lower, DEC);
		//
		//Serial.print("Period to RPM Test: ");
		//Serial.println(convert_period_to_rpm(500));
		//
		//Serial.print("RPM to Period Test: ");
		//Serial.println(convert_rpm_to_period(454));
			
		lastDebugPrintTime = millis();
	}
}

void update_all_inputs(){

	// Update all the switches
	SW_ON_OFF.update();
	SW_CONTROL_MODE.update();
	SW_CALIBRATE_COOLANT_MAX.update();
	SW_CALIBRATE_COOLANT_MIN.update();
	
	// Don't need to update SW_CALIBRATE_FLOW_RATE_MAX or MIN
	//SW_CALIBRATE_FLOW_RATE_MAX.update();
	//SW_CALIBRATE_FLOW_RATE_MIN.update();
	
	boolean power_on_old_state = power_on;
	
	power_on = SW_ON_OFF.read();
	
	just_powered_on = (power_on && !power_on_old_state);
	
	if(SW_CONTROL_MODE.read()){
		control_mode = MANUAL;
	} else {
		control_mode = AUTO;
	}
	
	// Map the value from the coolant flow rate dial to a percentage
	coolant_flow_rate = map(analogRead(PIN_FLOWRATE), 0, 1023, 0, 100);
	
	// Read the coolant level - only do this once a second, as it's not going to change very quickly
	if(millis() - last_coolant_level_check_time > coolant_level_check_period){
		// Map the value from the coolant level sensor to a percentage
		uint8_t CoolantLevelRange = CalibratedCoolantLevelMinDistance - CalibratedCoolantLevelMaxDistance;
		coolant_level = map(CalibratedCoolantLevelMinDistance - coolant_level_distance_sensor.getDistanceCentimeter(), 0, CoolantLevelRange, 0, 100);
		last_coolant_level_check_time = millis();
	}
	
	coolant_level_too_low = (digitalRead(PIN_COOLANT_CUTOFF) == HIGH);
}

boolean has_enough_coolant(){
	
	boolean has_enough;
	uint8_t coolant_compare_value;
	
	// Check the float switch first
	if(coolant_level_too_low){
		return false;
	}
	
	// Coolant level hysteresis
	if(coolant_level_was_lower){
		coolant_compare_value = coolant_cutoff_level + coolant_level_hysteresis;
	} else {
		coolant_compare_value = coolant_cutoff_level - coolant_level_hysteresis;
	}
	
	has_enough = coolant_level > coolant_compare_value;
	
	// Record this last response so that it can be used on the next check
	coolant_level_was_lower = !has_enough;
	
	return has_enough;
}

boolean coolant_should_flow(){
	
	return (control_mode == MANUAL || (control_mode == AUTO && is_chuck_turning()));
}

boolean is_chuck_turning(){

	return (get_spindle_period() < spindle_speed_period_moving);
}

void pump_coolant(uint8_t flow_rate_percent){
	
	// Map the flow rate percentage to a specific PWM value
	uint8_t flow_rate_pwm = map(flow_rate_percent, 0, 100, CalibratedFlowRateMin, CalibratedFlowRateMax);
	
	// Drive the pump at the given power - inverse due to driver setup
	analogWrite(PIN_PUMP_DRIVER, 255 - flow_rate_pwm);
}

boolean flow_rate_cal_buttons_pressed(){

	return (SW_CALIBRATE_FLOW_RATE_MAX.read() || SW_CALIBRATE_FLOW_RATE_MIN.read());
}

void save_configuration(){

	set_bargraph(10, true);
	set_bargraph_flashing(true, 300);
	
	// Save the current calibration / configuration values to EEPROM
	save_config();
	
	// Pause for half a second to show the calibration stored animation
	delay(500);

	// Clear the bargraph output
	set_bargraph((uint8_t) 0);	
}
