// 
// 
// 

#include "Configuration.h"

uint8_t CONFIG_ADDR_SCHEMA = 1;
uint8_t CONFIG_ADDR_COOLANT_LEVEL_MAX = 2;
uint8_t CONFIG_ADDR_COOLANT_LEVEL_MIN = 3;
uint8_t CONFIG_ADDR_FLOW_RATE_MAX = 4;
uint8_t CONFIG_ADDR_FLOW_RATE_MIN = 5;

uint8_t ConfigSchemaCurrentVersion = 1;

uint8_t CalibratedCoolantLevelMaxDistance;
uint8_t CalibratedCoolantLevelMinDistance;
uint16_t CalibratedFlowRateMin;
uint16_t CalibratedFlowRateMax;


void read_config(){
	
	uint8_t SavedSchemaVersion = EEPROM.readByte(CONFIG_ADDR_SCHEMA);
	
	// If the schema hasn't changed since the data was saved, use the saved data
	if(SavedSchemaVersion == ConfigSchemaCurrentVersion){
		CalibratedCoolantLevelMaxDistance = EEPROM.readByte(CONFIG_ADDR_COOLANT_LEVEL_MAX);
		CalibratedCoolantLevelMinDistance = EEPROM.readByte(CONFIG_ADDR_COOLANT_LEVEL_MIN);
		CalibratedFlowRateMin = EEPROM.readInt(CONFIG_ADDR_FLOW_RATE_MIN);
		CalibratedFlowRateMax = EEPROM.readInt(CONFIG_ADDR_FLOW_RATE_MAX);
	} else {
		// Otherwise, the schema has changed or has never been used, so use default values
		CalibratedCoolantLevelMaxDistance = 3;
		CalibratedCoolantLevelMinDistance = 37;
		CalibratedFlowRateMin = 0;
		CalibratedFlowRateMax = 255;
	}	 
}

void save_config(){
	
	EEPROM.writeByte(CONFIG_ADDR_SCHEMA, ConfigSchemaCurrentVersion);
	
	EEPROM.writeByte(CONFIG_ADDR_COOLANT_LEVEL_MAX, CalibratedCoolantLevelMaxDistance);
	EEPROM.writeByte(CONFIG_ADDR_COOLANT_LEVEL_MIN, CalibratedCoolantLevelMinDistance);
	
	EEPROM.writeInt(CONFIG_ADDR_FLOW_RATE_MAX, CalibratedFlowRateMax);
	EEPROM.writeInt(CONFIG_ADDR_FLOW_RATE_MIN, CalibratedFlowRateMin);
}
