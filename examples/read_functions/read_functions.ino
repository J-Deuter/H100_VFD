#include "H100_VFD.h"

// Slave address of VFD, default is 1
constexpr  uint8_t SLAVE_ADDRESS = 0x01;

// Control pin for MAX485 TX/RX enable (RE/DE)
constexpr uint8_t MAX485_PIN = 24;

// Define vfd object with parameters
H100VFD vfd(SLAVE_ADDRESS, Serial2, MAX485_PIN);

void setup() {
  // put your setup code here, to run once:  
  Serial.begin(9600); // Start serial for debug

  delay(500); // Small delay for serial monitor
  Serial.println("");

  if(!vfd.begin(19200)){
    Serial.println(F("VFD failed to start"));
    Serial.print("Last Error: ");
    Serial.println(vfd.getLastErrorStr()); // Print last error
    return; // Stop program if VFD does not initialize
  }
    
  // Get Status as a string
  Serial.print(F("Status: ")); // Note using 'F("")' stores the string in flash memory instead of RAM
  Serial.println(vfd.getStatusStr()); // Get and display status as a readable string
  delay(10); // Wait between modbus commands

  Serial.println(F("\nReading inverter functions..."));

  // Read all inverter functions, returns adjusted float value   
  // To read raw integer value use read16BitParameter or read32BitParameter functions
  Serial.print(F("Output Frequency: "));
  Serial.println(vfd.readInverterFunction(Function::OUTPUT_FREQUENCY));
  delay(10); // Wait between modbus commands 
  Serial.print(F("Output Current: "));
  Serial.println(vfd.readInverterFunction(Function::OUTPUT_CURRENT));
  delay(10); // Wait between modbus commands
  Serial.print(F("Output Voltage: "));
  Serial.println(vfd.readInverterFunction(Function::OUTPUT_VOLTAGE));
  delay(10); // Wait between modbus commands
  Serial.print(F("Output Torque: "));
  Serial.println(vfd.readInverterFunction(Function::OUTPUT_TORQUE));
  delay(10); // Wait between modbus commands
  Serial.print(F("DC Voltage: "));
  Serial.println(vfd.readInverterFunction(Function::DC_VOLTAGE));
  delay(10); // Wait between modbus commands
  Serial.print(F("Inverter Temperature: "));
  Serial.println(vfd.readInverterFunction(Function::INVERTER_TEMPERATURE));
  delay(10); // Wait between modbus commands
  Serial.print(F("Power: "));
  Serial.println(vfd.readInverterFunction(Function::POWER));
  delay(10); // Wait between modbus commands
  Serial.print(F("Energy Consumption: "));
  Serial.println(vfd.readInverterFunction(Function::ENERGY_CONSUMPTION));
  delay(10); // Wait between modbus commands
  Serial.print(F("Hours Of Power On: "));
  Serial.println(vfd.readInverterFunction(Function::HOURS_OF_POWER_ON));
  delay(10); // Wait between modbus commands
  Serial.print(F("Number Of Power Ons: "));
  Serial.println(vfd.readInverterFunction(Function::NUMBER_OF_POWER_ON));
  Serial.println(vfd.lastErrorCode, HEX);
  delay(10); // Wait between modbus commands
  Serial.print(F("S Terminal Input Status: "));
  Serial.println(vfd.readInverterFunction(Function::S_TERMINAL_INPUT_STATUS));
  delay(10); // Wait between modbus commands
  Serial.print(F("AI1 Terminal Input Value: "));
  Serial.println(vfd.readInverterFunction(Function::AI1_TERMINAL_INPUT_VALUE));
  delay(10); // Wait between modbus commands
  Serial.print(F("AI2 Terminal Input Value: "));
  Serial.println(vfd.readInverterFunction(Function::AI2_TERMINAL_INPUT_VALUE));
  delay(10); // Wait between modbus commands
  Serial.print(F("Y Terminal Output Status: "));
  Serial.println(vfd.readInverterFunction(Function::Y_TERMINAL_OUTPUT_STATUS));
  delay(10); // Wait between modbus commands
  Serial.print(F("AO1 Terminal Output Value: "));
  Serial.println(vfd.readInverterFunction(Function::AO1_TERMINAL_OUTPUT_VALUE));
  delay(10); // Wait between modbus commands
  Serial.print(F("AO2 Terminal Output Value: "));
  Serial.println(vfd.readInverterFunction(Function::AO2_TERMINAL_OUTPUT_VALUE));
  delay(10); // Wait between modbus commands

}

void loop() {
  // put your main code here, to run repeatedly:

}
