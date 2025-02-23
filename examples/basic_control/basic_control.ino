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

  // Need short delay between modbus commands to prevent errors
  // Can reduce delay but must varrify function
  delay(10);
  
  // Get Status as a string
  Serial.print(F("Status String: ")); // Note using 'F("")' stores the string in flash memory instead of RAM
  Serial.println(vfd.getStatusStr()); // Get and display status as a readable string
  delay(10); // Wait between modbus commands

  // Set Frequancy to 100%
  vfd.setFrequency(vfd.MaxSpeed); // Max speed value will set frequacny to 100% of max frequancy value (P5.08)
  delay(10); // Wait between modbus commands

  // Command VFD start
  vfd.setCommand(Command::START);
  Serial.println("Forward 100%");
  delay(20000); // Wait for full ramp, depends on acceleration settings

  // Slow to 50%
  vfd.setFrequency(vfd.MaxSpeed*0.5); // Sets frequency to half of maximum 
  Serial.println("50%");
  delay(10000); //Wait for slowdown

  // Set motor to start at same frequancy in reverce
  vfd.setCommand(Command::START_REVERSE);
  Serial.println("Reverce 50%");
  delay(20000); // Wait for full reverce

  // Stop the VFD
  vfd.setCommand(Command::STOP);
  Serial.println("Stop");
  delay(10); // Wait between modbus commands
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
