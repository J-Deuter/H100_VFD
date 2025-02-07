#include "H100_VFD.h"

constexpr  uint8_t SLAVE_ADDRESS = 0x01;

constexpr uint8_t MAX485_PIN = 26;

H100VFD vfd(SLAVE_ADDRESS, Serial2, MAX485_PIN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if(!vfd.begin(19200))
    Serial.println(F("VFD failed to start"));

  Serial.print(vfd.getStatusCode());
  Serial.print("\t");
  delay(100); // Wait between modbus commands

  Serial.println(vfd.getStatusStr());
  delay(100); // Wait between modbus commands

  Serial.print(F("Inverter Temp: "));
  Serial.println(vfd.readInverterFunction(Function::INVERTER_TEMPERATURE));
  delay(100); // Wait between modbus commands
  
  vfd.setFrequency(100000); // 100%
  delay(100); // Wait between modbus commands

  vfd.setCommand(Command::START);

  delay(8000); // Wait for ramp

  vfd.setCommand(Command::REVERSE);

  delay(10000); // Wait for full reverce

  vfd.setFrequency(50000); //Half freq

  delay(5000); // Wait for slowdown

  vfd.setCommand(Command::STOP);
  delay(100); // Wait between modbus commands

  Serial.print("P2.18 value: ");
  Serial.println(vfd.read16BitParameter(218)); // Read param P2.18, Max speed setting

}

void loop() {
  // put your main code here, to run repeatedly:

}
