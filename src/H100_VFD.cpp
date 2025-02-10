#include "H100_VFD.h"

// Define the static member variable
uint8_t H100VFD::_MAX485ControlPin = 0;

// Constructor: Set up Modbus and control pin.
H100VFD::H100VFD(uint8_t slaveAddress, HardwareSerial &serial, uint8_t MAX485ControlPin)
: _slaveAddress(slaveAddress), _serial(serial) {
    _node.begin(slaveAddress, serial);  

    // Set the static MAX485 control pin if not already set
    if (_MAX485ControlPin == 0) {  // Only set it if not already initialized
        _MAX485ControlPin = MAX485ControlPin;
    }
}

// Initialize the VFD connection using a specific serial port.
bool H100VFD::begin(uint32_t baudRate = 19200) {

    // Check if the baud rate is valid
    switch (baudRate) {
        case 2400:
        case 4800:
        case 9600:
        case 14400:
        case 19200:
        case 38400:
            H100VFD::_baudRate = baudRate; // Set baud rate if valid
            break;
        default:
            lastErrorCode = H100VFD::INVALID_BAUD;
            return false; // Invalid baud rate
    } 

    // Start Serial Communications
    _serial.end();
    _serial.begin(baudRate);    // Baud rate for Modbus communication.

    pinMode(_MAX485ControlPin, OUTPUT);
    digitalWrite(_MAX485ControlPin, LOW);    // Start in receive mode

    // Set up Modbus transmission callbacks
    _node.preTransmission(H100VFD::preTransmission); // Use class scope
    _node.postTransmission(H100VFD::postTransmission); // Use class scope

    uint16_t P147 = read16BitParameter(147);

    if(lastErrorCode != ModbusMaster::ku8MBSuccess){
      return false;
    }
      
    switch (P147){
    case 0:
      MaxSpeed = 100000;
      break;
    case 1:
      MaxSpeed = 10000;
      break;
    case 2:
      MaxSpeed = 1000;
      break;
    case 3:
      MaxSpeed = 100;
      break;
    default:
      lastErrorCode = INVALID_READ_VALUE;
      break;
    }

    return true; 

}

// Manage MAX485 direction: HIGH before transmission.
void H100VFD::preTransmission() {
  digitalWrite(_MAX485ControlPin, HIGH);
}

// Manage MAX485 direction: LOW after transmission.
void H100VFD::postTransmission() {
  digitalWrite(_MAX485ControlPin, LOW);
}

// Read a 16-bit register value.
uint16_t H100VFD::read16Bit(uint16_t registerAddress) {
  uint8_t result = _node.readHoldingRegisters(registerAddress, 1);
  lastErrorCode = result;
  if (result == _node.ku8MBSuccess) {
    return _node.getResponseBuffer(0);
  }
  return 0xFFFF;  // Error code.
}

// Read a 32-bit value (two registers).
uint32_t H100VFD::read32Bit(uint16_t registerAddress) {
  uint8_t result = _node.readHoldingRegisters(registerAddress, 2);
  lastErrorCode = result;
  if (result == _node.ku8MBSuccess) {
    return ((uint32_t)_node.getResponseBuffer(0) << 16) | _node.getResponseBuffer(1);
  }
  return 0xFFFFFFFF;  // Error code.
}

// Write a 16-bit value to a register.
bool H100VFD::write16Bit(uint16_t registerAddress, uint16_t value) {
  uint8_t result = _node.writeSingleRegister(registerAddress, value);
  lastErrorCode = result;
  return (result == _node.ku8MBSuccess);
}

// Write a 32-bit value (two registers).
bool H100VFD::write32Bit(uint16_t registerAddress, uint32_t value) {
  _node.setTransmitBuffer(0, highWord(value));
  _node.setTransmitBuffer(1, lowWord(value));
  uint8_t result = _node.writeMultipleRegisters(registerAddress, 2);
  lastErrorCode = result;
  return (result == _node.ku8MBSuccess);
}

// Set the motor frequency (32-bit operation).
bool H100VFD::setFrequency(uint32_t frequency) {
  return write32Bit(SPEED_ADDR, frequency);
}

// Map user-friendly commands to the actual 16-bit hex codes.
bool H100VFD::setCommand(Command cmd) {
  return write16Bit(CONTROL_ADDR, cmd);
}

// Read the status register (16-bit).
uint16_t H100VFD::getStatusCode() {
  return read16Bit(STATUS_ADDR);
}

// Read the fault register (16-bit).
uint32_t H100VFD::getFaultCode() {
  return read32Bit(FAULT_ADDR);
}

// Get human-readable status description from 16-bit status codes.
String H100VFD::getStatusStr() {
    uint16_t statusCode = getStatusCode();
    if (statusCode == 0xFFFF) 
        return "Error reading status.";

    String status = "";

    if (statusCode & Status::POWERING_OFF) {
      status += F("Powering Off\t");
    }
    if (statusCode & Status::STOPPING) {
      status += F("Stopping\t");
    }
    if (statusCode & Status::RUNNING) {
      status += F("Running\t");
    }
    if (statusCode & Status::START_FUNCTION_START) {
      status += F("Start Function Start\t");
    }
    if (statusCode & Status::PARAM_SELF_LEARN_START) {
      status += F("Parameter Self Learn Start\t");
    }
    if (statusCode & Status::OPERATING) {
      status += F("Operating\t");
    }
    if (statusCode & Status::READY) {
      status += F("Ready\t");
    }
    if (statusCode & Status::FAULT) {
      status += F("Fault: ");
      status += getFaultStr();
    }
    if (statusCode & Status::ALARM) {
      status += F("Alarm\t");
    }
    if (statusCode & Status::STO_STATUS) {
      status += F("STO Active\t");
    }

    return status;
}   

// Get human-readable fault description from 16-bit fault codes.
String H100VFD::getFaultStr() {
    uint32_t faultCode = getFaultCode();
    if (faultCode == 0xFFFFFFFF) 
        return "Error reading fault.";

    String fault = ""; // Initialize an empty string

    if (faultCode & Fault::SYSTEM_ABNORMALITY) {
      fault += F("System Abnormality\t");
    }
    if (faultCode & Fault::GROUND_FAULT) {
      fault += F("Ground Fault\t");
    }
    if (faultCode & Fault::SHORT_CIRCUIT_TO_GND) {
      fault += F("Short Circuit to Ground\t");
    }
    if (faultCode & Fault::OUTPUT_SHORT_CIRCUIT) {
      fault += F("Output Short Circuit\t");
    }
    if (faultCode & Fault::OUTPUT_OVERCURRENT) {
      fault += F("Output Overcurrent\t");
    }
    if (faultCode & Fault::DC_BUS_OVERVOLTAGE) {
      fault += F("DC Bus Overvoltage\t");
    }
    if (faultCode & Fault::DC_BUS_UNDERVOLTAGE) {
      fault += F("DC Bus Undervoltage\t");
    }
    if (faultCode & Fault::INVERTER_OVERHEATING) {
      fault += F("Inverter Overheating\t");
    }
    if (faultCode & Fault::RECTIFIER_OVERHEATING) {
      fault += F("Rectifier Overheating\t");
    }
    if (faultCode & Fault::U_PHASE_MISSING) {
      fault += F("U Phase Missing\t");
    }
    if (faultCode & Fault::V_PHASE_MISSING) {
      fault += F("V Phase Missing\t");
    }
    if (faultCode & Fault::W_PHASE_MISSING) {
      fault += F("W Phase Missing\t");
    }
    if (faultCode & Fault::NO_MOTOR_CONNECTION) {
      fault += F("No Motor Connection\t");
    }
    if (faultCode & Fault::INPUT_PHASE_LOSS) {
      fault += F("Input Phase Loss\t");
    }
    if (faultCode & Fault::INVERTER_OVERLOAD) {
      fault += F("Inverter Overload\t");
    }
    if (faultCode & Fault::OVERTORQUE) {
      fault += F("Overtorque\t");
    }
    if (faultCode & Fault::MOTOR_OVERHEATING) {
      fault += F("Motor Overheating\t");
    }
    if (faultCode & Fault::MOTOR_OVERLOAD) {
      fault += F("Motor Overload\t");
    }
    if (faultCode & Fault::CURRENT_LIMIT) {
      fault += F("Current Limit\t");
    }
    if (faultCode & Fault::INPUT_POWER_DOWN) {
      fault += F("Input Power Down\t");
    }

    if (fault == "")
      return "No faults";
    
    return fault;
}

// Read a 32-bit inverter function register.
double H100VFD::readInverterFunction(Function function) {
    uint16_t address = static_cast<uint16_t>(function);  // Convert to uint16_t
    uint32_t valueRaw = read32Bit(address);
    if (valueRaw == 0xFFFFFFFF){
        return NAN;
    }
    double value = double(valueRaw);

    switch (function) {
        case Function::OUTPUT_FREQUENCY:     value /= 10.0f;     break; // 1 decimal place
        case Function::OUTPUT_CURRENT:       value /= 100.0f;    break; // 2 decimal places
        case Function::OUTPUT_VOLTAGE:       value /= 10.0f;     break; // 1 decimal place
        case Function::OUTPUT_TORQUE:        value /= 1000.0f;   break; // 3 decimal places
        case Function::DC_VOLTAGE:           value /= 10.0f;     break; // 1 decimal place
        case Function::POWER:                value /= 1000.0f;   break; // 3 decimal places
        case Function::ENERGY_CONSUMPTION:   value /= 1000.0f;   break; // 3 decimal places
        case Function::HOURS_OF_POWER_ON:    value /= 1000.0f;   break; // 3 decimal places
        case Function::AI1_TERMINAL_INPUT_VALUE: value /= 1000.0f; break; // 3 decimal places
        case Function::AI2_TERMINAL_INPUT_VALUE: value /= 1000.0f; break; // 3 decimal places
        case Function::AO1_TERMINAL_OUTPUT_VALUE: value /= 1000.0f; break; // 3 decimal places
        case Function::AO2_TERMINAL_OUTPUT_VALUE: value /= 1000.0f; break; // 3 decimal places
    }
    return value;
}

uint16_t H100VFD::read16BitParameter(uint16_t paramNumber){
    return read16Bit(paramNumber-1);
}

uint32_t H100VFD::read32BitParameter(uint16_t paramNumber){
    return read32Bit((paramNumber-1+16384));
}

bool H100VFD::write16BitParameter(uint16_t paramNumber, uint16_t value){
    return write32Bit((paramNumber-1), value);
}

bool H100VFD::write32BitParameter(uint16_t paramNumber, uint32_t value){
    return write32Bit((paramNumber-1+16384), value);
}

String H100VFD::getLastErrorStr() {
    switch (lastErrorCode) {
      case ModbusMaster::ku8MBSuccess:
          return F("Success");
      case ModbusMaster::ku8MBIllegalFunction:
          return F("Illegal function");
      case ModbusMaster::ku8MBIllegalDataAddress:
          return F("Illegal data address");
      case ModbusMaster::ku8MBIllegalDataValue:
          return F("Illegal data value");
      case ModbusMaster::ku8MBSlaveDeviceFailure:
          return F("Slave device failure");
      case ModbusMaster::ku8MBInvalidSlaveID:
          return F("Invalid slave ID");
      case ModbusMaster::ku8MBInvalidFunction:
          return F("Invalid function");
      case ModbusMaster::ku8MBResponseTimedOut:
          return F("Response timed out");
      case ModbusMaster::ku8MBInvalidCRC:
          return F("Invalid CRC");
      case H100VFD::INVALID_BAUD:
          return F("Invalid Baud Rate");
      default:
          return F("Unknown error");
    }
}