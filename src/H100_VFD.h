#ifndef H100_VFD_H
#define H100_VFD_H

#include <Arduino.h>
#include <ModbusMaster.h>

// Define commands as enums for readability.
enum Command {
    START            = 1 << 0,   // 0b0000000000000001
    REVERSE          = 1 << 1,   // 0b0000000000000010
    START_REVERSE    = 1 << 2,   // 0b0000000000000100
    JOG              = 1 << 3,   // 0b0000000000001000
    STOP             = 1 << 4,   // 0b0000000000010000
    EMERGENCY_STOP   = 1 << 5,   // 0b0000000000100000
    SAFE_STOP        = 1 << 6,   // 0b0000000001000000
    RESET            = 1 << 7,   // 0b0000000010000000
    PARAM_SELF_LEARN = 1 << 9,   // 0b0000001000000000
    PAUSE            = 1 << 11,  // 0b0000100000000000
    UP               = 1 << 13,  // 0b0010000000000000
    DOWN             = 1 << 14   // 0b0100000000000000
};

// Define status as enums for readability
enum Status {
    POWERING_OFF           = 1 << 0,  // 0b0000000000000001
    STOPPING               = 1 << 1,  // 0b0000000000000010
    RUNNING                = 1 << 2,  // 0b0000000000000100
    START_FUNCTION_START   = 1 << 3,  // 0b0000000000001000
    PARAM_SELF_LEARN_START = 1 << 4,  // 0b0000000000010000
    OPERATING              = 1 << 5,  // 0b0000000000100000
    READY                  = 1 << 6,  // 0b0000000001000000
    FAULT                  = 1 << 10, // 0b0000010000000000
    ALARM                  = 1 << 11, // 0b0000100000000000
    STO_STATUS             = 1 << 12  // 0b0001000000000000
};

// Define fault codes as enums for readability
enum Fault {
    SYSTEM_ABNORMALITY     = 1 << 1,   // 0b0000000000000010
    GROUND_FAULT           = 1 << 4,   // 0b0000000000010000
    SHORT_CIRCUIT_TO_GND   = 1 << 5,   // 0b0000000000100000
    OUTPUT_SHORT_CIRCUIT   = 1 << 6,   // 0b0000000001000000
    OUTPUT_OVERCURRENT     = 1 << 7,   // 0b0000000010000000
    DC_BUS_OVERVOLTAGE     = 1 << 8,   // 0b0000000100000000
    DC_BUS_UNDERVOLTAGE    = 1 << 9,   // 0b0000001000000000
    INVERTER_OVERHEATING   = 1 << 10,  // 0b0000010000000000
    RECTIFIER_OVERHEATING  = 1 << 13,  // 0b0010000000000000
    U_PHASE_MISSING        = 1 << 14,  // 0b0100000000000000
    V_PHASE_MISSING        = 1 << 15,  // 0b1000000000000000
    W_PHASE_MISSING        = 1 << 16,  // 0b00010000000000000  (Note: shifted left by 16)
    NO_MOTOR_CONNECTION    = 1 << 19,  // 0b00000000000100000000
    INPUT_PHASE_LOSS       = 1 << 20,  // 0b00000000001000000000
    INVERTER_OVERLOAD      = 1 << 21,  // 0b00000000010000000000
    OVERTORQUE             = 1 << 22,  // 0b00000000100000000000
    MOTOR_OVERHEATING      = 1 << 24,  // 0b00000100000000000000
    MOTOR_OVERLOAD         = 1 << 25,  // 0b00001000000000000000
    CURRENT_LIMIT          = 1 << 26,  // 0b00010000000000000000
    INPUT_POWER_DOWN       = 1 << 27   // 0b00100000000000000000
};
// Define 32-bit addresses for inverter functions as enums
enum class Function : uint16_t {
    OUTPUT_FREQUENCY          = 0x43FC,  // P10.21
    OUTPUT_CURRENT            = 0x43FD,  // P10.22
    OUTPUT_VOLTAGE            = 0x43FE,  // P10.23
    OUTPUT_TORQUE             = 0x43FF,  // P10.24
    DC_VOLTAGE                = 0x4400,  // P10.25
    INVERTER_TEMPERATURE      = 0x4401,  // P10.26
    POWER                     = 0x4405,  // P10.30
    ENERGY_CONSUMPTION        = 0x4406,  // P10.31
    HOURS_OF_POWER_ON         = 0x440F,  // P10.40
    NUMBER_OF_POWER_ON        = 0x441F,  // P10.41
    S_TERMINAL_INPUT_STATUS   = 0x442D,  // P10.70
    AI1_TERMINAL_INPUT_VALUE  = 0x442E,  // P10.71
    AI2_TERMINAL_INPUT_VALUE  = 0x442F,  // P10.72
    Y_TERMINAL_OUTPUT_STATUS  = 0x4431,  // P10.74
    AO1_TERMINAL_OUTPUT_VALUE = 0x4432,  // P10.75
    AO2_TERMINAL_OUTPUT_VALUE = 0x4433   // P10.76 
};

class H100VFD {
public:
    H100VFD(uint8_t slaveAddress, HardwareSerial &serial, uint8_t MAX485ControlPin);

    bool begin(uint32_t baudRate = 19200);

    // VFD control functions
    bool setFrequency(uint32_t frequency);
    bool setCommand(Command cmd);

    // Status and Fault functions
    uint16_t getStatusCode();
    uint32_t getFaultCode();
    String getStatusStr();
    String getFaultStr();

    // Inverter function reading with scaling
    double readInverterFunction(Function function);

    // Parameter access functions
    uint16_t read16BitParameter(uint16_t paramNumber);
    uint32_t read32BitParameter(uint16_t paramNumber);
    bool write16BitParameter(uint16_t paramNumber, uint16_t value);
    bool write32BitParameter(uint16_t paramNumber, uint32_t value);

    // Error code variables and functions
    uint8_t lastErrorCode;       // Last error code
    String getLastErrorStr();   // Error code as human readable string
    static constexpr uint8_t INVALID_BAUD = 0xA0; // Error for invalid baud rate
    static constexpr uint8_t INVALID_READ_VALUE = 0xA1; // Value read is outside expected bounds

    uint32_t MaxSpeed;

private:
   private:
    ModbusMaster _node;
    HardwareSerial& _serial;
    uint8_t _slaveAddress;
    static uint8_t _MAX485ControlPin;
    uint32_t _baudRate;
    

    // Define Modbus registers (addresses) - Moved to private
    static constexpr uint16_t SPEED_ADDR = 0xC121; 
    static constexpr uint16_t CONTROL_ADDR = 0x8122;
    static constexpr uint16_t STATUS_ADDR = 0x03F6;
    static constexpr uint16_t FAULT_ADDR = 0x43F7;

    // Helper functions for Modbus communication
    uint16_t read16Bit(uint16_t registerAddress);
    uint32_t read32Bit(uint16_t registerAddress);
    bool write16Bit(uint16_t registerAddress, uint16_t value);
    bool write32Bit(uint16_t registerAddress, uint32_t value);

    // MAX485 control
    static void preTransmission();
    static void postTransmission();
};

#endif
