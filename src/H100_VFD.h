/**
 * @file H100_VFD.h
 * @brief Library for controlling the H100 series VFD via Modbus RTU.
 *
 * This library provides an interface for sending commands, reading status,
 * and configuring parameters of the H100 VFD using RS485 (Modbus RTU) communication.
 *
 * @author J. Deuter
 * @date 2025-02-21
 * @version 1.0
 */

#ifndef H100_VFD_H
#define H100_VFD_H

#include <Arduino.h>
#include <ModbusMaster.h>

/**
 * @brief Defines command codes for controlling the VFD
 */
enum Command {
    START            = 1 << 0,   /**< Start the VFD */
    REVERSE          = 1 << 1,   /**< Reverse the direction */
    START_REVERSE    = 1 << 2,   /**< Start in reverse */
    JOG              = 1 << 3,   /**< Jog mode */
    STOP             = 1 << 4,   /**< Stop operation */
    EMERGENCY_STOP   = 1 << 5,   /**< Emergency stop */
    SAFE_STOP        = 1 << 6,   /**< Safe stop */
    RESET            = 1 << 7,   /**< Reset command */
    PARAM_SELF_LEARN = 1 << 9,   /**< Motor parameter self-learn */
    PAUSE            = 1 << 11,  /**< Pause operation */
    UP               = 1 << 13,  /**< Increase parameter */
    DOWN             = 1 << 14   /**< Decrease parameter */
};

/**
 * @brief Defines status codes for the VFD
 */
enum Status {
    POWERING_OFF           = 1 << 0,  /**< Powering off */
    STOPPING               = 1 << 1,  /**< Stopping */
    RUNNING                = 1 << 2,  /**< Running */
    START_FUNCTION_START   = 1 << 3,  /**< Start function initiated */
    PARAM_SELF_LEARN_START = 1 << 4,  /**< Parameter self-learning in progress */
    OPERATING              = 1 << 5,  /**< Operating */
    READY                  = 1 << 6,  /**< Ready state */
    FAULT                  = 1 << 10, /**< Fault detected */
    ALARM                  = 1 << 11, /**< Alarm triggered */
    STO_STATUS             = 1 << 12  /**< Safe torque off status */
};

/**
 * @brief Defines fault codes for the VFD
 */
enum Fault {
    SYSTEM_ABNORMALITY     = 1 << 1,   /**< System abnormality */
    GROUND_FAULT           = 1 << 4,   /**< Ground fault */
    SHORT_CIRCUIT_TO_GND   = 1 << 5,   /**< Short circuit to ground */
    OUTPUT_SHORT_CIRCUIT   = 1 << 6,   /**< Output short circuit */
    OUTPUT_OVERCURRENT     = 1 << 7,   /**< Output overcurrent */
    DC_BUS_OVERVOLTAGE     = 1 << 8,   /**< DC bus overvoltage */
    DC_BUS_UNDERVOLTAGE    = 1 << 9,   /**< DC bus undervoltage */
    INVERTER_OVERHEATING   = 1 << 10,  /**< Inverter overheating */
    RECTIFIER_OVERHEATING  = 1 << 13,  /**< Rectifier overheating */
    U_PHASE_MISSING        = 1 << 14,  /**< U phase missing */
    V_PHASE_MISSING        = 1 << 15,  /**< V phase missing */
    W_PHASE_MISSING        = 1 << 16,  /**< W phase missing */
    NO_MOTOR_CONNECTION    = 1 << 19,  /**< No motor connection */
    INPUT_PHASE_LOSS       = 1 << 20,  /**< Input phase loss */
    INVERTER_OVERLOAD      = 1 << 21,  /**< Inverter overload */
    OVERTORQUE             = 1 << 22,  /**< Over torque */
    MOTOR_OVERHEATING      = 1 << 24,  /**< Motor overheating */
    MOTOR_OVERLOAD         = 1 << 25,  /**< Motor overload */
    CURRENT_LIMIT          = 1 << 26,  /**< Current limit exceeded */
    INPUT_POWER_DOWN       = 1 << 27   /**< Input power down */
};

/**
 * @brief Defines function addresses for reading inverter parameters
 */
enum class Function : uint16_t {
    OUTPUT_FREQUENCY          = 0x43FC,  /**< Output frequency */
    OUTPUT_CURRENT            = 0x43FD,  /**< Output current */
    OUTPUT_VOLTAGE            = 0x43FE,  /**< Output voltage */
    OUTPUT_TORQUE             = 0x43FF,  /**< Output torque */
    DC_VOLTAGE                = 0x4400,  /**< DC voltage */
    INVERTER_TEMPERATURE      = 0x4401,  /**< Inverter temperature */
    POWER                     = 0x4405,  /**< Power output */
    ENERGY_CONSUMPTION        = 0x4406,  /**< Energy consumption */
    HOURS_OF_POWER_ON         = 0x440F,  /**< Hours of power on */
    NUMBER_OF_POWER_ON        = 0x441F,  /**< Number of power on cycles */
    S_TERMINAL_INPUT_STATUS   = 0x442D,  /**< S terminal input status */
    AI1_TERMINAL_INPUT_VALUE  = 0x442E,  /**< Analog input 1 value */
    AI2_TERMINAL_INPUT_VALUE  = 0x442F,  /**< Analog input 2 value */
    Y_TERMINAL_OUTPUT_STATUS  = 0x4431,  /**< Y terminal output status */
    AO1_TERMINAL_OUTPUT_VALUE = 0x4432,  /**< Analog output 1 value */
    AO2_TERMINAL_OUTPUT_VALUE = 0x4433   /**< Analog output 2 value */
};

/**
 * @brief Class for interfacing with an H100 series VFD via RS485.
 */
class H100VFD {
    public:

        /**
         * @brief Construct a new H100VFD object
         * 
         * @param slaveAddress Modbus slave address, Check P01.41
         * @param serial Hardware serial port
         * @param MAX485ControlPin Control pin for MAX485 DE, RE
         */
        H100VFD(uint8_t slaveAddress, HardwareSerial &serial, uint8_t MAX485ControlPin);
    
        /**
         * @brief Start VFD communication at specified baud
         * 
         * @param baudRate Baud rate for communication, Check P01.42 @li 0: 2400 @li 1: 4800 @li 2: 9600 @li 3: 19200 @li 4: 38400
         * @return true, if startup is successful 
         * @return false, if startup fails, check last error
         */
        bool begin(uint32_t baudRate = 19200);
    
        // ----- VFD control functions ----- //

        bool setFrequency(uint32_t frequency);

        /**
         * @brief Send a command to the VFD
         * 
         * @param cmd Command code, Use Command enum e.x. Command::Start
         * @return true, if command was sent successfully 
         * @return false, if error durring transmission 
         */
        bool setCommand(Command cmd);
    
        
        // ----- Status and Fault functions ----- //

        /**
         * @brief Get Status Code of VFD
         * 
         * @return uint16_t Status code 
         */
        uint16_t getStatusCode();

        /**
         * @brief Get Fault Code
         * 
         * @return uint32_t Fault Code
         */
        uint32_t getFaultCode();

        /**
         * @brief Get the Status as a String object
         * @details Returns a human readable string that represents VFD status
         * 
         * @return String 
         */
        String getStatusStr();

        /**
         * @brief Get the Fault as a String object
         * @details Returns a human readable string that represents VFD fault
         * 
         * @return String 
         */
        String getFaultStr();
    

        // ----- Parameter access/edit functions ----- //

        /**
         * @brief Reads inverter function and scales value
         * @note Use read parameter functions to return more accurate integer values
         * 
         * @param function Function enum, e.x. Function::INVERTER_TEMPERATURE
         * @return double 
         */
        double readInverterFunction(Function function);

        /**
         * @brief Read 16bit parameter from VFD 
         * @note Not all parameter values fit in 16bit object
         * 
         * @param paramNumber VFD parameter number i.e P1.47 becomes 147
         * @return uint16_t 16bit value of parameter
         */
        uint16_t read16BitParameter(uint16_t paramNumber);

        /**
         * @brief Read 32bit parameter from VFD 
         * 
         * @param paramNumber VFD parameter number i.e P1.47 becomes 147
         * @return uint32_t 32bit value of parameter
         */
        uint32_t read32BitParameter(uint16_t paramNumber);

        /**
         * @brief write 16bit parameter
         * 
         * @param paramNumber VFD parameter number i.e P1.47 becomes 147
         * @param value 16bit value to be stored in parameter
         * @return true, if value was sent successfully
         * @return false, if error occurred
         */
        bool write16BitParameter(uint16_t paramNumber, uint16_t value);

        /**
         * @brief write 32bit parameter
         * 
         * @param paramNumber VFD parameter number i.e P1.47 becomes 147
         * @param value 32bit value to be stored in parameter
         * @return true, if value was sent successfully
         * @return false, if error occurred
         */
        bool write32BitParameter(uint16_t paramNumber, uint32_t value);
    
        // ----- Error code variables and functions ----- //

        /**
         * @brief Last error code received durring communication 
         * @details Error codes are from ModbusMaster library with the addition of a few H100 specific errors
         */
        uint8_t lastErrorCode;       // Last error code

        /**
         * @brief Gets the last Error as a String object
         * @details Returns a human readable string that represents the last error
         * 
         * @return String 
         */
        String getLastErrorStr(); 

        /**
         * @brief Error code for invalid baud rate
         * 
         */
        static constexpr uint8_t INVALID_BAUD = 0xA0; 

        /**
         * @brief Error code for read values outside of expected bounds
         * 
         */
        static constexpr uint8_t INVALID_READ_VALUE = 0xA1; 
    
        /**
         * @brief Maximum speed value governed by P1.47
         * 
         */
        uint32_t MaxSpeed;
    
    private:
    
        ModbusMaster _node; // Modbus node for all communication 
        HardwareSerial& _serial; // Hardware serial port for communication 
        uint8_t _slaveAddress;   // Modbus slave address 
        static uint8_t _MAX485ControlPin; // Control pin for MAX485 DE, RE
        uint32_t _baudRate; // Communication baud rate
        
    
        // Define VFD registers (addresses)
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
