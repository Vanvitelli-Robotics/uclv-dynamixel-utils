#include <iostream>
#include "uclv_dynamixel_utils/palm_sensor.hpp"

using namespace uclv::dynamixel_utils;

/**
 * @brief Constructor of the PalmSensor class.
 * 
 * This constructor initializes the PalmSensor class with the provided parameters.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 * @param portHandler Pointer to the port handler.
 * @param packetHandler Pointer to the packet handler.
 */

PalmSensor::PalmSensor(const std::string& serial_port, int baudrate, float protocol_version, 
             dynamixel::PortHandler *const& portHandler, dynamixel::PacketHandler *const& packetHandler
)
    : 
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version),
    portHandler_(portHandler),
    packetHandler_(packetHandler) {}

/**
 * @brief Constructor of the PalmSensor class.
 * 
 * This constructor initializes the PalmSensor class with the provided parameters.
 * It creates the port and packet handlers based on the serial port and protocol version.
 * 
 * @param serial_port The serial port to which the motor is connected.
 * @param baudrate The baudrate of the serial communication.
 * @param protocol_version The version of the protocol used.
 */
PalmSensor::PalmSensor(const std::string& serial_port, int baudrate, float protocol_version)
    :
    serial_port_(serial_port),
    baudrate_(baudrate),
    protocol_version_(protocol_version)
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    }

/**
 * @brief Default constructor of the PalmSensor class.
 * 
 * This constructor initializes the PalmSensor class with default values.
 */
PalmSensor::PalmSensor()
    :
    serial_port_(""),
    baudrate_(0),
    protocol_version_(0),
    portHandler_(nullptr),
    packetHandler_(nullptr) {}

/**
 * @brief Gets the ID of the main board.
 * 
 * @return The identifier of the main board.
 */
uint8_t PalmSensor::getId() {
    return id_;
}

/**
 * @brief Gets the address for the target position.
 * 
 * @return The address for the target position.
 */
uint16_t PalmSensor::getAddrPalmSensor() const {
    return addrPalmSensor_;
}

/**
 * @brief Reads the current state of the palm sensor
 * 
 * @param id The identifier of the motor.
 * @return Current distance measured
 */
uint16_t PalmSensor::readFromPalmSensor() {
    return read2FromAddress(id_, getAddrPalmSensor());
}

/**
 * @brief Reads a 2-byte value from a specified address on the motor.
 * 
 * @param id The identifier of the motor.
 * @param address The address to read the data from.
 * @return The data read from the address.
 */
uint16_t PalmSensor::read2FromAddress(uint8_t id, uint16_t address) {
    uint16_t data;
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, address, &data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to read from address: " << address << " for [ID:" << (unsigned int)id << "]" << std::endl;
    }
    
    return data;
}

