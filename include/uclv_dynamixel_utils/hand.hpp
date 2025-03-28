#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "colors.hpp"
#include "motor.hpp"
#include "fingermotor.hpp"
#include "wristmotor.hpp"
#include "palm_sensor.hpp"

namespace uclv::dynamixel_utils
{

    /**
     * @class Hand
     * @brief The Hand class represents a robotic hand with multiple motors.
     */
    class Hand
    {
    private:
        std::string serial_port_; ///< Serial port to which the hand is connected.
        int baudrate_;            ///< Baudrate for the serial communication.
        float protocol_version_;  ///< Protocol version used for communication.

        dynamixel::PortHandler *portHandler_;     ///< Pointer to the port handler.
        dynamixel::PacketHandler *packetHandler_; ///< Pointer to the packet handler.

        std::vector<std::shared_ptr<FingerMotor>> fingerMotors_; ///< Vector of finger motors.
        std::vector<std::shared_ptr<WristMotor>> wristMotors_;   ///< Vector of wrist motors.

        std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_ = nullptr; ///< Group sync write handler.
        std::unique_ptr<dynamixel::GroupSyncRead> groupSyncRead_ = nullptr;   ///< Group sync read handler.
        std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_ = nullptr;   ///< Group bulk read handler.
        std::unique_ptr<dynamixel::GroupBulkWrite> groupBulkWrite_ = nullptr; ///< Group bulk write handler.

        std::shared_ptr<PalmSensor> palm_sensor_ ;          // Pointer to the palm sensor

    public:
        /**
         * @brief Constructs a Hand object.
         *
         * @param serial_port The serial port to which the hand is connected.
         * @param baudrate The baudrate for the serial communication.
         * @param protocol_version The protocol version used for communication.
         * @param portHandler Pointer to the port handler.
         * @param packetHandler Pointer to the packet handler.
         */
        Hand(const std::string &serial_port, int baudrate, float protocol_version,
             dynamixel::PortHandler *const &portHandler, dynamixel::PacketHandler *const &packetHandler);

        /**
         * @brief Constructs a Hand object.
         *
         * @param serial_port The serial port to which the hand is connected.
         * @param baudrate The baudrate for the serial communication.
         * @param protocol_version The protocol version used for communication.
         */
        Hand(const std::string &serial_port, int baudrate, float protocol_version);

        /**
         * @brief Default constructor.
         */
        Hand();

        /**
         * @brief Destructor.
         */
        ~Hand();

        /**
         * @brief Initializes the hand.
         */
        bool initialize();

        /**
         * @brief Sets the serial port.
         *
         * @param serial_port The serial port to which the hand is connected.
         */
        void setSerialPort(const std::string &serial_port);

        /**
         * @brief Sets the baudrate.
         *
         * @param baudrate The baudrate for the serial communication.
         */
        void setBaudrate(int baudrate);

        /**
         * @brief Sets the protocol version.
         *
         * @param protocol_version The protocol version used for communication.
         */
        void setProtocolVersion(float protocol_version);

        /**
         * @brief Gets the serial port.
         *
         * @return The serial port to which the hand is connected.
         */
        std::string getSerialPort();

        /**
         * @brief Gets the baudrate.
         *
         * @return The baudrate for the serial communication.
         */
        int getBaudrate();

        /**
         * @brief Gets the protocol version.
         *
         * @return The protocol version used for communication.
         */
        float getProtocolVersion();

        /**
         * @brief Sets the port handler.
         *
         * @param portHandler Pointer to the port handler.
         */
        void setPortHandler(dynamixel::PortHandler *portHandler);

        /**
         * @brief Sets the packet handler.
         *
         * @param packetHandler Pointer to the packet handler.
         */
        void setPacketHandler(dynamixel::PacketHandler *packetHandler);

        /**
         * @brief Sets the serial port to low latency mode.
         *
         * @param serial_port The serial port to which the hand is connected.
         */
        void setSerialPortLowLatency(const std::string &serial_port);

        /**
         * @brief Creates a finger motor.
         *
         * @param id The identifier of the motor.
         * @return A shared pointer to the created finger motor.
         */
        std::shared_ptr<FingerMotor> createFingerMotor(uint8_t id);

        /**
         * @brief Creates a wrist motor.
         *
         * @param id The identifier of the motor.
         * @return A shared pointer to the created wrist motor.
         */
        std::shared_ptr<WristMotor> createWristMotor(uint8_t id);

        /**
         * @brief Adds a finger motor to the hand.
         *
         * @param id The identifier of the motor.
         * @return A constant reference to the vector of finger motors.
         */
        const std::vector<std::shared_ptr<FingerMotor>> &addFingerMotor(uint8_t id);

        /**
         * @brief Adds a wrist motor to the hand.
         *
         * @param id The identifier of the motor.
         * @return A constant reference to the vector of wrist motors.
         */
        const std::vector<std::shared_ptr<WristMotor>> &addWristMotor(uint8_t id);

        /**
         * @brief Gets the finger motors.
         *
         * @return A vector of shared pointers to the finger motors.
         */
        std::vector<std::shared_ptr<FingerMotor>> getFingerMotors();

        /**
         * @brief Gets the wrist motors.
         *
         * @return A vector of shared pointers to the wrist motors.
         */
        std::vector<std::shared_ptr<WristMotor>> getWristMotors();

        /**
         * @brief Prints the finger motors.
         */
        void printFingerMotors() const;

        /**
         * @brief Prints the wrist motors.
         */
        void printWristMotors() const;

        /**
         * @brief Removes a finger motor from the hand.
         *
         * @param id The identifier of the motor to be removed.
         */
        void removeFingerMotor(uint8_t id);

        /**
         * @brief Removes a wrist motor from the hand.
         *
         * @param id The identifier of the motor to be removed.
         */
        void removeWristMotor(uint8_t id);

        /**
         * @brief Moves a finger motor to a specified position.
         *
         * @param id The identifier of the motor.
         * @param position The target position.
         */
        void moveFingerMotor(const uint8_t &id, const float &position);

        /**
         * @brief Moves a wrist motor to a specified position.
         *
         * @param id The identifier of the motor.
         * @param position The target position.
         */
        void moveWristMotor(const uint8_t &id, const float &position);

        /**
         * @brief Reads the position of a finger motor.
         *
         * @param id The identifier of the motor.
         * @return The position of the motor.
         */
        uint16_t readFingerMotorPosition(const uint8_t &id);

        /**
         * @brief Reads the position of a wrist motor.
         *
         * @param id The identifier of the motor.
         * @return The position of the motor.
         */
        uint16_t readWristMotorPosition(const uint8_t &id);

        /**
         * @brief Moves multiple motors using bulk write.
         *
         * @param ids A vector of motor identifiers.
         * @param positions A vector of target positions.
         */
        void moveMotors(const std::vector<uint8_t> &ids, const std::vector<float> &positions);

        /**
         * @brief Reads the positions of multiple motors using bulk read.
         *
         * @param ids A vector of motor identifiers.
         * @return A vector of positions corresponding to the motor identifiers.
         */
        std::vector<uint32_t> readMotorsPositions(const std::vector<uint8_t> &ids);

        /******** CODE FOR PALM SENSOR *******/
        /**
         * @brief Gets the palm sensor.
         *
         * @return A shared pointer to the palm sensor.
         */
        std::shared_ptr<PalmSensor> getPalmSensor();

        /**
         * @brief Creates the palm sensor.
         *
         * @return A shared pointer to the created palm sensor.
         */
        std::shared_ptr<PalmSensor> createPalmSensor();

         /**
         * @brief Reads the current state of the palm sensor.
         *
         * @return Distance measured.
         */
        uint16_t readPalmSensorState();
    };
}
