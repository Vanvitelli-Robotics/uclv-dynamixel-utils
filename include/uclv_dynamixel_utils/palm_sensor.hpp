/** .hpp of hardware interface plugin **/

// Copyleft 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "colors.hpp"
#include <string>

namespace uclv::dynamixel_utils
{

    /**
     * @class PalmSensor
     * @brief The PalmSensor class represents a single motor connected to the system.
     */
    class PalmSensor
    {
    private:
        uint8_t id_ = 40;              ///< Identifier of the main board of left hand.
        std::string serial_port_;      ///< Serial port to which the motor is connected.
        int baudrate_;                 ///< Baudrate for the serial communication.
        float protocol_version_;       ///< Protocol version used for communication.

        uint16_t addrPalmSensor_ = 94; ///< Address for Torque Enable in RH8D's control table.

        dynamixel::PortHandler *portHandler_;     ///< Pointer to the port handler.
        dynamixel::PacketHandler *packetHandler_; ///< Pointer to the packet handler.

    public:
        /**
         * @brief Constructs a PalmSensor object with specified parameters.
         *
         * @param serial_port The serial port to which the motor is connected.
         * @param baudrate The baudrate for the serial communication.
         * @param protocol_version The protocol version used for communication.
         * @param portHandler Pointer to the port handler.
         * @param packetHandler Pointer to the packet handler.
         */
        PalmSensor(const std::string &serial_port, int baudrate, float protocol_version,
              dynamixel::PortHandler *const &portHandler, dynamixel::PacketHandler *const &packetHandler);

        /**
         * @brief Constructs a PalmSensor object with specified parameters.
         *
         * @param serial_port           The serial port to which the motor is connected.
         * @param baudrate              The baudrate for the serial communication.
         * @param protocol_version      The protocol version used for communication.
         */
        PalmSensor(const std::string &serial_port, int baudrate, float protocol_version);

        /**
         * @brief Default constructor.
         */
        PalmSensor();

        /**
         * @brief Gets the identifier of the main_board.
         *
         * @return Identifier of the main_board.
         */
        uint8_t getId();

        /**
         * @brief Gets the address for the palm sensor.
         *
         * @return The address for the palm sensor..
         */
        uint16_t getAddrPalmSensor() const;

        /**
         * @brief Reads the current state of palm sensor (distance).
         *
         * @return Current distance measured
         */
        uint16_t readFromPalmSensor();

        /**
         * @brief Reads two bytes from the specified address of the motor.
         *
         * @param id Identifier of the motor.
         * @param address Address to read from.
         * @return Read data.
         */
        uint16_t read2FromAddress(uint8_t id, uint16_t address);
    };
}
