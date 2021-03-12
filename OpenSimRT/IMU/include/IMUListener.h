/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file IMUListener.h
 *
 * @brief Implements a generic interface for IMU Listeners. It is based on the
 * Adapter Design Pattern to decouple the implementation of IMU data acquisition
 * drivers from the IMU dependencies (e.g., Wi-Fi, Bluetooth, OSC protocol and
 * oscpack library, etc).
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "InputDriver.h"
#include "internal/IMUExports.h"

#include <memory>
#include <string>

namespace OpenSimRT {

/**
 * @brief Interface class for IMU listener implementations.
 *
 */
template <typename T> class ListenerAdapter {
 public:
    ListenerAdapter() = default;
    virtual ~ListenerAdapter() = default;
    std::shared_ptr<InputDriver<T>> driver; // pointer to base class imu driver
    std::string name;                       // imu name
    std::string ip;                         // imu ip
    int port;                               // imu port

    /**
     * Push IMU data (indexed by the listener's port) to the driver's buffer.
     */
    void pushDataToManagerBuffer(const int& port, const T& input) {
        driver->buffer[port]->add(input);
    }
};

} // namespace OpenSimRT
