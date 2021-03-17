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
 * @file NGIMUInputDriver.h
 *
 * @brief Concrete implementation of the IMUInputDriver to receive data from the
 * NGIMU units [https://x-io.co.uk/ngimu/].
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "InputDriver.h"
#include "NGIMUData.h"
#include "ip/UdpSocket.h"
#include <Common/TimeSeriesTable.h>
#include <vector>

namespace OpenSimRT {

/**
 * @brief xio NGIMU Driver concrete implementation.
 */
class IMU_API NGIMUInputDriver : public InputDriver<NGIMUData> {
    /**
     * Representation of the data acquired from all IMU sensors.
     */
    typedef std::vector<std::pair<double, SimTK::Vector>> DataPack;

 public:
    NGIMUInputDriver() = default; // default ctor
    /**
     * Setup the listening sockets in the constructor.
     */
    NGIMUInputDriver(const std::vector<std::string>& imuLabels,
                     const std::vector<std::string>& ips,
                     const std::vector<int>& ports);
    ~NGIMUInputDriver() = default; // dtor

    /**
     * Setup listening sockets.
     */
    void setupInput(const std::vector<std::string>& imuLabels,
                    const std::vector<std::string>& ips,
                    const std::vector<int>& ports);

    /**
     * Setup NGIMU internal settings.
     */
    void setupTransmitters(const std::vector<std::string>& remoteIPs,
                           const std::vector<int>& remotePorts,
                           const std::string& localIP,
                           const std::vector<int>& localPorts);

    /**
     * Attaches sockets to listeners. (Implements the startListening function
     * of the base class.)
     */
    virtual void startListening() override;

    /**
     * Terminate IMU data acquisition. (Implements the startListening function
     * of the base class.)
     */
    virtual void stopListening() override;

    /**
     * Receive the NGIMU data stored in the driver's buffer. (Implements the
     * startListening function of the base class.)
     */
    virtual IMUDataList getData() const override;

    /**
     * Represent the list of NGIMUData as a SimTK::Vector.
     */
    static SimTK::Vector asVector(const IMUDataList& imuDataFrame);

    /**
     * Reconstruct a list of NGIMUData from a SimTK::Vector.
     */
    static IMUDataList fromVector(const SimTK::Vector& v);

    /**
     * Represent a list of NGIMUData as a list of std::pairs containing the
     * timestamp and the acompanied sensor measurement of each the sensor as a
     * SimTK::Vector, i.e., {<time, quaternion>_imu_1, <time,
     * acceleration>_imu_1 .... <time, quaternion>_imu_N, <time,
     * acceleration>_imu_N}.
     */
    static DataPack asPack(const IMUDataList& imuDataList);

    /**
     * Create an NGIMUData logger. Data from all sensors are stored with a
     * common timestamp.
     */
    OpenSim::TimeSeriesTable initializeLogger() const;

 private:
    SocketReceiveMultiplexer mux; // multipler for polling listener sockets
    std::vector<std::unique_ptr<UdpSocket>> udpSockets; // upd sockets
};
} // namespace OpenSimRT
