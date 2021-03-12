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
 * @file ViconDataStream.h
 *
 * \brief An interface with Vicon server for marker and force acquisition.
 *
 * Acknowledgement: This class has been mainly adapted from the RTOSIM project
 * [https://github.com/RealTimeBiomechanics/rtosim] by Pizzolato et al.
 * http://dx.doi.org/10.1080/10255842.2016.1240789
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "CircularBuffer.h"
#include "InverseDynamics.h"
#include "internal/ViconExports.h"

#include <DataStreamClient.h>
#include <SimTKcommon.h>
#include <map>

namespace OpenSimRT {
/**
 * \brief Connects with Vicon server and collects marker and force plate forces.
 */
class Vicon_API ViconDataStream {
 public:
    struct MarkerData {
        double time;
        std::map<std::string, SimTK::Vec3> markers;
    };

    struct ForceData {
        double time;
        std::map<std::string, ExternalWrench::Input> externalWrenches;
    };

    ViconDataStream(std::vector<SimTK::Vec3> labForcePlatePositions);

    void connect(std::string hostName);
    void initialize(ViconDataStreamSDK::CPP::Direction::Enum xAxis,
                    ViconDataStreamSDK::CPP::Direction::Enum yAxis,
                    ViconDataStreamSDK::CPP::Direction::Enum zAxis);
    void startAcquisition();

    CircularBuffer<2000, MarkerData> markerBuffer;
    CircularBuffer<2000, ForceData> forceBuffer;
    std::vector<std::string> markerNames;
    std::vector<std::string> forcePlateNames;

    bool shouldTerminate;

 private:
    void getFrame();

    ViconDataStreamSDK::CPP::Client client;
    std::vector<SimTK::Vec3> labForcePlatePositions;
    int forcePlates;
    double previousMarkerDataTime, previousForceDataTime;
};

/**
 * Convert string to Direction::Enum. The direction must have the same name as:
 *
 * # enum Enum
 * {
 *   Up,
 *   Down,
 *   Left,
 *   Right,
 *   Forward,
 *   Backward
 * };
 */
Vicon_API ViconDataStreamSDK::CPP::Direction::Enum
stringToDirection(std::string direction);

} // namespace OpenSimRT
