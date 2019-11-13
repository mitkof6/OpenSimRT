/**
 * @file ViconDataStream.h
 *
 * \brief An interface with Vicon server for marker and force acquisition.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef VICON_DATA_STREAM_H
#define VICON_DATA_STREAM_H

#include <iostream>
#include <DataStreamClient.h>
#include "InverseDynamics.h"
#include "SimulationUtils.h"
#include "internal/ViconExports.h"

/**
 * \brief Connects with Vicon server and collects marker and force plate forces.
 */
class Vicon_API ViconDataStream {
public:
    ViconDataStreamSDK::CPP::Client client;
    std::vector<SimTK::Vec3> labForcePlatePositions;
    std::vector<std::string> forcePlateNames;
    std::vector<std::string> markerNames;
    int forcePlates;
    double previousMarkerDataTime, previousForceDataTime;
    struct MarkerData {
        double time;
        std::map<std::string, SimTK::Vec3> markers;
    };
    struct ForceData {
        double time;
        std::map<std::string, ExternalWrench::Input> externalWrenches;
    };
    CircularBuffer<2000, MarkerData> markerBuffer;
    CircularBuffer<2000, ForceData> forceBuffer;
    bool shouldTerminate;
public:
    ViconDataStream(std::vector<SimTK::Vec3> labForcePlatePositions);
    void connect(std::string hostName);
    void initialize(ViconDataStreamSDK::CPP::Direction::Enum xAxis,
		    ViconDataStreamSDK::CPP::Direction::Enum yAxis,
		    ViconDataStreamSDK::CPP::Direction::Enum zAxis);
    void startAcquisition();
private:
    void getFrame();
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
Vicon_API ViconDataStreamSDK::CPP::Direction::Enum stringToDirection(std::string direction);

#endif
