/**
 * @file ViconDataStream.h
 *
 * \brief An interface with Vicon server for marker and force acquisition.
 *
 * NOTE: This class has been adapted from the RTOSIM project
 * [https://github.com/RealTimeBiomechanics/rtosim].
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef VICON_DATA_STREAM_H
#define VICON_DATA_STREAM_H

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
#endif
