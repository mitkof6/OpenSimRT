/**
 * @file NGIMUData.h
 *
 * @brief Struct representation of the data acquired from NGIMU units
 * [https://x-io.co.uk/ngimu/]
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "TimeDefinitions.h"
#include "internal/IMUExports.h"

#include <SimTKcommon.h>

namespace OpenSimRT {

/**
 * @brief A struct representation of the data acquired from an NGIMU unit. It
 * contains quaternion estimations, raw sensor data (acceleration, gyroscope,
 * magnetometer, barometer), linear acceleration values and altitude
 * measurements.
 */
struct IMU_API NGIMUData {
    /**
     * Alternative representation of the NGIMUData.
     */
    typedef std::vector<std::pair<double, SimTK::Vector>> NGIMUPack;

    struct Quaternion {
        SimTK::Quaternion q;
        double timeStamp;
    };
    struct Sensors {
        SimTK::Vec3 acceleration; // in g
        SimTK::Vec3 gyroscope;    // in circ/sec
        SimTK::Vec3 magnetometer; // in uT
        SimTK::Vec1 barometer;    // in hPa
        double timeStamp;
    };
    struct LinearAcceleration {
        SimTK::Vec3 acceleration;
        double timeStamp;
    };
    struct Altitude {
        SimTK::Vec1 measurement;
        double timeStamp;
    };

    Sensors sensors;
    Quaternion quaternion;
    LinearAcceleration linear;
    Altitude altitude;

    /**
     * Total size in bytes of the NGIMUData object.
     */
    static constexpr int size() { return 18; }

    /**
     * Represent the sensor values from NGIMUData as a SimTK::Vector.
     */
    SimTK::Vector asVector() const;

    /**
     * Get sensor values from a SimTK::Vector.
     */
    void fromVector(const SimTK::Vector& v);

    /**
     * Represent NGIMUData as a NGIMUPack (a list of std::pairs, where each pair
     * contains the timeStamp and the accompanied measurement of the sensor in
     * the IMU as a SimTK::Vector, e.g. {<time, acceleration>, <time,
     * quaternion>}). Each sensor in the IMU can have its own sampling rate, and
     * thus the timestamp in each measurement can differ from one to another.
     */
    NGIMUPack getAsPack() const;

    /**
     * Assign values to an NGIMUData object from a NGIMUPack (a list of pairs of
     * SimTK::Vectors containing the timeStamps and measurements of each sensor
     * in a IMU).
     */
    void setFromPack(const NGIMUPack& pack);
};
} // namespace OpenSimRT
