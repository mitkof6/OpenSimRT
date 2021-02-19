/**
 * @file NGIMUInputDriver.h
 *
 * @brief Concrete implementation of the IMUInputDriver to receive NGIMU data
 * from file.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "InputDriver.h"
#include "NGIMUData.h"

#include <Common/TimeSeriesTable.h>
#include <condition_variable>
#include <exception>
#include <thread>

namespace OpenSimRT {

/**
 * @brief xio NGIMU Input driver for streaming data from file.
 */
class IMU_API NGIMUInputFromFileDriver : public InputDriver<NGIMUData> {
 public:
    /**
     * Create a NGIMU driver that streams data from file at a constant rate.
     */
    NGIMUInputFromFileDriver(const std::string& fileName,
                             const double& sendRate);
    ~NGIMUInputFromFileDriver(); // dtor

    /**
     * Implements the startListening of the base class. Create a thread that
     * streams the data from file at a constant rate.
     */
    virtual void startListening() override;

    /**
     * Determine if the stream from file has ended.
     */
    bool shouldTerminate();

    /**
     * Get data from file as a list of NGIMUData. Implements the stopListening
     * of the base class.
     */
    virtual IMUDataList getData() override;

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a std::vector<NGIMUData>. (i.e., time and row
     * from table)
     */
    std::pair<double, IMUDataList> getFrame();

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a SimTK::Vector. (i.e., time and row from table)
     */
    std::pair<double, SimTK::Vector> getFrameAsVector();

 protected:
    /**
     * Reconstruct a list of NGIMU from a SimTK::Vector.
     */
    IMUDataList fromVector(const SimTK::Vector&);

    // hide it from public since it does nothing
    void stopListening() override {}

 private:
    OpenSim::TimeSeriesTable table;
    double rate;

    // buffers
    SimTK::RowVector frame;
    double time;

    // thread related variables
    std::thread t;
    std::mutex mu;
    std::condition_variable cond;
    std::exception_ptr exc_ptr;
    bool newRow = false;
};
} // namespace OpenSimRT
