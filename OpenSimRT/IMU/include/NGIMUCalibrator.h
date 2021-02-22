/**
 * @file NGIMUCalibrator.h
 *
 * @brief Calibration of the IMU data required before using the IK module.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "InverseKinematics.h"
#include "NGIMUInputDriver.h"
#include "internal/IMUExports.h"

#include <Simulation/Model/Model.h>
#include <string>

namespace OpenSimRT {

/**
 * Class used to calibrate NGIMUData for solving the IK module with orientation
 * data. Initially, it calibrates the IK tasks based on the data from the
 * initial pose of the subject that corresponds to the osim model pose. The
 * subject should stand in pose for a number of seconds or a specific number of
 * samples acquired from stream. After that the calibrator is used to transform
 * the received data in the dynamic simulation based on the initial
 * measurements. The transformations applied are the transformation of the
 * ground-reference of the sensors to the OpenSim's ground reference frame, and
 * the a heading transformation of the subject so that the heading axis of the
 * subject coinsides with anterior axis in OpenSim (X-axis).
 */
class IMU_API NGNGIMUCalibrator {
 public:
    /**
     * Construct a calibrator object.
     */
    NGNGIMUCalibrator(const OpenSim::Model& otherModel,
                      InputDriver<NGIMUData>* const driver,
                      const std::vector<std::string>& observationOrder);

    /**
     * Time duration (in seconds) of the static phase during calibration.
     */
    void recordTime(const double& timeout);

    /**
     * Time duration (in number number of acquired samples) of the static phase
     * during calibration.
     */
    void recordNumOfSamples(const size_t& numSamples);

    /**
     * Set the rotation sequence of the axes (in degrees) that form the
     * transformation between the sensor's reference frame and the OpenSim's
     * ground reference frame.
     */
    SimTK::Rotation setGroundOrientationSeq(const double& xDegrees,
                                            const double& yDegrees,
                                            const double& zDegrees);
    /**
     * Compute the transformation for the heading correction based on the
     * measurements acquired during the static phase.
     */
    SimTK::Rotation computeHeadingRotation(const std::string& baseImuName,
                                           const std::string& imuDirectionAxis);
    /**
     * Calibrate the IK IMUTasks prior the construction of the IK module.
     */
    void calibrateIMUTasks(std::vector<InverseKinematics::IMUTask>& imuTasks);

    /**
     * Calibrate NGIMU data acquired from stream and create the IK input.
     */
    InverseKinematics::Input
    transform(const std::pair<double, std::vector<NGIMUData>>& imuData,
              const SimTK::Array_<SimTK::Vec3>& markerData);

 private:
    /**
     * Compute the mean pose from the data acquired during the static phase.
     * TODO hasn't been implemented appropriatly yet. Only return one sample.
     */
    void computeAvgStaticPose();

    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<InputDriver<NGIMUData>> m_driver;
    std::vector<NGIMUData> initIMUData;
    std::vector<std::vector<NGIMUData>> quatTable;
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;
    std::vector<std::string> imuBodiesObservationOrder;
    SimTK::Rotation R_GoGi;
    SimTK::Rotation R_heading;
};
} // namespace OpenSimRT
