/**
 * @file InverseKinematics.h
 *
 * \brief Utilities for performing inverse kinematics.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Common/MarkerData.h>
#include <simbody/internal/AssemblyCondition_Markers.h>
#include <simbody/internal/AssemblyCondition_OrientationSensors.h>
#include "internal/RealTimeExports.h"

// forward declaration
class CSVLogger;

/**
 * \brief Solves the inverse kinematics problem.
 *
 * Important note: If the model contains constraints, then the tracking is very
 * slow. Constraint tracking could be controlled by changing the
 * constraintsWeight. If the constraintsWeight is set to SimTK::Infinity, then
 * the constraints are tracked perfectly. If it is set to zero, then the problem
 * is relaxed thus can be solved faster. Note that, there are models that use
 * coordinate coupled constraints which eventually are not important for the
 * tracking as day depend on some coordinate (e.g., patella as a function of
 * knee angle). In this case, they could be ignored without affecting the
 * overall solution and speed up the process.
 */
class RealTime_API InverseKinematics {
 public:
    struct MarkerTask {
        std::string name;
	std::string marker;
        double weight;
    };
    struct IMUTask {
	std::string name;
	std::string body;
	SimTK::Rotation orientation;
	double weight;
    };
    struct Input {
	double t;
	SimTK::Array_<SimTK::Vec3> markerObservations;
	SimTK::Array_<SimTK::Rotation> imuObservations;
    };
    struct Output {
	double rms;
	double t;
	SimTK::Vector q;
    };
    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<SimTK::Assembler> assembler;
    SimTK::ReferencePtr<SimTK::Markers> markerAssemblyConditions;
    SimTK::ReferencePtr<SimTK::OrientationSensors> imuAssemblyConditions;
    SimTK::ReferencePtr<CSVLogger> logger;
    bool assembled;
 public:
    InverseKinematics(std::string modelFile,
		      double constraintsWeight,
                      const std::vector<MarkerTask>& markerTasks,
		      const std::vector<IMUTask>& imuTasks);
    Output solve(const Input& input);
};

/**
 * Creates marker tasks and observation order from IKTaskSet. 
 */
RealTime_API void createMarkerTasksFromIKTaskSet(
    const OpenSim::Model& model,
    const OpenSim::IKTaskSet& ikTaskSet,
    std::vector<InverseKinematics::MarkerTask>& markerTasks,
    std::vector<std::string>& observationOrder);

/**
 * An interface of InverseKinematics with MarkerData. This function reads the
 * marker data (marker names extracted from column names) and constructs the
 * InverseKinematics::MarkerTask tasks and well as the observation order.
 */
RealTime_API void createMarkerTasksFromMarkerData(
    const OpenSim::Model& model,
    const OpenSim::MarkerData& markerData,
    std::vector<InverseKinematics::MarkerTask>& markerTasks,
    std::vector<std::string>& observationOrder);

/**
 * TODO
 */
RealTime_API void createMarkerTasksFromMarkerNames(
    const OpenSim::Model& model,
    const std::vector<std::string>& markerNames,
    std::vector<InverseKinematics::MarkerTask>& markerTasks,
    std::vector<std::string>& observationOrder);

/**
 * An interface of InverseKinematics with MarkerData in case that imu
 * measurements are stored into a .trc file. No relative orientations of the IMU
 * sensor with respect to a body is specified. The column names must refer to
 * existing bodies in the model. This function reads the marker data and
 * constructs the InverseKinematics::IMUTask tasks and well as the observation
 * order.
 */
RealTime_API void createIMUTasksFromMarkerData(
    const OpenSim::Model& model,
    const OpenSim::MarkerData& markerData,
    std::vector<InverseKinematics::IMUTask>& imuTasks,
    std::vector<std::string>& observationOrder);

/**
 * When the IMUs are assigned manually.
 */
RealTime_API void createIMUTasksFromObservationOrder(
    const OpenSim::Model& model,
    const std::vector<std::string>& observationOrder,
    std::vector<InverseKinematics::IMUTask>& imuTasks);

/**
 * An interface of InverseKinematics with MarkerFrame. This function constructs
 * the InverseKinematics::Input from the MarkerFrame. In case that, the .trc
 * file contains the absolute orientation of orientation sensors only, one can
 * set the isIMU flag to populate the IMU observations in the
 * InverseKinematics::Input instead of the marker observations.
 */
RealTime_API InverseKinematics::Input getFrameFromMarkerData(
    int i,
    OpenSim::MarkerData& markerData,
    const std::vector<std::string>& observationOrder,
    bool isIMU);

#endif
