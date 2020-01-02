/**
 * @file InverseKinematics.h
 *
 * \brief Utilities for performing inverse kinematics.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "internal/RealTimeExports.h"

#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <simbody/internal/AssemblyCondition_Markers.h>
#include <simbody/internal/AssemblyCondition_OrientationSensors.h>

namespace OpenSimRT {

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
 *
 * TODO:
 *
 * 1) Support for IKCoordinateTask
 */
class RealTime_API InverseKinematics {
 public: /* public data structures */
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

 public: /* public interface */
    /**
     * Inverse kinematics constructor, that accepts a model, the marker tasks
     * (if any), the IMU tasks (if any), the constraint weight (Infinity) and
     * accuracy (1.0e-5). Reducing the value of constraint weight can
     * significantly reduce the delay.
     */
    InverseKinematics(const OpenSim::Model& model,
                      const std::vector<MarkerTask>& markerTasks,
                      const std::vector<IMUTask>& imuTasks,
                      double constraintsWeight, double accuracy);
    /**
     * Track an input frame (marker and/or IMU target positions/orientation).
     */
    Output solve(const Input& input);
    /**
     * Initialize inverse kinematics log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed kinematics.
     */
    OpenSim::TimeSeriesTable initializeLogger();

 public: /* static methods */
    /**
     * Creates marker tasks and observation order from IKTaskSet.
     */
    static void
    createMarkerTasksFromIKTaskSet(const OpenSim::Model& model,
                                   const OpenSim::IKTaskSet& ikTaskSet,
                                   std::vector<MarkerTask>& markerTasks,
                                   std::vector<std::string>& observationOrder);
    /**
     * An interface of InverseKinematics with MarkerData. This function reads
     * the marker data (marker names extracted from column names) and constructs
     * the InverseKinematics::MarkerTask tasks and well as the observation
     * order.
     */
    static void
    createMarkerTasksFromMarkerData(const OpenSim::Model& model,
                                    const OpenSim::MarkerData& markerData,
                                    std::vector<MarkerTask>& markerTasks,
                                    std::vector<std::string>& observationOrder);
    /**
     * Creates marker tasks and observation order from marker names.
     */
    static void createMarkerTasksFromMarkerNames(
            const OpenSim::Model& model,
            const std::vector<std::string>& markerNames,
            std::vector<MarkerTask>& markerTasks,
            std::vector<std::string>& observationOrder);
    /**
     * An interface of InverseKinematics with MarkerData in case that IMU
     * measurements are stored into a .trc file. No relative orientations of the
     * IMU sensor with respect to a body is specified. The column names must
     * refer to existing bodies in the model. This function reads the marker
     * data and constructs the InverseKinematics::IMUTask tasks and well as the
     * observation order.
     */
    static void
    createIMUTasksFromMarkerData(const OpenSim::Model& model,
                                 const OpenSim::MarkerData& markerData,
                                 std::vector<IMUTask>& imuTasks,
                                 std::vector<std::string>& observationOrder);
    /**
     * When the IMUs are assigned manually.
     */
    static void createIMUTasksFromObservationOrder(
            const OpenSim::Model& model,
            const std::vector<std::string>& observationOrder,
            std::vector<IMUTask>& imuTasks);
    /**
     * An interface of InverseKinematics with MarkerFrame. This function
     * constructs the InverseKinematics::Input from the MarkerFrame. In case
     * that, the .trc file contains the absolute orientation of orientation
     * sensors only, one can set the isIMU flag to populate the IMU observations
     * in the InverseKinematics::Input instead of the marker observations.
     */
    static Input
    getFrameFromMarkerData(int i, OpenSim::MarkerData& markerData,
                           const std::vector<std::string>& observationOrder,
                           bool isIMU);

 private: /* private members */
    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<SimTK::Assembler> assembler;
    SimTK::ReferencePtr<SimTK::Markers> markerAssemblyConditions;
    SimTK::ReferencePtr<SimTK::OrientationSensors> imuAssemblyConditions;
    bool assembled;
};

} // namespace OpenSimRT

#endif
