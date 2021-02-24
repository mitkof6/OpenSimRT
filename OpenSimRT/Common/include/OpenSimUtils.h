/**
 * @file OpenSimUtils.h
 *
 * \brief Common OpenSim operations and functions.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef OPENSIM_UTILS_H
#define OPENSIM_UTILS_H

#include "internal/CommonExports.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {
// Type definitions from a moment arm function that accepts a vector and returns
// a matrix.
typedef SimTK::Matrix (*MomentArmFunctionT)(const SimTK::Vector& q);

struct Common_API OpenSimUtils {
    // Generates a unique identifier
    static int generateUID();
    // Extract model's coordinate names in multibody tree order.
    static std::vector<std::string>
    getCoordinateNamesInMultibodyTreeOrder(const OpenSim::Model& model);
    // Extract model's coordinate names in normal order.
    static std::vector<std::string>
    getCoordinateNames(const OpenSim::Model& model);
    // Extract model's muscle names.
    static std::vector<std::string> getMuscleNames(const OpenSim::Model& model);
    // Extract model's actuator names.
    static std::vector<std::string>
    getActuatorNames(const OpenSim::Model& model);
    // Disable actuators.
    static void disableActuators(OpenSim::Model& model);
    // Remove actuators.
    static void removeActuators(OpenSim::Model& model);
    // Get ordered generalized coordinates from storage.
    static OpenSim::TimeSeriesTable
    getMultibodyTreeOrderedCoordinatesFromStorage(const OpenSim::Model& model,
                                                  const std::string stoFilePath,
                                                  double samplingInterval);
    // Load moment arm from a dynamic library
    static MomentArmFunctionT
    getMomentArmFromDynamicLibrary(const OpenSim::Model& model,
                                   std::string libraryPath);

    /**
     * Update the SimTK::State.
     */
    static void updateState(const OpenSim::Model& model, SimTK::State& state,
                            const SimTK::Vector& q, const SimTK::Vector& qDot);
};

} // namespace OpenSimRT

#endif
