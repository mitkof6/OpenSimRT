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
 * @file RealTimeAnalysisLegacy.h
 *
 * @brief This file contains a facade class as a convinient interface for
 * performing musculoskeletal analysis in mocap scenarios.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "CircularBuffer.h"
#include "InverseDynamics.h"
#include "InverseKinematics.h"
#include "JointReaction.h"
#include "MuscleOptimization.h"
#include "Visualization.h"
#include "internal/RealTimeExports.h"
#include <iostream>

namespace OpenSimRT {

/**
 * \brief TODO
 */
struct RealTime_API MotionCaptureInput {
    InverseKinematics::Input ikFrame;
    std::vector<ExternalWrench::Input> externalWrenches;
};

/**
 * \brief A higher order function that is an interface between the motion
 * capture system and the InverseKinematics input. Termination of the
 * acquisition should be handled by throwing an exception.
 */
typedef std::function<MotionCaptureInput()> DataAcquisitionFunction;

/**
 * \brief TODO
 */
class RealTime_API RealTimeAnalysisLegacy {
 public:
    struct UnfilteredData {
        double t;
        SimTK::Vector q;
        std::vector<ExternalWrench::Input> externalWrenches;
        SimTK::Vector toVector();
        int size();
    };
    struct FilteredData {
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
        std::vector<ExternalWrench::Input> externalWrenches;
    };
    struct Parameters {
        bool useVisualizer;
        bool solveMuscleOptimization;
        double fc;          // cut of frequency
        double filterOrder; // order of the IIR filter
        int samplesDelay;
        std::vector<InverseKinematics::MarkerTask> ikMarkerTasks;
        std::vector<InverseKinematics::IMUTask> ikIMUTasks;
        double ikConstraintsWeight;
        double ikAccuracy;
        std::vector<ExternalWrench::Parameters> wrenchParameters;
        MuscleOptimization::OptimizationParameters muscleOptimizationParameters;
        MomentArmFunctionT momentArmFunction;
        DataAcquisitionFunction dataAcquisitionFunction;
        std::vector<std::string> reactionForceOnBodies;
    } parameters;
    SimTK::ReferencePtr<InverseKinematics> inverseKinematics;
    SimTK::ReferencePtr<InverseDynamics> inverseDynamics;
    SimTK::ReferencePtr<MuscleOptimization> muscleOptimization;
    SimTK::ReferencePtr<JointReaction> jointReaction;
    SimTK::ReferencePtr<BasicModelVisualizer> visualizer;
    std::vector<ForceDecorator*> GRFDecorators;
    std::vector<ForceDecorator*> reactionForceDecorators;
    CircularBuffer<2000, UnfilteredData> buffer;
    OpenSim::TimeSeriesTable qFiltered, qDotFiltered, qDDotFiltered;
    double previousAcquisitionTime;

 public:
    RealTimeAnalysisLegacy(const OpenSim::Model& model,
                           const Parameters& parameters);
    void run();
    void exportResults(std::string dir);

 private:
    void acquisition();
    void processing();
    FilteredData filterData();
};

} // namespace OpenSimRT
