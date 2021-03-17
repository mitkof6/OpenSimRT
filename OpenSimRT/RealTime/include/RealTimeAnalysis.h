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
 * @file RealTimeAnalysis.h
 *
 * @brief This file contains a facade class as a convinient interface for
 * performing musculoskeletal analysis in mocap scenarios.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>, Filip Konstantinos
 * <filip.k@ece.upatras.gr>
 */
#pragma once

#include "CircularBuffer.h"
#include "InverseDynamics.h"
#include "InverseKinematics.h"
#include "JointReaction.h"
#include "MuscleOptimization.h"
#include "OpenSimUtils.h"
#include "RealTimeAnalysis.h"
#include "SignalProcessing.h"
#include "internal/RealTimeExports.h"
#include <atomic>

namespace OpenSimRT {
/**
 * @brief Convinient struct container of the marker/orientation data and the
 * external measured wrenches in Mocap scenarios.
 */
struct RealTime_API MotionCaptureInput {
    InverseKinematics::Input IkFrame;
    std::vector<ExternalWrench::Input> ExternalWrenches;
};

/**
 * @brief A higher order function that is an interface between the motion
 * capture system and the InverseKinematics input. Termination of the
 * acquisition should be handled by throwing an exception.
 */
typedef std::function<MotionCaptureInput()> DataAcquisitionFunction;

/**
 * @brief Provides a convinient interface for performing RT musculoskeletal
 * analysis. It creates one thread for the data acquisition, IK and filtering,
 * and one processing thread for the rest the analysis (ID, SO and JR).
 */
class RealTime_API RealTimeAnalysis {
 public:
    struct FilteredData {
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
        std::vector<ExternalWrench::Input> externalWrenches;

        /**
         * Assigns the struct fields of FilteredData from SimTK::Vectors. The
         * first nq elements of the input SimTK::Vectors contain the filtered
         * generalized coordinates (and derivatives), and the rest elements
         * correspond to the filtered wrench data.
         */
        void fromVector(const double& time, const SimTK::Vector& x,
                        const SimTK::Vector& xd, const SimTK::Vector& xdd,
                        const int& nq);
    };

    struct Output {
        // filtered IK
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
        SimTK::Vector grfRightWrench;
        SimTK::Vector grfLeftWrench;

        // ID
        SimTK::Vector tau;

        // SO
        SimTK::Vector am;
        SimTK::Vector fm;
        SimTK::Vector residuals;

        // JRA
        SimTK::Vector_<SimTK::SpatialVec> reactionWrenches;
        SimTK::Vector reactionWrenchVector; // alternative representation
    };

    struct Parameters {
        // acquisition function
        DataAcquisitionFunction dataAcquisitionFunction;

        // lp smooth filter parameters
        LowPassSmoothFilter::Parameters filterParameters;

        // ik parameters
        std::vector<InverseKinematics::MarkerTask> ikMarkerTasks;
        std::vector<InverseKinematics::IMUTask> ikIMUTasks;
        double ikConstraintsWeight;
        double ikAccuracy;

        // id + jr parameters
        std::vector<ExternalWrench::Parameters> wrenchParameters;

        // so parameters
        bool solveMuscleOptimization;
        MuscleOptimization::OptimizationParameters muscleOptimizationParameters;
        MomentArmFunctionT momentArmFunction;
    };

    struct Loggers {
        // ik
        OpenSim::TimeSeriesTable qLogger;
        OpenSim::TimeSeriesTable qDotLogger;
        OpenSim::TimeSeriesTable qDDotLogger;

        // id
        OpenSim::TimeSeriesTable tauLogger;

        // so
        OpenSim::TimeSeriesTable fmLogger;
        OpenSim::TimeSeriesTable amLogger;
        OpenSim::TimeSeriesTable residualLogger;

        // jr
        OpenSim::TimeSeriesTable jrLogger;
    };

 public:
    // ctor
    RealTimeAnalysis(const OpenSim::Model& model, const Parameters& parameters);
    virtual ~RealTimeAnalysis() = default; // dtor

    /**
     * Start the simulation. It creates one thread for acquiring data, solving
     * IK and filtering, and one thread for performing the rest of the analysis
     * (ID, SO and JR). The threads are detached and terminated when simulation
     * ends or signaled from the main thread.
     */
    void run();

    /**
     * Check the termination flag if it has been raised.
     */
    bool shouldTerminate();

    /**
     * Set the termination flag to terminate analysis threads.
     */
    void shouldTerminate(bool flag);

    /**
     * Thread safe fetch function of analysis results.
     */
    Output getResults();

    /**
     * Initialize module loggers.
     */
    Loggers initializeLoggers();

 protected:
    /**
     * This function is meant to be used in a separate thread to handle the data
     * acquisition.
     */
    virtual void acquisition();

    /**
     * This function is meant to be used in a separate thread to handle the data
     * processing.
     */
    virtual void processing();

    /**
     * Prepare the input data for filtering.
     */
    SimTK::Vector prepareUnfilteredData(
            const SimTK::Vector& q,
            const std::vector<ExternalWrench::Input>& externalWrenches) const;

    OpenSim::Model model;
    Parameters parameters; // RealTimeAnalysis parameters
    Loggers log;           // loggers
    Output output;         // analysis results
    double previousAcquisitionTime;
    double previousProcessingTime;

    // modules
    SimTK::ReferencePtr<LowPassSmoothFilter> lowPassFilter;
    SimTK::ReferencePtr<InverseKinematics> inverseKinematics;
    SimTK::ReferencePtr<InverseDynamics> inverseDynamics;
    SimTK::ReferencePtr<MuscleOptimization> muscleOptimization;
    SimTK::ReferencePtr<JointReaction> jointReaction;

    // data buffer
    CircularBuffer<1, LowPassSmoothFilter::Output> buffer;

    // termination flag
    std::atomic_bool terminationFlag;

    // thread synch variables
    std::mutex mu;
    std::condition_variable cond;
    std::atomic_bool notifyParentThread;
};
} // namespace OpenSimRT
