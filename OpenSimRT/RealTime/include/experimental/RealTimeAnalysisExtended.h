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
 * @file RealTimeAnalysisExtended.h
 *
 * @brief This file contains a facade class that extends the RealTimeAnalysis
 * class by including experimental features, such as GRFMPrediction and
 * MarkerReconstruction.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "GRFMPrediction.h"
#include "MarkerReconstruction.h"
#include "RealTimeAnalysis.h"
#include "internal/RealTimeExports.h"
#include <SimTKcommon/SmallMatrix.h>

namespace OpenSimRT {

/**
 * Phase detector update function that uses external measurements.
 */
typedef std::function<void()> ExternalPhaseDetectorUpdateFunction;

/**
 * Phase detector update function that uses internal estimations from the
 * musculoskeletal analysis.
 */
typedef std::function<void(const double& t, const SimTK::Vector& q,
                           const SimTK::Vector& qd, const SimTK::Vector& qdd)>
        InternalPhaseDetectorUpdateFunction;

/**
 * @brief Extends the facade RealTimeAnalysis class to include the experimental
 * features. Provides a convinient interface for performing RT musculoskeletal
 * analysis. It creates one thread for the data acquisition, marker
 * reconstruction IK and filtering, and one processing thread for the rest the
 * analysis (GRF&M prediction, ID, SO and JR).
 */
class RealTime_API RealTimeAnalysisExtended : public RealTimeAnalysis {
 public:
    /**
     * Select if the detector updates its state with INTERNAL estimations or
     * EXTERNAL measurements.
     */
    enum class PhaseDetectorUpdateMethod { INTERNAL, EXTERNAL };

    // Extends RealTimeAnalysis parameters
    struct Parameters : RealTimeAnalysis::Parameters {
        // chose to solve GRFMPrediction
        bool useGRFMPrediction;
        // grfm module parameters
        GRFMPrediction::Parameters grfmParameters;
        // select if the detector update function is EXTERNAL or INTERNAL
        PhaseDetectorUpdateMethod detectorUpdateMethod;
        // reference to detector
        SimTK::ReferencePtr<GaitPhaseDetector> phaseDetector;
        // detector update function (with external measurements)
        ExternalPhaseDetectorUpdateFunction externalPhaseDetectorUpdateFunction;
        // detector update function (with internal estimations)
        InternalPhaseDetectorUpdateFunction internalPhaseDetectorUpdateFunction;
    };

 public:
    // ctor
    RealTimeAnalysisExtended(const OpenSim::Model& model,
                             const Parameters& parameters);

 private:
    /**
     * This function is meant to be used in a separate thread to handle the data
     * acquisition. It performs the IK and filtering of the generalized
     * coordinates. Overrides the base acquisition function in order to include
     * the experimental features.
     */
    void acquisition() override;

    /**
     * This function is meant to be used in a separate thread to handle the data
     * processing. It performs the ID, SO and JR analysis. Overrides the base
     * processing function in order to include the experimental features.
     */
    void processing() override;

    // modules
    SimTK::ReferencePtr<GRFMPrediction> grfmPrediction;
    SimTK::ReferencePtr<MarkerReconstruction> markerReconstruction;

    Parameters parameters;
};
} // namespace OpenSimRT
