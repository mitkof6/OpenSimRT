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
 * @file ExternalForceBasedPhaseDetector.h
 *
 * @brief This file contains a class of a force-based gait phase detector in
 * which externaly measured force-like quantiies (e.g., pressure insoles)
 * indicate whether each foot is in STANCE or SWING phase.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "internal/RealTimeExports.h"

namespace OpenSimRT {

/**
 * @brief Implements an force-based gait phase event detector in which externaly
 * measured force-like quantiies (e.g., pressure insoles) indicate whether each
 * foot is in STANCE or SWING phase when they exceed the given threshold or not,
 * respectively.
 */
class RealTime_API ExternalForceBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Input {
        double t;
        double rForce; // right force-like measurement
        double lForce; // left force-like measurement
    };

    struct Parameters {
        double threshold; // stance/swing threshold
        int windowSize;   // windowSize to determine HS/TO events
    };

    // ctor
    ExternalForceBasedPhaseDetector(const Parameters& parameters);

    /**
     * Update the detector given the force-like measurements.
     */
    void updDetector(const Input& input);

 private:
    Parameters parameters;
};
} // namespace OpenSimRT
