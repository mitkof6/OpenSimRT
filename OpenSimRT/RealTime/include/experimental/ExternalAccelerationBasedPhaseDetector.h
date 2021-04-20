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
 * @file ExternalAccelerationBasedPhaseDetector.h
 *
 * @brief This file contains a class of an acceleration-based gait phase
 * detector in which the foot accelerations are measured from external sensors
 * (accelerometers). The acceleration magnitude indicates whether the foot is in
 * SWING or STANCE phase.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */

#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "SignalProcessing.h"
#include "internal/RealTimeExports.h"
#include <SimTKcommon/internal/ReferencePtr.h>
#include <Simulation/Model/Model.h>

namespace OpenSimRT {

/**
 * @brief Implements an acceleration-based gait phase detector in which the
 * acceleration is measured from external sensors (accelerometers). It uses LP
 * and HP filters to filter the measured acceleration and it determines if each
 * foot is in SWING or STANCE phase when it exceeds the given threshold or not,
 * respectively.
 */
class RealTime_API ExternalAccelerationBasedPhaseDetector
        : public GaitPhaseDetector {
 public:
    struct Input {
        double time;
        SimTK::Vec3 rightFootAcceleration; // R foot acceleration
        SimTK::Vec3 leftFootAcceleration;  // L foot acceleration
    };

    struct Parameters {
        double threshold; // stance/swing threshold
        int windowSize;   // windowSize to determine HS/TO events

        // filter parameters
        double samplingFreq; // signal frequency
        int lpFilterOrder;   // lp filter order
        int hpFilterOrder;   // hp filter order
        double lpCutoffFreq; // lp cut-off frequency
        double hpCutoffFreq; // hp cut-off frequency
    };

    // ctor
    ExternalAccelerationBasedPhaseDetector(const Parameters& parameters);

    /**
     * Update the detector given the measured foot accelerations.
     */
    void updDetector(const Input& input);

 private:
    SimTK::ReferencePtr<ButterworthFilter> bwLPFilter;
    SimTK::ReferencePtr<ButterworthFilter> bwHPFilter;

    Parameters parameters;
};
} // namespace OpenSimRT
