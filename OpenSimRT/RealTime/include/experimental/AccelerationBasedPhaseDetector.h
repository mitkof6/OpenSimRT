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
 * @file AccelerationBasedPhaseDetector.h
 *
 * @brief Concrete implementation of the the GaitPhaseDetector based on foot
 * acceleration values estimated in the model from kinematic data.
 *
 * @Author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "SignalProcessing.h"

#include <SimTKcommon.h>
#include <Simulation/Model/Model.h>
#include <string>

namespace OpenSimRT {

/**
 * Gait phase detector implementation based on foot acceleration values
 * estimated in the model from kinematic data. Contact on ground is determined
 * when the acceleration of determined points in foot crosses a given threshold.
 */
class RealTime_API AccelerationBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Parameters {
        int windowSize;          // windowSize to determine HS/TO events
        double heelAccThreshold; // threshold at heel point
        double toeAccThreshold;  // threshold at toe point

        std::string rFootBodyName;       // right foot name
        std::string lFootBodyName;       // left foot name
        SimTK::Vec3 rHeelLocationInFoot; // acceleration point in R heel
        SimTK::Vec3 lHeelLocationInFoot; // acceleration point in L heel
        SimTK::Vec3 rToeLocationInFoot;  // acceleration point in R toe
        SimTK::Vec3 lToeLocationInFoot;  // acceleration point in L toe

        // bw filter Parameters
        double samplingFrequency; // signal frequency (fs)
        double accLPFilterFreq; // foot acceleration lp filter target freq (fc)
        double velLPFilterFreq; // foot velocity lp filter target freq (fc)
        double posLPFilterFreq; // foot position lp filter target freq (fc)
        int accLPFilterOrder;   // foot acceleration lp filter order
        int velLPFilterOrder;   // foot velocity lp filter order
        int posLPFilterOrder;   // foot position lp filter order

        // order of differentiators
        int posDiffOrder; // position differentiator order
        int velDiffOrder; // velocity differentiator order
    };

    // ctor
    AccelerationBasedPhaseDetector(const OpenSim::Model& otherModel,
                                   const Parameters& parameters);

    /**
     * Update detector using the kinematic data.
     */
    void updDetector(const GRFMPrediction::Input& input);

 private:
    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;

    // buffers with size = consecutive values. Hold values for both heel and
    // toe that indicate when the acceleration exceeds the threshold.
    SlidingWindow<SimTK::Vec2> rSlidingWindow;
    SlidingWindow<SimTK::Vec2> lSlidingWindow;

    // bw filters
    SimTK::ReferencePtr<ButterworthFilter> posFilter;
    SimTK::ReferencePtr<ButterworthFilter> velFilter;
    SimTK::ReferencePtr<ButterworthFilter> accFilter;

    // differentiators
    SimTK::ReferencePtr<NumericalDifferentiator> posDiff;
    SimTK::ReferencePtr<NumericalDifferentiator> velDiff;

    // attached station points to foot
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;
};
} // namespace OpenSimRT
