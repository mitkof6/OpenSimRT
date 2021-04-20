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
 */

#include "ExternalAccelerationBasedPhaseDetector.h"
#include "SignalProcessing.h"
#include <SimTKcommon.h>

using namespace OpenSimRT;

ExternalAccelerationBasedPhaseDetector::ExternalAccelerationBasedPhaseDetector(
        const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          parameters(otherParameters) {
    // construct filters
    bwLPFilter = new ButterworthFilter(6, parameters.lpFilterOrder,
                                       (2 * parameters.lpCutoffFreq) /
                                               parameters.samplingFreq,
                                       ButterworthFilter::FilterType::LowPass,
                                       IIRFilter::InitialValuePolicy::Zero);
    bwHPFilter = new ButterworthFilter(6, parameters.hpFilterOrder,
                                       (2 * parameters.hpCutoffFreq) /
                                               parameters.samplingFreq,
                                       ButterworthFilter::FilterType::HighPass,
                                       IIRFilter::InitialValuePolicy::Zero);
}

void ExternalAccelerationBasedPhaseDetector::updDetector(const Input& input) {
    // combine measurements as single Vector
    SimTK::Vector v(6, 0.0);
    v(0, 3) = SimTK::Vector(input.rightFootAcceleration);
    v(3, 3) = SimTK::Vector(input.leftFootAcceleration);

    // filter measurements
    auto fAcc = bwHPFilter->filter(bwLPFilter->filter(v));
    auto rAcc = SimTK::Vec3(&fAcc[0]);
    auto lAcc = SimTK::Vec3(&fAcc[3]);

    // update detector internal state
    updDetectorState(input.time, parameters.threshold - rAcc.norm(),
                     parameters.threshold - lAcc.norm());
}
