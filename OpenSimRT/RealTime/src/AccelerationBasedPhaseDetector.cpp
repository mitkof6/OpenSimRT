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
#include "AccelerationBasedPhaseDetector.h"

#include "GRFMPrediction.h"
#include "OpenSimUtils.h"

#include <algorithm>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;

AccelerationBasedPhaseDetector::AccelerationBasedPhaseDetector(
        const Model& otherModel, const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          model(*otherModel.clone()), parameters(otherParameters) {
    // initialize buffers with consecutive values indicating the acceleration
    // exceeds the threshold
    rSlidingWindow.init(Array_<Vec2>(2, Vec2(0.0)));
    lSlidingWindow.init(Array_<Vec2>(2, Vec2(0.0)));

    // initialize bw filters
    posFilter = new ButterworthFilter(12, parameters.posLPFilterOrder,
                                      (2 * parameters.posLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    velFilter = new ButterworthFilter(12, parameters.velLPFilterOrder,
                                      (2 * parameters.velLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    accFilter = new ButterworthFilter(12, parameters.accLPFilterOrder,
                                      (2 * parameters.accLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    // initialize differentiators
    posDiff = new NumericalDifferentiator(12, parameters.posDiffOrder);
    velDiff = new NumericalDifferentiator(12, parameters.velDiffOrder);

    // add station points to the model to compute acceleration
    heelStationR = new Station(model.getBodySet().get(parameters.rFootBodyName),
                               parameters.rHeelLocationInFoot);
    heelStationL = new Station(model.getBodySet().get(parameters.lFootBodyName),
                               parameters.lHeelLocationInFoot);
    toeStationR = new Station(model.getBodySet().get(parameters.rFootBodyName),
                              parameters.rToeLocationInFoot);
    toeStationL = new Station(model.getBodySet().get(parameters.lFootBodyName),
                              parameters.lToeLocationInFoot);
    model.addModelComponent(heelStationR.get());
    model.addModelComponent(heelStationL.get());
    model.addModelComponent(toeStationR.get());
    model.addModelComponent(toeStationL.get());

    // initialize system
    state = model.initSystem();
}

void AccelerationBasedPhaseDetector::updDetector(
        const GRFMPrediction::Input& input) {
    // update detector simtk state
    OpenSimUtils::updateState(model, state, input.q, input.qDot);
    model.realizeAcceleration(state);

    // get station position
    auto rHeelPos = heelStationR->getLocationInGround(state);
    auto rToePos = toeStationR->getLocationInGround(state);
    auto lHeelPos = heelStationL->getLocationInGround(state);
    auto lToePos = toeStationL->getLocationInGround(state);

    // prepare for filtering
    Vector v(12);
    v(0, 3) = Vector(rHeelPos);
    v(3, 3) = Vector(lHeelPos);
    v(6, 3) = Vector(rToePos);
    v(9, 3) = Vector(lToePos);

    // apply filters and differentiators
    auto x = posFilter->filter(v);
    auto xDot = velFilter->filter(posDiff->diff(input.t, x));
    auto xDDot = accFilter->filter(velDiff->diff(input.t, xDot));

    // get station accelerations
    auto rHeelAcc = Vec3(&xDDot[0]);
    auto lHeelAcc = Vec3(&xDDot[3]);
    auto rToeAcc = Vec3(&xDDot[6]);
    auto lToeAcc = Vec3(&xDDot[9]);

    // append to sliding windows a state value representing if the acceleration
    // exceeded the threshold.
    rSlidingWindow.insert(
            Vec2((rToeAcc.norm() - parameters.toeAccThreshold > 0) ? 1 : 0,
                 (rHeelAcc.norm() - parameters.heelAccThreshold > 0) ? 1 : 0));
    lSlidingWindow.insert(
            Vec2((lToeAcc.norm() - parameters.toeAccThreshold > 0) ? 1 : 0,
                 (lHeelAcc.norm() - parameters.heelAccThreshold > 0) ? 1 : 0));

    // NOTE: state diagram logic
    // | acc_heel | acc_toe | Leg State |
    // |----------|---------|-----------|
    // |    0     |    0    |  STANCE   |
    // |    0     |    1    |  STANCE   |
    // |    1     |    0    |  STANCE   |
    // |    1     |    1    |  SWING    |

    // determine change in leg state based on the states in windows. If both toe
    // and heel acceleration states are 1 (i.e, Vec2(1,1)) it results in SWING,
    // else results in STANCE (NOTE: opposite of forces)
    double rPhase = (rSlidingWindow.equal(Vec2(1, 1))) ? -1 : 1;
    double lPhase = (lSlidingWindow.equal(Vec2(1, 1))) ? -1 : 1;

    // update detector internal state
    updDetectorState(input.t, rPhase, lPhase);
}
