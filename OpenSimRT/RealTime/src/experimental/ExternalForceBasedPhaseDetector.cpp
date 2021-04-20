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
#include "ExternalForceBasedPhaseDetector.h"
#include <algorithm>

using namespace OpenSimRT;

ExternalForceBasedPhaseDetector::ExternalForceBasedPhaseDetector(
        const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          parameters(otherParameters) {}

void ExternalForceBasedPhaseDetector::updDetector(const Input& input) {
    // update detector internal state
    updDetectorState(input.t, input.rForce - parameters.threshold,
                     input.lForce - parameters.threshold);
}
