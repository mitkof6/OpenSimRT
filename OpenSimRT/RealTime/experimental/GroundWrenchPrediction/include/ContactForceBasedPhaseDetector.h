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
 * @file ContactForceBasedPhaseDetector.h
 *
 * @brief Concrete implementation of the the GaitPhaseDetector based on virtual
 * contact forces on attached contact surfaces to the model.
 *
 * @Author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "GaitPhaseDetector.h"

#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <string>

namespace OpenSimRT {

/**
 * Gait phase detector implementation based on virtual contact forces. The
 * method attaches virtual contact surfaces to the model (contact spheres on
 * heels and toes, and contact plane as ground). When in contact, the surfaces
 * apply forces to each other. Contact on ground is determined when the
 * generated forces exceed a given threshold.
 */
class RealTime_API ContactForceBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Parameters {
        int windowSize;   // windowSize to determine HS/TO events
        double threshold; // force threshold
        SimTK::Vec3 plane_origin; // contact plane origin

        // constact sphere Parameters
        double sphereRadius;
        SimTK::Vec3 rHeelSphereLocation;
        SimTK::Vec3 rToeSphereLocation;
        SimTK::Vec3 lHeelSphereLocation;
        SimTK::Vec3 lToeSphereLocation;
        std::string rFootBodyName;
        std::string lFootBodyName;
    };
    // ctor
    ContactForceBasedPhaseDetector(const OpenSim::Model&,
                                   const Parameters& parameters);
    /**
     * Update detector using the kinematic data.
     */
    void updDetector(const GRFMPrediction::Input& input);

 private:
    // contact force elements added to a copy of the original model
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftContactForce;

    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;
};
} // namespace OpenSimRT
