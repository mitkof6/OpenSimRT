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
 * @file JointReaction.h
 *
 * \brief Utilities for calculating the joint reaction loads.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "InverseDynamics.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {

/**
 * \brief Calculates the joint reaction loads as applied on child bodies
 * expressed in ground.
 *
 * TODO: implement re-express in different frame of interest
 */
class RealTime_API JointReaction {
 public: /* public data structures */
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector fm;
        std::vector<ExternalWrench::Input> externalWrenches;
    };
    struct Output {
        double t;
        SimTK::Vector_<SimTK::SpatialVec> reactionWrench; // [m, f]^T
    };

 public: /* public interface */
    JointReaction(
            const OpenSim::Model& model,
            const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& input);
    /**
     * Transform the joint reactions into a Vector arranged as
     * [force[0], moment[0], point[0], ..., force[n - 1], moment[n -
     * 1], point[n - 1]].
     */
    SimTK::Vector asForceMomentPoint(const Output& jrOutput);
    /**
     * Initialize inverse dynamics log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed generalized
     * forces.
     */
    OpenSim::TimeSeriesTable initializeLogger();

 private: /* private data members */
    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;
};

} // namespace OpenSimRT
