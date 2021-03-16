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
 * @file InverseDynamics.h
 *
 * \brief Utilities for performing inverse dynamics.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

namespace OpenSimRT {

/**
 * \brief Represents a wrench (force and torque) which is applied on a
 * body through some point.
 */
class RealTime_API ExternalWrench : public OpenSim::Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExternalWrench, OpenSim::Force);

 public: /* public data structures */
    struct RealTime_API Parameters {
        std::string appliedToBody;
        std::string forceExpressedInBody;
        std::string pointExpressedInBody;
    };
    struct RealTime_API Input {
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 torque;
        SimTK::Vector toVector() const;
        void fromVector(const SimTK::Vector& in);
        static int size();
    };

 public: /* public interface */
    ExternalWrench(const Parameters& parameters);
    Input& getInput();

 public: /* public static interface */
    /**
     * Creates the GRF labels (point, force, torque) required to parse
     * an .mot file. @see getWrenchFromStorage
     */
    static std::vector<std::string>
    createGRFLabelsFromIdentifiers(std::string pointIdentifier,
                                   std::string forceIdentifier,
                                   std::string torqueIdentifier);
    /**
     * An interface between InverseDynamics::Input and ExternalForce.
     */
    static ExternalWrench::Input
    getWrenchFromExternalForce(double t, const OpenSim::ExternalForce& force);
    /**
     * An interface between InverseDynamics::Input and storage. Labels must be
     * properly ordered (point, force, torque).
     */
    static ExternalWrench::Input
    getWrenchFromStorage(double t, const std::vector<std::string>& labels,
                         const OpenSim::Storage& storage);
    /**
     * Initialize wrench log storage. Use this to create a TimeSeriesTable that
     * can be appended with the computed moments.
     */
    static OpenSim::TimeSeriesTable initializeLogger();

 protected: /* protected interface */
    void computeForce(const SimTK::State& state,
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                      SimTK::Vector& generalizedForces) const override;

 private: /* private data members */
    Parameters parameters;
    Input input;
};

/**
 * \brief Performs inverse dynamics calculations.
 */
class RealTime_API InverseDynamics {
 public: /* public data structures */
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
        std::vector<ExternalWrench::Input> externalWrenches;
    };
    struct Output {
        double t;
        SimTK::Vector tau;
    };

 public: /* public interface */
    InverseDynamics(
            const OpenSim::Model& model,
            const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& input);
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
