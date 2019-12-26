/**
 * @file InverseDynamics.h
 *
 * \brief Utilities for performing inverse dynamics.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

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

 public:
    struct RealTime_API Parameters {
        std::string appliedToBody;
        std::string forceExpressedInBody;
        std::string pointExpressedInBody;
    };
    struct RealTime_API Input {
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 torque;
        SimTK::Vector toVector();
        void fromVector(const SimTK::Vector& in);
        static int size();
    };

 public:
    ExternalWrench(const Parameters& parameters);
    Input& getInput();
 protected:
    void computeForce(const SimTK::State& state,
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                      SimTK::Vector& generalizedForces) const override;

 private:
    Parameters parameters;
    Input input;
};

/**
 * \brief Performs inverse dynamics calculations.
 */
class RealTime_API InverseDynamics {
 public:
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

 public:
    InverseDynamics(
            const OpenSim::Model& model,
            const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& inputState);

 public:
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

 private:
    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;
};

} // namespace OpenSimRT

#endif
