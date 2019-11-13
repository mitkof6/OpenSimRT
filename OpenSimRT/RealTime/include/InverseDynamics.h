/**
 * @file InverseDynamics.h
 *
 * \brief Utilities for performing inverse dynamics.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include "internal/RealTimeExports.h"

// forward declaration
class CSVLogger;

/**
 * \brief Represents a wrench (force and torque) which is applied on a body
 * through some point.
 */
class RealTime_API ExternalWrench : public OpenSim::Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExternalWrench, OpenSim::Force);
 public:
    struct RealTime_API Parameters {
        std::string appliedToBody;
        std::string forceExpressedInBody;
        std::string pointExpressedInBody;
    } parameters;
    struct RealTime_API Input {
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 torque;
        SimTK::Vector toVector();
        void fromVector(const SimTK::Vector& in);
        static int size();
    } input;
    SimTK::ReferencePtr<CSVLogger> logger;
 public:
    ExternalWrench(const Parameters& parameters);
 protected:
    void computeForce(const SimTK::State& state,
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                      SimTK::Vector& generalizedForces) const override;
};

/**
 * \brief Performs inverse dynamics calculations.
 */
class RealTime_API InverseDynamics {
 public:
    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;
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
    SimTK::ReferencePtr<CSVLogger> logger;
 public:
    InverseDynamics(std::string modelFile,
                    const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& inputState);
};

/**
 * Creates the GRF labels (point, force, torque) required to parse an .mot
 * file. @see getWrenchFromStorage
 */
RealTime_API std::vector<std::string> createGRFLabelsFromIdentifiers(std::string pointIdentifier,
								     std::string forceIdentifier,
								     std::string torqueIdentifier);

/**
 * An interface between InverseDynamics::Input and ExternalForce.
 */
RealTime_API ExternalWrench::Input getWrenchFromExternalForce(double t,
							      const OpenSim::ExternalForce& force);

/**
 * An interface between InverseDynamics::Input and storage. Labels must be
 * properly ordered (point, force, torque).
 */
RealTime_API ExternalWrench::Input getWrenchFromStorage(double t,
							const std::vector<std::string>& labels,
							const OpenSim::Storage& storage);

#endif
