/**
 * @file JointReaction.h
 *
 * \brief Utilities for calculating the joint reaction loads.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef JOINT_REACTION_H
#define JOINT_REACTION_H

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
     * Initialize inverse dynamics log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed generalized
     * forces.
     */
    OpenSim::TimeSeriesTable initializeLogger();
    /**
     * Converts Output to a SimTK::Vector ([f[1], m[1], f[2], m[2],
     * ...]), ignoring the ground body.
     */
    SimTK::Vector convertOutput(const Output& output);

 private: /* private data members */
    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;
};

} // namespace OpenSimRT

#endif
