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

#endif
