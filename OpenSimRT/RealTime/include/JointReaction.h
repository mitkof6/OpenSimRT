/**
 * @file JointReaction.h
 *
 * \brief Utilities for calculating the joint reaction loads.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef JOINT_REACTION_H
#define JOINT_REACTION_H

#include <OpenSim/Simulation/Model/Model.h>
#include "InverseDynamics.h"
#include "internal/RealTimeExports.h"

// forward declaration
class CSVLogger;

/**
 * \brief Calculates the joint reaction loads as applied on child bodies
 * expressed in ground.
 *
 * TODO: implement re-express in different frame of interest
 */
class RealTime_API JointReaction {
 public:
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
    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;
    SimTK::ReferencePtr<CSVLogger> logger;
 public:
    JointReaction(std::string modelFile,
                  const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& input);
};

#endif
