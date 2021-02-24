/**
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
