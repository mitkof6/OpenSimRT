/**
 * @file GroundReactionForcePrediction.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2019-12-10
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef GROUND_REACTION_FORCE_PREDICTION
#define GROUND_REACTION_FORCE_PREDICTION

#include "internal/RealTimeExports.h"

#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

class RealTime_API GaitPhaseDetector; // todo
class RealTime_API ContactForceBasedPhaseDetector;

//==============================================================================
/**
 * \brief Defines the gait phases that are supported.
 */
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase {
        INVALID,
        SWING,
        STANCE
    };

    enum class GaitPhase {
        INVALID,
        RIGHT_SWING,
        LEFT_SWING,
        DOUBLE_SUPPORT
    };

    enum class LeadingLeg {
        INVALID,
        RIGHT,
        LEFT
    };
};

// =============================================================================
class RealTime_API GRFPrediction {
    typedef std::function<double(double)> TransitionFuction;
 public:
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
    };

    struct Output {
        double t;
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 moment;
        SimTK::Vector asVector();
    };

    GRFPrediction(const OpenSim::Model&);
    std::vector<Output> solve(const Input& input);

 private:
    TransitionFuction reactionComponentTransition;
    TransitionFuction anteriorForceTransition;

    double currentTime, Tds;
    SimTK::Vector_<SimTK::SpatialVec> bodyVelocities;
    SimTK::Vector_<SimTK::SpatialVec> bodyAccelerations;

    OpenSim::Model model;
    SimTK::State state;
    GaitPhaseState::GaitPhase gaitphase;
    SimTK::ReferencePtr<ContactForceBasedPhaseDetector> gaitPhaseDetector;

    void computeReactionForces(SimTK::Vec3& rightReactionForce,
            SimTK::Vec3& leftReactionForce);
    void computeReactionMoments(const SimTK::Vec3& rightReactionForce,
            const SimTK::Vec3& leftReactionForce,
            SimTK::Vec3& rightReactionMoment,
            SimTK::Vec3& leftReactionMoment);
    void computeReactionPoint(const SimTK::Vec3& rightReactionForce,
            const SimTK::Vec3& leftReactionForce,
            const SimTK::Vec3& rightReactionMoment,
            const SimTK::Vec3& leftReactionMoment,
            SimTK::Vec3& rightPoint,
            SimTK::Vec3& leftPoint);
};

// =============================================================================
/**
 * \brief Detects gait phase cycles based on ground reaction force
 * predicted by contacts.
 */
class RealTime_API GaitPhaseDetector {
 public:
    GaitPhaseDetector() = default;
    virtual ~GaitPhaseDetector() = default;
    virtual GaitPhaseState::GaitPhase getPhase(const GRFPrediction::Input& input) = 0;

 protected:
    GaitPhaseState::LeadingLeg leadingLeg;
    GaitPhaseState::LegPhase phaseR, phaseL;
    GaitPhaseState::GaitPhase gaitPhase;
    virtual void updLegPhase() = 0;
    virtual void updGaitPhase() = 0;

};

class RealTime_API ContactForceBasedPhaseDetector : GaitPhaseDetector {
 public:
    ContactForceBasedPhaseDetector(const OpenSim::Model& model);

    // getters
    GaitPhaseState::GaitPhase getPhase(const GRFPrediction::Input& input) override;
    const GaitPhaseState::LeadingLeg getLeadingLeg();
    const double getHeelStrikeTime();

 private:
    void updLegPhase() override;
    void updGaitPhase() override;
    void detectRightHeelStrike();
    void detectLeftHeelStrike();

    double Ths;
    OpenSim::Model model;
    SimTK::State state;

    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightHeelContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftHeelContactForce;
};

// class RealTime_API VelocityBasedPhaseDetector : GaitPhaseDetector {
//  public:
//     VelocityBasedPhaseDetector(OpenSim::Model* model);
//     std::vector<GaitPhase> getPhase(const SimTK::State& state) override;

//  private:
//     void updPhase(const SimTK::State& state) override;

//     SimTK::ReferencePtr<OpenSim::Model> model;

//     // * Note: Markers are also possible, but can go missing
//     SimTK::ReferencePtr<OpenSim::Station> heelStationR;
//     SimTK::ReferencePtr<OpenSim::Station> heelStationL;
//     SimTK::ReferencePtr<OpenSim::Station> toeStationR;
//     SimTK::ReferencePtr<OpenSim::Station> toeStationL;
//     SimTK::ReferencePtr<OpenSim::Station> pelvisStation;
// };
#endif // !GROUND_REACTION_FORCE_PREDICTION