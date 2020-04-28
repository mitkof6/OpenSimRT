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

template <typename T> struct SlidingWindow {
    std::vector<T> data;
    void init(std::vector<T>&& aData) {
        data = std::forward<std::vector<T>>(aData);
    }
    void insert(const T& x) {
        data.erase(data.begin());
        data.push_back(x);
    }
};

//==============================================================================
/**
 * \brief Defines the gait phases that are supported.
 */
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase { INVALID, SWING, STANCE };

    enum class GaitPhase { INVALID, RIGHT_SWING, LEFT_SWING, DOUBLE_SUPPORT };

    enum class LeadingLeg { INVALID, RIGHT, LEFT };
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

    struct Parameters {
        double stance_threshold;
        SimTK::Vec3 contact_plane_origin;
        SimTK::UnitVec3 contact_plane_normal;
    } parameters;

    SimTK::ReferencePtr<ContactForceBasedPhaseDetector> gaitPhaseDetector;

    GRFPrediction(const OpenSim::Model&, const Parameters&);
    std::vector<Output> solve(const Input& input);

 private:
    TransitionFuction anteriorForceTransition;
    TransitionFuction reactionComponentTransition;

    double t, Tds, Tss;

    OpenSim::Model model;
    SimTK::State state;
    GaitPhaseState::GaitPhase gaitphase;

    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    void seperateReactionComponents(
            const SimTK::Vec3& totalReactionComponent,
            const TransitionFuction& anteriorForceFunction,
            const TransitionFuction& reactionComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    void computeReactionPoint(SimTK::Vec3& rightPoint, SimTK::Vec3& leftPoint);
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
    // virtual GaitPhaseState::GaitPhase
    // getPhase(const GRFPrediction::Input& input) = 0;

 protected:
    GaitPhaseState::LeadingLeg leadingLeg;
    SlidingWindow<GaitPhaseState::LegPhase> phaseWindowR, phaseWindowL;
};

class RealTime_API ContactForceBasedPhaseDetector : GaitPhaseDetector {
 public:
    ContactForceBasedPhaseDetector(const OpenSim::Model&,
                                   const GRFPrediction::Parameters&);
    void updDetector(const GRFPrediction::Input& input);

    // getters
    GaitPhaseState::GaitPhase getPhase();
    const GaitPhaseState::LeadingLeg getLeadingLeg();
    const double getHeelStrikeTime();
    const double getToeOffTime();
    const double getDoubleSupportDuration();
    const double getSingleSupportDuration();

    bool isDetectorReady();

 private:
    GaitPhaseState::LegPhase updLegPhase(const OpenSim::HuntCrossleyForce*);
    GaitPhaseState::GaitPhase updGaitPhase(const GaitPhaseState::LegPhase&,
                                           const GaitPhaseState::LegPhase&);

    double Ths, Tto, Tds, Tss;

    OpenSim::Model model;
    SimTK::State state;
    GaitPhaseState::GaitPhase gaitPhase;
    GRFPrediction::Parameters parameters;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftContactForce;
};

#endif // !GROUND_REACTION_FORCE_PREDICTION