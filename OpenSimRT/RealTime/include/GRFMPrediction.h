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

namespace OpenSim {
/**
 * \brief Defines the gait phases that are supported.
 */
class RealTime_API GaitPhase : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(GaitPhase, Object);

 public:
    OpenSim_DECLARE_PROPERTY(phase, int, "Phase enum value.");
    /*
     *  ------------------------------------
     * |            ALWAYS_ACTIVE           |
     *  ------------------------------------
     * |      STANCE      |      SWING      |
     *  ------------------------------------
     * |  DOUBLE_SUPPORT  |        -        |
     *  ------------------------------------
     */
    enum Phase {
        INVALID = 0,
        STANCE = 1 << 0,                                  // 0001
        SWING = 1 << 1,                                   // 0010
        DOUBLE_SUPPORT = (1 << 2 | STANCE),               // 0101
        ACTIVE_ALWAYS = (STANCE | SWING | DOUBLE_SUPPORT) // 0111
    };

    GaitPhase();
    GaitPhase(Phase phase);
    explicit GaitPhase(int phase);
    operator int() const;
    bool operator==(Phase other) const;
    bool operator!=(Phase other) const;
    bool operator==(GaitPhase other) const;
    bool operator!=(GaitPhase other) const;
    std::string toString() const;
};

/**
 * \brief Detects gait phase cycles based on ground reaction force
 * predicted by contacts.
 */
class RealTime_API GaitPhaseDetector : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(GaitPhaseDetector, Component);

 private: // explicit
    /**
     * \brief An event handler that is triggered when the foot touches or
     * leaves the ground. This is used to identify the gait phase.
     */
    class StanceSwingEvent : public SimTK::TriggeredEventHandler {
     public:
        StanceSwingEvent(const HuntCrossleyForce& contact,
                         const HuntCrossleyForce& contactContra,
                         GaitPhase& phase, double threshold);

     protected:
        SimTK::Real getValue(const SimTK::State& s) const override;
        void handleEvent(SimTK::State& s, SimTK::Real accuracy,
                         bool& shouldTerminate) const override;

     private:
        const HuntCrossleyForce& contact;
        const HuntCrossleyForce& contactContra;
        GaitPhase& phase;
        double threshold;
        mutable double contactValue, contactValueContra;
        void updatePhase() const;
    };

 public:
    mutable GaitPhase phaseR, phaseL;
    OpenSim_DECLARE_PROPERTY(stance_threshold, double,
                             "Threshold to be identified as stance.");
    OpenSim_DECLARE_SOCKET(contact_force_right, HuntCrossleyForce,
                           "The contact force of the right leg.");
    OpenSim_DECLARE_SOCKET(contact_force_left, HuntCrossleyForce,
                           "The contact force of the left leg.");
    GaitPhaseDetector();

 protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
};
} // namespace OpenSim

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
    };

    struct Parameters {
        double stance_threshold;
        std::string rightFootContactForceName;
        std::string leftFootContactForceName;
    } parameters;

    GRFPrediction(const OpenSim::Model&, const Parameters&);
    void solve(const Input& input);

    OpenSim::TimeSeriesTable initializeLogger();

 private:
    TransitionFuction reactionComponentTransition;
    TransitionFuction anteriorForceTransition;

    double Tds, Tp, k1, k2;
    SimTK::Vector_<SimTK::SpatialVec> bodyVelocities;
    SimTK::Vector_<SimTK::SpatialVec> bodyAccelerations;

    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<OpenSim::GaitPhaseDetector> gaitPhaseDetector;
};
#endif // !GROUND_REACTION_FORCE_PREDICTION