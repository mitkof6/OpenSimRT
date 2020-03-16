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

//==============================================================================
/**
 * \brief Defines the gait phases that are supported.
 */
class RealTime_API GaitPhase {
 public:
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
    int phase;

    GaitPhase();
    GaitPhase(Phase phase);
    explicit GaitPhase(int phase);
    operator int() const;
    GaitPhase& operator=(const GaitPhase& rhs);
    bool operator==(Phase other) const;
    bool operator!=(Phase other) const;
    bool operator==(GaitPhase other) const;
    bool operator!=(GaitPhase other) const;
    std::string toString() const;
};

class RealTime_API GaitPhaseDetector;
class RealTime_API ContactForceBasedPhaseDetector;

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

    double Tds, Ths, Tto, k1, k2;
    GaitPhase phaseR, phaseL;
    SimTK::Vector_<SimTK::SpatialVec> bodyVelocities;
    SimTK::Vector_<SimTK::SpatialVec> bodyAccelerations;

    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<ContactForceBasedPhaseDetector> gaitPhaseDetector;
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

    virtual std::vector<GaitPhase> getPhase(const GRFPrediction::Input& input) = 0;

 protected:
    GaitPhase phaseR, phaseL;
    virtual void updPhase(const GRFPrediction::Input& input) = 0;
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

class RealTime_API ContactForceBasedPhaseDetector : GaitPhaseDetector {
 public:
    // ContactForceBasedPhaseDetector(OpenSim::Model* model);
    ContactForceBasedPhaseDetector(const OpenSim::Model& model);
    std::vector<GaitPhase> getPhase(const GRFPrediction::Input& input) override;

    enum LeadingLeg {
        INVALID,
        RIGHT,
        LEFT
    };
    int leadingLeg;

    double t0, duration;
 private:
    void updPhase(const GRFPrediction::Input& input) override;

    // SimTK::ReferencePtr<OpenSim::Model> model;
    OpenSim::Model model;
    SimTK::State state;

    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightHeelContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightToeContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftHeelContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftToeContactForce;
};

#endif // !GROUND_REACTION_FORCE_PREDICTION