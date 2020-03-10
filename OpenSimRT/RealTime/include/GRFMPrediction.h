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

// =============================================================================
/**
 * \brief Detects gait phase cycles based on ground reaction force
 * predicted by contacts.
 */
class RealTime_API GaitPhaseDetector {
 public:

    struct Parameters {
        double stance_threshold;
    } parameters;

    GaitPhaseDetector(OpenSim::Model* model, const Parameters& parameters);
    std::vector<GaitPhase> getPhase(const SimTK::State& state);

 private:
    SimTK::ReferencePtr<OpenSim::Model> model;
    GaitPhase phaseR, phaseL;
    double avgPelvisVel;
    void updPhase(const SimTK::State& state);

    // * Note: Markers are also possible, but can go missing
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;
    SimTK::ReferencePtr<OpenSim::Station> pelvisStation;
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
    };

    GRFPrediction(const OpenSim::Model&, const GaitPhaseDetector::Parameters&);
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
    SimTK::ReferencePtr<GaitPhaseDetector> gaitPhaseDetector;
};
#endif // !GROUND_REACTION_FORCE_PREDICTION