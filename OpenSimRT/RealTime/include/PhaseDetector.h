#ifndef PHASE_DETECTOR_H
#define PHASE_DETECTOR_H

#include "GRFMPrediction.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {

// Detects gait phase cycles based on ground reaction force predicted by
// contacts.
class RealTime_API GaitPhaseDetector {
 public:
    GaitPhaseDetector() = default;
    virtual ~GaitPhaseDetector() = default;

 protected:
    GaitPhaseState::LeadingLeg leadingLeg;
    SlidingWindow<GaitPhaseState::LegPhase> phaseWindowR, phaseWindowL;
};

// Gait phase detector implementation based on contact forces
class RealTime_API ContactForceBasedPhaseDetector : GaitPhaseDetector {
 public:
    using DetectEventFunction =
            std::function<bool(const SlidingWindow<GaitPhaseState::LegPhase>&)>;

    // constructor
    ContactForceBasedPhaseDetector(
            const OpenSim::Model&,
            const GRFMPrediction::Parameters& parameters);

    // update detector based on the IK results
    void updDetector(const GRFMPrediction::Input& input);

    // determine if detector state is valid
    bool isDetectorReady();

    // getters
    GaitPhaseState::GaitPhase getPhase() { return gaitPhase; };
    const GaitPhaseState::LeadingLeg getLeadingLeg() { return leadingLeg; };
    const double getHeelStrikeTime() { return Ths; };
    const double getToeOffTime() { return Tto; };
    const double getDoubleSupportDuration() { return Tds; };
    const double getSingleSupportDuration() { return Tss; };

 private:
    // update leg phase (Swing - Stance) based on contact-force values
    GaitPhaseState::LegPhase updLegPhase(const OpenSim::HuntCrossleyForce*);

    // update gait phase (Double - Single support) based on leg phase
    GaitPhaseState::GaitPhase updGaitPhase(const GaitPhaseState::LegPhase&,
                                           const GaitPhaseState::LegPhase&);
    DetectEventFunction detectHS; // function to determine heel-strike
    DetectEventFunction detectTO; // function to determine toe-off

    // time constants. DS and SS period, and time of HS and TO events
    double Ths, Tto, Tds, Tss;

    OpenSim::Model model;
    SimTK::State state;
    GRFMPrediction::Parameters parameters;
    GaitPhaseState::GaitPhase gaitPhase; // current gait phase

    // contact force elements added to a copy of the original model
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftContactForce;
};
} // namespace OpenSimRT
#endif PHASE_DETECTOR_H
