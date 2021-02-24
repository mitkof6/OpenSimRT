/**
 * @file GaitPhaseDetector.h
 *
 * @brief Interface class for defining gait phase detectors that determine gait
 * related state.
 *
 * @Author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "GRFMPrediction.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon.h>
#include <Simulation/Model/Model.h>
#include <utility>

namespace OpenSimRT {

/**
 *  Interface class for event detection algorithms and gait-cycle related state.
 */
class RealTime_API GaitPhaseDetector {
 public:
    /**
     * Determine if the detector is ready when all internal state is valid.
     */
    bool isDetectorReady();

    /**
     * Get the current gait phase.
     */
    GaitPhaseState::GaitPhase getPhase();

    /**
     * Get the current LeadingLeg state.
     */
    const GaitPhaseState::LeadingLeg getLeadingLeg();

    /**
     * Get the most recent HS event (left of right limb).
     */
    const double getHeelStrikeTime();

    /**
     * Get the most recent TO event (left of right limb).
     */
    const double getToeOffTime();

    /**
     * Get the most recent DS time duration.
     */
    const double getDoubleSupportDuration();

    /**
     * Get the most recent SS time duration.
     */
    const double getSingleSupportDuration();

 protected:
    // Function type for event detection methods in a Sliding Window.
    template <typename T>
    using DetectEventFunction = std::function<bool(const SlidingWindow<T>&)>;

    GaitPhaseDetector(const int& windowSize); // ctor
    virtual ~GaitPhaseDetector() = default;   // dtor

    /**
     * Update the detector state by passing values that determine each leg
     * phase. Positive values correspond to the STANCE phase and negative values
     * to the SWING phase.
     */
    void updDetectorState(const double& t, const double& rValue,
                          const double& lValue);

    /**
     * Update gait phase based on the leg phases.
     */
    GaitPhaseState::GaitPhase
    updGaitPhase(const GaitPhaseState::LegPhase& phaseR,
                 const GaitPhaseState::LegPhase& phaseL);

    /**
     * Update leg phase based on input value. If x > 0 then output is STANCE,
     * else the output is SWING.
     */
    GaitPhaseState::LegPhase updLegPhase(const double& x);

    // Compute time events separately for each leg.
    struct TimeConstant {
        double right = -1;
        double left = -1;
    };
    TimeConstant Ths, Tto; // exact time of HS and TO events of the R/L foot
    double Tds, Tss;       // double-support and single-support time-periods
    int windowSize;        // buffer size

    GaitPhaseState::GaitPhase gaitPhase;   // current gait phase
    GaitPhaseState::LeadingLeg leadingLeg; // current leadingLeg state

    // functions to determine heel-strike and toe-off events
    DetectEventFunction<GaitPhaseState::LegPhase> detectHS, detectTO;

    // n-sized buffer with the current and previous GaitPhase states
    SlidingWindow<GaitPhaseState::LegPhase> phaseWindowR, phaseWindowL;
};

} // namespace OpenSimRT
