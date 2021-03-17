/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file GRFMPrediction.h
 *
 * @brief Estimate the ground reaction forces, moments and cop during gait from
 * kinematic data.
 *
 * @Author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "Exception.h"
#include "InverseDynamics.h"
#include "SlidingWindow.h"
#include "internal/RealTimeExports.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon.h>

namespace OpenSimRT {

class RealTime_API GaitPhaseDetector;

/*******************************************************************************/

// Gait related states.
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase { INVALID, SWING, STANCE };

    enum class GaitPhase { INVALID, RIGHT_SWING, LEFT_SWING, DOUBLE_SUPPORT };

    enum class LeadingLeg { INVALID, RIGHT, LEFT };
};

/**
 * Ground Reaction Force & Moment Prediction method. It uses an external
 * detection algorithm for determining gait related events (heel-strike,
 * toe-off, etc.). It estimates the total GRF&M from kinematic data during gait
 * using either the Newton-Euler equations or by solving the ID. Seperation into
 * right/left components is achieved using the Smooth Transition Assumption
 * funtions [Ren et al. https://doi.org/10.1016/j.jbiomech.2008.06.001]. The CoP
 * is estimated assuming a smooth transition between two specified station
 * points in heel and toe.
 */
class RealTime_API GRFMPrediction {
    /**
     * Funtion type used to describe the Smooth Transition Assumption fucntions.
     */
    typedef std::function<double(const double&)> TransitionFuction;

    /**
     * Function type used to describe the CoP trajectory on foot during gait.
     */
    typedef std::function<SimTK::Vec3(const double&, const SimTK::Vec3&)>
            CoPTrajectory;

 public:
    /**
     * Select method for computing the total reactions loads.
     */
    enum class Method { NewtonEuler, InverseDynamics };

    struct Parameters {
        int directionWindowSize;
        Method method;                                  // Newton-Euler or ID
        std::string pelvisBodyName;                     // pelvis body name
        std::string rStationBodyName, lStationBodyName; // foot body names
        SimTK::Vec3 rHeelStationLocation, lHeelStationLocation; // begin of cop
        SimTK::Vec3 rToeStationLocation, lToeStationLocation;   // end of cop
    };

    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
    };

    struct RealTime_API Output {
        double t;
        ExternalWrench::Input right, left;
    };

    GRFMPrediction(const OpenSim::Model&, const Parameters&,
                   GaitPhaseDetector*); // ctor

    /**
     * Select the name of the method used to compute the total reaction loads
     * F_ext and M_ext. Computation is performed using either the Newton-Euler
     * equations of motion, or the by solving the Inverse-Dynamics (ID) method.
     */
    static Method selectMethod(const std::string& methodName);

    /**
     * Compute the ground reaction forces, moments and cop.
     */
    Output solve(const Input& input);

 private:
    // transition functions based on the STA
    TransitionFuction reactionComponentTransition; // STA fucntions
    // TransitionFuction anteriorForceTransition; // STA function for F_x

    // function that describes the CoP trajectory during gait
    CoPTrajectory copPosition;

    SimTK::Vec3 totalForceAtThs;  // F_ext at Ths
    SimTK::Vec3 totalMomentAtThs; // M_ext at Ths
    double Tds, Tss; //  double support and single support time period

    // gait direction based on the average direction of the pelvis anterior axis
    SlidingWindow<SimTK::Vec3> gaitDirectionBuffer;

    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;

    // gait phase detection
    SimTK::ReferencePtr<GaitPhaseDetector> gaitPhaseDetector;
    GaitPhaseState::GaitPhase gaitphase; // gait phase during simulation

    // station points forming the cop trajectory
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    /**
     * Compute the rotation matrix required to transform the estimated total
     * GRF&M from the global reference frame to the the average heading
     * direction frame during gait computed based on the anterior axis of the
     * pelvis local frame.
     */
    SimTK::Rotation computeGaitDirectionRotation(const std::string& bodyName);

    /**
     * Compute the total reaction components F_ext and M_ext based either on the
     * Newton-Euler method or by solving ID.
     */
    void computeTotalReactionComponents(const Input& input,
                                        SimTK::Vec3& totalReactionForce,
                                        SimTK::Vec3& totalReactionMoment);

    /**
     * Separate the total reaction components into R/L foot reaction components.
     */
    void seperateReactionComponents(
            const double& time, const SimTK::Vec3& totalReactionComponent,
            const SimTK::Vec3& totalReactionAtThs,
            const TransitionFuction& anteriorComponentFunction,
            const TransitionFuction& verticalComponentFunction,
            const TransitionFuction& lateralComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    /**
     * Compute the CoP on each foot.
     */
    void computeReactionPoint(const double& t, SimTK::Vec3& rightPoint,
                              SimTK::Vec3& leftPoint);
};

} // namespace OpenSimRT
