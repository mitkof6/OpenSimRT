/**
 * @file MuscleOptimization.h
 *
 * \brief Utilities for solving the muscle redundancy problem.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef MUSCLE_OPTIMIZATION_H
#define MUSCLE_OPTIMIZATION_H

#include "OpenSimUtils.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <functional>

namespace OpenSimRT {

// forward declaration
class TorqueBasedTarget;

/**
 * \brief Solves the muscle optimization problem.
 */
class RealTime_API MuscleOptimization {
 public:
    OpenSim::Model model;
    SimTK::ReferencePtr<SimTK::Optimizer> optimizer;
    SimTK::ReferencePtr<TorqueBasedTarget> target;
    SimTK::Vector parameterSeeds;
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector tau;
    };
    struct Output {
        double t;
        SimTK::Vector am;
        SimTK::Vector fm;
        SimTK::Vector residuals;
    };
    struct OptimizationParameters {
        double convergenceTolerance; // 1e-0
        int maximumIterations;       // 50
        int objectiveExponent;       // 2
    };

 public:
    MuscleOptimization(const OpenSim::Model& model,
                       const OptimizationParameters& optimizationParameters,
                       const MomentArmFunctionT& momentArmFunction);
    Output solve(const Input& input);
    /**
     * Initialize muscle optimization log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed kinematics.
     */
    OpenSim::TimeSeriesTable initializeMuscleLogger();
};

/**
 * \brief Muscle optimization criterion.
 *
 *    min  Σ (f_m^i / f_max^i)^p
 *    s.t. τ = R f_m
 *         f_m >= 0
 *
 * TODO: include muscle physiology (e.g., f_m <= f(l, lDot))
 */
class RealTime_API TorqueBasedTarget : public SimTK::OptimizerSystem {
 public:
    int p;
    SimTK::ReferencePtr<OpenSim::Model> model;
    SimTK::State state;
    SimTK::Matrix R;
    SimTK::Vector fMax, tau;
    MomentArmFunctionT calcMomentArm;

 public:
    TorqueBasedTarget(OpenSim::Model* model, int objectiveExponent,
                      const MomentArmFunctionT& momentArmFunction);
    void prepareForOptimization(const MuscleOptimization::Input& input);
    SimTK::Vector extractMuscleForces(const SimTK::Vector& x) const;

 protected:
    int objectiveFunc(const SimTK::Vector& x, bool newCoefficients,
                      SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector& x, bool newCoefficients,
                     SimTK::Vector& gradient) const override;
    int constraintFunc(const SimTK::Vector& x, bool newCoefficients,
                       SimTK::Vector& constraints) const override;
    int constraintJacobian(const SimTK::Vector& x, bool newCoefficients,
                           SimTK::Matrix& jac) const override;
};

} // namespace OpenSimRT

#endif
