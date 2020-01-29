/**
 * @file MuscleOptimization.h
 *
 * \brief Utilities for solving the muscle redundancy problem.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef MUSCLE_OPTIMIZATION_H
#define MUSCLE_OPTIMIZATION_H

#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <functional>

namespace OpenSimRT {

// forward declaration
class TorqueBasedTarget;

typedef SimTK::Matrix (*MomentArmFunction)(const SimTK::Vector& q);
// typedef std::function<SimTK::Matrix(const SimTK::Vector& q)>
// MomentArmFunction;

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
        SimTK::OptimizerAlgorithm algorithm =
                SimTK::OptimizerAlgorithm::InteriorPoint;
        double convergenceTolerance; // 1e-0
        int maximumIterations;       // 50
        int objectiveExponent;       // 2
    };

 public:
    MuscleOptimization(const OpenSim::Model& model,
                       const OptimizationParameters& optimizationParameters,
                       const MomentArmFunction& momentArmFunction);
    Output solve(const Input& input);
    /**
     * Initialize inverse kinematics log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed kinematics.
     */
    OpenSim::TimeSeriesTable initializeLogger();
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
    int npa;
    SimTK::ReferencePtr<OpenSim::Model> model;
    SimTK::State state;
    SimTK::Matrix R;
    SimTK::Vector fMax, tau;
    MomentArmFunction calcMomentArm;
    std::vector<int> multibodyOrderIndex;

    // Model muscles and coordinates that are used.
    std::vector<int> activeMuscleIndices, activeCoordinateIndices;

 public:
    TorqueBasedTarget(OpenSim::Model* model, int objectiveExponent,
                      const MomentArmFunction& momentArmFunction);
    void prepareForOptimization(const MuscleOptimization::Input& input);
    SimTK::Vector extractMuscleForces(const SimTK::Vector& x) const;
    SimTK::Vector extractResidualForces(const SimTK::Vector& x) const;

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
