/**
 * @file MuscleOptimization.h
 *
 * \brief Utilities for solving the muscle redundancy problem.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef MUSCLE_OPTIMIZATION_H
#define MUSCLE_OPTIMIZATION_H

#include <functional>
#include <OpenSim/Simulation/Model/Model.h>
#include "SimulationUtils.h"
#include "internal/RealTimeExports.h"

// forward declaration
class CSVLogger;
class OptimizationTarget;

typedef std::function<SimTK::Matrix(const SimTK::Vector& q)> MomentArmFunction;

/**
 * \brief Solves the muscle optimization problem for non-linear muscle model.
 */
class RealTime_API MuscleOptimization {
 public:
    SimTK::Vector parameterSeeds; // controls to minimize
    SimTK::ReferencePtr<CSVLogger> logger;

    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qdot;
        SimTK::Vector tau;
    };
    struct Output {
        double t;
        SimTK::Vector am;
        SimTK::Vector fm;
        SimTK::Vector residuals;
    };
    struct OptimizationParameters {
        SimTK::OptimizerAlgorithm algorithm = SimTK::OptimizerAlgorithm::InteriorPoint;
        double convergenceTolerance = 1e-6; // set 1e-0 for linear muscle
        int maximumIterations = 50;
        int objectiveExponent = 2;
    };

 private:
    OpenSim::Model model;
    SimTK::ReferencePtr<OptimizationTarget> target;  // system for optimization
    SimTK::ReferencePtr<SimTK::Optimizer> optimizer; // target-based optimizer

 public:
    MuscleOptimization(std::string modelFile,
                       const OptimizationParameters& optimizationParameters,
                       const MomentArmFunction& momentArmFunction,
                       OptimizationTarget* optimizationTarget);

    // solve optimization problem
    Output solve(const Input& input);
};

/*******************************************************************************/
/**
 * @brief Base class for Optimization targets.
 *
 */
class RealTime_API OptimizationTarget : public SimTK::OptimizerSystem {
    friend class MuscleOptimization;

 protected:
    SimTK::ReferencePtr<OpenSim::Model> model; // pointer to MuscleOptimization::model
    SimTK::State state;

    int p;   // objectiveExponent
    int nma; // counter of muscle actuators

    SimTK::Matrix R;    // moment arm
    SimTK::Vector xMax; // xMax = [am_max; F^opt_coordAct]
    SimTK::Vector tau, fmMax;

    MomentArmFunction calcMomentArm; // model-specific function

    // Model muscles and coordinates that are used.
    std::vector<int> activeMuscleIndices, activeCoordinateIndices;

 public:
    OptimizationTarget() = default;
    virtual ~OptimizationTarget() = default; // run the correct destructor when delete

 private:
    // different target implementations, require different upper bounds.
    // (Goal: maximize code reuse.)
    virtual double setUpperBound(const OpenSim::Muscle* m) = 0;

    // on demand "contruction" of the target object inside MuscleOptimization class
    void build();

 protected:
    virtual void prepareForOptimization(const MuscleOptimization::Input& input) = 0;
    virtual SimTK::Vector extractMuscleActivations(const SimTK::Vector& x) const = 0;
    virtual SimTK::Vector extractMuscleForces(const SimTK::Vector& x) const = 0;
    virtual SimTK::Vector extractResidualForces(const SimTK::Vector& x) const = 0;

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
class RealTime_API TorqueBasedTargetLinearMuscle : public OptimizationTarget {
 public:
    TorqueBasedTargetLinearMuscle() = default;

 private:
    void prepareForOptimization(const MuscleOptimization::Input& input) override;
    SimTK::Vector extractMuscleActivations(const SimTK::Vector& x) const override;
    SimTK::Vector extractMuscleForces(const SimTK::Vector& x) const override;
    SimTK::Vector extractResidualForces(const SimTK::Vector& x) const override;

    // set upperBound equal to MaxIsometrixcForce
    double setUpperBound(const OpenSim::Muscle* m) override;

 protected:
    int objectiveFunc(const SimTK::Vector& x,
                      bool newCoefficients,
                      SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector& x,
                     bool newCoefficients,
                     SimTK::Vector& gradient) const override;
    int constraintFunc(const SimTK::Vector& x,
                       bool newCoefficients,
                       SimTK::Vector& constraints) const override;
    int constraintJacobian(const SimTK::Vector& x,
                           bool newCoefficients,
                           SimTK::Matrix& jac) const override;
};

/**
 * @brief Muscle Optimization Criterion
 *
 * min Σ (|x^i| / |x^i_max|)^p, where x = [am; fRes]
 * s.t. τ = R * (fmMax * (am*fmL*fmV + fmPE) * cosAlpha) + tauRes,
 *      x_min^i <= x^i <= x_max^i
 *
 */
class RealTime_API TorqueBasedTargetNonLinearMuscle : public OptimizationTarget {
 public:
    TorqueBasedTargetNonLinearMuscle() = default;

 private:
    void prepareForOptimization(const MuscleOptimization::Input& input) override;
    SimTK::Vector extractMuscleActivations(const SimTK::Vector& x) const override;
    SimTK::Vector extractMuscleForces(const SimTK::Vector& x) const override;
    SimTK::Vector extractResidualForces(const SimTK::Vector& x) const override;

    // set upper bound equal to maximum activation level (am_Max = 1.0)
    double setUpperBound(const OpenSim::Muscle* m) override;

 private:
    // muscle-related values
    SimTK::Vector fmL, fmV, fmPE, cosAlpha;

 protected:
    int objectiveFunc(const SimTK::Vector& x,
                      bool newCoefficients,
                      SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector& x,
                     bool newCoefficients,
                     SimTK::Vector& gradient) const override;
    int constraintFunc(const SimTK::Vector& x,
                       bool newCoefficients,
                       SimTK::Vector& constraints) const override;
    int constraintJacobian(const SimTK::Vector& x,
                           bool newCoefficients,
                           SimTK::Matrix& jac) const override;
};

#endif
