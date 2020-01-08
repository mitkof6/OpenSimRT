#include "MuscleOptimization.h"

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#include "Exception.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

/******************************************************************************/

// abstract function that returns muscle properties based on input
// std::function
Vector getMuscleRelatedValues(const Model& model,
                              const vector<int>& activeMuscleIndices,
                              function<double(const Muscle&)> operation) {
    const auto& muscleSet = model.getMuscles();
    Vector vec(activeMuscleIndices.size());
    for (int i = 0; i < activeMuscleIndices.size(); i++) {
        vec[i] = operation(muscleSet[activeMuscleIndices[i]]);
    }
    return vec;
}

Matrix calculateMomentArm(const State& s, const Model& model,
                          const vector<int>& activeCoordinateIndices,
                          const vector<int>& activeMuscleIndices) {
    const auto& coordinateSet = model.getCoordinateSet();
    const auto& muscleSet = model.getMuscles();
    Matrix R(activeCoordinateIndices.size(), activeMuscleIndices.size());
    for (int i = 0; i < activeCoordinateIndices.size(); i++) {
        for (int j = 0; j < activeMuscleIndices.size(); j++) {
            R[i][j] = muscleSet[activeMuscleIndices[j]].computeMomentArm(
                    s, coordinateSet[activeCoordinateIndices[i]]);
        }
    }
    return R;
}

/******************************************************************************/
// MuscleOptimization

MuscleOptimization::MuscleOptimization(
        const Model& otherModel,
        const MuscleOptimization::OptimizationParameters&
                optimizationParameters,
        const MomentArmFunction& momentArmFunction,
        OptimizationTarget* optimizationTarget)
    : model(*otherModel.clone()), target(optimizationTarget) {

    // configure optimizer
    target->model = model;
    target->calcMomentArm = momentArmFunction;
    target->p = optimizationParameters.objectiveExponent;
    target->build();

    optimizer = new SimTK::Optimizer(*target, optimizationParameters.algorithm);
    optimizer->setConvergenceTolerance(
            optimizationParameters.convergenceTolerance);
    optimizer->setMaxIterations(optimizationParameters.maximumIterations);
    optimizer->setDiagnosticsLevel(0); // for debugging
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(false);
    if (optimizationParameters.algorithm ==
        SimTK::OptimizerAlgorithm::InteriorPoint) {
        optimizer->setLimitedMemoryHistory(100);
        optimizer->setAdvancedBoolOption("warm_start", true);
        optimizer->setAdvancedRealOption("expect_infeasible_problem", false);
        optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
        optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    }
    parameterSeeds = SimTK::Vector(target->getNumParameters(), 0.0);
}

MuscleOptimization::Output
MuscleOptimization::solve(const MuscleOptimization::Input& input) {
    SimTK::State state;
    try {
        target->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
    } catch (std::exception& e) {
        // optimization may find a feasible solution and fail
        std::cout << "Failed at time: " << input.t << std::endl
                  << e.what() << std::endl;
        // exit(-1);
    }

    // resutls
    auto fm = target->extractMuscleForces(parameterSeeds);
    auto residuals = target->extractResidualForces(parameterSeeds);
    auto am = target->extractMuscleActivations(parameterSeeds);

    return MuscleOptimization::Output{input.t, am, fm, residuals};
}

/******************************************************************************/
// Base Class: OptimizationTarget

void OptimizationTarget::build() {
    // create a reserve actuator for each generalized coordinate in
    // the model
    auto& cs = model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        string name = cs.get(i).getName() + "_reserve";
        if (!model->getForceSet().contains(name)) {
            CoordinateActuator* actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            actuator->setMinControl(-SimTK::Infinity);
            actuator->setMaxControl(SimTK::Infinity);
            actuator->setOptimalForce(1);
            model->addForce(actuator);
        }
    }

    setNumEqualityConstraints(cs.getSize());
    setNumLinearEqualityConstraints(cs.getSize());

    // parameter bounds
    nma = 0;
    auto& as = model->getActuators();
    int na = as.getSize();
    xMax = Vector(na, 0.0);
    Vector lowerBounds(na, 0.0), upperBounds(na, 0.0);
    for (int i = 0; i < na; ++i) {
        auto muscle = dynamic_cast<const Muscle*>(&as[i]);
        auto coordAct = dynamic_cast<const CoordinateActuator*>(&as[i]);
        if (muscle) {
            nma++;
            xMax[i] = setUpperBound(muscle);
            lowerBounds[i] = 0.0;
            upperBounds[i] = xMax[i];
        } else if (coordAct) {
            xMax[i] = coordAct->getOptimalForce();
            lowerBounds[i] = coordAct->get_min_control() * xMax[i];
            upperBounds[i] = coordAct->get_max_control() * xMax[i];
        } else {
            THROW_EXCEPTION("unsupported type of actuator");
        }
    }
    setNumParameters(na);
    setParameterLimits(lowerBounds, upperBounds);

    // Initialize the system and get the state representing the state system
    state = model->initSystem();

    // TODO: check if muscles are disabled and append active muscle coordinates
    const auto& muscleSet = model->getMuscles();
    for (int i = 0; i < muscleSet.getSize(); i++) {
        activeMuscleIndices.push_back(i);
    }

    // TODO initialize active model coordinates
    const auto& coordinateSet = model->getCoordinateSet();
    for (int i = 0; i < coordinateSet.getSize(); i++) {
        activeCoordinateIndices.push_back(i);
    }

    // compute state-independent muscle parameters
    fmMax = getMuscleRelatedValues(*model, activeMuscleIndices,
                                   [&](const Muscle& m) -> double {
                                       return m.getMaxIsometricForce();
                                   });
}

/*******************************************************************************/
// Concrete class: TorqueBasedTargetLinearMuscle

double TorqueBasedTargetLinearMuscle::setUpperBound(const OpenSim::Muscle* m) {
    return m->getMaxIsometricForce();
}

void TorqueBasedTargetLinearMuscle::prepareForOptimization(
        const MuscleOptimization::Input& input) {
    tau = input.tau;
    R = calcMomentArm(input.q);
}

Vector
TorqueBasedTargetLinearMuscle::extractMuscleActivations(const Vector& x) const {
    return x.elementwiseDivide(xMax)(0, nma);
}

Vector
TorqueBasedTargetLinearMuscle::extractMuscleForces(const Vector& x) const {
    return x(0, nma);
}

Vector
TorqueBasedTargetLinearMuscle::extractResidualForces(const Vector& x) const {
    return x(nma, getNumParameters() - nma);
}

int TorqueBasedTargetLinearMuscle::objectiveFunc(const Vector& x,
                                                 bool newCoefficients,
                                                 Real& rP) const {
    rP = 0.0;
    for (int i = 0; i < getNumParameters(); ++i) {
        rP += pow(abs(x[i]) / xMax[i], p);
    }
    return 0;
}

int TorqueBasedTargetLinearMuscle::gradientFunc(const Vector& x,
                                                bool newCoefficients,
                                                Vector& gradient) const {
    for (int i = 0; i < getNumParameters(); ++i) {
        if (x[i] < 0) {
            gradient[i] = -1.0 * p * pow(abs(x[i]) / xMax[i], p - 1);
        } else {
            gradient[i] = p * pow(abs(x[i]) / xMax[i], p - 1);
        }
    }
    return 0;
}

int TorqueBasedTargetLinearMuscle::constraintFunc(const Vector& x,
                                                  bool newCoefficients,
                                                  Vector& constraints) const {
    constraints = R * extractMuscleForces(x) + extractResidualForces(x) - tau;
    return 0;
}

int TorqueBasedTargetLinearMuscle::constraintJacobian(const Vector& x,
                                                      bool newCoefficients,
                                                      Matrix& jac) const {
    int n = getNumConstraints();
    jac(0, 0, n, nma) = R;
    jac(0, nma, n, n) = 1.0;
    return 0;
}

/*******************************************************************************/
// Concrete class: TorqueBasedTargetNonLinearMuscle

double
TorqueBasedTargetNonLinearMuscle::setUpperBound(const OpenSim::Muscle* m) {
    return 1.0; // Max activation level
}

void TorqueBasedTargetNonLinearMuscle::prepareForOptimization(
        const MuscleOptimization::Input& input) {
    // tau computed from ID
    tau = input.tau;

    // update filtered generalized coordinates/velocities
    state.updQ() = input.q;
    state.updU() = input.qdot;

    // Make sure the muscles states are in equilibrium
    model->equilibrateMuscles(state);

    // calculate moment arm
    R = calcMomentArm(input.q);
    // R = calculateMomentArm(state, *model,
    //         activeCoordinateIndices,
    //         activeMuscleIndices);

    // compute state-dependent muscle multipliers
    fmL = getMuscleRelatedValues(
            *model, activeMuscleIndices, [&](const Muscle& m) -> double {
                return m.getActiveForceLengthMultiplier(state);
            });
    fmV = getMuscleRelatedValues(*model, activeMuscleIndices,
                                 [&](const Muscle& m) -> double {
                                     return m.getForceVelocityMultiplier(state);
                                 });
    fmPE = getMuscleRelatedValues(*model, activeMuscleIndices,
                                  [&](const Muscle& m) -> double {
                                      return m.getPassiveForceMultiplier(state);
                                  });
    cosAlpha = getMuscleRelatedValues(*model, activeMuscleIndices,
                                      [&](const Muscle& m) -> double {
                                          return m.getCosPennationAngle(state);
                                      });
}

Vector TorqueBasedTargetNonLinearMuscle::extractMuscleActivations(
        const Vector& x) const {
    return x(0, nma);
}

Vector
TorqueBasedTargetNonLinearMuscle::extractMuscleForces(const Vector& x) const {
    auto am = extractMuscleActivations(x);
    return fmMax
            .elementwiseMultiply(
                    am.elementwiseMultiply(fmL).elementwiseMultiply(fmV) + fmPE)
            .elementwiseMultiply(cosAlpha);
}

Vector
TorqueBasedTargetNonLinearMuscle::extractResidualForces(const Vector& x) const {
    return x(nma, getNumParameters() - nma);
}

int TorqueBasedTargetNonLinearMuscle::objectiveFunc(const Vector& x,
                                                    bool newCoefficients,
                                                    Real& rP) const {
    rP = 0.0;
    for (int i = 0; i < getNumParameters(); ++i) {
        rP += pow(abs(x[i]) / xMax[i], p);
    }
    return 0;
}

int TorqueBasedTargetNonLinearMuscle::gradientFunc(const Vector& x,
                                                   bool newCoefficients,
                                                   Vector& gradient) const {
    for (int i = 0; i < getNumParameters(); ++i) {
        if (x[i] < 0) {
            gradient[i] = -1.0 * p * pow(abs(x[i]) / xMax[i], p - 1);
        } else {
            gradient[i] = p * pow(abs(x[i]) / xMax[i], p - 1);
        }
    }
    return 0;
}

int TorqueBasedTargetNonLinearMuscle::constraintFunc(
        const Vector& x, bool newCoefficients, Vector& constraints) const {
    constraints = R * extractMuscleForces(x) + extractResidualForces(x) - tau;
    return 0;
}

int TorqueBasedTargetNonLinearMuscle::constraintJacobian(const Vector& x,
                                                         bool newCoefficients,
                                                         Matrix& jac) const {
    int n = getNumConstraints();
    jac(0, 0, n, nma) = R.colScale(fmMax.elementwiseMultiply(fmL)
                                           .elementwiseMultiply(fmV)
                                           .elementwiseMultiply(cosAlpha));
    jac(0, nma, n, n) = 1.0;
    return 0;
}
