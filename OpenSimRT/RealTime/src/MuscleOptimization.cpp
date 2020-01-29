#include "MuscleOptimization.h"

#include "Exception.h"
#include "OpenSimUtils.h"

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Matrix calculateMomentArm(const State& s, const Model& model,
                          const vector<int>& activeCoordinateIndices,
                          const vector<int>& activeMuscleIndices) {
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();
    const auto& muscles = model.getMuscles();
    Matrix R(activeCoordinateIndices.size(), activeMuscleIndices.size());
    for (int i = 0; i < activeCoordinateIndices.size(); i++) {
        for (int j = 0; j < activeMuscleIndices.size(); j++) {
            const Coordinate& coord = *coordinates[i];
            R[i][j] = muscles[activeMuscleIndices[j]].computeMomentArm(
                    s, const_cast<Coordinate&>(coord));
        }
    }
    return R;
}

/*******************************************************************************/

MuscleOptimization::MuscleOptimization(
        const Model& modelOther,
        const MuscleOptimization::OptimizationParameters&
                optimizationParameters,
        const MomentArmFunction& momentArmFunction)
        : model(*modelOther.clone()) {
    // configure optimizer
    target = new TorqueBasedTarget(&model,
                                   optimizationParameters.objectiveExponent,
                                   momentArmFunction);
    optimizer = new Optimizer(*target, optimizationParameters.algorithm);
    optimizer->setConvergenceTolerance(
            optimizationParameters.convergenceTolerance);
    optimizer->setMaxIterations(optimizationParameters.maximumIterations);
    optimizer->setDiagnosticsLevel(0); // for debugging
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(false);
    if (optimizationParameters.algorithm == OptimizerAlgorithm::InteriorPoint) {
        optimizer->setLimitedMemoryHistory(100);
        optimizer->setAdvancedBoolOption("warm_start", true);
        optimizer->setAdvancedRealOption("expect_infeasible_problem", false);
        optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
        optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    }
    parameterSeeds = Vector(target->getNumParameters(), 0.0);
}

MuscleOptimization::Output
MuscleOptimization::solve(const MuscleOptimization::Input& input) {
    try {
        target->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
    } catch (exception& e) {
        // optimization may find a feasible solution and fail
        cout << "Failed at time: " << input.t << endl << e.what() << endl;
        // exit(-1);
    }
    auto fm = target->extractMuscleForces(parameterSeeds);
    auto residuals = target->extractResidualForces(parameterSeeds);
    auto am = target->extractMuscleForces(
            parameterSeeds.elementwiseDivide(target->fMax));
    return MuscleOptimization::Output{input.t, am, fm, residuals};
}

TimeSeriesTable MuscleOptimization::initializeLogger() {
    auto columnNames = OpenSimUtils::getMuscleNames(model);

    TimeSeriesTable m;
    m.setColumnLabels(columnNames);
    return m;
}

/*******************************************************************************/

TorqueBasedTarget::TorqueBasedTarget(Model* model, int objectiveExponent,
                                     const MomentArmFunction& momentArmFunction)
        : model(model), p(objectiveExponent), calcMomentArm(momentArmFunction) {
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
            cout << "Add coordinate actuator: " << actuator->getName() << endl;
        }
    }
    setNumEqualityConstraints(cs.getSize());
    setNumLinearEqualityConstraints(cs.getSize());

    // parameter bounds
    npa = 0;
    auto& as = model->getActuators();
    int na = as.getSize();
    fMax = Vector(na, 0.0);
    Vector lowerBounds(na, 0.0), upperBounds(na, 0.0);
    for (int i = 0; i < na; ++i) {
        auto muscle = dynamic_cast<const Muscle*>(&as[i]);
        auto pathAct = dynamic_cast<const PathActuator*>(&as[i]);
        auto coordAct = dynamic_cast<const CoordinateActuator*>(&as[i]);
        if (muscle) {
            npa++;
            fMax[i] = muscle->getMaxIsometricForce();
            lowerBounds[i] = 0.0;
            upperBounds[i] = Infinity;
        } else if (pathAct) {
            npa++;
            fMax[i] = pathAct->getOptimalForce();
            lowerBounds[i] = 0.0;
            upperBounds[i] = Infinity;
        } else if (coordAct) {
            fMax[i] = coordAct->getOptimalForce();
            lowerBounds[i] = coordAct->getMinControl() * fMax[i];
            upperBounds[i] = coordAct->getMaxControl() * fMax[i];
        } else {
            THROW_EXCEPTION("unsupported type of actuator");
        }
    }
    setNumParameters(na);
    setParameterLimits(lowerBounds, upperBounds);

    // coordinate indexes - //! for Analytic solution
    const auto& coordinateSet = model->getCoordinateSet();
    for (int i = 0; i < coordinateSet.getSize(); ++i) {
        activeCoordinateIndices.push_back(i);
    }

    // muscle indexes - //! for Analytic solution
    const auto& muscleSet = model->getMuscles();
    for (int i = 0; i < muscleSet.getSize(); ++i) {
        activeMuscleIndices.push_back(i);
    }

    // initialize system - //! for Analytic solution
    state = model->initSystem();
}

void TorqueBasedTarget::prepareForOptimization(
        const MuscleOptimization::Input& input) {

    // update state - //! for Analytic solution
    state.updQ() = input.q;
    model->getMultibodySystem().realize(state, Stage::Position);

    tau = input.tau;
    R = calcMomentArm(input.q);
    // R = calculateMomentArm(state, *model,
    //         activeCoordinateIndices,
    //         activeMuscleIndices);
}
Vector TorqueBasedTarget::extractMuscleForces(const Vector& x) const {
    return x(0, npa);
}

Vector TorqueBasedTarget::extractResidualForces(const Vector& x) const {
    return x(npa, getNumParameters() - npa);
}

int TorqueBasedTarget::objectiveFunc(const Vector& x, bool newCoefficients,
                                     Real& rP) const {
    rP = 0.0;
    for (int i = 0; i < getNumParameters(); ++i) {
        rP += pow(abs(x[i]) / fMax[i], p);
    }
    return 0;
}

int TorqueBasedTarget::gradientFunc(const Vector& x, bool newCoefficients,
                                    Vector& gradient) const {
    for (int i = 0; i < getNumParameters(); ++i) {
        if (x[i] < 0) {
            gradient[i] = -1.0 * p * pow(abs(x[i]) / fMax[i], p - 1);
        } else {
            gradient[i] = p * pow(abs(x[i]) / fMax[i], p - 1);
        }
    }
    return 0;
}

int TorqueBasedTarget::constraintFunc(const Vector& x, bool newCoefficients,
                                      Vector& constraints) const {
    constraints = R * extractMuscleForces(x) + extractResidualForces(x) - tau;
    return 0;
}

int TorqueBasedTarget::constraintJacobian(const Vector& x, bool newCoefficients,
                                          Matrix& jac) const {
    int n = getNumConstraints();
    jac(0, 0, n, npa) = R;
    jac(0, npa, n, n) = 1.0;
    return 0;
}

/*******************************************************************************/
