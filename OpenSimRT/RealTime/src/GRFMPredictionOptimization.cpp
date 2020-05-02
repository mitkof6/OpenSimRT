#include "GRFMPredictionOptimization.h"

#include "Exception.h"

#include <OpenSim/OpenSim.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;
using namespace OpenSimRT;

// help functions
//==============================================================================
Vector multiplyByJacobianT(const State& state, const Model& model,
                           const Vector_<SpatialVec>& spatialForces) {
    Vector generalizedForces;
    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(
            state, spatialForces, generalizedForces);
    return generalizedForces;
}

template <typename T>
void updateState(const T& input, const Model& model, State& state,
                 const Stage& stage) {
    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < coordinateSet.size(); ++i) {
        coordinateSet[i]->setValue(state, input.q[i]);
        coordinateSet[i]->setSpeedValue(state, input.qd[i]);
    }
    model.getMultibodySystem().realize(state, stage);
}

//==============================================================================

ContactForceActuator::ContactForceActuator(const Model& model,
                                           const std::string& bodyName,
                                           const Vec3& pointOrigin,
                                           const Vec3& planeOrigin,
                                           const std::string& aName) {
    name = aName;

    optimal_force = 200.0;
    // static_friction = 0.8;
    // kinetic_friction = 0.5;
    // contact_tolerance = 0;
    // contact_transition_zone = 0.01;
    // v_trans = 0.1;

    minMultipliers.resize(getNumMultipliers());
    maxMultipliers.resize(getNumMultipliers());
    minMultipliers = Vector(3, -Infinity);
    minMultipliers[1] = 0;
    maxMultipliers = Vector(3, Infinity);

    point_body = bodyName;
    point_location = pointOrigin;
    plane_origin = planeOrigin;
    plane_normal = Vec3(0, 1, 0);

    pointBody =
            ReferencePtr<const OpenSim::Body>(model.getBodySet().get(bodyName));
}

//==============================================================================
// getters/setters
std::string ContactForceActuator::getName() const { return name; }

void ContactForceActuator::setName(const std::string& otherName) {
    name = otherName;
}

Vector ContactForceActuator::getMaxMultipliers() const {
    return maxMultipliers;
}

Vector ContactForceActuator::getMinMultipliers() const {
    return minMultipliers;
}

double ContactForceActuator::getOptimalForce() const { return optimal_force; }

void ContactForceActuator::setOptimalForce(double force) {
    optimal_force = force;
}

int ContactForceActuator::getNumMultipliers() { return 3; }

int ContactForceActuator::getNumMultiplierConstraints() { return 4; }

//==============================================================================

void ContactForceActuator::computeForceVectors(
        const State& state, const Vector& multipliers,
        ContactWrench& planeWrench, ContactWrench& bodyWrench) const {
    // calculate force directions with respect to plane in ground frame
    const UnitVec3 normal(plane_normal);
    // const UnitVec3 shear1 = normal.perp();
    // const UnitVec3 shear2 = UnitVec3(normal % shear1);

    // transform contact point into ground frame (=force station for pointBody)
    bodyWrench.point =
            pointBody->findStationLocationInGround(state, point_location);

    // compute contact point to contact plane distance
    const auto distance = ~(bodyWrench.point - plane_origin) * normal;

    // projection of contact point onto contact plane
    planeWrench.point = bodyWrench.point - distance * normal;

    // // compute relative in-plane velocity of contact point
    // const Vec3 point_vel = pointBody->getVelocityInGround(state)[1];
    // const Vec3 v_rel_in_plane = point_vel - normal * (~point_vel * normal);
    // const auto velocity = v_rel_in_plane.norm();

    // // compute reference contact strength (depending on point to plane
    // distance) double force_ref = 0; if (contact_tolerance < 0 || distance <
    // 0)
    //     force_ref = optimal_force;
    // else if (distance > 0 &&
    //          distance <= contact_tolerance + contact_transition_zone) {
    //     const double arg = (-2.0 * (std::abs(distance) - contact_tolerance) +
    //                         contact_transition_zone) /
    //                        contact_transition_zone;
    //     force_ref = 0.5 * optimal_force * (1 + std::tanh(arg * Pi));
    // }

    // // detect if we are in the static or in the kinetic regime
    // if (velocity <= v_trans) {
    //     f_on_plane = (-multipliers[0] * shear1 + multipliers[1] * normal +
    //                   multipliers[2] * shear2) *
    //                  force_ref;
    // } else {
    //     const Vec3 shear =
    //             multipliers[1] * kinetic_friction *
    //             v_rel_in_plane.normalize();
    //     f_on_plane = (-multipliers[1] * normal + shear) * force_ref;
    // }
    double force_ref = (distance <= 0) ? optimal_force : 0;
    planeWrench.force =
            Vec3(multipliers[0], multipliers[1], multipliers[2]) * force_ref;

    // force on point body acts along the opposite direction
    planeWrench.moment = Vec3(0); // todo
    bodyWrench.force = -planeWrench.force;
    bodyWrench.moment = -planeWrench.moment;
}
// void ContactForceActuator::computeForceGradient(
//         const SimTK::State& state, const SimTK::Vector& multipliers) const {
//     const UnitVec3 normal(plane_normal);
//     const auto& point =
//             pointBody->findStationLocationInGround(state, point_location);
//     const auto distance = ~(point - plane_origin) * normal;

//     double force_ref = (distance <= 0) ? optimal_force : 0;
// }

//==============================================================================

ContactForceAnalysis::ContactForceAnalysis(const Model& otherModel,
                                           const Parameters& parameters)
        : model(*otherModel.clone()), numContactMultipliers(0) {
    // contact force elements
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0133, -0.01, 0.0054),
            parameters.platform_offset, "heel_lat_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0133, -0.01, -0.015),
            parameters.platform_offset, "heel_med_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0805, -0.01, 0.0268),
            parameters.platform_offset, "mid_lat_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.110, -0.01, -0.030),
            parameters.platform_offset, "mid_med_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.176, 0.005, -0.029),
            parameters.platform_offset, "met_1_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.174, 0.005, -0.002),
            parameters.platform_offset, "met_2_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.166, 0.005, 0.0168),
            parameters.platform_offset, "met_3_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.1520, 0.005, 0.029),
            parameters.platform_offset, "met_4_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.1435, 0.005, 0.044),
            parameters.platform_offset, "met_5_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.233, 0.015, -0.0187),
            parameters.platform_offset, "toe_1_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.247, 0.015, 0.0133),
            parameters.platform_offset, "toe_2_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.230, 0.015, 0.028),
            parameters.platform_offset, "toe_3_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.210, 0.015, 0.040),
            parameters.platform_offset, "toe_4_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.197, 0.015, 0.052),
            parameters.platform_offset, "toe_5_r"));

    // left
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0133, -0.01, -0.0054),
            parameters.platform_offset, "heel_lat_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0133, -0.01, 0.015),
            parameters.platform_offset, "heel_med_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0805, -0.01, -0.0268),
            parameters.platform_offset, "mid_lat_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.110, -0.01, 0.030),
            parameters.platform_offset, "mid_med_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.176, 0.005, 0.029),
            parameters.platform_offset, "met_1_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.174, 0.005, 0.002),
            parameters.platform_offset, "met_2_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.166, 0.005, -0.0168),
            parameters.platform_offset, "met_3_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.152, 0.005, -0.029),
            parameters.platform_offset, "met_4_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.1435, 0.005, -0.044),
            parameters.platform_offset, "met_5_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.233, 0.015, 0.0187),
            parameters.platform_offset, "toe_1_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.247, 0.015, -0.0133),
            parameters.platform_offset, "toe_2_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.230, 0.015, 0.010),
            parameters.platform_offset, "toe_3_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.210, 0.015, -0.04),
            parameters.platform_offset, "toe_4_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.197, 0.015, -0.052),
            parameters.platform_offset, "toe_5_l"));

    // number of multipliers and constraints
    for (int i = 0; i < contacts.size(); ++i) {
        numContactMultipliers +=
                contacts[i]->getNumMultipliers(); // contacts x 3
    }

    // create new static optimization problem
    problem = new ContactForceAnalysisTarget(&model, this);

    // create optimizer
    optimizer = new Optimizer(*problem, OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(parameters.convergence_tolerance);
    optimizer->setConstraintTolerance(parameters.constraint_tolerance);
    optimizer->setMaxIterations(300);
    optimizer->setDiagnosticsLevel(0);
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(true);
    optimizer->setLimitedMemoryHistory(100);
    optimizer->setAdvancedBoolOption("warm_start", true);
    optimizer->setAdvancedRealOption("expect_infeasible_problem", false);
    optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->setAdvancedStrOption("nlp_scaling_method", "none");
    problem->initParameters(parameterSeeds, 0.0);
}

Vector ContactForceAnalysis::Output::asVector() {
    auto output = Vector(9);
    output[0] = point[0];
    output[1] = point[1];
    output[2] = point[2];
    output[3] = force[0];
    output[4] = force[1];
    output[5] = force[2];
    output[6] = moment[0];
    output[7] = moment[1];
    output[8] = moment[2];
    return output;
}

std::vector<ContactForceAnalysis::Output>
ContactForceAnalysis::solve(const Input& input) {
    try {
        // problem->initParameters(parameterSeeds, 1);
        problem->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
    } catch (SimTK::Exception::OptimizerFailed& e) {
        // std::cerr << e.what() << std::endl;
    }

    auto grf = problem->getContactForces(parameterSeeds);
    auto mixR = model.getBodySet().get("calcn_r").getMobilizedBodyIndex();
    auto mixL = model.getBodySet().get("calcn_l").getMobilizedBodyIndex();

    const auto& rightReactionForce = -grf[mixR][1];
    const auto& leftReactionForce = -grf[mixL][1];

    // std::cout << parameterSeeds << std::endl;
    std::cout << input.t << "," << rightReactionForce[0] << ","
              << rightReactionForce[1] << "," << rightReactionForce[2]
              << std::endl;

    // results
    Output rightLegOutput;
    rightLegOutput.t = input.t;
    rightLegOutput.force = rightReactionForce;
    rightLegOutput.moment = Vec3(0);
    rightLegOutput.point = Vec3(0);

    Output leftLegOutput;
    leftLegOutput.t = input.t;
    leftLegOutput.force = leftReactionForce;
    leftLegOutput.moment = Vec3(0);
    leftLegOutput.point = Vec3(0);

    return {rightLegOutput, leftLegOutput};
}

//==============================================================================
ContactForceAnalysisTarget::ContactForceAnalysisTarget(
        Model* otherModel, ContactForceAnalysis* otherAnalysis)
        : model(otherModel), analysis(otherAnalysis) {
    // init system
    state = model->initSystem();

    // contacts per body
    for (int i = 0; i < analysis->contacts.size(); ++i) {
        contactsPerBodyMap.emplace(analysis->contacts[i]->point_body,
                                   analysis->contacts[i]);
    }

    // disable muscles
    for (int i = 0; i < model->getMuscles().getSize(); ++i) {
        model->updMuscles()[i].setAppliesForce(state, false);
    }

    // // disable controllers
    // model->setAllControllersEnabled(false);

    // set the structure of the problem
    setNumParameters(analysis->numContactMultipliers); // contacts x 3
    setNumEqualityConstraints(model->getCoordinateSet().getSize());
    setNumLinearEqualityConstraints(model->getCoordinateSet().getSize());

    // set limits
    Vector lb(getNumParameters()), ub(getNumParameters());
    const int numMult = ContactForceActuator::getNumMultipliers();
    int rowOffset = 0;
    for (int i = 0; i < analysis->contacts.size(); ++i) {
        lb(rowOffset, numMult) = analysis->contacts[i]->getMinMultipliers();
        ub(rowOffset, numMult) = analysis->contacts[i]->getMaxMultipliers();
        rowOffset += numMult;
    }
    setParameterLimits(lb, ub);
}

void ContactForceAnalysisTarget::prepareForOptimization(
        const ContactForceAnalysis::Input& input) const {
    // update state
    updateState(input, *model, state, Stage::Dynamics);

    // get applied mobility (generalized) forces
    const Vector& appliedMobilityForces =
            model->getMultibodySystem().getMobilityForces(state,
                                                          Stage::Dynamics);

    // get all applied body forces, e.g. gravity
    const Vector_<SpatialVec>& appliedBodyForces =
            model->getMultibodySystem().getRigidBodyForces(state,
                                                           Stage::Dynamics);
    // tau = M*(q) * q'' + C(q, q') + G(q)
    model->getMultibodySystem()
            .getMatterSubsystem()
            .calcResidualForceIgnoringConstraints(state, appliedMobilityForces,
                                                  appliedBodyForces, input.qdd,
                                                  tau);
}

void ContactForceAnalysisTarget::initParameters(Vector& parameters,
                                                double&& value) const {
    // ToDo: initialize to meaningful values e.g. between lb and ub.
    parameters = Vector(getNumParameters(), value);
}

int ContactForceAnalysisTarget::constraintFunc(const Vector& parameters,
                                               bool new_parameters,
                                               Vector& errors) const {
    // tau + E(lambda) = 0
    errors = tau +
             multiplyByJacobianT(state, *model, getContactForces(parameters));
    return 0;
}

int ContactForceAnalysisTarget::constraintJacobian(const Vector& parameters,
                                                   bool new_parameters,
                                                   Matrix& jacobian) const {
    // jacobian = multiplyByJacobianT(state, *model,... ) // todo
    return 0;
}

int ContactForceAnalysisTarget::objectiveFunc(const Vector& parameters,
                                              bool new_parameters,
                                              Real& objective) const {
    objective = ~parameters * parameters;
    return 0;
}

int ContactForceAnalysisTarget::gradientFunc(const Vector& parameters,
                                             bool new_parameters,
                                             Vector& gradient) const {
    gradient = 2 * parameters;
    return 0;
}

Vector_<SpatialVec>
ContactForceAnalysisTarget::getContactForces(const Vector& multipliers) const {
    Vector_<SpatialVec> appliedContactForces;
    ContactForceActuator::ContactWrench planeWrench, bodyWrench;
    appliedContactForces.resize(model->getMatterSubsystem().getNumBodies())
            .setToZero();

    const int numMult = ContactForceActuator::getNumMultipliers();
    int rowOffset = 0;
    for (auto elem : contactsPerBodyMap) {
        const auto& mbix =
                model->getBodySet().get(elem.first).getMobilizedBodyIndex();

        elem.second->computeForceVectors(state, multipliers(rowOffset, numMult),
                                         planeWrench, bodyWrench);
        rowOffset += numMult;
        appliedContactForces[mbix][0] += bodyWrench.moment;
        appliedContactForces[mbix][1] += bodyWrench.force;
    }

    return appliedContactForces;
}
