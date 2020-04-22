#include "GRFMPredictionOptimization.h"

#include "Exception.h"

#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim;
using namespace OpenSimRT;

// help functions
//==============================================================================
Vector calcGravity(const State& s, const Model& model) {
    Vector g;
    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(
            s, model.getGravityForce().getBodyForces(s), g);
    return -g;
}

Vector calcCoriolis(const State& s, const Model& model) {
    Vector c;
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
            s, Vector(0), Vector_<SpatialVec>(0), Vector(0), c);
    return c;
}

Matrix calcM(const State& s, const Model& model) {
    Matrix M;
    model.getMatterSubsystem().calcM(s, M);
    return M;
}

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

ContactForceActuator::ContactForceActuator() {
    name = "contact_force_element";
    optimal_force = 100.0;
    static_friction = 0.8;
    kinetic_friction = 0.5;
    contact_tolerance = 0;
    contact_transition_zone = 0.01;
    v_trans = 0.1;

    plane_body = "ground";
    plane_origin = Vec3(0, 0, 0);
    plane_normal = Vec3(0, 1, 0);

    point_body = "ground";
    point_location = Vec3(0, 0, 0);

    minMultipliers.resize(getNumMultipliers());
    minMultipliers = Vector(3, 0.0);
    maxMultipliers.resize(getNumMultipliers());
    maxMultipliers = Vector(3, 1.0);
    planeBody = nullptr;
    pointBody = nullptr;
}

ContactForceActuator::ContactForceActuator(const Model& model,
                                           const std::string& bodyName,
                                           const Vec3& point,
                                           const std::string& aName)
        : ContactForceActuator() {
    point_body = bodyName;
    point_location = point;
    name = aName;
    pointBody =
            ReferencePtr<const OpenSim::Body>(model.getBodySet().get(bodyName));
    planeBody = ReferencePtr<const OpenSim::Body>(
            model.getBodySet().get("Platform"));
}

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

void ContactForceActuator::computeForceVectors(
        const State& state, const Vector& multipliers, Vec3& f_on_plane,
        Vec3& f_on_point, Vec3& station_on_plane, Vec3& station_on_body) const {
    // calculate force directions with respect to plane in ground frame
    const UnitVec3 normal(
            planeBody->expressVectorInGround(state, plane_normal));
    // calculate arbitrary vector that is perpendicular to plane normal
    const UnitVec3 shear1 = normal.perp();
    const UnitVec3 shear2 = UnitVec3(normal % shear1);

    // transform contact point into ground frame (=force station for pointBody)
    station_on_body =
            pointBody->findStationLocationInGround(state, point_location);

    // transform plane origin into ground frame
    const Vec3 p_origin =
            planeBody->findStationLocationInGround(state, plane_origin);

    // compute contact point to contact plane distance
    const auto distance = ~(station_on_body - p_origin) * normal;

    // projection of contact point onto contact plane
    station_on_plane = station_on_body - distance * normal;

    // compute relative in-plane velocity of contact point
    const Vec3 point_vel = pointBody->getVelocityInGround(state)[1];
    const Vec3 plane_vel = planeBody->getVelocityInGround(state)[1];
    const Vec3 v_rel = point_vel - plane_vel;

    // project v_rel into plane
    const Vec3 v_rel_in_plane = v_rel - normal * (~v_rel * normal);
    const auto velocity = v_rel_in_plane.norm();

    // compute reference contact strength (depending on point to plane distance)
    double force_ref = 0;
    if (contact_tolerance < 0 ||
        distance < 0) // negative values ==> distance has no effect
        force_ref = optimal_force;
    else if (distance > 0 &&
             distance <= contact_tolerance + contact_transition_zone) {
        const double arg = (-2.0 * (std::abs(distance) - contact_tolerance) +
                            contact_transition_zone) /
                           contact_transition_zone;
        force_ref = 0.5 * optimal_force * (1 + std::tanh(arg * Pi));
    }

    // detect if we are in the static or in the kinetic regime
    if (velocity <= v_trans) {
        f_on_plane = (-multipliers[0] * shear1 + multipliers[1] * normal +
                      multipliers[2] * shear2) *
                     force_ref;
    } else {
        const Vec3 shear =
                multipliers[1] * kinetic_friction * v_rel_in_plane.normalize();
        f_on_plane = (-multipliers[1] * normal + shear) * force_ref;
    }

    // force on point body acts along the opposite direction
    f_on_point = -f_on_plane;
}

//==============================================================================

ContactForceAnalysis::ContactForceAnalysis(const Model& otherModel,
                                           const Parameters& parameters)
        : model(*otherModel.clone()), numContactMultipliers(0) {
    // platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    platform->attachGeometry(new Brick(Vec3(1, 0.0075, 1))); // todo
    model.addBody(platform);

    // weld joint
    auto platformToGround = new WeldJoint(
            "PlatformToGround", model.getGround(), Vec3(0), Vec3(0), *platform,
            Vec3(-0.6, -0.035, 0), Vec3(0)); // todo
    model.addJoint(platformToGround);

    // contact force elements
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0133, -0.0423, 0.0054), "heel_lat_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0133, -0.0423, -0.015), "heel_med_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.0805, -0.0173, 0.0268), "mid_lat_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.11, -0.0173, -0.03), "mid_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.176, -0.029, -0.029), "met_1_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.174, -0.028, -0.002), "met_2_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.166, -0.028, 0.0168), "met_3_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.152, -0.028, 0.029), "met_4_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.1435, -0.028, 0.044), "met_5_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.233, -0.02, -0.0187), "toe_1_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.247, -0.02, 0.0133), "toe_2_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.23, -0.02, 0.028), "toe_3_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.21, -0.02, 0.04), "toe_4_r"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_r", Vec3(0.197, -0.02, 0.052), "toe_5_r"));

    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0133, -0.0423, -0.0054), "heel_lat_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0133, -0.0423, 0.015), "heel_med_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.0805, -0.0173, -0.0268), "mid_lat_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.11, -0.0173, 0.03), "mid_med_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.176, -0.029, 0.029), "met_1_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.174, -0.028, 0.002), "met_2_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.166, -0.028, -0.0168), "met_3_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.152, -0.028, -0.029), "met_4_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.1435, -0.028, -0.044), "met_5_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.233, -0.02, 0.0187), "toe_1_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.247, -0.02, -0.0133), "toe_2_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.23, -0.02, -0.028), "toe_3_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.21, -0.02, -0.04), "toe_4_l"));
    contacts.push_back(new ContactForceActuator(
            model, "calcn_l", Vec3(0.197, -0.02, -0.052), "toe_5_l"));

    // number of multipliers and constraints
    for (int i = 0; i < contacts.size(); ++i) {
        numContactMultipliers += contacts[i]->getNumMultipliers();
    }

    // create new static optimization problem
    problem = new ContactForceAnalysisTarget(&model, this);

    // create optimizer
    const OptimizerAlgorithm algorithm = OptimizerAlgorithm::InteriorPoint;
    optimizer = new Optimizer(*problem, algorithm);
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
    problem->initParameters(parameterSeeds);
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
        problem->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
    } catch (SimTK::Exception::OptimizerFailed& e) {
        // std::cerr << e.what() << std::endl;
    }

    auto grf = problem->getForces(parameterSeeds);
    auto mixR = model.getBodySet().get("calcn_r").getMobilizedBodyIndex();
    auto mixL = model.getBodySet().get("calcn_l").getMobilizedBodyIndex();

    const auto& rightReactionForce = grf[mixR][1];
    const auto& leftReactionForce = grf[mixL][1];

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

    // disable controllers
    model->setAllControllersEnabled(false);

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

void ContactForceAnalysisTarget::initParameters(Vector& parameters) const {
    // ToDo: initialize to meaningful values e.g. between lb and ub.
    parameters = Vector(getNumParameters(), 0.0);
}

int ContactForceAnalysisTarget::constraintFunc(const Vector& parameters,
                                               bool new_parameters,
                                               Vector& errors) const {
    // tau + E(lambda) = 0
    errors = tau + multiplyByJacobianT(state, *model,
                                       getContactForces(state, parameters));
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
ContactForceAnalysisTarget::getContactForces(const State& state,
                                             const Vector& multipliers) const {
    Vector_<SpatialVec> appliedContactForces;
    Vec3 F_plane, F_point, P_plane, P_point;
    appliedContactForces.resize(model->getNumBodies() + 1); // +1 for ground

    const int numMult = ContactForceActuator::getNumMultipliers();
    int rowOffset = 0;
    for (int i = 0; i < model->getBodySet().getSize(); ++i) {
        const auto& body = model->getBodySet()[i];
        const auto& bodyName = body.getName();
        const auto& mix = body.getMobilizedBodyIndex();

        if (contactsPerBodyMap.find(bodyName) != contactsPerBodyMap.end()) {
            const auto& contactsInBody =
                    contactsPerBodyMap.equal_range(bodyName);
            for (auto itr = contactsInBody.first; itr != contactsInBody.second;
                 ++itr) {
                itr->second->computeForceVectors(
                        state, multipliers(rowOffset, numMult), F_plane,
                        F_point, P_plane, P_point);
                rowOffset += numMult;
                appliedContactForces[mix][0] += Vec3(0); // todo
                appliedContactForces[mix][1] += F_point;
            }
        }
    }
    return appliedContactForces;
}

SimTK::Vector_<SimTK::SpatialVec>
ContactForceAnalysisTarget::getForces(const Vector& parameters) const {
    return getContactForces(state, parameters);
}
