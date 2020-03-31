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

//==============================================================================

ContactPointOnPlane::ContactPointOnPlane() {
    name = "contact_force_element";
    optimal_force = 100.0;
    static_friction = 0.8;
    kinetic_friction = 0.5;
    contact_tolerance = -1.0;
    contact_transition_zone = 0.01;
    v_trans = 0.1;

    plane_body = "ground";
    plane_origin = Vec3(0, 0, 0);
    plane_normal = Vec3(0, 1, 0);

    point_body = "ground";
    point_location = Vec3(0, 0, 0);

    minMultipliers.resize(getNumMultipliers());
    minMultipliers[0] = 0;         // normal component
    minMultipliers[1] = -Infinity; // shear 1
    minMultipliers[2] = -Infinity; // shear 2
    maxMultipliers.resize(getNumMultipliers());
    maxMultipliers = Infinity; // all componentes
    planeBody = nullptr;
    pointBody = nullptr;
    distance = 0;
    velocity = 0;
}

ContactPointOnPlane::ContactPointOnPlane(const Model& model,
                                         const std::string& bodyName,
                                         const Vec3& point,
                                         const std::string& aName)
        : ContactPointOnPlane() {
    point_body = bodyName;
    point_location = point;
    name = aName;
    pointBody = &model.getBodySet().get(bodyName);
    planeBody = &model.getBodySet().get("Platform");
}

std::string ContactPointOnPlane::getName() const { return name; }

void ContactPointOnPlane::setName(const std::string& otherName) {
    name = otherName;
}

Vector ContactPointOnPlane::getMaxMultipliers() const { return maxMultipliers; }

Vector ContactPointOnPlane::getMinMultipliers() const { return minMultipliers; }

double ContactPointOnPlane::getOptimalForce() const { return optimal_force; }

void ContactPointOnPlane::setOptimalForce(double force) {
    optimal_force = force;
}

int ContactPointOnPlane::getNumMultipliers() { return 3; }

int ContactPointOnPlane::getNumMultiplierConstraints() { return 4; }

void ContactPointOnPlane::computeForceVectors(
        const State& state, const Vector& multipliers, Vec3& f_on_plane,
        Vec3& f_on_point, Vec3& station_on_planeBody,
        Vec3& station_on_pointBody) const {

    // calculate force directions with respect to plane in ground frame
    const UnitVec3 normal(planeBody->expressVectorInGround(state, plane_normal));
    // calculate arbitrary vector that is perpendicular to plane normal
    const UnitVec3 shear1 = normal.perp();
    const UnitVec3 shear2 = UnitVec3(normal % shear1);

    // transform contact point into ground frame (=force station for pointBody)
    station_on_pointBody =
            pointBody->findStationLocationInGround(state, point_location);

    // transform plane origin into ground frame
    const Vec3 p_origin =
            planeBody->findStationLocationInGround(state, plane_origin);

    // compute contact point to contact plane distance
    distance = std::abs(~(station_on_pointBody - p_origin) * normal);

    // calculate orthogonal projection of the contact point onto the contact
    // plane (=force station for planeBody)
    station_on_planeBody = station_on_pointBody - distance * normal;

    // compute relative in-plane velocity of contact point
    const Vec3 point_vel = pointBody->getVelocityInGround(state)[1];
    const Vec3 plane_vel = planeBody->getVelocityInGround(state)[1];
    const Vec3 v_rel = point_vel - plane_vel;

    // project v_rel into plane
    const Vec3 v_rel_in_plane = v_rel - normal * (~v_rel * normal);
    velocity = v_rel_in_plane.norm();

    // compute reference contact strength (depending on point to plane distance)
    double force_ref = 0;
    if (contact_tolerance < 0) // negative values ==> distance has no effect
        force_ref = optimal_force;
    else if (distance <= contact_tolerance + contact_transition_zone) {
        const double arg = (-2.0 * (distance - contact_tolerance) +
                            contact_transition_zone) /
                           contact_transition_zone;
        force_ref = 0.5 * optimal_force * (1 + std::tanh(arg * Pi));
    }

    // detect if we are in the static or in the kinetic regime, if so: caculate
    // force on plane body (normal component in opposite direction of plane
    // normal!)
    if (velocity <= v_trans) {
        f_on_plane = (-multipliers[0] * normal + multipliers[1] * shear1 +
                           multipliers[2] * shear2) *
                          force_ref;
    } else {
        // shear force in opposite direction of in-plane velocity
        const Vec3 shear =
                multipliers[0] * kinetic_friction * v_rel_in_plane.normalize();
        // total force
        f_on_plane = (-multipliers[0] * normal + shear) * force_ref;
    }

    // force on point body acts along the opposite direction
    f_on_point = -f_on_plane;
}


// void ContactPointOnPlane::computeForceVectors(
//         const State& state, const Vector& multipliers, Vec3& f_on_plane,
//         Vec3& f_on_point, Vec3& station_on_planeBody,
//         Vec3& station_on_pointBody) const {

//     // calculate force directions with respect to plane in ground frame
//     const UnitVec3 normal(0, 1, 0); // y

//     // transform contact point into ground frame (=force station for pointBody)
//     station_on_pointBody =
//             pointBody->findStationLocationInGround(state, point_location);

//     // transform plane origin into ground frame
//     const Vec3 p_origin =
//             planeBody->findStationLocationInGround(state, plane_origin);

//     // projection of the contact point onto the contact  plane
//     distance = std::abs(~(station_on_pointBody - p_origin) * normal);
//     station_on_planeBody = station_on_pointBody - distance * normal;

//     f_on_plane = multipliers * optimal_force;
//     f_on_point = -f_on_plane;
// }

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

    // create contact force elements // todo
    contacts.push_back(
            new ContactPointOnPlane(model, "calcn_r", Vec3(0, 0, 0), "HeelR"));
    contacts.push_back(
            new ContactPointOnPlane(model, "calcn_l", Vec3(0, 0, 0), "HeelL"));

    // number of multipliers and constraints
    for (int i = 0; i < contacts.size(); ++i) {
        numContactMultipliers += contacts[i]->getNumMultipliers();
    }

    // create new static optimization problem
    problem = new ContactForceAnalysisTarget(&model, this);

    // create optimizer
    const OptimizerAlgorithm algorithm = OptimizerAlgorithm::InteriorPoint;
    Optimizer* optimizer = new Optimizer(*problem, algorithm);
    optimizer->setConvergenceTolerance(parameters.convergence_tolerance);
    optimizer->setConstraintTolerance(parameters.constraint_tolerance);
    optimizer->setMaxIterations(300);
    optimizer->setDiagnosticsLevel(3);
    optimizer->useNumericalGradient(true);
    optimizer->useNumericalJacobian(true);
    optimizer->setLimitedMemoryHistory(100);
    optimizer->setAdvancedBoolOption("warm_start", true);
    // optimizer->setAdvancedRealOption("expect_infeasible_problem", false);
    optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    // optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->setAdvancedStrOption("nlp_scaling_method", "none");
    problem->initParameters(parameterSeeds);
}

void ContactForceAnalysis::solve(const Input& input) {
    try {
        problem->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
        std::cout << "time=" << time << " "
                  << problem->printPerformance(parameterSeeds) << std::endl;
    } catch (SimTK::Exception::OptimizerFailed& e) {
        std::cerr << "time=" << time << " optimization failed!" << std::endl;
    }
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
    const int numMult = ContactPointOnPlane::getNumMultipliers();
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
    state.updQ() = input.q;
    state.updQDot() = input.qd;
    state.updQDotDot() = input.qdd;
    model->realizeDynamics(state);

    // update accelerations
    qdd = input.qdd;
}

void ContactForceAnalysisTarget::initParameters(Vector& parameters) const {
    // ToDo: initialize to meaningful values e.g. between lb and ub.
    parameters = 0;
}

int ContactForceAnalysisTarget::constraintFunc(const Vector& parameters,
                                               bool new_parameters,
                                               Vector& errors) const {
    Vector Mqdd;
    model->getMatterSubsystem().multiplyByM(state, qdd, Mqdd);

    // M*(q) * q'' + C(q, q') + G(q) + E(lambda) = 0
    errors = Mqdd + calcCoriolis(state, *model) + calcGravity(state, *model) +
             multiplyByJacobianT(state, *model, getContactForces(state, parameters));
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
ContactForceAnalysisTarget::getContactForces(const State& state, const Vector& multipliers) const {
    Vector_<SpatialVec> appliedContactForces;
    Vec3 F_plane, F_point, P_plane, P_point;
    appliedContactForces.resize(model->getNumBodies() + 1); // +1 for ground

    const int numMult = ContactPointOnPlane::getNumMultipliers();
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
                appliedContactForces[mix][1] += F_plane;
            }
        }
    }
    return appliedContactForces;
}

String
ContactForceAnalysisTarget::printPerformance(const Vector& parameters) const {
    double objective;
    Vector errors(getNumConstraints());
    objectiveFunc(parameters, true, objective);
    constraintFunc(parameters, true, errors);
    std::stringstream ss;
    ss << "performance=" << objective
       << " constraint violation=" << sqrt(~errors * errors);
    return ss.str();
}
