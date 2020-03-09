#include "GRFMPrediction.h"
#include "Simulation.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Muscle.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*******************************************************************************/

GaitPhase::GaitPhase() { constructProperty_phase(Phase::INVALID); }

GaitPhase::GaitPhase(Phase phase) : GaitPhase() { set_phase(phase); }

GaitPhase::GaitPhase(int phase) : GaitPhase() { set_phase(phase); }

GaitPhase::operator int() const { return get_phase(); }

bool GaitPhase::operator==(Phase other) const { return get_phase() == other; }

bool GaitPhase::operator!=(Phase other) const { return get_phase() != other; }

bool GaitPhase::operator==(GaitPhase other) const {
    return get_phase() == other.get_phase();
}

bool GaitPhase::operator!=(GaitPhase other) const {
    return get_phase() != other.get_phase();
}

std::string GaitPhase::toString() const {
    switch (get_phase()) {
    case INVALID: return "INVALID"; break;
    case STANCE: return "STANCE"; break;
    case SWING: return "SWING"; break;
    case DOUBLE_SUPPORT: return "DOUBLE_SUPPORT"; break;
    case ACTIVE_ALWAYS: return "ACTIVE_ALWAYS"; break;
    default: throw OpenSim::Exception("invalid gait phase");
    }
}

//==============================================================================
GaitPhaseDetector::StanceSwingEvent::StanceSwingEvent(
        const HuntCrossleyForce& contact,
        const HuntCrossleyForce& contactContra, GaitPhase& phase,
        double threshold)
        : SimTK::TriggeredEventHandler(SimTK::Stage::Dynamics),
          contact(contact), contactContra(contactContra), phase(phase),
          threshold(threshold) {
    // default is both, but to be sure we set them explicitly
    getTriggerInfo().setTriggerOnRisingSignTransition(true);
    getTriggerInfo().setTriggerOnFallingSignTransition(true);
}

SimTK::Real
GaitPhaseDetector::StanceSwingEvent::getValue(const SimTK::State& s) const {
    // 0-3 force, 3-6 moment they must be negated
    auto contactWrench = contact.getRecordValues(s);
    auto contactWrenchContra = contactContra.getRecordValues(s);
    SimTK::Vec3 legForce(-contactWrench[0], -contactWrench[1],
                         -contactWrench[2]);
    SimTK::Vec3 legForceContra(-contactWrenchContra[0], -contactWrenchContra[1],
                               -contactWrenchContra[2]);

    // f > threshold
    contactValue = legForce.norm() - threshold;
    contactValueContra = legForceContra.norm() - threshold;
    // in case that phase is invalid (initial state)
    if (phase == GaitPhase::Phase::INVALID) { updatePhase(); }

    // cout << contactValue << " "
    //      << contactValueContra << endl;

    return contactValue;
}

void GaitPhaseDetector::StanceSwingEvent::handleEvent(
        SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate) const {
    updatePhase();
}

void GaitPhaseDetector::StanceSwingEvent::updatePhase() const {
    if (contactValue > 0) {
        phase = GaitPhase::Phase::STANCE;
    } else {
        phase = GaitPhase::Phase::SWING;
    }

    // if (contactValue > 0 && contactValueContra > 0) {
    // 	phase = GaitPhase::Phase::DOUBLE_SUPPORT;
    // }
}

//==============================================================================
GaitPhaseDetector::GaitPhaseDetector() : Component() {
    constructProperty_stance_threshold(0.1);
    phaseR = GaitPhase::Phase::INVALID;
    phaseL = GaitPhase::Phase::INVALID;
}

void GaitPhaseDetector::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    auto self = const_cast<GaitPhaseDetector*>(this);
    double threshold = self->get_stance_threshold();
    auto& contactR = getConnectee<HuntCrossleyForce>("contact_force_right");
    auto& contactL = getConnectee<HuntCrossleyForce>("contact_force_left");

    // right leg event
    auto stanceSwingR =
            new StanceSwingEvent(contactR, contactL, self->phaseR, threshold);
    system.addEventHandler(stanceSwingR);

    // left leg event
    auto stanceSwingL =
            new StanceSwingEvent(contactL, contactR, self->phaseL, threshold);
    system.addEventHandler(stanceSwingL);
}

//==============================================================================
/**
 * Calculates the Coriolis contribution \f$ \tau_c \f$. We assume that
 * \f$ \tau_c \f$ is on the same side with the joint space accelerations
 * (i.e. \f$ M \ddot{q} + \tau_c = \tau \f$).
 */
Vector calcCoriolis(const State& s, const Model& model) {
    Vector c;
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
            s, Vector(0), Vector_<SpatialVec>(0), Vector(0), c);
    return c;
}

GRFPrediction::GRFPrediction(const Model& otherModel, const Parameters& otherParameteres)
    : model(*otherModel.clone()), parameters(otherParameteres) {

    // leg phase detector
    gaitPhaseDetector = new GaitPhaseDetector();
    gaitPhaseDetector->setName("gait_phase_detector");
    gaitPhaseDetector->set_stance_threshold(parameters.stance_threshold);
    gaitPhaseDetector->connectSocket_contact_force_right(
            model.updForceSet().get(parameters.rightFootContactForceName));
    gaitPhaseDetector->connectSocket_contact_force_left(
            model.updForceSet().get(parameters.leftFootContactForceName));
    model.addComponent(gaitPhaseDetector);
    model.finalizeConnections();

    // define transition functions for reaction components
    reactionComponentTransition = [&](const double& time) -> double {
              return exp(-pow((time / Tds), 3));
          };
    anteriorForceTransition = [&](const double& time) -> double {
              return k1 * exp(-pow((time - Tp) / Tds, 2)) - k2 * time / Tds;
          };

    // disable muscles, otherwise they apply passive forces
    state = model.initSystem();
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        model.updMuscles()[i].setAppliesForce(state, false);
    }
}

void GRFPrediction::solve(const GRFPrediction::Input& input) {
    // get reference to matter subsystem
    const auto& matter = model.getMatterSubsystem();

    // update state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updQDot() = input.qDot;
    state.updQDotDot() = input.qDDot;

    // realize to dynamics stage so that all model velocities, accelerations and
    // forces are computed
    model.getMultibodySystem().realize(state, Stage::Dynamics);

    // const auto& component = model.getComponent("gait_phase_detector_r");
    // const auto& gaitPhaseDetector = dynamic_cast<const GaitPhaseDetector*>(&component);

    cout << gaitPhaseDetector->phaseL.toString() << " "
         << gaitPhaseDetector->phaseR.toString() << endl;

    // compute body velocities and accelerations
    matter.multiplyBySystemJacobian(state, input.qDot, bodyVelocities);
    matter.calcBodyAccelerationFromUDot(state, input.qDDot, bodyAccelerations);

    // get the vector of spatial forces (meaning the gravitational moment about
    // and force at the body origin, \e not necessarily the center of mass)
    // currently being applied to each of the mobilized bodies.
    const auto& gravityForces = model.getGravityForce().getBodyForces(state);

    // get applied mobility (generalized) forces generated by components of the
    // model, like actuators
    const Vector& appliedMobilityForces =
            model.getMultibodySystem().getMobilityForces(state,
                                                         Stage::Dynamics);

    // get all applied body forces [m, f]^T
    const Vector_<SpatialVec>& appliedBodyForces =
            model.getMultibodySystem().getRigidBodyForces(state,
                                                          Stage::Dynamics);

    // reaction force is the sum of forces applied due to body accelerations
    Vec3 reactionForce, externalMoment;
    for (int i = 0; i < model.getNumBodies(); ++i) {
        // get body segment
        const auto& body = model.getBodySet()[i];
        const auto& idx = body.getMobilizedBodyIndex();
        if (body.getName() == "ground") continue;

        // calculate F_ext = sum(m_i * (a_i - g) ) //* Yes, the sign is minus(-)
        reactionForce +=
                body.getMass() * (bodyAccelerations[idx][1] - model.getGravity());

        // calculate M_ext = sum( I_i * omega_dot + omega x (I_i * omega) -
        // sum_j( r_ij x F_ij )), where i: each body, j: endpoint of the ith
        // body. The negative term is the total torque applied to the body's
        // com.
        // const auto& I = matter.getArticulatedBodyInertia(
        //                               state, idx)
        //                         .getInertia();
        const auto& I = body.getInertia();

        // angular velocity-dependent force on the body due to rotational
        // inertia
        const auto& gyroscopicForce =
                matter.getGyroscopicForce(state, idx);

        // total angular velocity-dependent force acting on this body, including
        // forces due to Coriolis acceleration and gyroscopic forces due to
        // rotational inertia.
        const auto& centrifugalForce = matter.getTotalCentrifugalForces(
                state, idx);

        // estimate of ground reaction moment
        externalMoment += I * bodyAccelerations[idx][0] +
                          cross(bodyVelocities[idx][0], I * bodyVelocities[idx][0]);
    }

    // Vector logV(18);
    // auto point = Vec3(0, 0, 0);

    // int offset = 0;
    // while (offset < 18) {
    //     for (int i = 0; i < 3; ++i) {
    //         logV[i + offset] = point[i];
    //         logV[i + offset + 3] = reactionForce[i];
    //         logV[i + offset + 6] = externalMoment[i];
    //     }
    //     offset += 9;
    // }
    // center of pressure
    // logger->addRow(input.t, logV);
}

TimeSeriesTable GRFPrediction::initializeLogger() {
    std::vector<std::string> columnNames{
        "r_ground_force_px", "r_ground_force_py", "r_ground_force_pz",
        "r_ground_force_vx", "r_ground_force_vy", "r_ground_force_vz",
        "r_ground_torque_x", "r_ground_torque_y", "r_ground_torque_z",
        "l_ground_force_px", "l_ground_force_py", "l_ground_force_pz",
        "l_ground_force_vx", "l_ground_force_vy", "l_ground_force_vz",
        "l_ground_torque_x", "l_ground_torque_y", "l_ground_torque_z"};

    TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}