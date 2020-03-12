#include "GRFMPrediction.h"
#include "Simulation.h"
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Muscle.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*******************************************************************************/

GaitPhase::GaitPhase() : phase(Phase::INVALID) {}

GaitPhase::GaitPhase(Phase otherPhase) : phase(otherPhase) {}

GaitPhase::GaitPhase(int otherPhase) : phase(otherPhase) {}

GaitPhase::operator int() const { return phase; }

GaitPhase& GaitPhase::operator=(const GaitPhase& rhs) {
    if (this == &rhs)
        return *this;
    phase = rhs.phase;
    return *this;
}

bool GaitPhase::operator==(Phase other) const { return phase == other; }

bool GaitPhase::operator!=(Phase other) const { return phase != other; }

bool GaitPhase::operator==(GaitPhase other) const {
    return phase == other.phase;
}

bool GaitPhase::operator!=(GaitPhase other) const {
    return phase != other.phase;
}

std::string GaitPhase::toString() const {
    switch (phase) {
    case INVALID: return "INVALID"; break;
    case STANCE: return "STANCE"; break;
    case SWING: return "SWING"; break;
    case DOUBLE_SUPPORT: return "DOUBLE_SUPPORT"; break;
    case ACTIVE_ALWAYS: return "ACTIVE_ALWAYS"; break;
    default: throw OpenSim::Exception("invalid gait phase");
    }
}
//==============================================================================
ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(const Model& otherModel):
    model(*otherModel.clone()){
    phaseR = GaitPhase::Phase::INVALID;
    phaseL = GaitPhase::Phase::INVALID;

    // platform body
    auto platform = new OpenSim::Body();
    platform->setName("Platform");
    platform->setMass(1);
    platform->setMassCenter(Vec3(0));
    platform->setInertia(Inertia(0));
    platform->attachGeometry(new Brick(Vec3(1, 0.0075, 1)));
    model.addBody(platform);

    // joint connection the platform to the ground
    auto platformToGround = new WeldJoint("PlatformToGround", model.getGround(),
        Vec3(0), Vec3(0), *platform, Vec3(-0.6, -0.035, 0), Vec3(0));
    model.addJoint(platformToGround);

    // contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->setLocation(Vec3(0.0, -0.0075, 0.0));
    platformContact->setOrientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setBody(*platform);
    model.addContactGeometry(platformContact);

    // contact spheres
    auto rightHeelContact = new ContactSphere();
    rightHeelContact->setName("RHeelContact");
    rightHeelContact->setRadius(0.04);
    rightHeelContact->setLocation(Vec3(0.012, -0.0015, -0.005));
    rightHeelContact->setBody(model.getBodySet().get("calcn_r"));
    model.addContactGeometry(rightHeelContact);

    auto leftHeelContact = new ContactSphere();
    leftHeelContact->setName("LHeelContact");
    leftHeelContact->setRadius(0.04);
    leftHeelContact->setLocation(Vec3(0.012, -0.0015, 0.005));
    leftHeelContact->setBody(model.getBodySet().get("calcn_l"));
    model.addContactGeometry(leftHeelContact);

    auto rightToeContact = new ContactSphere();
    rightToeContact->setName("RToeContact");
    rightToeContact->setRadius(0.04);
    rightToeContact->setLocation(Vec3(0.06, -0.01, -0.02));
    rightToeContact->setBody(model.getBodySet().get("toes_r"));
    model.addContactGeometry(rightToeContact);

    auto leftToeContact = new ContactSphere();
    leftToeContact->setName("LToeContact");
    leftToeContact->setRadius(0.04);
    leftToeContact->setLocation(Vec3(0.06, -0.01, 0.02));
    leftToeContact->setBody(model.getBodySet().get("toes_l"));
    model.addContactGeometry(leftToeContact);

    // right foot contact parameters
    auto rightFootContactParams = new HuntCrossleyForce::ContactParameters();
    rightFootContactParams->setStiffness(2000000);
    rightFootContactParams->setDissipation(1.0);
    rightFootContactParams->setStaticFriction(0.9);
    rightFootContactParams->setDynamicFriction(0.8);
    rightFootContactParams->setViscousFriction(0.6);
    rightFootContactParams->addGeometry("RHeelContact");
    rightFootContactParams->addGeometry("RToeContact");
    rightFootContactParams->addGeometry("PlatformContact");

    // left foot contact parameters
    auto leftFootContactParams = new HuntCrossleyForce::ContactParameters();
    leftFootContactParams->setStiffness(2000000);
    leftFootContactParams->setDissipation(1.0);
    leftFootContactParams->setStaticFriction(0.9);
    leftFootContactParams->setDynamicFriction(0.8);
    leftFootContactParams->setViscousFriction(0.6);
    leftFootContactParams->addGeometry("LHeelContact");
    leftFootContactParams->addGeometry("LToeContact");
    leftFootContactParams->addGeometry("PlatformContact");

    // right foot Force
    rightFootContactForce = new HuntCrossleyForce(rightFootContactParams);
    rightFootContactForce->setName("RightFootContactForce");
    model.addForce(rightFootContactForce);

    // left foot Force
    leftFootContactForce = new HuntCrossleyForce(leftFootContactParams);
    leftFootContactForce->setName("LeftFootContactForce");
    model.addForce(leftFootContactForce);

    state = model.initSystem();
}

vector<GaitPhase> ContactForceBasedPhaseDetector::getPhase(const GRFPrediction::Input& input) {
    updPhase(input);
    return {phaseR, phaseL};
}

void ContactForceBasedPhaseDetector::updPhase(const GRFPrediction::Input& input) {

    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    int i = 0;
    for (const auto& coord : coordinateSet){
        coord->setValue(state, input.q[i]);
        coord->setSpeedValue(state, input.qDot[i]);
        ++i;
    }

    model.realizeDynamics(state);
    auto rightContactWrench = rightFootContactForce->getRecordValues(state);
    auto leftContactWrench = leftFootContactForce->getRecordValues(state);

    Vec3 rightLegForce(-rightContactWrench[0], -rightContactWrench[1],
                       -rightContactWrench[2]);
    Vec3 leftLegForce(-leftContactWrench[0], -leftContactWrench[1],
                      -leftContactWrench[2]);

    double threshold = 20; // todo

    // f > threshold
    auto rightContactValue = rightLegForce.norm() - threshold;
    auto leftContactValue = leftLegForce.norm() - threshold;

    // cout << rightLegForce.norm() << endl;

    if (rightContactValue > 0) {
        phaseR = GaitPhase::Phase::STANCE;
    } else {
        phaseR = GaitPhase::Phase::SWING;
    }

    if (leftContactValue > 0) {
        phaseL = GaitPhase::Phase::STANCE;
    } else {
        phaseL = GaitPhase::Phase::SWING;
    }

}
//==============================================================================
// VelocityBasedPhaseDetector::VelocityBasedPhaseDetector(Model* otherModel) :
//         model(otherModel) {
//     phaseR = GaitPhase::Phase::INVALID;
//     phaseL = GaitPhase::Phase::INVALID;

//     // add station point to the model
//     heelStationR =
//         new Station(model->getBodySet().get("calcn_r"), Vec3(0.012, -0.0015, -0.005));
//     heelStationL =
//         new Station(model->getBodySet().get("calcn_l"), Vec3(0.012, -0.0015, 0.005));
//     toeStationR =
//         new Station(model->getBodySet().get("toes_r"), Vec3(0.06, -0.01, -0.02));
//     toeStationL =
//         new Station(model->getBodySet().get("toes_l"), Vec3(0.06, -0.01, 0.02));

//     const auto& pelvisBody = model->getBodySet().get("pelvis");
//     pelvisStation = new Station(pelvisBody, pelvisBody.getMassCenter());

//     heelStationR->setName("heel_station_point_r");
//     heelStationL->setName("heel_station_point_l");
//     toeStationR->setName("toe_station_point_r");
//     toeStationL->setName("toe_station_point_l");
//     pelvisStation->setName("pelvis_station_point");

//     model->addModelComponent(heelStationR);
//     model->addModelComponent(heelStationL);
//     model->addModelComponent(toeStationR);
//     model->addModelComponent(toeStationL);
//     model->addModelComponent(pelvisStation);
// }

// vector<GaitPhase> VelocityBasedPhaseDetector::getPhase(const SimTK::State& state) {
//     updPhase(state);
//     return {phaseR, phaseL};
// }
// void VelocityBasedPhaseDetector::updPhase(const SimTK::State& state) {
//     model->realizeAcceleration(state);
//     const auto& rHeel = heelStationR->getVelocityInGround(state).norm();
//     const auto& lHeel = heelStationL->getVelocityInGround(state).norm();
//     const auto& rToe = toeStationR->getVelocityInGround(state).norm();
//     const auto& lToe = toeStationL->getVelocityInGround(state).norm();
//     const auto& pelvis = pelvisStation->getVelocityInGround(state).norm();

//     double threshold = 20; // todo
//     // cout << state.getTime() << ", " <<  rHeel << ", " << lHeel << ", "
//     //      << rToe << ", " << lToe << ", " << pelvis << endl;

//     // // update phase
//     // if (rHeel > threshold && rToe > threshold) {
//     //     phaseR = GaitPhase::Phase::SWING;
//     // } else {
//     //     phaseR = GaitPhase::Phase::STANCE;
//     // }

//     // if (lHeel > threshold && lToe > threshold) {
//     //     phaseL = GaitPhase::Phase::SWING;
//     // } else {
//     //     phaseL = GaitPhase::Phase::STANCE;
//     // }
// }


//==============================================================================

GRFPrediction::GRFPrediction(const Model& otherModel)
    : model(*otherModel.clone()) {

    // leg phase detector
    gaitPhaseDetector = new ContactForceBasedPhaseDetector(model);

    // disable muscles, otherwise they apply passive forces
    state = model.initSystem();
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        model.updMuscles()[i].setAppliesForce(state, false);
    }

    // define transition functions for reaction components
    k1 = exp(4.0 / 9.0);
    k2 = exp(-12.0 / 9.0) / 2.0;
    reactionComponentTransition = [&](const double& time) -> double {
              return exp(-pow((time / Tds), 3));
          };
    anteriorForceTransition = [&](const double& time) -> double {
              return k1 * exp(-pow((time - Tp) / Tds, 2)) - k2 * time / Tds;
          };
}

void GRFPrediction::solve(const GRFPrediction::Input& input) {
    auto phase = gaitPhaseDetector->getPhase(input);
    // cout << "Time: " << input.t << " "
    //     << "Right: " << phase[0].toString() << " "
    //     << "Left: " << phase[1].toString() << endl;

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