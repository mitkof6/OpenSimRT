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
    if (this == &rhs) return *this;
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
ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel)
        : model(*otherModel.clone()) {
    phaseR = GaitPhase::Phase::INVALID;
    phaseL = GaitPhase::Phase::INVALID;
    leadingLeg = LeadingLeg::LEFT;

    double heeSphereRadius = 0.035; // todo
    double toeSphereRadius = 0.03; // todo

    // platform
    auto platform = new OpenSim::Body();
    platform->setName("Platform");
    platform->setMass(1);
    platform->setMassCenter(Vec3(0));
    platform->setInertia(Inertia(0));
    platform->attachGeometry(new Brick(Vec3(1, 0.0075, 1)));
    model.addBody(platform);

    // weld joint
    auto platformToGround =
            new WeldJoint("PlatformToGround", model.getGround(), Vec3(0),
                          Vec3(0), *platform, Vec3(-0.6, -0.035, 0), Vec3(0));
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
    rightHeelContact->setRadius(heeSphereRadius);
    rightHeelContact->setLocation(Vec3(0.012, -0.0015, -0.005));
    rightHeelContact->setOrientation(Vec3(Pi / 2, 0, 0));
    rightHeelContact->setBody(model.getBodySet().get("calcn_r"));
    model.addContactGeometry(rightHeelContact);
    auto leftHeelContact = new ContactSphere();
    leftHeelContact->setName("LHeelContact");
    leftHeelContact->setRadius(heeSphereRadius);
    leftHeelContact->setLocation(Vec3(0.012, -0.0015, 0.005));
    leftHeelContact->setOrientation(Vec3(Pi / 2, 0, 0));
    leftHeelContact->setBody(model.getBodySet().get("calcn_l"));
    model.addContactGeometry(leftHeelContact);
    auto rightToeContact = new ContactSphere();
    rightToeContact->setName("RToeContact");
    rightToeContact->setRadius(toeSphereRadius);
    rightToeContact->setLocation(Vec3(0.055, 0.008, -0.01));
    rightToeContact->setOrientation(Vec3(Pi / 2, 0, 0));
    rightToeContact->setBody(model.getBodySet().get("toes_r"));
    model.addContactGeometry(rightToeContact);
    auto leftToeContact = new ContactSphere();
    leftToeContact->setName("LToeContact");
    leftToeContact->setRadius(toeSphereRadius);
    leftToeContact->setLocation(Vec3(0.055, 0.008, 0.01));
    leftToeContact->setOrientation(Vec3(Pi / 2, 0, 0));
    leftToeContact->setBody(model.getBodySet().get("toes_l"));
    model.addContactGeometry(leftToeContact);

    // right foot contact parameters
    auto rightHeelContactParams = new HuntCrossleyForce::ContactParameters();
    rightHeelContactParams->setStiffness(2e6);
    rightHeelContactParams->setDissipation(1.0);
    rightHeelContactParams->setStaticFriction(0.9);
    rightHeelContactParams->setDynamicFriction(0.8);
    rightHeelContactParams->setViscousFriction(0.6);
    rightHeelContactParams->addGeometry("RHeelContact");
    rightHeelContactParams->addGeometry("PlatformContact");
    auto rightToeContactParams = new HuntCrossleyForce::ContactParameters();
    rightToeContactParams->setStiffness(2e6);
    rightToeContactParams->setDissipation(1.0);
    rightToeContactParams->setStaticFriction(0.9);
    rightToeContactParams->setDynamicFriction(0.8);
    rightToeContactParams->setViscousFriction(0.6);
    rightToeContactParams->addGeometry("RToeContact");
    rightToeContactParams->addGeometry("PlatformContact");

    // left foot contact parameters
    auto leftHeelContactParams = new HuntCrossleyForce::ContactParameters();
    leftHeelContactParams->setStiffness(2e6);
    leftHeelContactParams->setDissipation(1.0);
    leftHeelContactParams->setStaticFriction(0.9);
    leftHeelContactParams->setDynamicFriction(0.8);
    leftHeelContactParams->setViscousFriction(0.6);
    leftHeelContactParams->addGeometry("LHeelContact");
    leftHeelContactParams->addGeometry("PlatformContact");
    auto leftToeContactParams = new HuntCrossleyForce::ContactParameters();
    leftToeContactParams->setStiffness(2e6);
    leftToeContactParams->setDissipation(1.0);
    leftToeContactParams->setStaticFriction(0.9);
    leftToeContactParams->setDynamicFriction(0.8);
    leftToeContactParams->setViscousFriction(0.6);
    leftToeContactParams->addGeometry("LToeContact");
    leftToeContactParams->addGeometry("PlatformContact");

    // right foot Force
    rightHeelContactForce = new HuntCrossleyForce(rightHeelContactParams);
    rightHeelContactForce->setName("RightHeelContactForce");
    model.addForce(rightHeelContactForce);
    rightToeContactForce = new HuntCrossleyForce(rightToeContactParams);
    rightToeContactForce->setName("RightToeContactForce");
    model.addForce(rightToeContactForce);

    // left foot Force
    leftHeelContactForce = new HuntCrossleyForce(leftHeelContactParams);
    leftHeelContactForce->setName("LeftHeelContactForce");
    model.addForce(leftHeelContactForce);
    leftToeContactForce = new HuntCrossleyForce(leftToeContactParams);
    leftToeContactForce->setName("LeftToeContactForce");
    model.addForce(leftToeContactForce);

    // initialize system
    state = model.initSystem();
}

vector<GaitPhase>
ContactForceBasedPhaseDetector::getPhase(const GRFPrediction::Input& input) {
    updPhase(input);
    return {phaseR, phaseL};
}

void ContactForceBasedPhaseDetector::updPhase(
        const GRFPrediction::Input& input) {
    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    int i = 0;
    for (const auto& coord : coordinateSet) {
        coord->setValue(state, input.q[i]);
        coord->setSpeedValue(state, input.qDot[i]);
        ++i;
    }

    model.realizeDynamics(state);
    auto rightHeelContactWrench = rightHeelContactForce->getRecordValues(state);
    auto rightToeContactWrench = rightToeContactForce->getRecordValues(state);
    auto leftHeelContactWrench = leftHeelContactForce->getRecordValues(state);
    auto leftToeContactWrench = leftToeContactForce->getRecordValues(state);

    Vec3 rightLegForce(-rightHeelContactWrench[0] - rightToeContactWrench[0],
                       -rightHeelContactWrench[1] - rightToeContactWrench[1],
                       -rightHeelContactWrench[2] - rightToeContactWrench[2]);
    Vec3 leftLegForce(-leftHeelContactWrench[0] - leftToeContactWrench[0],
                      -leftHeelContactWrench[1] - leftToeContactWrench[1],
                      -leftHeelContactWrench[2] - leftToeContactWrench[2]);

    double threshold = 60; // todo

    // f > threshold
    auto rightContactValue = rightLegForce.norm() - threshold;
    auto leftContactValue = leftLegForce.norm() - threshold;
    // cout << rightLegForce.norm() << ", " << leftLegForce.norm() << endl;

    if (rightContactValue > 0) {
        if (phaseR == GaitPhase::Phase::SWING) {
            leadingLeg = LeadingLeg::RIGHT;
            t0 = input.t;
        }
        phaseR = GaitPhase::Phase::STANCE;
    } else {
        phaseR = GaitPhase::Phase::SWING;
    }

    if (leftContactValue > 0) {
        if (phaseL == GaitPhase::Phase::SWING) {
            leadingLeg = LeadingLeg::LEFT;
            t0 = input.t;
        }
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
//         new Station(model->getBodySet().get("calcn_r"), Vec3(0.012, -0.0015,
//         -0.005));
//     heelStationL =
//         new Station(model->getBodySet().get("calcn_l"), Vec3(0.012, -0.0015,
//         0.005));
//     toeStationR =
//         new Station(model->getBodySet().get("toes_r"), Vec3(0.06, -0.01,
//         -0.02));
//     toeStationL =
//         new Station(model->getBodySet().get("toes_l"), Vec3(0.06, -0.01,
//         0.02));

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

// vector<GaitPhase> VelocityBasedPhaseDetector::getPhase(const SimTK::State&
// state) {
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
        : model(*otherModel.clone()), Tds(-1.0) {
    phaseR = GaitPhase::Phase::INVALID;
    phaseL = GaitPhase::Phase::INVALID;

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
    Tds = 0.16; // todo
    auto Tp = Tds / 3.0;
    reactionComponentTransition = [&](const double& t) -> double {
        return exp(-pow((t / Tds / 2.0), 3));
    };
    // anteriorForceTransition = [&](const double& t) -> double {
    //     return k1 * exp(-pow((t - Tp) / Tds / 2.0, 2)) - k2 * t / Tds / 2.0;
    // };

    double A, B, K, M, m1, m2, c; // todo
    A = 0; B = 50; K = 1; M = 0.3; c = 0.02; m1 = Tds / 2.0; m2 = Tp;
    auto anteriorForceTransition = [&](const double& t) -> double {
        return A + K / (1.0 + exp(B * (t - m1))) +
                M * exp(-pow((t - m2), 2) / (2 * pow(c, 2)));
    };
}

Vector GRFPrediction::Output::asVector() {
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

vector<GRFPrediction::Output>
GRFPrediction::solve(const GRFPrediction::Input& input) {
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
        reactionForce += body.getMass() *
                         (bodyAccelerations[idx][1] - model.getGravity());

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
        const auto& gyroscopicForce = matter.getGyroscopicForce(state, idx);

        // total angular velocity-dependent force acting on this body, including
        // forces due to Coriolis acceleration and gyroscopic forces due to
        // rotational inertia.
        const auto& centrifugalForce =
                matter.getTotalCentrifugalForces(state, idx);

        // estimate of ground reaction moment
        externalMoment +=
                I * bodyAccelerations[idx][0] +
                cross(bodyVelocities[idx][0], I * bodyVelocities[idx][0]);
    }

    Vec3 rightReactionForce;
    Vec3 leftReactionForce;
    Vec3 trailingReactionForce;
    Vec3 leadingReactionForce;
    if (phase[0] == GaitPhase::Phase::STANCE &&
        phase[1] == GaitPhase::Phase::STANCE) {
        // double support
        // todo  Trasnition fucntion apply on the trailing leg
        // todo transition function input is the time duration of the DS phase
        double time = input.t - gaitPhaseDetector->t0;

        trailingReactionForce[0] =
                // reactionForce[0] * reactionComponentTransition(time);
                reactionForce[0] * anteriorForceTransition(time);
        trailingReactionForce[1] =
                reactionForce[1] * reactionComponentTransition(time);
        trailingReactionForce[2] =
                reactionForce[2] * reactionComponentTransition(time);

        leadingReactionForce[0] = reactionForce[0] - trailingReactionForce[0];
        leadingReactionForce[1] = reactionForce[1] - trailingReactionForce[1];
        leadingReactionForce[2] = reactionForce[2] - trailingReactionForce[2];



        if (gaitPhaseDetector->leadingLeg ==
                ContactForceBasedPhaseDetector::LeadingLeg::RIGHT) {
            rightReactionForce = leadingReactionForce;
            leftReactionForce = trailingReactionForce;
        } else {
            rightReactionForce = trailingReactionForce;
            leftReactionForce = leadingReactionForce;
        }

    } else if (phase[0] == GaitPhase::Phase::STANCE &&
               phase[1] == GaitPhase::Phase::SWING) {
        // right stance
        rightReactionForce = reactionForce;
        leftReactionForce = Vec3(0);

    } else if (phase[0] == GaitPhase::Phase::SWING &&
               phase[1] == GaitPhase::Phase::STANCE) {
        // left stance
        rightReactionForce = Vec3(0);
        leftReactionForce = reactionForce;

    } else {
        // invalid
        rightReactionForce = Vec3(0);
        leftReactionForce = Vec3(0);
    }

    Output rightLegOutput;
    rightLegOutput.t = input.t;
    rightLegOutput.force = rightReactionForce;
    rightLegOutput.moment = externalMoment;
    rightLegOutput.point = Vec3(0);

    Output leftLegOutput;
    leftLegOutput.t = input.t;
    leftLegOutput.force = leftReactionForce;
    leftLegOutput.moment = externalMoment;
    leftLegOutput.point = Vec3(0);

    return {rightLegOutput, leftLegOutput};
}