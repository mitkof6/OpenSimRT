#include "GRFMPrediction.h"

#include "Simulation.h"

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel)
        : model(*otherModel.clone()) {
    phaseR = GaitPhaseState::LegPhase::INVALID;
    phaseL = GaitPhaseState::LegPhase::INVALID;
    leadingLeg = GaitPhaseState::LeadingLeg::LEFT; // todo

    // platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    platform->attachGeometry(new Brick(Vec3(1, 0.0075, 1))); // todo
    model.addBody(platform);

    // weld joint
    auto platformToGround = new WeldJoint(
            "PlatformToGround", model.getGround(), Vec3(0), Vec3(0), *platform,
            Vec3(-0.6, -0.035, 0), Vec3(0)); // todo
    model.addJoint(platformToGround);

    // contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->setLocation(Vec3(0.0, -0.0075, 0.0)); // todo
    platformContact->setOrientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setBody(*platform);
    model.addContactGeometry(platformContact);

    // contact spheres
    double heelSphereRadius = 0.035; // todo
    double toeSphereRadius = 0.03;   // todo
    auto rightHeelContact = new ContactSphere();
    auto leftHeelContact = new ContactSphere();
    auto rightToeContact = new ContactSphere();
    auto leftToeContact = new ContactSphere();
    rightHeelContact->setLocation(Vec3(0.012, -0.0015, -0.005)); // todo
    leftHeelContact->setLocation(Vec3(0.012, -0.0015, 0.005));   // todo
    rightToeContact->setLocation(Vec3(0.055, 0.008, -0.01));     // todo
    leftToeContact->setLocation(Vec3(0.055, 0.008, 0.01));       // todo
    rightHeelContact->setBody(model.getBodySet().get("calcn_r"));
    leftHeelContact->setBody(model.getBodySet().get("calcn_l"));
    rightToeContact->setBody(model.getBodySet().get("toes_r"));
    leftToeContact->setBody(model.getBodySet().get("toes_l"));
    rightHeelContact->setRadius(heelSphereRadius);
    leftHeelContact->setRadius(heelSphereRadius);
    rightToeContact->setRadius(toeSphereRadius);
    leftToeContact->setRadius(toeSphereRadius);
    rightHeelContact->setName("RHeelContact");
    leftHeelContact->setName("LHeelContact");
    rightToeContact->setName("RToeContact");
    leftToeContact->setName("LToeContact");
    model.addContactGeometry(rightHeelContact);
    model.addContactGeometry(leftHeelContact);
    model.addContactGeometry(rightToeContact);
    model.addContactGeometry(leftToeContact);

    // contact parameters // todo
    double stiffness = 2e6;
    double dissipation = 1.0;
    double staticFriction = 0.9;
    double dynamicFriction = 0.8;
    double viscousFriction = 0.6;

    auto rightContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);
    auto leftContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);

    rightContactParams->addGeometry("PlatformContact");
    rightContactParams->addGeometry("RHeelContact");
    rightContactParams->addGeometry("RToeContact");
    leftContactParams->addGeometry("PlatformContact");
    leftContactParams->addGeometry("LHeelContact");
    leftContactParams->addGeometry("LToeContact");

    // contact forces
    rightHeelContactForce = new OpenSim::HuntCrossleyForce(rightContactParams);
    leftHeelContactForce = new OpenSim::HuntCrossleyForce(leftContactParams);
    rightHeelContactForce->setName("RightHeelContactForce");
    leftHeelContactForce->setName("LeftHeelContactForce");
    model.addForce(rightHeelContactForce);
    model.addForce(leftHeelContactForce);

    // initialize system
    state = model.initSystem();
}

GaitPhaseState::GaitPhase
ContactForceBasedPhaseDetector::getPhase(const GRFPrediction::Input& input) {
    // update model state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updQDot() = input.qDot;
    state.updQDotDot() = input.qDDot;
    model.realizeDynamics(state);

    // update phase
    updLegPhase();
    updGaitPhase();
    return gaitPhase;
}

void ContactForceBasedPhaseDetector::updGaitPhase() {
    if (phaseR == GaitPhaseState::LegPhase::STANCE &&
        phaseL == GaitPhaseState::LegPhase::STANCE) {
        gaitPhase = GaitPhaseState::GaitPhase::DOUBLE_SUPPORT;

    } else if (phaseL == GaitPhaseState::LegPhase::SWING &&
               phaseR == GaitPhaseState::LegPhase::STANCE) {
        gaitPhase = GaitPhaseState::GaitPhase::LEFT_SWING;

    } else if (phaseR == GaitPhaseState::LegPhase::SWING &&
               phaseL == GaitPhaseState::LegPhase::STANCE) {
        gaitPhase = GaitPhaseState::GaitPhase::RIGHT_SWING;

    } else {
        gaitPhase = GaitPhaseState::GaitPhase::INVALID;
    }
}
void ContactForceBasedPhaseDetector::updLegPhase() {
    // get contact force values
    auto rightContactWrench = rightHeelContactForce->getRecordValues(state);
    auto leftContactWrench = leftHeelContactForce->getRecordValues(state);

    // right/left leg total contact force
    Vec3 rightLegForce(-rightContactWrench[0],
                       -rightContactWrench[1],
                       -rightContactWrench[2]);
    Vec3 leftLegForce(-leftContactWrench[0],
                      -leftContactWrench[1],
                      -leftContactWrench[2]);

    // f > threshold
    double threshold = 25; // todo
    auto rightContactValue = rightLegForce.norm() - threshold;
    auto leftContactValue = leftLegForce.norm() - threshold;

    // monitor and update right leg phase
    if (rightContactValue > 0) {
        detectRightHeelStrike();
        phaseR = GaitPhaseState::LegPhase::STANCE;
    } else {
        phaseR = GaitPhaseState::LegPhase::SWING;
    }

    // monitor and update left leg phase
    if (leftContactValue > 0) {
        detectLeftHeelStrike();
        phaseL = GaitPhaseState::LegPhase::STANCE;
    } else {
        phaseL = GaitPhaseState::LegPhase::SWING;
    }
}

const GaitPhaseState::LeadingLeg
ContactForceBasedPhaseDetector::getLeadingLeg() {
    return leadingLeg;
};

const double ContactForceBasedPhaseDetector::getHeelStrikeTime() { return Ths; }

void ContactForceBasedPhaseDetector::detectRightHeelStrike() {
    if (phaseR == GaitPhaseState::LegPhase::SWING) {
        leadingLeg = GaitPhaseState::LeadingLeg::RIGHT;
        Ths = state.getTime();
    }
}

void ContactForceBasedPhaseDetector::detectLeftHeelStrike() {
    if (phaseL == GaitPhaseState::LegPhase::SWING) {
        leadingLeg = GaitPhaseState::LeadingLeg::LEFT;
        Ths = state.getTime();
    }
}
//==============================================================================

GRFPrediction::GRFPrediction(const Model& otherModel)
        : model(*otherModel.clone()), Tds(-1.0) {
    // gait phase detector
    gaitPhaseDetector = new ContactForceBasedPhaseDetector(model);

    // disable muscles, otherwise they apply passive forces
    state = model.initSystem();
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        model.updMuscles()[i].setAppliesForce(state, false);
    }

    // define transition functions for reaction components
    Tds = 0.08; // todo
    auto Tp = 2.0 * Tds / 3.0;
    auto k1 = exp(4.0 / 9.0);
    auto k2 = k1 * exp(-16.0 / 9.0) / 2.0;
    reactionComponentTransition = [&](const double& t) -> double {
        return exp(-pow((t / Tds), 3));
    };
    anteriorForceTransition = [&](const double& t) -> double {
        return k1 * exp(-pow((t - Tp) / Tds, 2)) - k2 * t / Tds;
    };

    // double A, B, K, M, m1, m2, c; // todo
    // A = 0; B = 50; K = 1; M = 0.3; c = 0.02; m1 = Tds / 2.0; m2 = Tp;
    // A = 0; K = 1; B = 27.96243188;
    // M = -0.04883987; c = 0.02999439; m1 = 0.11871468; m2 = 0.21028451;
    // anteriorForceTransition = [&](const double& t) -> double {
    //     return A + K / (1.0 + exp(B * (t - m1))) +
    //             M * exp(-pow((t - m2), 2) / (2.0 * pow(c, 2)));
    // };
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
    // update class state variables
    currentTime = input.t;
    gaitphase = gaitPhaseDetector->getPhase(input);

    // update model state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updQDot() = input.qDot;
    state.updQDotDot() = input.qDDot;
    model.realizeVelocity(state);

    // compute body velocities and accelerations
    const auto& matter = model.getMatterSubsystem();
    matter.multiplyBySystemJacobian(state, input.qDot, bodyVelocities);
    matter.calcBodyAccelerationFromUDot(state, input.qDDot, bodyAccelerations);

    // force
    Vec3 rightReactionForce, leftReactionForce;
    computeReactionForces(rightReactionForce, leftReactionForce);

    // moment
    Vec3 rightReactionMoment, leftReactionMoment;
    computeReactionMoments(rightReactionForce, leftReactionForce,
                           rightReactionMoment, leftReactionMoment);

    // point
    Vec3 rightPoint, leftPoint;
    computeReactionPoint(rightReactionForce, leftReactionForce,
                         rightReactionMoment, leftReactionMoment, rightPoint,
                         leftPoint);

    // results
    Output rightLegOutput;
    rightLegOutput.t = input.t;
    rightLegOutput.force = rightReactionForce;
    rightLegOutput.moment = rightReactionMoment;
    rightLegOutput.point = rightPoint;

    Output leftLegOutput;
    leftLegOutput.t = input.t;
    leftLegOutput.force = leftReactionForce;
    leftLegOutput.moment = leftReactionMoment;
    leftLegOutput.point = leftPoint;

    return {rightLegOutput, leftLegOutput};
}

void GRFPrediction::computeReactionForces(Vec3& rightReactionForce,
                                          Vec3& leftReactionForce) {
    // compute total reaction force as the sum of forces applied to each body
    // com due to linear accelerations
    Vec3 reactionForce;
    for (int i = 0; i < model.getNumBodies(); ++i) {
        // get body segment
        const auto& body = model.getBodySet()[i];
        const auto& idx = body.getMobilizedBodyIndex();

        // F_ext = sum(m_i * (a_i - g) )
        reactionForce += body.getMass() *
                         (bodyAccelerations[idx][1] - model.getGravity());
    }

    Vec3 trailingReactionForce, leadingReactionForce;
    if (gaitphase == GaitPhaseState::GaitPhase::DOUBLE_SUPPORT) {
        // double support
        double time = currentTime - gaitPhaseDetector->getHeelStrikeTime();

        // trailing leg force
        trailingReactionForce[0] =
                reactionForce[0] * reactionComponentTransition(time); // todo
                // reactionForce[0] * anteriorForceTransition(time);
        trailingReactionForce[1] =
                reactionForce[1] * reactionComponentTransition(time);
        trailingReactionForce[2] =
                reactionForce[2] * reactionComponentTransition(time);

        // leading leg force
        leadingReactionForce = reactionForce - trailingReactionForce;

        // assign forces to output
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT:
            rightReactionForce = leadingReactionForce;
            leftReactionForce = trailingReactionForce;
            break;
        case GaitPhaseState::LeadingLeg::LEFT:
            rightReactionForce = trailingReactionForce;
            leftReactionForce = leadingReactionForce;
            break;
        default: cerr << "invalid LeadingLeg state!" << endl; break;
        }

    } else if (gaitphase == GaitPhaseState::GaitPhase::LEFT_SWING) {
        // right stance
        rightReactionForce = reactionForce;
        leftReactionForce = Vec3(0);

    } else if (gaitphase == GaitPhaseState::GaitPhase::RIGHT_SWING) {
        // left stance
        rightReactionForce = Vec3(0);
        leftReactionForce = reactionForce;

    } else {
        // invalid
        rightReactionForce = Vec3(0);
        leftReactionForce = Vec3(0);
    }
}
/**
 * @brief calculate M_ext = sum( I_i * omega_dot + omega x (I_i * omega) -
    sum_j( r_ij x F_ij )), where i: each body, j: endpoint of the ith
    body. The negative term is the total torque applied to the body's
    com.
 *
 * @param rightReactionMoment
 * @param leftReactionMoment
 */
void GRFPrediction::computeReactionMoments(const Vec3& rightReactionForce,
                                           const Vec3& leftReactionForce,
                                           Vec3& rightReactionMoment,
                                           Vec3& leftReactionMoment) {
    Vec3 externalMoment;
    for (int i = 0; i < model.getNumBodies(); ++i) {
        // get body segment
        const auto& body = model.getBodySet()[i];
        const auto& idx = body.getMobilizedBodyIndex();

        // const auto& gravityForces =
        // model.getGravityForce().getBodyForces(state); const
        // Vector_<SpatialVec>& appliedBodyForces =
        //         model.getMultibodySystem().getRigidBodyForces(state,
        //                                                       Stage::Dynamics);
        // const auto& gyroscopicForce = matter.getGyroscopicForce(state, idx);
        // const auto& centrifugalForce =
        //         matter.getTotalCentrifugalForces(state, idx);

        // estimate of ground reaction moment
        const auto& I = body.getInertia();
        externalMoment +=
                I * bodyAccelerations[idx][0] +
                cross(bodyVelocities[idx][0], I * bodyVelocities[idx][0]);
    }
    rightReactionMoment = Vec3(0); // todo
    leftReactionMoment = Vec3(0);  // todo
}

void GRFPrediction::computeReactionPoint(const Vec3& rightReactionForce,
                                         const Vec3& leftReactionForce,
                                         const Vec3& rightReactionMoment,
                                         const Vec3& leftReactionMoment,
                                         SimTK::Vec3& rightPoint,
                                         SimTK::Vec3& leftPoint) {
    rightPoint = Vec3(0); // todo;
    leftPoint = Vec3(0);  // todo;
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