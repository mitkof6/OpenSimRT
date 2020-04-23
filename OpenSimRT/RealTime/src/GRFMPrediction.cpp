#include "GRFMPrediction.h"

#include "Simulation.h"

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define NEWTON_EULER_METHOD
// #define ID_METHOD

static Vec3 pointProjectionOnPlane(const Vec3& point, const Vec3& planeOrigin,
                                   const Vec3& planeNormal) {
    return point - dot(point - planeOrigin, planeNormal) * planeNormal;
}

template <typename T>
void updateState(const T& input, const Model& model, State& state,
                 const Stage& stage) {
    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < coordinateSet.size(); ++i) {
        coordinateSet[i]->setValue(state, input.q[i]);
        coordinateSet[i]->setSpeedValue(state, input.qDot[i]);
    }
    state.updTime() = input.t;
    model.getMultibodySystem().realize(state, stage);
}

//==============================================================================
ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel)
        : model(*otherModel.clone()), Ths(-1), Tto(-1), Tds(-1), Tss(-1) {
    phaseWindowR.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});
    phaseWindowL.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});
    leadingLeg = GaitPhaseState::LeadingLeg::INVALID;

    // platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    platform->attachGeometry(new Brick(Vec3(100, 0.0075, 100))); // todo. dont need it
    model.addBody(platform);

    // weld joint // todo couple platform to pelvis (tx,ty)
    auto platformToGround = new WeldJoint(
            "PlatformToGround", model.getGround(), Vec3(0), Vec3(0), *platform,
            Vec3(0, -0.9, 0), Vec3(0)); // todo
    model.addJoint(platformToGround);

    // double pelvis_r_range[2] = {-Pi, Pi};
    // double pelvis_t_range[2] = {-1, 2};

    // //! New coordinates cause issues with state update.
    // Coordinate& plane_rz =
    //         platformToPelvis->updCoordinate(PlanarJoint::Coord::RotationZ);
    // plane_rz.setName("plane_rz");
    // plane_rz.setRange(pelvis_r_range);
    // plane_rz.setDefaultValue(convertDegreesToRadians(0));
    // // plane_rz.setDefaultLocked(true);

    // Coordinate& plane_tx =
    //         platformToPelvis->updCoordinate(PlanarJoint::Coord::TranslationX);
    // plane_tx.setName("plane_tx");
    // plane_tx.setRange(pelvis_t_range);
    // plane_tx.setDefaultValue(0);

    // Coordinate& plane_ty =
    //         platformToPelvis->updCoordinate(PlanarJoint::Coord::TranslationY);
    // plane_ty.setName("plane_ty");
    // plane_ty.setRange(pelvis_t_range);
    // plane_ty.setDefaultValue(1);

    // contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->setLocation(Vec3(0.0, -0.0075, 0.0)); // todo
    platformContact->setOrientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setBody(*platform);
    model.addContactGeometry(platformContact);

    // contact spheres
    auto rightHeelContact = new ContactSphere();
    auto leftHeelContact = new ContactSphere();
    auto rightToeContact = new ContactSphere();
    auto leftToeContact = new ContactSphere();
    rightHeelContact->setLocation(Vec3(0.012, -0.0015, -0.005));  // todo
    leftHeelContact->setLocation(Vec3(0.012, -0.0015, 0.005));    // todo
    rightToeContact->setLocation(Vec3(0.055, 0.008, -0.01));      // todo
    leftToeContact->setLocation(Vec3(0.055, 0.008, 0.01));        // todo
    rightHeelContact->setBody(model.getBodySet().get("calcn_r")); // todo
    leftHeelContact->setBody(model.getBodySet().get("calcn_l"));  // todo
    rightToeContact->setBody(model.getBodySet().get("toes_r"));   // todo
    leftToeContact->setBody(model.getBodySet().get("toes_l"));    // todo
    rightHeelContact->setRadius(0.035);                           // todo
    leftHeelContact->setRadius(0.035);                            // todo
    rightToeContact->setRadius(0.03);                             // todo
    leftToeContact->setRadius(0.03);                              // todo
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
    rightContactForce = new OpenSim::HuntCrossleyForce(rightContactParams);
    leftContactForce = new OpenSim::HuntCrossleyForce(leftContactParams);
    rightContactForce->setName("RightContactForce");
    leftContactForce->setName("LeftContactForce");
    model.addForce(rightContactForce);
    model.addForce(leftContactForce);

    // initialize system
    state = model.initSystem();
}

void ContactForceBasedPhaseDetector::updDetector(
        const GRFPrediction::Input& input) {
    // update model state
    updateState(input, model, state, Stage::Dynamics);

    // update leg phase
    auto phaseR = updLegPhase(rightContactForce);
    auto phaseL = updLegPhase(leftContactForce);

    // push to sliding window
    phaseWindowR.insert(phaseR);
    phaseWindowL.insert(phaseL);

    // detect HS and TO events - help functions
    auto detectHS = [&](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::SWING &&
                w.data[1] == GaitPhaseState::LegPhase::STANCE)
                       ? true
                       : false;
    };

    auto detectTO = [&](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::STANCE &&
                w.data[1] == GaitPhaseState::LegPhase::SWING)
                       ? true
                       : false;
    };

    // udpate time constants
    Tto = (detectTO(phaseWindowR) || detectTO(phaseWindowL)) ? state.getTime()
                                                             : Tto;
    Ths = (detectHS(phaseWindowR) || detectHS(phaseWindowL)) ? state.getTime()
                                                             : Ths;
    Tds = (Tto > Ths && Ths > 0) ? Tto - Ths : Tds;
    Tss = (Ths > Tto && Tto > 0) ? Ths - Tto : Tss;

    // udpate leading leg
    leadingLeg = [&]() {
        if (detectHS(phaseWindowR))
            return GaitPhaseState::LeadingLeg::RIGHT;
        else if (detectHS(phaseWindowL))
            return GaitPhaseState::LeadingLeg::LEFT;
        else
            return leadingLeg; // yield previous value
    }();

    // update gait phase
    gaitPhase = updGaitPhase(phaseR, phaseL);
}

GaitPhaseState::GaitPhase ContactForceBasedPhaseDetector::updGaitPhase(
        const GaitPhaseState::LegPhase& phaseR,
        const GaitPhaseState::LegPhase& phaseL) {
    GaitPhaseState::GaitPhase phase;
    if (phaseR == GaitPhaseState::LegPhase::STANCE &&
        phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::DOUBLE_SUPPORT;

    } else if (phaseL == GaitPhaseState::LegPhase::SWING &&
               phaseR == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::LEFT_SWING;

    } else if (phaseR == GaitPhaseState::LegPhase::SWING &&
               phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::RIGHT_SWING;

    } else {
        phase = GaitPhaseState::GaitPhase::INVALID;
    }
    return phase;
}

GaitPhaseState::LegPhase ContactForceBasedPhaseDetector::updLegPhase(
        const OpenSim::HuntCrossleyForce* contact) {
    // contact force
    auto contactWrench = contact->getRecordValues(state);
    Vec3 contactForce(-contactWrench[0], -contactWrench[1], -contactWrench[2]);

    // f > threshold
    return [](const double& f) {
        if (f > 0)
            return GaitPhaseState::LegPhase::STANCE;
        else if (f <= 0)
            return GaitPhaseState::LegPhase::SWING;
        else
            return GaitPhaseState::LegPhase::INVALID;
    }(contactForce.norm() - threshold);
}

bool ContactForceBasedPhaseDetector::isDetectorReady() {
    return (Ths >= 0 && Tto >= 0 && Tds >= 0 && Tss >= 0 &&
            leadingLeg != GaitPhaseState::LeadingLeg::INVALID &&
            gaitPhase != GaitPhaseState::GaitPhase::INVALID)
                   ? true
                   : false;
}

const double ContactForceBasedPhaseDetector::getHeelStrikeTime() { return Ths; }

const double ContactForceBasedPhaseDetector::getToeOffTime() { return Tto; }

const double ContactForceBasedPhaseDetector::getDoubleSupportDuration() {
    return Tds;
}

const double ContactForceBasedPhaseDetector::getSingleSupportDuration() {
    return Tss;
}

const GaitPhaseState::LeadingLeg
ContactForceBasedPhaseDetector::getLeadingLeg() {
    return leadingLeg;
};

GaitPhaseState::GaitPhase ContactForceBasedPhaseDetector::getPhase() {
    return gaitPhase;
}

//==============================================================================
//==============================================================================

GRFPrediction::GRFPrediction(const Model& otherModel)
        : model(*otherModel.clone()) {
    // gait phase detector
    gaitPhaseDetector = new ContactForceBasedPhaseDetector(model);

    // add station point to the model
    heelStationR = new Station(model.getBodySet().get("calcn_r"),
                               Vec3(0.014, -0.0168, -0.0055));
    heelStationL = new Station(model.getBodySet().get("calcn_l"),
                               Vec3(0.014, -0.0168, 0.0055));
    toeStationR = new Station(model.getBodySet().get("calcn_r"),
                              Vec3(0.24, -0.0168, -0.00117));
    toeStationL = new Station(model.getBodySet().get("calcn_l"),
                              Vec3(0.24, -0.0168, 0.00117));
    heelStationR->setName("heel_station_point_r");
    heelStationL->setName("heel_station_point_l");
    toeStationR->setName("toe_station_point_r");
    toeStationL->setName("toe_station_point_l");
    model.addModelComponent(heelStationR);
    model.addModelComponent(heelStationL);
    model.addModelComponent(toeStationR);
    model.addModelComponent(toeStationL);

    // initialize system
    state = model.initSystem();

    // disable muscles, otherwise they apply passive forces
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        model.updMuscles()[i].setAppliesForce(state, false);
    }

    // define transition functions for reaction components
    auto k1 = exp(4.0 / 9.0);
    auto k2 = k1 * exp(-16.0 / 9.0) / 2.0;
    reactionComponentTransition = [&](const double& t) -> double {
        return exp(-pow((2.0 * t / Tds), 3));
    };
    anteriorForceTransition = [&](const double& t) -> double {
        auto Tp = Tds / 3.0;
        return k1 * exp(-pow(2.0 * (t - Tp) / Tds, 2)) - 2.0 * k2 * t / Tds;
    };

    // todo. use sigmoid with "bump"
    // double A, B, K, M, m1, m2, c;
    // A = 0; B = 50; K = 1; M = 0.3; c = 0.02; m1 = Tds; m2 = Tp;
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
    t = input.t;
    gaitPhaseDetector->updDetector(input);

    if (gaitPhaseDetector->isDetectorReady()) {
        // update model state
        updateState(input, model, state, Stage::Dynamics);

        // get matter subsystem
        const auto& matter = model.getMatterSubsystem();

        // total forces / moments
        // =========================================================================
        // method 1:
        // =========================================================================
#ifdef ID_METHOD
        // get applied mobility (generalized) forces generated by components of
        // the model, like actuators
        const Vector& appliedMobilityForces =
                model.getMultibodySystem().getMobilityForces(state,
                                                             Stage::Dynamics);

        // get all applied body forces like those from contact
        const Vector_<SpatialVec>& appliedBodyForces =
                model.getMultibodySystem().getRigidBodyForces(state,
                                                              Stage::Dynamics);

        // perform inverse dynamics
        Vector tau;
        model.getMultibodySystem()
                .getMatterSubsystem()
                .calcResidualForceIgnoringConstraints(
                        state, appliedMobilityForces, appliedBodyForces,
                        input.qDDot, tau);

        //==========================================================================
        // spatial forces/moments in pelvis wrt the ground
        Vector_<SpatialVec> spatialGenForces;
        matter.multiplyBySystemJacobian(state, tau, spatialGenForces);
        const auto& pelvisJoint =
                model.getJointSet().get("ground_pelvis"); // todo
        const auto& idx = pelvisJoint.getChildFrame().getMobilizedBodyIndex();
        const auto& totalReactionForce = spatialGenForces[idx][1];
        const auto& totalReactionMoment = spatialGenForces[idx][0];
#endif

        // =========================================================================
        // method 2:
        //==========================================================================
#ifdef NEWTON_EULER_METHOD
        // compute body velocities and accelerations
        SimTK::Vector_<SimTK::SpatialVec> bodyVelocities;
        SimTK::Vector_<SimTK::SpatialVec> bodyAccelerations;
        matter.multiplyBySystemJacobian(state, input.qDot, bodyVelocities);
        matter.calcBodyAccelerationFromUDot(state, input.qDDot,
                                            bodyAccelerations);

        // compute total force / moments
        Vec3 totalReactionForce, totalReactionMoment;
        for (int i = 0; i < model.getNumBodies(); ++i) {
            const auto& body = model.getBodySet()[i];
            const auto& bix = body.getMobilizedBodyIndex();

            // F_ext = sum(m_i * (a_i - g) )
            totalReactionForce += body.getMass() * (bodyAccelerations[bix][1] -
                                                    model.getGravity());

            // M_ext = sum( I_i * omega_dot + omega x (I_i * omega)
            // - sum_j( r_ij x F_ij )) // todo
            const auto& I = body.getInertia();
            totalReactionMoment +=
                    I * bodyAccelerations[bix][0] +
                    cross(bodyVelocities[bix][0], I * bodyVelocities[bix][0]);
        }
#endif
        // =========================================================================

        // smooth transition assumption - forces
        Vec3 rightReactionForce, leftReactionForce;
        seperateReactionComponents(totalReactionForce,
                                   reactionComponentTransition, // todo
                                   reactionComponentTransition,
                                   rightReactionForce, leftReactionForce);

        // smooth transition assumption - moments
        Vec3 rightReactionMoment, leftReactionMoment;
        seperateReactionComponents(totalReactionMoment,
                                   reactionComponentTransition,
                                   reactionComponentTransition,
                                   rightReactionMoment, leftReactionMoment);
        // point
        Vec3 rightPoint, leftPoint;
        computeReactionPoint(rightPoint, leftPoint);

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
    } else {
        Output rightLegOutput;
        rightLegOutput.t = input.t;
        rightLegOutput.force = Vec3(0.0);
        rightLegOutput.moment = Vec3(0.0);
        rightLegOutput.point = Vec3(0.0);

        Output leftLegOutput;
        leftLegOutput.t = input.t;
        leftLegOutput.force = Vec3(0.0);
        leftLegOutput.moment = Vec3(0.0);
        leftLegOutput.point = Vec3(0.0);

        return {rightLegOutput, leftLegOutput};
    }
}

void GRFPrediction::seperateReactionComponents(
        const Vec3& totalReactionComponent,
        const TransitionFuction& anteriorComponentFunction,
        const TransitionFuction& reactionComponentFunction,
        Vec3& rightReactionComponent, Vec3& leftReactionComponent) {
    Vec3 trailingReactionComponent, leadingReactionComponent;
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        double time = t - gaitPhaseDetector->getHeelStrikeTime();
        Tds = gaitPhaseDetector->getDoubleSupportDuration();

        // trailing leg component
        trailingReactionComponent[0] =
                totalReactionComponent[0] * anteriorComponentFunction(time);
        trailingReactionComponent[1] =
                totalReactionComponent[1] * reactionComponentFunction(time);
        trailingReactionComponent[2] =
                totalReactionComponent[2] * reactionComponentFunction(time);

        // leading leg component
        leadingReactionComponent =
                totalReactionComponent - trailingReactionComponent;

        // assign to output
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT: {
            rightReactionComponent = leadingReactionComponent;
            leftReactionComponent = trailingReactionComponent;
        } break;
        case GaitPhaseState::LeadingLeg::LEFT: {
            rightReactionComponent = trailingReactionComponent;
            leftReactionComponent = leadingReactionComponent;
        } break;
        case GaitPhaseState::LeadingLeg::INVALID: {
            cerr << "STA: invalid LeadingLeg state!" << endl;
        } break;
        }
    } break;

    case GaitPhaseState::GaitPhase::LEFT_SWING: {
        rightReactionComponent = totalReactionComponent;
        leftReactionComponent = Vec3(0);
    } break;

    case GaitPhaseState::GaitPhase::RIGHT_SWING: {
        rightReactionComponent = Vec3(0);
        leftReactionComponent = totalReactionComponent;
    } break;

    default: { // Look mom, I'm flying!
        rightReactionComponent = Vec3(0);
        leftReactionComponent = Vec3(0);
    } break;
    }
}

void GRFPrediction::computeReactionPoint(SimTK::Vec3& rightPoint,
                                         SimTK::Vec3& leftPoint) {
    // get Tss
    Tss = gaitPhaseDetector->getSingleSupportDuration();

    // CoP trajectory (linear transition from heel -> metatarsal)
    auto copPosition = [&](const double& t, const Vec3& d) -> Vec3 {
        const auto omega = 2.0 * Pi / Tss;
        return -2.0 * d / (3.0 * Pi) *
               (sin(omega * t) - 1.0 / 8.0 * sin(2.0 * omega * t) -
                3.0 / 4.0 * omega * t);
    };

    // determine gait phase
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        // determine leading / trailing leg
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT: {
            rightPoint = heelStationR->getLocationInGround(state);
            leftPoint = toeStationL->getLocationInGround(state);
        } break;
        case GaitPhaseState::LeadingLeg::LEFT: {
            rightPoint = toeStationR->getLocationInGround(state);
            leftPoint = heelStationL->getLocationInGround(state);
        } break;
        case GaitPhaseState::LeadingLeg::INVALID: {
            cerr << "CoP: invalid LeadingLeg state!" << endl;
        } break;
        }
    } break;

    case GaitPhaseState::GaitPhase::LEFT_SWING: {
        const auto d = toeStationR->getLocationInGround(state) -
                       heelStationR->getLocationInGround(state);
        auto time = t - gaitPhaseDetector->getToeOffTime();

        leftPoint = toeStationL->getLocationInGround(state);
        rightPoint =
                heelStationR->getLocationInGround(state) + copPosition(time, d);
    } break;

    case GaitPhaseState::GaitPhase::RIGHT_SWING: {
        const auto d = toeStationL->getLocationInGround(state) -
                       heelStationL->getLocationInGround(state);
        auto time = t - gaitPhaseDetector->getToeOffTime();

        rightPoint = toeStationR->getLocationInGround(state);
        leftPoint =
                heelStationL->getLocationInGround(state) + copPosition(time, d);
    } break;

    default: {
        rightPoint = Vec3(0.0);
        leftPoint = Vec3(0.0);
    } break;
    }
}