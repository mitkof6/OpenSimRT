#include "GRFMPrediction.h"

#include "PhaseDetector.h"
#include "Simulation.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

#define NEWTON_EULER_METHOD
// #define ID_METHOD

#define BUFFER_SIZE 10 // sliding window size for mean gait direction

// compute the projection of a point or vector on a arbitrary plane
static Vec3 projectionOnPlane(const Vec3& point, const Vec3& planeOrigin,
                              const Vec3& planeNormal) {
    return point - dot(point - planeOrigin, planeNormal) * planeNormal;
}

//==============================================================================

GRFMPrediction::GRFMPrediction(const Model& otherModel,
                               const Parameters& otherParameters)
        : model(*otherModel.clone()), parameters(otherParameters) {
    // reserve memory size for computing the mean gait direction
    gaitDirectionBuffer.setSize(BUFFER_SIZE);

    // gait phase detector
    gaitPhaseDetector = new ContactForceBasedPhaseDetector(model, parameters);

    // add station points to the model for the CoP trajectory
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

    // exponential STA by Ren et al.
    exponentialTransition = [&](const double& t) -> double {
        return exp(-pow((2.0 * t / Tds), 3));
    };

    // // Anterior GRF (f_x) sigmoid_with_bump parameters
    // auto k1 = exp(4.0 / 9.0);
    // auto k2 = k1 * exp(-16.0 / 9.0) / 2.0;
    // anteriorForceTransition = [=](const double& t) -> double {
    //     auto Tp = Tds / 3.0;
    //     return k1 * exp(-pow(2.0 * (t - Tp) / Tds, 2)) - 2.0 * k2 * t / Tds;
    // };

    // Anterior GRF (f_x) sigmoid_with_bump parameters
    anteriorForceTransition = [&](const double& t) -> double {
        double A = 0;
        double K = 1;
        double B = 25.974456748001113;
        double M = 0.7520304912335662;
        double m1 = 0.4470939749685837;
        double m2 = 0.43955467948725574;
        double c = 0.03350169202846608;
        return A + K / (1.0 + exp(B * (t - m1 * Tds))) +
               M * exp(-pow((t - m2 * Tds), 2) / (2.0 * pow(c, 2)));
    };

    // Vertical GRF (f_y) logistic parameters
    verticalForceTransition = [&](const double& t) -> double {
        double A = 1.1193681826492452;
        double K = -1.6038212670613377;
        double C = 3.037706815056258;
        double B = 61.29534891423534;
        double Q = 0.5067639996015457;
        double m = 0.9396455521564754;
        double v = 0.42306183726767493;
        return A + K / pow((C + Q * exp(-B * (t - m * Tds))), v);
    };

    // Lateral GRF (f_z) logistic parameters
    lateralForceTransition = [&](const double& t) -> double {
        double A = 1.1320519858489442;
        double K = -2.80801945666771;
        double C = 3.2237306408637867;
        double B = 55.194506993478576;
        double Q = 0.3923571148441916;
        double m = 0.7334108958330988;
        double v = 0.7682144104771099;
        return A + K / pow((C + Q * exp(-B * (t - m * Tds))), v);
    };

    // CoP trajectory (linear transition from heel -> metatarsal)
    copPosition = [&](const double& t, const Vec3& d) -> Vec3 {
        const auto omega = 2.0 * Pi / Tss;
        return -2.0 * d / (3.0 * Pi) *
               (sin(omega * t) - 1.0 / 8.0 * sin(2.0 * omega * t) -
                3.0 / 4.0 * omega * t);
    };
}

Vector GRFMPrediction::Output::asVector() {
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

vector<GRFMPrediction::Output>
GRFMPrediction::solve(const GRFMPrediction::Input& input) {
    // update state variables of this instance
    t = input.t;
    gaitPhaseDetector->updDetector(input);

    if (gaitPhaseDetector->isDetectorReady()) {
        // update model state
        updateState(input, model, state, Stage::Dynamics);

        // compute mean gait direction from pelvis
        const auto& body = model.getBodySet().get("pelvis");
        auto mob = model.getMatterSubsystem().getMobilizedBody(
                body.getMobilizedBodyIndex());
        auto X_GB = mob.getBodyTransform(state).R();
        gaitDirectionBuffer.insert((~X_GB * Vec3(1, 0, 0)).normalize());
        auto tempDirection =
                gaitDirectionBuffer.mean(); // direct passing has issues...
        auto gaitDirection =
                projectionOnPlane(tempDirection, Vec3(0), Vec3(0, 1, 0));

        // rotation about the vertical axis to transform the reaction components
        // from the opensim global reference frame to the gait-direction
        // reference frame
        auto R = SimTK::Rotation(
                atan(dot(Vec3(0, 1, 0), cross(gaitDirection, Vec3(1, 0, 0)))),
                Vec3(0, 1, 0));

        // get matter subsystem
        const auto& matter = model.getMatterSubsystem();

        // total forces / moments
        // =========================================================================
        // method 1: compute total forces/moment from pelvis using ID
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
        const auto& pelvisJoint = model.getJointSet().get("ground_pelvis");
        const auto& idx = pelvisJoint.getChildFrame().getMobilizedBodyIndex();
        auto& totalReactionForce = spatialGenForces[idx][1];
        auto& totalReactionMoment = spatialGenForces[idx][0];
        totalReactionForce = R * totalReactionForce;
        totalReactionMoment = R * totalReactionMoment;
        // totalReactionMoment[0] = 0;
        // totalReactionMoment[2] = 0;
#endif

        // =========================================================================
        // method 2: compute the reaction forces/moment based on the
        // Newton-Euler equations
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
            // - sum_j( r_ij x F_ij )) // TODO negative term is still
            // unspecified...
            const auto& I = body.getInertia();
            totalReactionMoment +=
                    I * bodyAccelerations[bix][0] +
                    cross(bodyVelocities[bix][0], I * bodyVelocities[bix][0]);
        }
        totalReactionForce = R * totalReactionForce;
        totalReactionMoment = R * totalReactionMoment;
        // totalReactionMoment[0] = 0;
        // totalReactionMoment[2] = 0;

#endif
        // =========================================================================

        // smooth transition assumption - forces
        Vec3 rightReactionForce, leftReactionForce;
        seperateReactionComponents(totalReactionForce, anteriorForceTransition,
                                   verticalForceTransition,
                                   lateralForceTransition, rightReactionForce,
                                   leftReactionForce);

        // smooth transition assumption - moments
        Vec3 rightReactionMoment, leftReactionMoment;
        seperateReactionComponents(totalReactionMoment, exponentialTransition,
                                   exponentialTransition, exponentialTransition,
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

    } else { // when detector is not ready, return zero values.
        // TODO think of something better...
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

void GRFMPrediction::seperateReactionComponents(
        const Vec3& totalReactionComponent,
        const TransitionFuction& anteriorComponentFunction,
        const TransitionFuction& verticalComponentFunction,
        const TransitionFuction& lateralComponentFunction,
        Vec3& rightReactionComponent, Vec3& leftReactionComponent) {
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        // time since last HS
        double time = t - gaitPhaseDetector->getHeelStrikeTime();

        // previous DS time-period
        Tds = gaitPhaseDetector->getDoubleSupportDuration();

        // compute the trailing and leading leg reaction components
        Vec3 trailingReactionComponent, leadingReactionComponent;

        // trailing leg component
        trailingReactionComponent[0] =
                totalReactionComponent[0] * anteriorComponentFunction(time);
        trailingReactionComponent[1] =
                totalReactionComponent[1] * verticalComponentFunction(time);
        trailingReactionComponent[2] =
                totalReactionComponent[2] * lateralComponentFunction(time);

        // leading leg component
        leadingReactionComponent =
                totalReactionComponent - trailingReactionComponent;

        // assign to output based on the current leading/trailing leg
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

    default: {
        rightReactionComponent = Vec3(0);
        leftReactionComponent = Vec3(0);
    } break;
    }
}

void GRFMPrediction::computeReactionPoint(SimTK::Vec3& rightPoint,
                                          SimTK::Vec3& leftPoint) {
    // get previous SS time-period
    Tss = gaitPhaseDetector->getSingleSupportDuration();

    // determine gait phase
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        // first determine leading / trailing leg
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT: {
            rightPoint =
                    projectionOnPlane(heelStationR->getLocationInGround(state),
                                      parameters.contact_plane_origin,
                                      parameters.contact_plane_normal);
            leftPoint =
                    projectionOnPlane(toeStationL->getLocationInGround(state),
                                      parameters.contact_plane_origin,
                                      parameters.contact_plane_normal);
        } break;
        case GaitPhaseState::LeadingLeg::LEFT: {
            rightPoint =
                    projectionOnPlane(toeStationR->getLocationInGround(state),
                                      parameters.contact_plane_origin,
                                      parameters.contact_plane_normal);
            leftPoint =
                    projectionOnPlane(heelStationL->getLocationInGround(state),
                                      parameters.contact_plane_origin,
                                      parameters.contact_plane_normal);
        } break;
        case GaitPhaseState::LeadingLeg::INVALID: {
            cerr << "CoP: invalid LeadingLeg state!" << endl;
        } break;
        }
    } break;

    case GaitPhaseState::GaitPhase::LEFT_SWING: {
        // distance between heel and toe station points on foot
        const auto d = toeStationR->getLocationInGround(state) -
                       heelStationR->getLocationInGround(state);

        // time since last toe-off event
        auto time = t - gaitPhaseDetector->getToeOffTime();

        // result CoP
        leftPoint = Vec3(0);
        rightPoint = projectionOnPlane(
                heelStationR->getLocationInGround(state) + copPosition(time, d),
                parameters.contact_plane_origin,
                parameters.contact_plane_normal);
    } break;

    case GaitPhaseState::GaitPhase::RIGHT_SWING: {
        // distance between heel and toe station points on foot
        const auto d = toeStationL->getLocationInGround(state) -
                       heelStationL->getLocationInGround(state);

        // time since last toe-off event
        auto time = t - gaitPhaseDetector->getToeOffTime();

        // result CoP
        rightPoint = Vec3(0);
        leftPoint = projectionOnPlane(heelStationL->getLocationInGround(state) +
                                              copPosition(time, d),
                                      parameters.contact_plane_origin,
                                      parameters.contact_plane_normal);
    } break;

    default: {
        rightPoint = Vec3(0.0);
        leftPoint = Vec3(0.0);
    } break;
    }
}
