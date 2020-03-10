#include "GRFMPrediction.h"
#include "Simulation.h"
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
GaitPhaseDetector::GaitPhaseDetector(Model* otherModel, const Parameters& otherParameters) :
    model(otherModel), parameters(otherParameters) {
    phaseR = GaitPhase::Phase::INVALID;
    phaseL = GaitPhase::Phase::INVALID;

    // add station point to the model
    heelStationR =
        new Station(model->getBodySet().get("calcn_r"), Vec3(0.01, 0.02, -0.005));
    heelStationL =
        new Station(model->getBodySet().get("calcn_l"), Vec3(0.01, 0.02, 0.005));
    toeStationR =
        new Station(model->getBodySet().get("toes_r"), Vec3(0.04, 0.02, 0.014));
    toeStationL =
        new Station(model->getBodySet().get("toes_l"), Vec3(0.04, 0.02, -0.014));

    const auto& pelvisBody = model->getBodySet().get("pelvis");
    pelvisStation = new Station(pelvisBody, pelvisBody.getMassCenter());

    heelStationR->setName("heel_station_point_r");
    heelStationL->setName("heel_station_point_l");
    toeStationR->setName("toe_station_point_r");
    toeStationL->setName("toe_station_point_l");
    pelvisStation->setName("pelvis_station_point");

    model->addModelComponent(heelStationR);
    model->addModelComponent(heelStationL);
    model->addModelComponent(toeStationR);
    model->addModelComponent(toeStationL);
    model->addModelComponent(pelvisStation);
}

vector<GaitPhase> GaitPhaseDetector::getPhase(const SimTK::State& state){
    updPhase(state);
    return {phaseR, phaseL};
}
void GaitPhaseDetector::updPhase(const SimTK::State& state) {
    model->realizeAcceleration(state);
    const auto& rHeelVel = heelStationR->getVelocityInGround(state).norm();
    const auto& lHeelVel = heelStationL->getVelocityInGround(state).norm();
    const auto& rToeVel = toeStationR->getVelocityInGround(state).norm();
    const auto& lToeVel = toeStationL->getVelocityInGround(state).norm();
    const auto& pelvisVel = pelvisStation->getVelocityInGround(state).norm();

    double threshold = 0.25; // todo
    // cout << rHeelVel << ", " << lHeelVel << ", "
    //      << rToeVel << ", " << lToeVel << ", " << pelvisVel << endl;

    // check for invalid phase
    if (phaseR == GaitPhase::Phase::INVALID) {
        if (rHeelVel < 0.6 * threshold) {
            phaseR = GaitPhase::Phase::STANCE;
        } else if (rToeVel > 1.9 * threshold) {
            phaseR = GaitPhase::Phase::SWING;
        }
    }

    if (phaseL == GaitPhase::Phase::INVALID) {
        if (lHeelVel < 0.6 * threshold) {
            phaseL = GaitPhase::Phase::STANCE;
        } else if (lToeVel > 1.9 * threshold) {
            phaseL = GaitPhase::Phase::SWING;
        }
    }

    // update phase
    if (phaseR == GaitPhase::Phase::SWING &&
        rHeelVel < 0.6 * threshold) {
        phaseR = GaitPhase::Phase::STANCE;

    } else if (phaseR == GaitPhase::Phase::STANCE &&
               rToeVel > 1.9 * threshold) {
        phaseR = GaitPhase::Phase::SWING;
    }

    if (phaseL == GaitPhase::Phase::SWING &&
        lHeelVel < 0.6 * threshold) {
        phaseL = GaitPhase::Phase::STANCE;

    } else if (phaseL == GaitPhase::Phase::STANCE &&
               lToeVel > 1.9 * threshold) {
        phaseL = GaitPhase::Phase::SWING;
    }
}


//==============================================================================

GRFPrediction::GRFPrediction(const Model& otherModel,
     const GaitPhaseDetector::Parameters& parameters)
    : model(*otherModel.clone()) {

    // leg phase detector
    gaitPhaseDetector = new GaitPhaseDetector(&model, parameters);

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
    // get reference to matter subsystem
    const auto& matter = model.getMatterSubsystem();

    // // update state
    // state.updTime() = input.t;
    // state.updQ() = input.q;
    // state.updQDot() = input.qDot;
    // state.updQDotDot() = input.qDDot;

    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    int i = 0;
    for (const auto& coord : coordinateSet){
        coord->setValue(state, input.q[i]);
        coord->setSpeedValue(state, input.qDot[i]);
        ++i;
    }

    auto phase = gaitPhaseDetector->getPhase(state);
    cout << "Right: " << phase[0].toString() << " "
        << "Left: " << phase[1].toString() << endl;

    // // realize to dynamics stage so that all model velocities, accelerations and
    // // forces are computed
    // model.getMultibodySystem().realize(state, Stage::Dynamics);

    // // compute body velocities and accelerations
    // matter.multiplyBySystemJacobian(state, input.qDot, bodyVelocities);
    // matter.calcBodyAccelerationFromUDot(state, input.qDDot, bodyAccelerations);

    // // get the vector of spatial forces (meaning the gravitational moment about
    // // and force at the body origin, \e not necessarily the center of mass)
    // // currently being applied to each of the mobilized bodies.
    // const auto& gravityForces = model.getGravityForce().getBodyForces(state);

    // // get applied mobility (generalized) forces generated by components of the
    // // model, like actuators
    // const Vector& appliedMobilityForces =
    //         model.getMultibodySystem().getMobilityForces(state,
    //                                                      Stage::Dynamics);

    // // get all applied body forces [m, f]^T
    // const Vector_<SpatialVec>& appliedBodyForces =
    //         model.getMultibodySystem().getRigidBodyForces(state,
    //                                                       Stage::Dynamics);

    // // reaction force is the sum of forces applied due to body accelerations
    // Vec3 reactionForce, externalMoment;
    // for (int i = 0; i < model.getNumBodies(); ++i) {
    //     // get body segment
    //     const auto& body = model.getBodySet()[i];
    //     const auto& idx = body.getMobilizedBodyIndex();
    //     if (body.getName() == "ground") continue;

    //     // calculate F_ext = sum(m_i * (a_i - g) ) //* Yes, the sign is minus(-)
    //     reactionForce +=
    //             body.getMass() * (bodyAccelerations[idx][1] - model.getGravity());

    //     // calculate M_ext = sum( I_i * omega_dot + omega x (I_i * omega) -
    //     // sum_j( r_ij x F_ij )), where i: each body, j: endpoint of the ith
    //     // body. The negative term is the total torque applied to the body's
    //     // com.
    //     // const auto& I = matter.getArticulatedBodyInertia(
    //     //                               state, idx)
    //     //                         .getInertia();
    //     const auto& I = body.getInertia();

    //     // angular velocity-dependent force on the body due to rotational
    //     // inertia
    //     const auto& gyroscopicForce =
    //             matter.getGyroscopicForce(state, idx);

    //     // total angular velocity-dependent force acting on this body, including
    //     // forces due to Coriolis acceleration and gyroscopic forces due to
    //     // rotational inertia.
    //     const auto& centrifugalForce = matter.getTotalCentrifugalForces(
    //             state, idx);

    //     // estimate of ground reaction moment
    //     externalMoment += I * bodyAccelerations[idx][0] +
    //                       cross(bodyVelocities[idx][0], I * bodyVelocities[idx][0]);
    // }

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