/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "InverseDynamics.h"

#include "Exception.h"
#include "OpenSimUtils.h"
#include "Utils.h"

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Muscle.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

/******************************************************************************/

Vector ExternalWrench::Input::toVector() {
    Vector out(9);
    out[0] = point[0];
    out[1] = point[1];
    out[2] = point[2];
    out[3] = force[0];
    out[4] = force[1];
    out[5] = force[2];
    out[6] = torque[0];
    out[7] = torque[1];
    out[8] = torque[2];
    return out;
}

void ExternalWrench::Input::fromVector(const Vector& in) {
    point[0] = in[0];
    point[1] = in[1];
    point[2] = in[2];
    force[0] = in[3];
    force[1] = in[4];
    force[2] = in[5];
    torque[0] = in[6];
    torque[1] = in[7];
    torque[2] = in[8];
}

int ExternalWrench::Input::size() { return 9; }

ExternalWrench::ExternalWrench(const ExternalWrench::Parameters& parameters)
        : Force(), parameters(parameters) {}

ExternalWrench::Input& ExternalWrench::getInput() { return input; }

void ExternalWrench::computeForce(const State& state,
                                  Vector_<SpatialVec>& bodyForces,
                                  Vector& generalizedForces) const {
    // get references
    const auto& engine = getModel().getSimbodyEngine();
    const auto& appliedToBody =
            getModel().getBodySet().get(parameters.appliedToBody);
    const auto& forceExpressedInBody = getModel().getComponent<PhysicalFrame>(
            parameters.forceExpressedInBody);
    const auto& pointExpressedInBody = getModel().getComponent<PhysicalFrame>(
            parameters.pointExpressedInBody);

    // re-express point in applied body frame
    Vec3 point = input.point;
    point = pointExpressedInBody.findStationLocationInAnotherFrame(
            state, point, appliedToBody);

    // re-express force in ground frame
    Vec3 force = input.force;
    force = forceExpressedInBody.expressVectorInGround(state, force);

    // add-in force to the corresponding slot in bodyForces
    getModel().getMatterSubsystem().addInStationForce(
            state, appliedToBody.getMobilizedBodyIndex(), point, force,
            bodyForces);

    // re-express torque in ground frame
    Vec3 torque = input.torque;
    torque = forceExpressedInBody.expressVectorInGround(state, torque);

    // add-in torque in the corresponding slot in bodyForces
    getModel().getMatterSubsystem().addInBodyTorque(
            state, appliedToBody.getMobilizedBodyIndex(), torque, bodyForces);
}

vector<string>
ExternalWrench::createGRFLabelsFromIdentifiers(string pointIdentifier,
                                               string forceIdentifier,
                                               string torqueIdentifier) {
    if (pointIdentifier.size() == 0 || forceIdentifier.size() == 0 ||
        torqueIdentifier.size() == 0) {
        THROW_EXCEPTION("empty identifiers not allowed");
    }
    vector<string> temp;
    temp.push_back(pointIdentifier + "x");
    temp.push_back(pointIdentifier + "y");
    temp.push_back(pointIdentifier + "z");
    temp.push_back(forceIdentifier + "x");
    temp.push_back(forceIdentifier + "y");
    temp.push_back(forceIdentifier + "z");
    temp.push_back(torqueIdentifier + "x");
    temp.push_back(torqueIdentifier + "y");
    temp.push_back(torqueIdentifier + "z");
    return temp;
}

ExternalWrench::Input
ExternalWrench::getWrenchFromExternalForce(double t,
                                           const ExternalForce& force) {
    ExternalWrench::Input input;
    input.point = force.getPointAtTime(t);
    input.force = force.getForceAtTime(t);
    input.torque = force.getTorqueAtTime(t);
    return input;
}

ExternalWrench::Input
ExternalWrench::getWrenchFromStorage(double t, const vector<string>& labels,
                                     const Storage& storage) {
    if (labels.size() != 9) {
        THROW_EXCEPTION("labels dimension does not agree with ExternalWrench");
    }

    // assumes labels are pre-ordered (point, force, torque)
    auto columns = storage.getColumnLabels();
    Vector row(columns.getSize() - 1, 0.0);
    storage.getDataAtTime(t, columns.getSize() - 1, row);

    Vector collect(columns.getSize() - 1, 0.0);
    for (int i = 0; i < labels.size(); ++i) {
        int ind = columns.findIndex(labels[i]);
        if (ind < 0) {
            THROW_EXCEPTION("label: " + labels[i] +
                            " does not exist in storage");
        }
        collect[i] = row[ind - 1];
    }
    ExternalWrench::Input input;
    input.fromVector(collect);
    return input;
}

TimeSeriesTable ExternalWrench::initializeLogger() {
    vector<string> columnNames;
    columnNames.push_back("p_x");
    columnNames.push_back("p_y");
    columnNames.push_back("p_z");
    columnNames.push_back("f_x");
    columnNames.push_back("f_y");
    columnNames.push_back("f_z");
    columnNames.push_back("tau_x");
    columnNames.push_back("tau_y");
    columnNames.push_back("tau_z");

    TimeSeriesTable wrench;
    wrench.setColumnLabels(columnNames);
    return wrench;
}

/******************************************************************************/

InverseDynamics::InverseDynamics(
        const OpenSim::Model& otherModel,
        const vector<ExternalWrench::Parameters>& wrenchParameters)
        : model(*otherModel.clone()) {
    // add externally applied forces
    for (int i = 0; i < wrenchParameters.size(); ++i) {
        auto wrench = new ExternalWrench(wrenchParameters[i]);
        externalWrenches.push_back(wrench);
        model.addForce(wrench);
    }

    // disable muscles, otherwise they apply passive forces
    state = model.initSystem();
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        model.updMuscles()[i].setAppliesForce(state, false);
    }
}

InverseDynamics::Output
InverseDynamics::solve(const InverseDynamics::Input& input) {
    // update state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updU() = input.qDot;
    state.updUDot() = input.qDDot;

    // update external wrenches
    if (input.externalWrenches.size() != externalWrenches.size()) {
        THROW_EXCEPTION("input wrenches dimensionality mismatch " +
                        toString(input.externalWrenches.size()) +
                        " != " + toString(externalWrenches.size()));
    }

    for (int i = 0; i < input.externalWrenches.size(); ++i) {
        externalWrenches[i]->getInput() = input.externalWrenches[i];
    }

    // realize to dynamics stage so that all model forces are computed
    model.getMultibodySystem().realize(state, Stage::Dynamics);

    // get applied mobility (generalized) forces generated by components of the
    // model, like actuators
    const Vector& appliedMobilityForces =
            model.getMultibodySystem().getMobilityForces(state,
                                                         Stage::Dynamics);

    // get all applied body forces like those from contact
    const Vector_<SpatialVec>& appliedBodyForces =
            model.getMultibodySystem().getRigidBodyForces(state,
                                                          Stage::Dynamics);

    // perform inverse dynamics
    Output output;
    output.t = input.t;
    model.getMultibodySystem()
            .getMatterSubsystem()
            .calcResidualForceIgnoringConstraints(state, appliedMobilityForces,
                                                  appliedBodyForces,
                                                  input.qDDot, output.tau);
    return output;
}

TimeSeriesTable InverseDynamics::initializeLogger() {
    auto columnNames =
            OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);

    TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}

/*******************************************************************************/
