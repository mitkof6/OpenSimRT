#include "JointReaction.h"
#include "Exception.h"

#include <OpenSim/Simulation/Model/Actuator.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

/*******************************************************************************/

JointReaction::JointReaction(
        const OpenSim::Model& otherModel,
        const vector<ExternalWrench::Parameters>& wrenchParameters)
        : model(*otherModel.clone()) {
    // add externally applied forces
    for (int i = 0; i < wrenchParameters.size(); ++i) {
        auto wrench = new ExternalWrench(wrenchParameters[i]);
        externalWrenches.push_back(wrench);
        model.addForce(wrench);
    }

    // init state
    state = model.initSystem();
}

JointReaction::Output JointReaction::solve(const JointReaction::Input& input) {
    if (model.getActuators().getSize() != input.fm.size()) {
        THROW_EXCEPTION("actuators and provided muscle forces are of different "
                        "dimensions");
    }
    // update state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updU() = input.qDot;

    // update external wrenches
    if (input.externalWrenches.size() != externalWrenches.size()) {
        THROW_EXCEPTION("input wrenches dimensionality mismatch " +
                        to_string(input.externalWrenches.size()) +
                        " != " + to_string(externalWrenches.size()));
    }

    for (int i = 0; i < input.externalWrenches.size(); ++i) {
        externalWrenches[i]->getInput() = input.externalWrenches[i];
    }

    // update muscle forces; here we iterate over the muscles and not
    // actuators because we want to be in line with OpenSim's
    // implementation, which considers only muscle forces
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        const ScalarActuator* act =
                dynamic_cast<const ScalarActuator*>(&model.getMuscles()[i]);
        if (act) {
            act->overrideActuation(state, true);
            act->setOverrideActuation(state, input.fm[i]);
        }
    }

    // calculate all joint reaction forces and moments applied to child bodies,
    // expressed in ground frame
    int nb = model.getNumBodies();
    Output output;
    output.t = input.t;
    output.reactionWrench = Vector_<SpatialVec>(nb);
    model.getMultibodySystem().realize(state, Stage::Acceleration);
    model.getMatterSubsystem().calcMobilizerReactionForces(
            state, output.reactionWrench);

    return output;
}

SimTK::Vector
JointReaction::asForceMomentPoint(const JointReaction::Output& jrOutput) {
    const int nj = model.getJointSet().getSize();
    const auto& joints = model.getJointSet();
    const auto& ground = model.getGround();

    Vector out(nj * 9);
    for (int i = 0; i < nj; ++i) {
        auto jointReaction =
                joints[i].calcReactionOnChildExpressedInGround(state);

        // find the point of application in immediate child frame, then
        // transform to the base frame of the child (expressedInBody)
        Vec3 childLocationInGlobal =
                joints[i].getChildFrame().getTransformInGround(state).p();
        auto pointOfApplication = ground.findStationLocationInGround(
                state, childLocationInGlobal);

        // transform SpatialVec of reaction forces and moments to the
        // requested base frame (expressedInBody)
        Vec3 force = ground.expressVectorInGround(state, jointReaction[1]);
        Vec3 moment = ground.expressVectorInGround(state, jointReaction[0]);

        /* place results in the truncated loads vectors*/
        out[i * 9 + 0] = force[0];
        out[i * 9 + 1] = force[1];
        out[i * 9 + 2] = force[2];
        out[i * 9 + 3] = moment[0];
        out[i * 9 + 4] = moment[1];
        out[i * 9 + 5] = moment[2];
        out[i * 9 + 6] = pointOfApplication[0];
        out[i * 9 + 7] = pointOfApplication[1];
        out[i * 9 + 8] = pointOfApplication[2];
    }
    return out;
}

TimeSeriesTable JointReaction::initializeLogger() {
    vector<string> columnNames;
    for (int i = 0; i < model.getNumJoints(); ++i) {
        const auto& joint = model.getJointSet()[i];
        auto label = joint.getName() + "_on_" +
                     joint.getChildFrame().findBaseFrame().getName() +
                     "_in_ground";
        columnNames.push_back(label + "_fx");
        columnNames.push_back(label + "_fy");
        columnNames.push_back(label + "_fz");
        columnNames.push_back(label + "_mx");
        columnNames.push_back(label + "_my");
        columnNames.push_back(label + "_mz");
        columnNames.push_back(label + "_px");
        columnNames.push_back(label + "_py");
        columnNames.push_back(label + "_pz");
    }

    TimeSeriesTable table;
    table.setColumnLabels(columnNames);
    return table;
}

/*******************************************************************************/
