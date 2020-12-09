#include "JointReaction.h"
#include "OpenSimUtils.h"
#include "Exception.h"
#include "Utils.h"
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
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

    // permit overriding of actuator force
    state = model.initSystem();
}

JointReaction::Output JointReaction::solve(const JointReaction::Input& input) {
    if (model.getActuators().getSize() != input.fm.size()) {
	THROW_EXCEPTION("actuators and provided muscle forces are of different dimensions");
    }
    // update state
    state.updTime() = input.t;
    state.updQ() = input.q;
    state.updU() = input.qDot;

    // update external wrenches
    if (input.externalWrenches.size() != externalWrenches.size()) {
        THROW_EXCEPTION("input wrenches dimensionality mismatch " +
                        toString(input.externalWrenches.size()) + " != " +
                        toString(externalWrenches.size()));
    }

    for (int i = 0; i < input.externalWrenches.size(); ++i) {
        externalWrenches[i]->getInput() = input.externalWrenches[i];
    }

    // update muscle forces
    const auto& fs = model.getForceSet();
    for(int i = 0, j = 0; i < fs.getSize(); i++)  {
        auto* act = dynamic_cast<ScalarActuator*>(&fs.get(i));
         if(act) {
             act->overrideActuation(state, true);
             act->setOverrideActuation(state, input.fm[i]);
             j++;
         }
    }

    // calculate all joint reaction forces and moments applied to child bodies,
    // expressed in ground frame
    int nb = model.getNumBodies();
    Output output;
    output.t = input.t;
    output.reactionWrench = Vector_<SpatialVec>(nb);
    model.getMultibodySystem().realize(state, Stage::Acceleration);
    model.getMatterSubsystem().calcMobilizerReactionForces(state, output.reactionWrench);

    return output;
}

TimeSeriesTable JointReaction::initializeLogger() {
    vector<string> jrColumnNames;
    for (int i = 0; i < model.getNumJoints(); ++i) {
        const auto& joint = model.getJointSet()[i];
        auto bodyName = joint.getChildFrame().findBaseFrame().getName();
        // auto bodyName =  joint.getChildFrame().getName();
        // string toReplace = "_offset";
        // size_t pos = bodyName.find(toReplace);
        // if (pos != string::npos)
        //     bodyName = bodyName.replace(pos, toReplace.length(), "");

        auto label = joint.getName() + "_on_" + bodyName + "_in_ground";
        jrColumnNames.push_back(label + "_fx");
        jrColumnNames.push_back(label + "_fy");
        jrColumnNames.push_back(label + "_fz");
        jrColumnNames.push_back(label + "_mx");
        jrColumnNames.push_back(label + "_my");
        jrColumnNames.push_back(label + "_mz");
    }

    TimeSeriesTable q;
    q.setColumnLabels(jrColumnNames);
    return q;
}

SimTK::Vector JointReaction::convertOutput(const JointReaction::Output& output) {
    Vector temp((output.reactionWrench.size() - 1) * 6, 0.0);
    for (int i = 1; i < output.reactionWrench.size(); ++i) {
        temp[(i - 1) * 6 + 0] = output.reactionWrench[i](1)[0];
        temp[(i - 1) * 6 + 1] = output.reactionWrench[i](1)[1];
        temp[(i - 1) * 6 + 2] = output.reactionWrench[i](1)[2];
        temp[(i - 1) * 6 + 3] = output.reactionWrench[i](0)[0];
        temp[(i - 1) * 6 + 4] = output.reactionWrench[i](0)[1];
        temp[(i - 1) * 6 + 5] = output.reactionWrench[i](0)[2];
    }
    return temp;
}

/*******************************************************************************/
