#include "JointReaction.h"
#include "InverseDynamics.h"
#include "SimulationUtils.h"
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Actuator.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

/*******************************************************************************/

JointReaction::JointReaction(string modelFile,
			     const vector<ExternalWrench::Parameters>& wrenchParameters)
    : model(modelFile) {
    // add externally applied forces
      for (int i = 0; i < wrenchParameters.size(); ++i) {
        auto wrench = new ExternalWrench(wrenchParameters[i]);
        externalWrenches.push_back(wrench);
        model.addForce(wrench);
    }

    // permit overriding of actuator force
    state = model.initSystem();

    // logger
    vector<string> jrColumnNames;
    jrColumnNames.push_back("time");
    for (int i = 0; i < model.getNumJoints(); ++i) {
        const auto& joint = model.getJointSet()[i];
        auto label = joint.getName() + "_on_" + joint.getChildFrame().getName() + "_in_ground";
        jrColumnNames.push_back(label + "_fx");
        jrColumnNames.push_back(label + "_fy");
        jrColumnNames.push_back(label + "_fz");
        jrColumnNames.push_back(label + "_mx");
        jrColumnNames.push_back(label + "_my");
        jrColumnNames.push_back(label + "_mz");
    }
    logger = new CSVLogger(jrColumnNames);
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
        externalWrenches[i]->input = input.externalWrenches[i];
    }
    
    // update muscle forces
    const auto& fs = model.getForceSet();
    for(int i = 0, j = 0; i < fs.getSize(); i++)  {
        auto* act = dynamic_cast<ScalarActuator*>(&fs.get(i));
         if(act) {
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

    // log
    vector<double> jrTemp;
    jrTemp.push_back(output.t);
    for (int i = 1; i < output.reactionWrench.size(); ++i) { // ignore ground body
	jrTemp.push_back(output.reactionWrench[i](1)[0]);
	jrTemp.push_back(output.reactionWrench[i](1)[1]);
	jrTemp.push_back(output.reactionWrench[i](1)[2]);
	jrTemp.push_back(output.reactionWrench[i](0)[0]);
	jrTemp.push_back(output.reactionWrench[i](0)[1]);
	jrTemp.push_back(output.reactionWrench[i](0)[2]);
    }
    logger->addRow(jrTemp);
    
    return output;
}

/*******************************************************************************/
