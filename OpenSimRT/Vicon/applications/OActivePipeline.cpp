/**
 * @file OActivePipeline.cpp
 *
 * \brief Real-time analysis for the estimation of knee reaction forces.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include "ViconDataStream.h"
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto hostName = ini.Get("VICON", "HOST_NAME", "");
    auto forcePlate00X = ini.GetReal("VICON", "FORCE_PLATE_00_X", 0);
    auto forcePlate00Y = ini.GetReal("VICON", "FORCE_PLATE_00_Y", 0);
    auto forcePlate00Z = ini.GetReal("VICON", "FORCE_PLATE_00_Z", 0);
    auto referenceFrameX = ini.Get("VICON", "REFERENCE_FRAME_AXIS_X", "");
    auto referenceFrameY = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Y", "");
    auto referenceFrameZ = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Z", "");
    auto subjectDir = DATA_DIR + ini.Get("VICON", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("VICON", "MODEL_FILE", "");
    auto grfRightApplyBody = ini.Get("VICON", "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed = ini.Get("VICON", "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed = ini.Get("VICON", "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");

    // setup vicon
    ViconDataStream vicon(vector<Vec3>{Vec3(forcePlate00X,
					    forcePlate00Y,
					    forcePlate00Z)});
    vicon.connect(hostName);
    vicon.initialize(stringToDirection(referenceFrameX),
		     stringToDirection(referenceFrameY),
		     stringToDirection(referenceFrameZ));

    // prepare marker tasks
    Model model(modelFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    createMarkerTasksFromMarkerNames(model, vicon.markerNames,
				     markerTasks, observationOrder);
    InverseKinematics ik(modelFile, 100, markerTasks,
                         vector<InverseKinematics::IMUTask>{});

    // ground reaction forces
    ExternalWrench::Parameters grfRightFootPar{grfRightApplyBody,
					       grfRightForceExpressed,
					       grfRightPointExpressed};
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    auto forcePlateNames = vicon.forcePlateNames;

    // acquisition function 
    auto dataAcquisitionFunction =
	[&]() -> MotionCaptureInput {
	    MotionCaptureInput output;
	    auto markerData = vicon.markerBuffer.get(1)[0];
	    auto forceData = vicon.forceBuffer.get(10, true)[0];
	    
	    // prepare ik data
	    output.ikFrame.t = markerData.time;
	    for (auto markerName : observationOrder) {
		output.ikFrame.markerObservations.push_back(markerData.markers[markerName]);
	    }
	    
	    // prepare wrench data
	    for (auto name : forcePlateNames) {
		output.externalWrenches.push_back(forceData.externalWrenches[name]);
	    }
	    return output;
	};

    // OActive pipeline setup
    RealTimeAnalysis::Parameters pipelineParameters;
    pipelineParameters.useVisualizer = true;
    pipelineParameters.solveMuscleOptimization = true;
    pipelineParameters.fc = 6.0;
    pipelineParameters.filterOrder = 50;
    pipelineParameters.samplesDelay = 10;
    pipelineParameters.modelFile = modelFile;
    pipelineParameters.ikMarkerTasks = markerTasks;
    pipelineParameters.ikConstraintsWeight = 100;
    pipelineParameters.wrenchParameters = wrenchParameters;
    pipelineParameters.dataAcquisitionFunction = dataAcquisitionFunction;
    pipelineParameters.momentArmFunction = momentArmSelector(modelFile);
    pipelineParameters.reactionForceOnBodies = vector<string>{"tibia_r"};
    RealTimeAnalysis pipeline(pipelineParameters);

    vicon.startAcquisition();
    pipeline.run();
    pipeline.exportResults(subjectDir + "results_rt/");
}

int main(int argc, char *argv[]) {
    try {
        run();
    } catch (exception &e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
