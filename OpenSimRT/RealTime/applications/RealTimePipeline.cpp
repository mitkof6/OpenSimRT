/**
 * @file RealTimePipeline.cpp
 *
 * \brief Apply real-time analysis from file.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include <thread>
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.Get("REAL_TIME_PIPELINE", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("REAL_TIME_PIPELINE", "MODEL_FILE", "");
    auto trcFile = subjectDir +  ini.Get("REAL_TIME_PIPELINE", "TRC_FILE", "");
    auto ikTaskSetFile = subjectDir +  ini.Get("REAL_TIME_PIPELINE", "IK_TASK_SET_FILE", "");
    auto grfMotFile = subjectDir + ini.Get("REAL_TIME_PIPELINE", "GRF_MOT_FILE", "");

    auto grfRightApplyBody = ini.Get("TESTS", "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed = ini.Get("TESTS", "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed = ini.Get("TESTS", "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier = ini.Get("TESTS", "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier = ini.Get("TESTS", "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier = ini.Get("TESTS", "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody = ini.Get("TESTS", "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed = ini.Get("TESTS", "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed = ini.Get("TESTS", "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier = ini.Get("TESTS", "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier = ini.Get("TESTS", "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier = ini.Get("TESTS", "GRF_LEFT_TORQUE_IDENTIFIER", "");
    
    Model model(modelFile);

    // prepare marker tasks
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    createMarkerTasksFromIKTaskSet(model, ikTaskSet,
                                   markerTasks, observationOrder);

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{grfRightApplyBody,
					       grfRightForceExpressed,
					       grfRightPointExpressed};
    auto grfRightLabels = createGRFLabelsFromIdentifiers(grfRightPointIdentifier,
							 grfRightForceIdentifier,
							 grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{grfLeftApplyBody,
					      grfLeftForceExpressed,
					      grfLeftPointExpressed};
    auto grfLeftLabels = createGRFLabelsFromIdentifiers(grfLeftPointIdentifier,
							grfLeftForceIdentifier,
							grfLeftTorqueIdentifier);
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // acquisition function (simulates acquisition from motion)
    auto dataAcquisitionFunction =
	[&]() -> MotionCaptureInput {
	    static int i = 0;
	    MotionCaptureInput input;

	    // get frame data
	    input.ikFrame = getFrameFromMarkerData(i, markerData, observationOrder, false);
	    double t = input.ikFrame.t;
	    
	    // get grf force
	    auto grfRightWrench = getWrenchFromStorage(t, grfRightLabels, grfMotion);
	    auto grfLeftWrench = getWrenchFromStorage(t, grfLeftLabels, grfMotion);
	    input.externalWrenches = {grfRightWrench, grfLeftWrench};
	    
	    // dummy delay to simulate real time
	    this_thread::sleep_for(chrono::milliseconds(16));
	    i++;
	    return input;
	};

    // pipeline
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
    pipelineParameters.reactionForceOnBodies = vector<string>{"tibia_r", "talus_l"};
    RealTimeAnalysis pipeline(pipelineParameters);
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
