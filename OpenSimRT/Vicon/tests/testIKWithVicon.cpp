/**
 * @file testIKWithVicon.cpp
 *
 * \brief Interface inverse kinematics with Vicon server.
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

    // visualizer
    BasicModelVisualizer visualizer(modelFile);

    vicon.startAcquisition();
    double previousTime= -1.0;
    while(!visualizer.shouldTerminate) {
	auto markerData = vicon.markerBuffer.get(1)[0];
	double t = markerData.time;
	if (previousTime >= t) {
    	    continue;
    	}

        // ik
        InverseKinematics::Input ikInput;
	ikInput.t = t;
        for (auto markerName : observationOrder) {
            ikInput.markerObservations.push_back(markerData.markers[markerName]);
        }
        auto ikOutput = ik.solve(ikInput);

        // visualizer
        visualizer.update(ikOutput.q);
	previousTime = t;
    }
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
