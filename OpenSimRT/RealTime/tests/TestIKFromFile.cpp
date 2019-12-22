/**
 * @file TestIKFromFile.cpp
 *
 * \brief Loads the marker trajectories and executes inverse kinematics in an
 * iterative manner in order to determine the model kinematics.
 *
 * @author Dimitar Stanev dimitar.stanev@epfl.ch
 */
#include <iostream>
#include <thread>
#include "Simulation.h"
#include "Visualization.h"
#include "CSVLogger.h"
#include "OpenSimUtils.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.getString("TESTS", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.getString("TESTS", "MODEL_FILE", "");
    auto trcFile = subjectDir +  ini.getString("TESTS", "TRC_FILE", "");
    auto ikTaskSetFile = subjectDir +  ini.getString("TESTS", "IK_TASK_SET_FILE", "");

    Model model(modelFile);

    // construct marker tasks from marker data (.trc)
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromIKTaskSet(model,
                                                      ikTaskSet,
                                                      markerTasks,
                                                      observationOrder);

    // initialize loggers
    auto coordinateColumnNames = ModelUtils::getCoordinateNames(model);
    coordinateColumnNames.insert(coordinateColumnNames.begin(), "time");
    CSVLogger q(coordinateColumnNames);

    // initialize ik
    InverseKinematics ik(model,
                         markerTasks,
                         vector<InverseKinematics::IMUTask>{},
                         100);

    // visualizer
    BasicModelVisualizer visualizer(modelFile);

    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = InverseKinematics::getFrameFromMarkerData(i,
                                                               markerData,
                                                               observationOrder,
                                                               false);

        // perform ik
        auto pose = ik.solve(frame);
        q.addRow(pose.t, pose.q);

        // visualize
        visualizer.update(pose.q);
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    // store results
    q.exportToFile(subjectDir + "results_rt/q.csv");
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
