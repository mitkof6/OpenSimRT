/**
 * @file TestIKFromFile.cpp
 *
 * \brief Loads the marker trajectories and executes inverse kinematics in an
 * iterative manner in order to determine the model kinematics.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"
#include "Utils.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <SimTKcommon/Scalar.h>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    // auto section = "RAJAGOPAL_2015";
    auto section = "TEST_IK_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");
    auto ikTaskSetFile =
            subjectDir + ini.getString(section, "IK_TASK_SET_FILE", "");

    // setup model
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // construct marker tasks from marker data (.trc)
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromIKTaskSet(
            model, ikTaskSet, markerTasks, observationOrder);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks,
                         vector<InverseKinematics::IMUTask>{}, SimTK::Infinity,
                         1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;

    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);

        // perform ik
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        auto pose = ik.solve(frame);

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

        // visualize
        visualizer.update(pose.q);

        // record
        qLogger.appendRow(pose.t, ~pose.q);

        // this_thread::sleep_for(chrono::milliseconds(10));
    }

    cout << "Mean delay: " << (double) sumDelayMS / markerData.getNumFrames()
         << " ms" << endl;

    OpenSimUtils::compareTables(
            qLogger,
            TimeSeriesTable(subjectDir + "real_time/inverse_kinematics/q.sto"),
            1e-3);

    // store results
    // STOFileAdapter::write(qLogger,
    //                       subjectDir + "real_time/inverse_kinematics/q.sto");
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
