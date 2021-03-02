/**
 * @file TestIKIMUFromFile.cpp
 *
 * \brief Perform inverse kinematics using IMU orientations from file.
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

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_IK_IMU_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");

    // setup model
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // create IMU tasks from marker data (.trc)
    MarkerData markerData(trcFile);
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> observationOrder;
    InverseKinematics::createIMUTasksFromMarkerData(model, markerData, imuTasks,
                                                    observationOrder);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, vector<InverseKinematics::MarkerTask>{},
                         imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;

    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, true);

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

    // Compare results with reference tables.
    OpenSimUtils::compareTables(
            qLogger,
            TimeSeriesTable(subjectDir + "real_time/inverse_kinematics/q.sto"));

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
