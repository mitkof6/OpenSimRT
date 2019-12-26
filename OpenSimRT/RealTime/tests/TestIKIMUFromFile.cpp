/**
 * @file TestIKIMUFromFile.cpp
 *
 * \brief Perform inverse kinematics using IMU orientations from file.
 *
 * @author Dimitar Stanev dimitar.stanev@epfl.ch
 */
#include <iostream>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "Simulation.h"
#include "INIReader.h"
#include "Visualization.h"
#include "OpenSimUtils.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.getString("NGIMU", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.getString("NGIMU", "MODEL_FILE", "");
    auto trcFile = subjectDir +  ini.getString("NGIMU", "TRC_FILE", "");

    Model model(modelFile);

    // create IMU tasks from marker data (.trc)
    MarkerData markerData(trcFile);
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> observationOrder;
    InverseKinematics::createIMUTasksFromMarkerData(model,
                                                    markerData,
                                                    imuTasks,
                                                    observationOrder);

    // initialize loggers
    auto coordinateColumnNames = ModelUtils::getCoordinateNames(model);
    TimeSeriesTable q;
    q.setColumnLabels(coordinateColumnNames);

    // initialize ik
    InverseKinematics ik(model,
                         vector<InverseKinematics::MarkerTask>{},
                         imuTasks,
                         100);

    // visualizer
    BasicModelVisualizer visualizer(model);

    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = InverseKinematics::getFrameFromMarkerData(i,
                                                               markerData,
                                                               observationOrder,
                                                               true);

        // perform ik
        auto pose = ik.solve(frame);

        // visualize
        visualizer.update(pose.q);

        // record
	q.appendRow(pose.t, ~pose.q);
    }

    // store results
    STOFileAdapter::write(q, subjectDir + "real_time/q.sto");
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
