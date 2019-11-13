/**
 * @file testIKIMU.cpp
 *
 * \brief Perform inverse kinematics using IMU orientations from file.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

// test the alternative smoothing filter and differentiation scheme
#define IIR_FILTER

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.Get("NGIMU", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("NGIMU", "MODEL_FILE", "");
    auto trcFile = subjectDir +  ini.Get("NGIMU", "TRC_FILE", "");
    
    Model model(modelFile);

    // prepare marker tasks
    MarkerData markerData(trcFile);
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> observationOrder;
    createIMUTasksFromMarkerData(model, markerData, imuTasks, observationOrder);

    // initialize loggers
    auto coordinateColumnNames = getCoordinateNames(model);
    coordinateColumnNames.insert(coordinateColumnNames.begin(), "time");
    CSVLogger qFiltered(coordinateColumnNames),
        qDotFiltered(coordinateColumnNames), qDDotFiltered(coordinateColumnNames);

    // filters and differentiator
#ifdef IIR_FILTER
    IIRFilter filter(model.getNumCoordinates(),
                     Vector(Vec3(1., -1.1429805, 0.4128016)),
                     Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                     IIRFilter::Signal);
    // SavitzkyGolay filter(model.getNumCoordinates(), 7);
    NumericalDifferentiator dq(model.getNumCoordinates(), 2);
    NumericalDifferentiator ddq(model.getNumCoordinates(), 2);
#else
    StateSpaceFilter filter(model.getNumCoordinates(), 6);
#endif

    // initialize ik
    InverseKinematics ik(modelFile, 100, vector<InverseKinematics::MarkerTask>{},
			 imuTasks);
    BasicModelVisualizer visualizer(modelFile);
    
    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = getFrameFromMarkerData(i, markerData, observationOrder, true);

        // perform ik
        auto pose = ik.solve(frame);

        // filter and differentiate results
#ifdef IIR_FILTER
        auto q = filter.filter(pose.q);
        auto qDot = dq.diff(pose.t, q);
        auto qDDot = ddq.diff(pose.t, qDot);
        qFiltered.addRow(pose.t, q);
        qDotFiltered.addRow(pose.t, qDot);
        qDDotFiltered.addRow(pose.t, qDDot);
	visualizer.update(q);
#else
        auto filterState = filter.filter(pose.t, pose.q);
        qFiltered.addRow(filterState.t, filterState.x);
        qDotFiltered.addRow(filterState.t, filterState.xDot);
        qDDotFiltered.addRow(filterState.t, filterState.xDDot);
	visualizer.update(filterState.x);
#endif
    }
    
    // store results
    ik.logger->exportToFile(subjectDir + "results_rt/q_unfiltered.csv");
    qFiltered.exportToFile(subjectDir + "results_rt/q_filtered.csv");
    qDotFiltered.exportToFile(subjectDir + "results_rt/qDot_filtered.csv");
    qDDotFiltered.exportToFile(subjectDir + "results_rt/qDDot_filtered.csv");
    // getchar();
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
