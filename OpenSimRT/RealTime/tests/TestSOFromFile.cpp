/**
 * @file TestSOFromFile.cpp
 *
 * \brief Loads results from OpenSim IK and externally applied forces and
 * executes the static optimization analysis in an iterative manner in order to
 * determine the muscle forces.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "Exception.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    cout << "Warning" << endl
         << "This test might fail on different machines. "
         << "The performance of the optimization depends on the underlying OS. "
         << "We think it has to do with how threads are scheduled by the OS. "
         << "We did not observed this behavior with OpenSim v3.3." << endl << endl;

    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_SO_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto idFile = subjectDir + ini.getString(section, "ID_FILE", "");

	auto momentArmLibraryPath = LIBRARY_OUTPUT_PATH + "/" +
        ini.getString(section, "MOMENT_ARM_LIBRARY", "");

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    auto convergenceTolerance = ini.getReal(section, "CONVERGENCE_TOLERANCE", 0);
    auto memoryHistory = ini.getReal(section, "MEMORY_HISTORY", 0);
    auto maximumIterations = ini.getInteger(section, "MAXIMUM_ITERATIONS", 0);
    auto objectiveExponent = ini.getInteger(section, "OBJECTIVE_EXPONENT", 0);

    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    model.initSystem();

	// load and verify moment arm function
	auto calcMomentArm =
		OpenSimUtils::getMomentArmFromDynamicLibrary(model, momentArmLibraryPath);

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // read external forces
    auto tauTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, idFile, 0.01);

    if (tauTable.getNumRows() != qTable.getNumRows()) {
        THROW_EXCEPTION("ik and id storages of different size " +
                        toString(qTable.getNumRows()) +
                        " != " + toString(tauTable.getNumRows()));
    }

    // initialize so
    MuscleOptimization::OptimizationParameters optimizationParameters;
    optimizationParameters.convergenceTolerance = convergenceTolerance;
    optimizationParameters.memoryHistory = memoryHistory;
    optimizationParameters.maximumIterations = maximumIterations;
    optimizationParameters.objectiveExponent = objectiveExponent;
    MuscleOptimization so(model, optimizationParameters, calcMomentArm);
    auto fmLogger = so.initializeMuscleLogger();
    auto amLogger = so.initializeMuscleLogger();
    // auto tauResLogger = so.initializeResidualLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;

    // loop through kinematic frames
    for (int i = 0; i < qTable.getNumRows(); i++) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto q = qTable.getRowAtIndex(i).getAsVector();
        auto tau = tauTable.getRowAtIndex(i).getAsRowVector();

        // perform id
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        auto soOutput = so.solve({t, q, ~tau});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

        // visualization
        visualizer.update(q, soOutput.am);

        // log data (use filter time to align with delay)
        fmLogger.appendRow(t, ~soOutput.fm);
        amLogger.appendRow(t, ~soOutput.am);

        // this_thread::sleep_for(chrono::milliseconds(10));
    }

    cout << "Mean delay: " << (double) sumDelayMS / qTable.getNumRows() << " ms"
         << endl;

    // Compare results with reference tables. Test might fail on a different
    // machine, possibly due to compilation differences.
    try {
        OpenSimUtils::compareTables(
            fmLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/muscle_optimization/fm.sto"),
            1e-1);
        OpenSimUtils::compareTables(
            amLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/muscle_optimization/am.sto"),
            1e-1);
    } catch (exception& e) {
        cout << e.what() << endl;
    }

    // store results
    // STOFileAdapter::write(fmLogger,
    //                       subjectDir + "real_time/muscle_optimization/fm.sto");
    // STOFileAdapter::write(amLogger,
    //                       subjectDir + "real_time/muscle_optimization/am.sto");
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
