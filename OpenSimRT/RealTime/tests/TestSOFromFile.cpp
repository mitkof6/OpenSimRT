/**
 * @file TestSOFromFile.cpp
 *
 * \brief Loads results from OpenSim IK and externally applied forces
 * and executes the static optimization analysis in an iterative
 * manner in order to determine the muscle forces.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "Exception.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

#include <OpenSim/Common/LoadOpenSimLibrary.h>
// necessary for loading dynamic libraries
#ifdef _WIN32
#    include <winbase.h>
#    include <windows.h>
#else
#    include <dlfcn.h>
#endif

// function prototype: SimTK::Matrix calcMomentArm(const SimTK::Vector& q)
typedef SimTK::Matrix (*CalcMomentArm)(const SimTK::Vector& q);

void run() {
    // load library
    auto momentArmLibHandle =
            OpenSim::LoadOpenSimLibrary("Gait1992MomentArm_rd", true);

    // get function pointer
#ifdef _WIN32
    CalcMomentArm calcMomentArm =
            (CalcMomentArm) GetProcAddress(momentArmLibHandle, "calcMomentArm");
#else
    CalcMomentArm calcMomentArm =
            (CalcMomentArm) dlsym(momentArmLibHandle, "calcMomentArm");
#endif

    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_SO_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto idFile = subjectDir + ini.getString(section, "ID_FILE", "");

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    Model model(modelFile);
    model.initSystem();

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

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    // initialize so
    MuscleOptimization::OptimizationParameters optimizationParameters;
    optimizationParameters.convergenceTolerance =
            1e-0; // set 1e-0 for linear muscle
    MuscleOptimization so(
            model, optimizationParameters, calcMomentArm,
            // new TorqueBasedTargetNonLinearMuscle()); // TODO change tolerance
            new TorqueBasedTargetLinearMuscle());

    // visualizer
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;

    // loop through kinematic frames
    for (int i = 0; i < qTable.getNumRows(); i++) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();
        auto tauRaw = tauTable.getRowAtIndex(i).getAsRowVector();

        // filter
        auto ikFiltered = ikFilter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;

        if (!ikFiltered.isValid) { continue; }

        // perform id
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        auto soOutput = so.solve({t, q, qDot, ~tauRaw});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

        // visualization
        visualizer.update(q, soOutput.am);

        // this_thread::sleep_for(chrono::milliseconds(10));
    }
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
