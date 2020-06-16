#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_GRFM_PREDICTION_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto resultsDir = DATA_DIR + ini.getString(section, "RESULTS_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0);

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // setup model
    Model model(modelFile);
    model.initSystem();

    // setup external forces
    auto grfRightLogger = ExternalWrench::initializeLogger();
    auto grfLeftLogger = ExternalWrench::initializeLogger();

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    ContactForceAnalysis::Parameters parameters;
    parameters.convergence_tolerance = 1e-4;
    parameters.constraint_tolerance = 1e-6;
    parameters.platform_offset = Vec3(0, platform_offset, 0);
    ContactForceAnalysis grfm(model, parameters);

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);

    // loop through kinematic frames
    for (int i = 0; i < qTable.getNumRows(); i++) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();

        // filter
        auto ikFiltered = ikFilter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;
        auto qDDot = ikFiltered.xDDot;

        if (!ikFiltered.isValid) { continue; }

        // perform grfm prediction
        auto grfmOutput = grfm.solve({ikFiltered.t, q, qDot, qDDot});

        // visualization
        visualizer.update(q);
        rightGRFDecorator->update(grfmOutput[0].point, grfmOutput[0].force);
        leftGRFDecorator->update(grfmOutput[1].point, grfmOutput[1].force);

        // log data (use filter time to align with delay)
        grfRightLogger.appendRow(grfmOutput[0].t, ~grfmOutput[0].asVector());
        grfLeftLogger.appendRow(grfmOutput[1].t, ~grfmOutput[1].asVector());

        // this_thread::sleep_for(chrono::milliseconds(10));
    }

    // store results
    // STOFileAdapter::write(tauLogger, resultsDir + "tau.sto");
    STOFileAdapter::write(grfRightLogger, resultsDir + "wrench_right.sto");
    STOFileAdapter::write(grfLeftLogger, resultsDir + "wrench_left.sto");
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
