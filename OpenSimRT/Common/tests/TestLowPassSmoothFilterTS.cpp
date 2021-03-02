/**
 * @file TestLowPassSmoothFilterTS.cpp
 *
 * \brief Tests thread-safe implementation of LowPassSmoothFilter.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "Exception.h"
#include "INIReader.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "OpenSimUtils.h"
#include <Common/Exception.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <Actuators/Thelen2003Muscle.h>
#include <SimTKcommon/Scalar.h>
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_LOW_PASS_SMOOTH_FILTER";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);
    auto calcDer = ini.getBoolean(section, "CALC_DER", true);

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    model.initSystem();

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // initialize filter
    LowPassSmoothFilterTS::Parameters parameters;
    parameters.numSignals = model.getNumCoordinates();
    parameters.memory = memory;
    parameters.delay = delay;
    parameters.cutoffFrequency = cutoffFreq;
    parameters.splineOrder = splineOrder;
    parameters.calculateDerivatives = calcDer;
    LowPassSmoothFilterTS filter(parameters);

    // logger
    auto columnNames =
        OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
    TimeSeriesTable q, qDot, qDDot;
    q.setColumnLabels(columnNames);
    qDot.setColumnLabels(columnNames);
    qDDot.setColumnLabels(columnNames);

    // filter update function in separate thread
    thread updateFilt([&]() {
        // loop through ik storage
        for (int i = 0; i < qTable.getNumRows(); i++) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();
            // update filter state
            filter.updState({t, qRaw});

            // sleep
            std::this_thread::sleep_for(chrono::milliseconds(5));
        }

        // WARNING: pass bad input to terminate filtering (exception can be
        // thrown only from the consumer thread, otherwise it's stuck in the
        // 'wait' state)
        filter.updState({-SimTK::Infinity, Vector(model.getNumCoordinates())});
    });

    // filter in main thread
    try {
        while (true) {
            // filter
            auto output = filter.filter();

            // record
            q.appendRow(output.t, ~output.x);
            qDot.appendRow(output.t, ~output.xDot);
            qDDot.appendRow(output.t, ~output.xDDot);
        }
    } catch (const std::exception& e) { cout << e.what() << "\n"; }
    updateFilt.join();

    // Compare results with reference tables. Make sure that M, D,
    // spline order, fc are the same as the test.
    SimTK_ASSERT_ALWAYS(memory == 35, "ensure that MEMORY = 35 in setup.ini for testing");
    SimTK_ASSERT_ALWAYS(delay == 14, "ensure that DELAY = 35 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(cutoffFreq == 6, "ensure that CUTOFF_FREQ = 6 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(splineOrder == 3, "ensure that SPLINE_ORDER = 3 setup.ini for testing");
    OpenSimUtils::compareTables(
            q, TimeSeriesTable(
                       subjectDir +
                       "real_time/filtering/proposed_filter/q_filtered.sto"));
    OpenSimUtils::compareTables(
            qDot,
            TimeSeriesTable(
                    subjectDir +
                    "real_time/filtering/proposed_filter/qDot_filtered.sto"));
    OpenSimUtils::compareTables(
            qDDot,
            TimeSeriesTable(
                    subjectDir +
                    "real_time/filtering/proposed_filter/qDDot_filtered.sto"));

    // // store results
    // STOFileAdapter::write(q,
    //                       subjectDir + "real_time/filtering/q_filtered_ts.sto");
    // STOFileAdapter::write(
    //         qDot, subjectDir + "real_time/filtering/qDot_filtered_ts.sto");
    // STOFileAdapter::write(
    //         qDDot, subjectDir + "real_time/filtering/qDDot_filtered_ts.sto");
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
