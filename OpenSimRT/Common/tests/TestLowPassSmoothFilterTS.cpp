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

#include <Common/Exception.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <SimTKcommon/Scalar.h>
#include <chrono>
#include <exception>
#include <iostream>
#include <numeric>
#include <stdexcept>
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
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);
    auto calcDer = ini.getBoolean(section, "CALC_DER", true);

    // read the motion file and use uniform sampling of 100Hz
    Storage ikQ(ikFile);
    ikQ.resampleLinear(0.01);

    LowPassSmoothFilterTS::Parameters parametersTS;
    parametersTS.numSignals = ikQ.getStateVector(0)->getSize();
    parametersTS.memory = memory;
    parametersTS.delay = delay;
    parametersTS.cutoffFrequency = cutoffFreq;
    parametersTS.splineOrder = splineOrder;
    parametersTS.calculateDerivatives = calcDer;
    LowPassSmoothFilterTS filterTS(parametersTS);

    // logger
    auto arrayStr = ikQ.getColumnLabels();
    vector<string> columns;
    osimToStd(arrayStr, columns);
    columns.erase(columns.begin());
    TimeSeriesTable q, qDot, qDDot;
    q.setColumnLabels(columns);
    qDot.setColumnLabels(columns);
    qDDot.setColumnLabels(columns);

    // filter update function in separate thread
    thread updateFilt([&]() {
        // loop through ik storage
        for (int i = 0; i < ikQ.getSize(); ++i) {
            // read storage entry
            auto stateVector = ikQ.getStateVector(i);
            double t = stateVector->getTime();
            auto x = Vector(stateVector->getSize(), &stateVector->getData()[0]);

            // update filter state
            filterTS.updState(LowPassSmoothFilterTS::Input{t, x});

            // sleep
            std::this_thread::sleep_for(chrono::milliseconds(5));
        }

        // bad input to terminate filtering
        filterTS.updState(LowPassSmoothFilterTS::Input{
                -SimTK::Infinity, Vector(ikQ.getStateVector(0)->getSize())});
    });

    // filter in main thread
    try {
        while (true) {
            // filter
            auto output = filterTS.filter();

            // record
            q.appendRow(output.t, ~output.x);
            qDot.appendRow(output.t, ~output.xDot);
            qDDot.appendRow(output.t, ~output.xDDot);
        }
    } catch (const std::exception& e) { cout << e.what() << "\n"; }
    updateFilt.join();

    // store results
    STOFileAdapter::write(q,
                          subjectDir + "real_time/filtering/q_filtered_ts.sto");
    STOFileAdapter::write(
            qDot, subjectDir + "real_time/filtering/qDot_filtered_ts.sto");
    STOFileAdapter::write(
            qDDot, subjectDir + "real_time/filtering/qDDot_filtered_ts.sto");
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
