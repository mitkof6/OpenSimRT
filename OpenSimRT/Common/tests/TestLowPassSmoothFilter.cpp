/**
 * @file TestLowPassSmoothFilter.cpp
 *
 * \brief Tests the LowPassSmoothFilter.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include "Settings.h"
#include "INIReader.h"
#include "Measure.h"
#include "Utils.h"
#include "SignalProcessing.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <iostream>
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

    // read the motion file
    Storage ikQ(ikFile);
    ikQ.resampleLinear(0.01);

    // initialize filter
    LowPassSmoothFilter::Parameters parameters;
    parameters.numSignals = ikQ.getStateVector(0)->getSize();
    parameters.history = 50;
    parameters.delay = 10;
    parameters.cutoffFrequency = 6;
    parameters.filterOrder = 25;
    parameters.splineOrder = 3;
    parameters.calculateDerivatives = true;
    LowPassSmoothFilter filter(parameters);

    // logger
    auto arrayStr = ikQ.getColumnLabels();
    vector<string> columns;
    osimToStd(arrayStr, columns);
    columns.erase(columns.begin());
    TimeSeriesTable q, qDot, qDDot;
    q.setColumnLabels(columns);
    qDot.setColumnLabels(columns);
    qDDot.setColumnLabels(columns);

    // loop through ik storage
    for (int i = 0; i < ikQ.getSize(); i++) {
        // read storage entry
        auto stateVector = ikQ.getStateVector(i);
        double t = stateVector->getTime();
        auto x = Vector(stateVector->getSize(), &stateVector->getData()[0]);

        // filter
        START_CHRONO();
        auto output = filter.filter(LowPassSmoothFilter::Input{t, x});
        END_CHRONO();

        // record
        if (output.isValid) {
            q.appendRow(output.t, ~output.x);
            qDot.appendRow(output.t, ~output.xDot);
            qDDot.appendRow(output.t, ~output.xDDot);
        }
    }

    // store results
    STOFileAdapter::write(q, subjectDir + "real_time/q_filtered.sto");
    STOFileAdapter::write(qDot, subjectDir + "real_time/qDot_filtered.sto");
    STOFileAdapter::write(qDDot, subjectDir + "real_time/qDDot_filtered.sto");
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
