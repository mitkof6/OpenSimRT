/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file TestButterworthFilter.cpp
 *
 * \brief Tests the Butterworth filter implementation.
 *
 * @author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <chrono>
#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_BUTTERWORTH_FILTER";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

    // filter parameters
    auto cutOffFreq = ini.getInteger(section, "CUTOFF_FREQ", 0);
    auto filtOrder = ini.getInteger(section, "FILTER_ORDER", 0);
    auto filtType = ini.getString(section, "FILTER_TYPE", "");

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    model.initSystem();

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // select appropriate filter type
    ButterworthFilter::FilterType type;
    if (filtType == "lp")
        type = ButterworthFilter::FilterType::LowPass;
    else if (filtType == "hp")
        type = ButterworthFilter::FilterType::HighPass;
    else
        THROW_EXCEPTION("Wrong input name. Selected appropriate name of filter "
                        "type. Options: 'lp' or 'hp'");

    // initialize filters
    ButterworthFilter filter(model.getNumCoordinates(), filtOrder,
                             (2 * cutOffFreq) / 100.0, type,
                             IIRFilter::InitialValuePolicy::Signal);

    // logger
    auto columnNames =
            OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
    TimeSeriesTable q;
    q.setColumnLabels(columnNames);

    // mean delay
    int sumDelayMS = 0;

    // loop through ik storage
    for (int i = 0; i < qTable.getNumRows(); ++i) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();

        // filter
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        auto qFiltered = filter.filter(qRaw);

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

        // record
        q.appendRow(t, ~qFiltered);
    }

    cout << "Mean delay: " << (double) sumDelayMS / qTable.getNumRows() << " ms"
         << endl;

    // Compare results with reference tables.
    OpenSimUtils::compareTables(
            q, TimeSeriesTable(subjectDir +
                               "real_time/filtering/bw_filter/q_filtered_" +
                               "F" + to_string(cutOffFreq) + "O" +
                               to_string(filtOrder) + "T" + filtType + ".sto"));

    // // store results
    // STOFileAdapter::write(
    //         q, subjectDir + "real_time/filtering/bw_filter/q_filtered_" + "F"
    //         +
    //                    to_string(cutOffFreq) + "O" + to_string(filtOrder) +
    //                    "T" + filtType + ".sto");
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
