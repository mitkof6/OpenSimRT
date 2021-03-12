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
 * @file UpperLimbIK.cpp
 *
 * @brief Online IK with NGIMU data on the upper body.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "NGIMUInputDriver.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "SyncManager.h"
#include "Visualization.h"

#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "UPPER_LIMB_NGIMU";

    // imu communication settings
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto LISTEN_PORTS = ini.getVector(section, "LISTEN_PORTS", vector<int>());

    // imu calibration Settings
    auto imuObservationOrder =
            ini.getVector(section, "IMU_BODIES", vector<string>());
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);

    // filter settings
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // SyncManager settings
    auto syncRate = ini.getReal(section, "SYNC_MANAGER_RATE", 0);
    auto syncThreshold = ini.getReal(section, "SYNC_MANAGER_THRESHOLD", 0);

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // setup model
    Object::RegisterType(Schutte1993Muscle_Deprecated());
    Model model(modelFile);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // imu driver
    NGIMUInputDriver driver;
    driver.setupInput(imuObservationOrder,
                      vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                      LISTEN_PORTS);
    driver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);
    auto imuLogger = driver.initializeLogger();

    // start listening
    thread listen(&NGIMUInputDriver::startListening, &driver);

    // imu calibrator
    IMUCalibrator clb = IMUCalibrator(model, &driver, imuObservationOrder);
    clb.recordTime(3); // record for 3 seconds
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // sensor data synchronization
    SyncManager manager(syncRate, syncThreshold);

    // setup filters
    LowPassSmoothFilter::Parameters filterParam;
    filterParam.numSignals = model.getNumCoordinates();
    filterParam.memory = memory;
    filterParam.delay = delay;
    filterParam.cutoffFrequency = cutoffFreq;
    filterParam.splineOrder = splineOrder;
    filterParam.calculateDerivatives = false;
    LowPassSmoothFilter filter(filterParam);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, {}, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataList = driver.getData();

            // change frame representation
            const auto imuDataAsPairs = driver.asPack(imuDataList);

            // synchronize data streams
            manager.appendPack(imuDataAsPairs);
            auto pack = manager.getPack();

            // proceed only if there are synced data
            if (pack.second.empty()) continue;

            // retrieve original representation of input data
            std::pair<double, std::vector<NGIMUData>> imuData;
            imuData.first = pack.first;
            imuData.second = driver.fromVector(pack.second[0]);

            // solve ik
            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});

            // filter
            auto ikFiltered = filter.filter({pose.t, pose.q});
            auto t = ikFiltered.t;
            auto q = ikFiltered.x;

            // proceed only if the filter output is valid
            if (!ikFiltered.isValid) continue;

            // visualize
            visualizer.update(q);

            // record
            qLogger.appendRow(t, ~q);
            imuLogger.appendRow(pose.t, ~pack.second[0]);
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        driver.stopListening();
        listen.join();

        // store results
        STOFileAdapter::write(
                qLogger, subjectDir + "real_time/inverse_kinematics/q.sto");
        CSVFileAdapter::write(imuLogger,
                              subjectDir + "experimental_data/ngimu_data.csv");
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
