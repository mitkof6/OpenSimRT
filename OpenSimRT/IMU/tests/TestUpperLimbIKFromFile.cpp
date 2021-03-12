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
 * @file TestLowerLimbIKFromFile.cpp
 *
 * @brief Test IK with prerecorded NGIMU data on the upper body.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "NGIMUInputFromFileDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"

#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <OpenSim/Common/STOFileAdapter.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "UPPER_LIMB_NGIMU_OFFLINE";
    // imu calibration settings
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0.0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0.0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0.0);
    auto imuObservationOrder =
            ini.getVector(section, "IMU_BODIES", vector<string>());

    // driver send rate
    auto rate = ini.getInteger(section, "DRIVER_SEND_RATE", 0);

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    // setup model
    Object::RegisterType(Schutte1993Muscle_Deprecated());
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(model, {}, markerTasks,
                                                        markerObservationOrder);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // ngimu input data driver from file
    NGIMUInputFromFileDriver driver(ngimuDataFile, rate);
    driver.startListening();

    // calibrator
    IMUCalibrator clb(model, &driver, imuObservationOrder);
    clb.recordNumOfSamples(10);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;
    int numFrames = 0;

    try { // main loop
        while (!driver.shouldTerminate()) {
            // get input from imus
            auto imuData = driver.getFrame();
            numFrames++;

            // solve ik
            chrono::high_resolution_clock::time_point t1;
            t1 = chrono::high_resolution_clock::now();

            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});

            chrono::high_resolution_clock::time_point t2;
            t2 = chrono::high_resolution_clock::now();
            sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
                                  .count();

            // visualize
            visualizer.update(pose.q);

            // record
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) { cout << e.what() << endl; }

    cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

    // // store results
    // STOFileAdapter::write(
    //         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
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
