/**
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
    BasicModelVisualizer visualizer(model);

    try { // main loop
        while (!driver.shouldTerminate()) {
            // get input from imus
            auto imuData = driver.getFrame();

            // solve ik
            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});

            // visualize
            visualizer.update(pose.q);

            // record
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) { cout << e.what() << endl; }

    // store results
    STOFileAdapter::write(qLogger,
                          subjectDir + "real_time/inverse_kinematics/q.sto");
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
