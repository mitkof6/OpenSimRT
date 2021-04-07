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
 * @file TestRTLFromFile.cpp
 *
 * \brief Apply real-time analysis from file.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "INIReader.h"
#include "InverseDynamics.h"
#include "RealTimeAnalysisLegacy.h"
#include "Settings.h"
#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_RT_PIPELINE_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");
    auto ikTaskSetFile =
            subjectDir + ini.getString(section, "IK_TASK_SET_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");

    auto grfRightApplyBody =
            ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier =
            ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier =
            ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier =
            ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody =
            ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier =
            ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier =
            ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier =
            ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");

    // Windows places executables in different folders. When ctest is
    // called on a Linux machine it runs the test from different
    // folders and thus the dynamic library might not be found
    // properly.
#ifndef WIN32
    auto momentArmLibraryPath =
            LIBRARY_OUTPUT_PATH + "/" +
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");
#else
    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");
#endif

    Model model(modelFile);

    // prepare marker tasks
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromIKTaskSet(
            model, ikTaskSet, markerTasks, observationOrder);

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // load moment arm function
    auto calcMomentArm = OpenSimUtils::getMomentArmFromDynamicLibrary(
            model, momentArmLibraryPath);

    // acquisition function (simulates acquisition from motion)
    auto dataAcquisitionFunction = [&]() -> MotionCaptureInput {
        static int i = 0;
        MotionCaptureInput input;

        // get frame data
        input.ikFrame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);
        double t = input.ikFrame.t;

        // get grf force
        auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);
        input.externalWrenches = {grfRightWrench, grfLeftWrench};

        // dummy delay to simulate real time
        this_thread::sleep_for(chrono::milliseconds(16));
        i++;
        return input;
    };

    // pipeline
    RealTimeAnalysisLegacy::Parameters pipelineParameters;
    pipelineParameters.useVisualizer = true;
    pipelineParameters.solveMuscleOptimization = true;
    pipelineParameters.fc = 6.0;
    pipelineParameters.filterOrder = 50;
    pipelineParameters.samplesDelay = 10;

    pipelineParameters.ikMarkerTasks = markerTasks;
    pipelineParameters.ikConstraintsWeight = 100;
    pipelineParameters.ikAccuracy = 1e-5;
    pipelineParameters.wrenchParameters = wrenchParameters;
    pipelineParameters.dataAcquisitionFunction = dataAcquisitionFunction;
    pipelineParameters.momentArmFunction = calcMomentArm;
    pipelineParameters.reactionForceOnBodies =
            vector<string>{"tibia_r", "talus_l"};
    RealTimeAnalysisLegacy pipeline(model, pipelineParameters);
    pipeline.run();
    pipeline.exportResults(subjectDir + "results_rt/");
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
