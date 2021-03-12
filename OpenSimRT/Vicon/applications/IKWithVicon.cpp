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
 * @file IKWithVicon.cpp
 *
 * \brief Interface inverse kinematics with Vicon server.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "INIReader.h"
#include "InverseKinematics.h"
#include "Settings.h"
#include "ViconDataStream.h"
#include "Visualization.h"

#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto hostName = ini.getString("VICON", "HOST_NAME", "");
    auto forcePlate00X = ini.getReal("VICON", "FORCE_PLATE_00_X", 0);
    auto forcePlate00Y = ini.getReal("VICON", "FORCE_PLATE_00_Y", 0);
    auto forcePlate00Z = ini.getReal("VICON", "FORCE_PLATE_00_Z", 0);
    auto referenceFrameX = ini.getString("VICON", "REFERENCE_FRAME_AXIS_X", "");
    auto referenceFrameY = ini.getString("VICON", "REFERENCE_FRAME_AXIS_Y", "");
    auto referenceFrameZ = ini.getString("VICON", "REFERENCE_FRAME_AXIS_Z", "");
    auto subjectDir = DATA_DIR + ini.getString("VICON", "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString("VICON", "MODEL_FILE", "");

    // setup vicon
    ViconDataStream vicon(
            vector<Vec3>{Vec3(forcePlate00X, forcePlate00Y, forcePlate00Z)});
    vicon.connect(hostName);
    vicon.initialize(stringToDirection(referenceFrameX),
                     stringToDirection(referenceFrameY),
                     stringToDirection(referenceFrameZ));

    // prepare marker tasks
    Model model(modelFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, vicon.markerNames, markerTasks, observationOrder);

    InverseKinematics ik(model, markerTasks, {}, SimTK::Infinity, 1e-5);

    // visualizer
    BasicModelVisualizer visualizer(model);

    vicon.startAcquisition();
    double previousTime = -1.0;
    while (true) {
        auto markerData = vicon.markerBuffer.get(1)[0];
        double t = markerData.time;
        if (previousTime >= t) { continue; }

        // ik
        InverseKinematics::Input ikInput;
        ikInput.t = t;
        for (auto markerName : observationOrder) {
            ikInput.markerObservations.push_back(
                    markerData.markers[markerName]);
        }
        auto ikOutput = ik.solve(ikInput);

        // visualizer
        visualizer.update(ikOutput.q);
        previousTime = t;
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
