/**
 * @file testViconSDK.cpp
 *
 * \brief Connects with Vicon server and read marker and force data.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "INIReader.h"
#include "Settings.h"
#include "ViconDataStream.h"

#include <iostream>

using namespace std;
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

    // setup vicon
    ViconDataStream vicon(
            vector<Vec3>{Vec3(forcePlate00X, forcePlate00Y, forcePlate00Z)});
    vicon.connect(hostName);
    vicon.initialize(stringToDirection(referenceFrameX),
                     stringToDirection(referenceFrameY),
                     stringToDirection(referenceFrameZ));
    vicon.startAcquisition();

    double previousTime = -1.0;
    while (true) {
        auto markerData = vicon.markerBuffer.get(1)[0];
        auto forceData = vicon.forceBuffer.get(10, true)[0];

        double t = forceData.time;
        if (previousTime >= t) { continue; }
        cout << "time: " << t << endl;

        for (auto marker : markerData.markers) {
            cout << marker.first << " " << marker.second << endl;
        }

        for (auto wrench : forceData.externalWrenches) {
            cout << wrench.first << " p: " << wrench.second.point
                 << " f: " << wrench.second.force
                 << " t: " << wrench.second.torque << endl;
        }
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
