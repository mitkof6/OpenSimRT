/**
 * @file testViconSDK.cpp
 *
 * \brief Connects with Vicon server and read marker and force data.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include "ViconDataStream.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace ViconDataStreamSDK::CPP;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto hostName = ini.Get("VICON", "HOST_NAME", "");
    auto forcePlate00X = ini.GetReal("VICON", "FORCE_PLATE_00_X", 0);
    auto forcePlate00Y = ini.GetReal("VICON", "FORCE_PLATE_00_Y", 0);
    auto forcePlate00Z = ini.GetReal("VICON", "FORCE_PLATE_00_Z", 0);
    auto referenceFrameX = ini.Get("VICON", "REFERENCE_FRAME_AXIS_X", "");
    auto referenceFrameY = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Y", "");
    auto referenceFrameZ = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Z", "");

    // setup vicon
    ViconDataStream vicon(vector<Vec3>{Vec3(forcePlate00X,
					    forcePlate00Y,
					    forcePlate00Z)});
    vicon.connect(hostName);
    vicon.initialize(stringToDirection(referenceFrameX),
		     stringToDirection(referenceFrameY),
		     stringToDirection(referenceFrameZ));
    vicon.startAcquisition();   

    double previousTime= -1.0;
    while (true) {  
    	auto markerData = vicon.markerBuffer.get(1)[0];
    	auto forceData = vicon.forceBuffer.get(10, true)[0];
	
	double t = forceData.time;
    	if (previousTime >= t) {
    	    continue;
    	}
	cout << "time: " << t << endl;

    	for (auto marker : markerData.markers) {
    	    cout << marker.first << " " << marker.second << endl;
    	}
	
    	for (auto wrench : forceData.externalWrenches) {
    	    cout << wrench.first
		 << " p: " << wrench.second.point
		 << " f: " << wrench.second.force
		 << " t: " << wrench.second.torque
		 << endl;
    	}
    	previousTime = t;
    }
}

int main(int argc, char *argv[]) {
    try {
        run();
    } catch (exception &e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
