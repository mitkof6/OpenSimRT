/**
 * @file testNGIMUReceive.cpp
 *
 * \brief Reads information from two NGIMU sensors configured in master slave
 * mode.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "INIReader.h"
#include "NGIMU.h"
#include "Settings.h"
#include "ip/UdpSocket.h"
#include "osc/OscPacketListener.h"
#include "osc/OscReceivedElements.h"

#include <OpenSim/Common/XsensDataReader.h>
#include <OpenSim/Common/XsensDataReaderSettings.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <iostream>
#include <thread>

using namespace osc;
using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_NGIMU_TRANSMIT";

    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto LISTEN_PORTS = ini.getVector(section, "LISTEN_PORTS", vector<int>());

    NGIMUManager manager(vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                         LISTEN_PORTS);
    manager.setupIMUs(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);

    thread listen(&NGIMUManager::startListeners, &manager);

    while (true) {
        InverseKinematics::Input input;
        manager.getObservations(input);
    }
    listen.join();
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        // we do not want this test to fail since device connection is required
        // return -1;
    }
    return 0;
}
