#include "INIReader.h"
#include "Manager.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"
#include "ip/UdpSocket.h"
#include "osc/OscPacketListener.h"
#include "osc/OscReceivedElements.h"

#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_NGIMU_UPPER_LIMB";
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto LISTEN_PORTS = ini.getVector(section, "LISTEN_PORTS", vector<int>());
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // setup model
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> observationOrder{IMU_BODIES[0], IMU_BODIES[1]};
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, observationOrder, imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, vector<InverseKinematics::MarkerTask>{},
                         imuTasks, SimTK::Infinity, 1e-5);

    // visualizer
    BasicModelVisualizer visualizer(model);

    // manager
    NGIMUManager manager;
    manager.setupListeners(vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                           LISTEN_PORTS);
    manager.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);

    std::this_thread::sleep_for(2s);

    // start listening
    thread listen(&NGIMUManager::startListeners, &manager);

    // main loop
    while (true) {
        // get input
        auto input = manager.getObservations();

        // solve ik
        auto pose = ik.solve(input);

        // visualize
        visualizer.update(pose.q);
    }
    listen.join();
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
