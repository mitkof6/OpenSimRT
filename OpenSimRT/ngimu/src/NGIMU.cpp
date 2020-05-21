#include "NGIMU.h"

#include "osc/OscReceivedElements.h"
#include "osc/OscOutboundPacketStream.h"
#include <chrono>

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;


#define OUTPUT_BUFFER_SIZE 1024
/*******************************************************************************/

void NGIMUListener::ProcessMessage(const ReceivedMessage& m,
                                   const IpEndpointName& remoteEndpoint) {
    try {
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            // double t = frameNumber * 1.0 / frameRate;
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            auto quaternion = IMUData::Quaternion{q1, q2, q3, q4};
            // data.add({t, quaternion});
            // frameNumber++;

            cout << "/quaternion " << q1 << ", " << q2 << ", " << q3 << ", "
                 << q4 << endl;
        }
    } catch (Exception& e) {
        cout << "error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}

// void NGIMUListener::setFrameRate(const double& fr) { frameRate = fr; }

NGIMUManager::NGIMUManager() { m_Manager = this; }

void NGIMUManager::setupIMUs(const std::vector<std::string>& remoteIPs,
                            const std::vector<int>& remotePorts,
                            const std::string& localIP,
                            const std::vector<int>& localPorts) {

     // message init
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    for (int i=0; i<remoteIPs.size(); ++i) {
        UdpTransmitSocket socket(IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // what ip imu shall send to
        p.Clear();
        p << osc::BeginMessage("/wifi/send/ip") << localIP.c_str()
          << osc::EndMessage;
        socket.Send(p.Data(), p.Size());

        // what port to send
        p.Clear();
        p << osc::BeginMessage("/wifi/send/port") << localPorts[i] << osc::EndMessage;
        socket.Send(p.Data(), p.Size());

        // bling!
        p.Clear();
        p << osc::BeginMessage("/identify") << osc::EndMessage;
        socket.Send(p.Data(), p.Size());
    }
}

void NGIMUManager::startListenersImp(const vector<string>& ips,
                                     const vector<int>& ports) {
    assert(ips.size() == ports.size());

    // setup listeners
    for (const auto& port : ports) {
        listeners.push_back(new NGIMUListener());
        udpSockets.push_back(new UdpSocket());
    }

    // bind socket, attach to mux, and run
    for (int i = 0; i < ports.size(); ++i) {
        udpSockets[i]->Bind(IpEndpointName(ips[i].c_str(), ports[i]));
        mux.AttachSocketListener(udpSockets[i],
                                 dynamic_cast<PacketListener*>(listeners[i]));
        std::cout << "Start Listening on port: " << ports[i] << std::endl;
    }
    mux.RunUntilSigInt();
    // mux.Break();
}

// InverseKinematics::Input NGIMUManager::getObservationsImp() {
//     // find minimum common time
//     map<int, vector<IMUData>> imuData;
//     int M = 15;
//     double tMin = SimTK::Infinity;
//     for (const auto& imu : imus) {
//         auto id = imu.first;
//         auto data = imu.second->data.get(M);
//         imuData[id] = data;
//         if (data[0].t < tMin) { tMin = data[0].t; }
//     }

//     // prepare observations
//     InverseKinematics::Input input;
//     for (const auto& imu : imuData) {
//         // find index of that corresponds to the synchronized frame
//         int id = imu.first;
//         int syncIndex = -1;
//         for (int i = 0; i < imu.second.size(); ++i) {
//             if (tMin == imu.second[i].t) {
//                 syncIndex = i;
//                 break;
//             }
//         }
//         if (syncIndex == -1) {
//             // THROW_EXCEPTION("imu data not synchronized");
//             syncIndex = 0; // just for demo
//         }

//         // get synchronized data frame
//         auto data = imu.second[syncIndex];
//         input.t = data.t;
//         auto q = data.quaternion;
//         // transform from NGIMU earth frame to OpenSim reference system
//         input.imuObservations.push_back(
//                 SimTK::Rotation(SimTK::Quaternion(-q.q1, q.q2, q.q4, -q.q3))
//                 * SimTK::Rotation(-SimTK::Pi / 2, SimTK::CoordinateAxis(0)));
//         // cout << id << " " << input.t << " " << q << endl;
//     }
//     // cout << endl;
//     return input;
// }