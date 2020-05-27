#include "Manager.h"
#include "IMUListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"

// #include <algorithm>

// #include "range/v3/action/action.hpp"

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;

#define OUTPUT_BUFFER_SIZE 1024

/*******************************************************************************/
std::ostream& operator<<(std::ostream& os, const IMUData::Quaternion& q) {
        return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
    }

NGIMUManager::NGIMUManager() { m_Manager = this; }

NGIMUManager::NGIMUManager(const std::vector<std::string>& ips,
                           const std::vector<int>& ports) {
    m_Manager = this;
    setupListeners(ips, ports);
}

void NGIMUManager::setupListeners(const std::vector<std::string>& ips,
                                  const std::vector<int>& ports) {
    for (int i = 0; i < ports.size(); ++i) {
        listeners.push_back(new NGIMUListener());
        udpSockets.push_back(new UdpSocket());

        listeners[i]->port = ports[i];
        listeners[i]->manager = this;

        data[ports[i]].reset();
    }
}

void NGIMUManager::setupTransmitters(const std::vector<std::string>& remoteIPs,
                             const std::vector<int>& remotePorts,
                             const std::string& localIP,
                             const std::vector<int>& localPorts) {
    // message init
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    for (int i = 0; i < remoteIPs.size(); ++i) {
        UdpTransmitSocket socket(
                IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // what ip imu shall send to
        p.Clear();
        p << osc::BeginMessage("/wifi/send/ip") << localIP.c_str()
          << osc::EndMessage;
        socket.Send(p.Data(), p.Size());

        // what port to send
        p.Clear();
        p << osc::BeginMessage("/wifi/send/port") << localPorts[i]
          << osc::EndMessage;
        socket.Send(p.Data(), p.Size());

        // bling!
        p.Clear();
        p << osc::BeginMessage("/identify") << osc::EndMessage;
        socket.Send(p.Data(), p.Size());
    }
}

void NGIMUManager::startListenersImp() {
    for (int i = 0; i < listeners.size(); ++i) {
        const auto& ip = listeners[i]->ip;
        const auto& port = listeners[i]->port;

        // bind socket, attach to mux, and run
        udpSockets[i]->Bind(IpEndpointName(ip.c_str(), port));
        mux.AttachSocketListener(udpSockets[i],
                                 dynamic_cast<PacketListener*>(listeners[i]));
        std::cout << "Start Listening on port: " << port << std::endl;
    }

    mux.RunUntilSigInt();
}

void NGIMUManager::getObservationsImp(InverseKinematics::Input& input) {
    for (auto& x : data) {
        auto imuData = x.second.get();

        auto& t = imuData.t;
        auto& q = imuData.quaternion;
        cout << q << endl;
        // transform from NGIMU earth frame to OpenSim reference system
        input.imuObservations.push_back(
                SimTK::Rotation(SimTK::Quaternion(-q.q1, q.q2, q.q4, -q.q3)) *
                SimTK::Rotation(-SimTK::Pi / 2, SimTK::CoordinateAxis(0)));
    }
}
