#include "Manager.h"

#include "IMUListener.h"
#include "InverseKinematics.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <iterator>
#include <tuple>
#include <type_traits>
#include <utility>

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;

#define OUTPUT_BUFFER_SIZE 1024

/*******************************************************************************/

// operator overload
std::ostream& operator<<(std::ostream& os, const IMUData::Quaternion& q) {
    return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
}

/**
 * @brief Type-agnostic variadic template function for sending messages to remote IP.
 */
template <typename... Args>
void sendMessage(UdpTransmitSocket& socket, const std::string& command,
                 Args&&... args) {
    // create tuple from args to allow different types
    using ArgsTuple = std::tuple<std::decay_t<Args>...>;
    ArgsTuple argTuple = {std::forward<Args>(args)...};

    // initialize message stream
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    // create the message...
    p << osc::BeginMessage(command.c_str());
    std::apply([&p](const Args&... arg) { ((p << arg), ...); }, argTuple);
    p << osc::EndMessage;

    // ...and send it
    socket.Send(p.Data(), p.Size());
}

/*******************************************************************************/
NGIMUManager::NGIMUManager() { m_Manager = this; }

NGIMUManager::NGIMUManager(const std::vector<std::string>& ips,
                           const std::vector<int>& ports)
        : NGIMUManager() {
    setupListeners(ips, ports);
}

void NGIMUManager::setupListeners(const std::vector<std::string>& ips,
                                  const std::vector<int>& ports) {
    for (int i = 0; i < ports.size(); ++i) {
        // create new listeners and sockets
        listeners.push_back(new NGIMUListener());
        udpSockets.push_back(new UdpSocket());

        // assign ports and manager to listeners
        listeners[i]->port = ports[i];
        listeners[i]->manager = this;

        // initialize manager buffer
        buffer[ports[i]] = new DoubleBuffer<IMUData>();
    }
}

void NGIMUManager::setupTransmitters(const std::vector<std::string>& remoteIPs,
                                     const std::vector<int>& remotePorts,
                                     const std::string& localIP,
                                     const std::vector<int>& localPorts) {
    for (int i = 0; i < remoteIPs.size(); ++i) {
        // create socket
        UdpTransmitSocket socket(
                IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // send commands to imu
        sendMessage(socket, "/wifi/send/ip", localIP.c_str());
        sendMessage(socket, "/wifi/send/port", localPorts[i]);
        sendMessage(socket, "/identify"); // bling!
    }
}

void NGIMUManager::startListenersImp() {
    for (int i = 0; i < listeners.size(); ++i) {
        // get IP and port info from listener
        const auto& ip = listeners[i]->ip;
        const auto& port = listeners[i]->port;

        // bind socket, attach to mux, and run
        udpSockets[i]->Bind(IpEndpointName(ip.c_str(), port));
        mux.AttachSocketListener(udpSockets[i],
                                 dynamic_cast<PacketListener*>(listeners[i]));
        std::cout << "Start Listening on port: " << port << std::endl;
    }
    // start listening..
    mux.RunUntilSigInt();
}

InverseKinematics::Input NGIMUManager::getObservationsImp() {
    InverseKinematics::Input input;
    for (auto& mapElement : buffer) {
        // thread-safe fetch data from buffer
        auto imuData = mapElement.second->get();

        // set aliases
        const auto& t = imuData.t;
        const auto& q = imuData.quaternion;

        // transform from NGIMU earth frame to OpenSim reference system
        input.imuObservations.push_back(
                SimTK::Rotation(SimTK::Quaternion(-q.q1, q.q2, q.q4, -q.q3)) *
                SimTK::Rotation(-SimTK::Pi / 2, SimTK::CoordinateAxis(0)));
    }
    return input;
}
