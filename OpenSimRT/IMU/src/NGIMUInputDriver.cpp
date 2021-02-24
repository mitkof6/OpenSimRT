#include "NGIMUInputDriver.h"

#include "NGIMUListener.h"

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;
using namespace OpenSim;

// size of stream buffer
#define OUTPUT_BUFFER_SIZE 1024

/**
 * Type-agnostic variadic template function for sending messages to
 * remote IP.
 */
template <typename... Args>
void sendMessage(UdpTransmitSocket& socket, const std::string& command,
                 Args&&... args) {
    // create tuple from args to allow different types
    using ArgsTuple = std::tuple<Args...>;
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

NGIMUInputDriver::NGIMUInputDriver(const std::vector<std::string>& imuLabels,
                                   const std::vector<std::string>& ips,
                                   const std::vector<int>& ports)
        : NGIMUInputDriver() {
    setupInput(imuLabels, ips, ports); // start streaming
}

void NGIMUInputDriver::setupInput(const std::vector<std::string>& imuLabels,
                                  const std::vector<std::string>& ips,
                                  const std::vector<int>& ports) {
    for (int i = 0; i < ports.size(); ++i) {
        // create new listeners and sockets
        listeners.push_back(make_shared<NGIMUListener>());
        udpSockets.push_back(make_unique<UdpSocket>());

        // assign ports and manager to listeners
        listeners[i]->name = imuLabels[i];
        listeners[i]->port = ports[i];
        listeners[i]->driver.reset(this);

        // initialize manager buffer
        buffer[ports[i]] =
                make_unique<CircularBuffer<CIRCULAR_BUFFER_SIZE, NGIMUData>>();
    }
}

void NGIMUInputDriver::setupTransmitters(
        const std::vector<std::string>& remoteIPs,
        const std::vector<int>& remotePorts, const std::string& localIP,
        const std::vector<int>& localPorts) {
    for (int i = 0; i < remoteIPs.size(); ++i) {
        // create socket
        UdpTransmitSocket socket(
                IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // send commands to imu
        sendMessage(socket, "/time",
                    TimeTag(tp2ntp(system_clock::now())));     // set IMU time
        sendMessage(socket, "/wifi/send/ip", localIP.c_str()); // pc ip
        sendMessage(socket, "/wifi/send/port", localPorts[i]); // port
        sendMessage(socket, "/wifi/client/lowpower", false);
        sendMessage(socket, "/rate/sensors", 60); // send rate is constant
        sendMessage(socket, "/rate/quaternion", 60);
        sendMessage(socket, "/rate/linear", 60);
        sendMessage(socket, "/rate/altitude", 60);
        sendMessage(socket, "/ahrs/magnetometer", false);
        sendMessage(socket, "/wifi/synchronisation/enabled", false);
        sendMessage(socket, "/identify"); // bling!
    }
}

void NGIMUInputDriver::startListening() {
    for (int i = 0; i < listeners.size(); ++i) {
        // get IP and port info from listener
        const auto& ip = listeners[i]->ip;
        const auto& port = listeners[i]->port;

        // bind socket, attach to mux, and run
        udpSockets[i]->Bind(IpEndpointName(ip.c_str(), port));
        mux.AttachSocketListener(
                udpSockets[i].get(),
                dynamic_cast<PacketListener*>(listeners[i].get()));
        std::cout << "Start Listening on port: " << port << std::endl;
    }
    // start listening..
    mux.RunUntilSigInt();
}

void NGIMUInputDriver::stopListening() { mux.Break(); }

NGIMUInputDriver::IMUDataList NGIMUInputDriver::getData() const {
    IMUDataList list;
    for (const auto& listener : listeners) {
        list.push_back(buffer[listener->port]->get(CIRCULAR_BUFFER_SIZE)[0]);
    }
    return list;
}

// transform all imu dataFrames into a single vector
SimTK::Vector NGIMUInputDriver::asVector(const IMUDataList& list) {
    int n = NGIMUData::size();
    int m = list.size();
    int i = 0;
    SimTK::Vector v(n * m);
    for (const auto& data : list) {
        v(i, n) = data.asVector();
        i += n;
    }
    return v;
}

NGIMUInputDriver::IMUDataList
NGIMUInputDriver::fromVector(const SimTK::Vector& v) {
    IMUDataList list;
    NGIMUData data;
    int n = NGIMUData::size();
    for (int i = 0; i < v.size(); i += n) {
        data.fromVector(v(i, n));
        list.push_back(data);
    }
    return list;
}

NGIMUInputDriver::DataPack
NGIMUInputDriver::asPack(const IMUDataList& imuDataFrame) {
    DataPack pack;
    for (const auto& imuData : imuDataFrame) {
        for (const auto& p : imuData.getAsPack()) { pack.push_back(p); }
    }
    return pack;
}

TimeSeriesTable NGIMUInputDriver::initializeLogger() const {
    vector<string> suffixes = {
            "_q1",       "_q2",       "_q3",      "_q4",        "_ax",
            "_ay",       "_az",       "_gx",      "_gy",        "_gz",
            "_mx",       "_my",       "_mz",      "_barometer", "_linAcc_x",
            "_linAcc_y", "_linAcc_z", "_altitude"};

    // create column names for each combination of imu names and measurement
    // suffixes
    vector<string> columnNames;
    for (const auto& listener : listeners) {
        for (const auto& suffix : suffixes) {
            columnNames.push_back(listener->name + suffix);
        }
    }

    // return table
    TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}
