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
#include "osc/OscOutboundPacketStream.h"
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

#define OUTPUT_BUFFER_SIZE 1024

class ExamplePacketListener : public osc::OscPacketListener {
 public:
    int _id;
    ExamplePacketListener(const int& id)
            : OscPacketListener(), _id(id){};

 protected:
    virtual void ProcessMessage(const osc::ReceivedMessage& m,
                                const IpEndpointName& remoteEndpoint) {
        (void) remoteEndpoint; // suppress unused parameter warning
        try {
            cout << m.AddressPattern() << endl;
            if (std::strcmp(m.AddressPattern(), "/quaternion") == 0) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float q1, q2, q3, q4;
                args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
                auto quaternion = NGIMUData::Quaternion{q1, q2, q3, q4};

                cout << "Port: " << _id << " /quaternion: " << q1 << ", " << q2 << ", " << q3
                     << ", " << q4 << endl;
            }
        } catch (osc::Exception& e) {
            // any parsing errors such as unexpected argument types, or
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: " << m.AddressPattern()
                      << ": " << e.what() << "\n";
        }
    }
};

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_NGIMU_TRANSMIT";

    auto PC_ADDRESSES = ini.getString(section, "PC_ADDRESSES", "0.0.0.0");
    auto IMU_CLIENT_ADDRESSES = ini.getString(section, "IMU_CLIENT_ADDRESSES", "0.0.0.0");
    auto IMU_UDP_ADDRESSES = ini.getString(section, "IMU_UDP_ADDRESSES", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto RECEIVE_PORTS = ini.getVector(section, "RECEIVE_PORTS", vector<int>());

    // message init
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    // transmitter
    auto startTransmit = [&](string ip, vector<int> ports) {
        for (const auto& port : ports) {
            UdpTransmitSocket socket(IpEndpointName(ip.c_str(), port));

            // p.Clear();
            // p << osc::BeginBundleImmediate << osc::BeginMessage("wifi/mode")
            //   << 0 << osc::EndMessage << osc::EndBundle;
            // socket.Send(p.Data(), p.Size());

            // p.Clear();
            // p << osc::BeginBundleImmediate << osc::BeginMessage("wifi/antenna")
            //   << 1 << osc::EndMessage << osc::EndBundle;
            // socket.Send(p.Data(), p.Size());

            p.Clear();
            p << osc::BeginBundleImmediate
              << osc::BeginMessage("/wifi/send/ip") << IMU_UDP_ADDRESSES.c_str()
              << osc::EndMessage << osc::EndBundle;
            socket.Send(p.Data(), p.Size());

            p.Clear();
            p << osc::BeginBundleImmediate
              << osc::BeginMessage("/wifi/send/port") << port
              << osc::EndMessage << osc::EndBundle;
            socket.Send(p.Data(), p.Size());

            p.Clear();
            p << osc::BeginBundleImmediate << osc::BeginMessage("/identify")
              << osc::EndMessage << osc::EndBundle;
            socket.Send(p.Data(), p.Size());
        }
    };

    auto listeners = vector<ExamplePacketListener*>();
    auto sockets = vector<UdpSocket*>();

    // listener
    auto startListener = [&](string ip, vector<int> ports) {
        auto mux = SocketReceiveMultiplexer();
        int i = 0;
        for (const auto& port : ports) {
            listeners.push_back(new ExamplePacketListener(port));
            sockets.push_back(
                    new UdpReceiveSocket(IpEndpointName(ip.c_str(), port)));

            mux.AttachSocketListener(sockets[i], listeners[i]);
            cout << "start listen on port: " << port << endl;
            i += 1;
        }
        mux.RunUntilSigInt();
    };

    startTransmit(IMU_CLIENT_ADDRESSES, SEND_PORTS); // works
    startListener(IMU_UDP_ADDRESSES, RECEIVE_PORTS);
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
