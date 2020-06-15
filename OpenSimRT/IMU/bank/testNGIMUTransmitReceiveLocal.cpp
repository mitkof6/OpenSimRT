#include "INIReader.h"
#include "Manager.h"
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
            if (std::strcmp(m.AddressPattern(), "/quaternion") == 0) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float q1, q2, q3, q4;
                args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
                auto quaternion = IMUData::Quaternion{q1, q2, q3, q4};

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
    auto section = "TEST_NGIMU";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto imuDataDir = subjectDir + ini.getString(section, "IMU_DATA_DIR", "");
    auto imuSettingsXML =
            subjectDir + ini.getString(section, "IMU_SETTINGS_XML", "");

    auto MASTER_IP = ini.getString(section, "MASTER_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto RECEIVE_PORTS = ini.getVector(section, "RECEIVE_PORTS", vector<int>());
    // int FRAME_RATE = ini.getInteger(section, "FRAME_RATE", 0);

    // setup data
    const auto xsensSettings = XsensDataReaderSettings(imuSettingsXML);
    const auto xsens = XsensDataReader(xsensSettings);
    const auto& dataTables = xsens.read(imuDataDir);

    // all data tables
    const auto& quatTable = xsens.getOrientationsTable(dataTables);
    const auto& magTable = xsens.getMagneticHeadingTable(dataTables);
    const auto& accelTable = xsens.getLinearAccelerationsTable(dataTables);
    const auto& angVelTable = xsens.getAngularVelocityTable(dataTables);

    // time
    const auto& time = quatTable.getIndependentColumn();

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

    // transmitter
    auto startTransmit = [time, quatTable](string ip, vector<int> ports) {
        // send
        char buffer[OUTPUT_BUFFER_SIZE];
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

        for (const auto& t : time) {
            // data
            for (int i = 0; i < quatTable.getNumColumns(); ++i) {
                const auto& q = quatTable.getRow(t)[i];
                UdpTransmitSocket socket(IpEndpointName(ip.c_str(), ports[i]));

                p.Clear();
                p << osc::BeginBundleImmediate
                  << osc::BeginMessage("/quaternion") << (float) q[0]
                  << (float) q[1] << (float) q[2] << (float) q[3]
                  << osc::EndMessage << osc::EndBundle;
                // cout << "Transmit package" << endl;
                socket.Send(p.Data(), p.Size());
            }
        }
    };

    thread transmit(startTransmit, MASTER_IP, SEND_PORTS);
    thread listen(startListener, "192.168.2.8", vector<int>{8000});
    transmit.join();
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
