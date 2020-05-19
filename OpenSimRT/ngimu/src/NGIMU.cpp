#include "NGIMU.h"

#include "ip/UdpSocket.h"
#include "osc/OscReceivedElements.h"

#include <cstring>
#include <thread>
#include <memory>

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;


/*******************************************************************************/

Manager::Manager(string masterIP, vector<int> ports) {
}

void Manager::startListener(string ip, int port) {
    // start new listener
    auto imu = new NGIMUListener();
    imu->manager = this;

    // keep listener in list
    imus.insert({port, imu});

    // create socket and start listening
    UdpListeningReceiveSocket socket(IpEndpointName(ip.c_str(), port), imu);
    cout << "start listen on port: " << port << endl;
    socket.RunUntilSigInt();
}


/*******************************************************************************/

void NGIMUListener::ProcessMessage(const ReceivedMessage& m,
                                   const IpEndpointName& remoteEndpoint) {
    try {
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            // double t = frameNumber * 1.0 / frameRate;
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            auto quaternion = NGIMUData::Quaternion{q1, q2, q3, q4};
            // data.add({t, quaternion});
            // frameNumber++;
        }
    } catch (Exception& e) {
        cout << "error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}
