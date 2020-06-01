#include "IMUListener.h"

#include "osc/OscOutboundPacketStream.h"

using namespace OpenSimRT;
using namespace osc;
using namespace std;

void ListenerAdapter::pushDataToManagerBuffer(const int& port,
                                              const IMUData& input) {
    manager->buffer[port].set(input);
}

void NGIMUListener::ProcessBundle(const ReceivedBundle& b,
                                  const IpEndpointName& remoteEndpoint) {
    // ngimu sends a single bundle with a single message and a time-tag.
    // time-tags for nested bundles are thus ignored in this case for now
    timeTag = TimeTag(b.TimeTag());
    for (ReceivedBundle::const_iterator i = b.ElementsBegin();
         i != b.ElementsEnd(); ++i) {
        if (i->IsBundle())
            ProcessBundle(ReceivedBundle(*i), remoteEndpoint);
        else
            ProcessMessage(ReceivedMessage(*i), remoteEndpoint);
    }
}

void NGIMUListener::ProcessMessage(const ReceivedMessage& m,
                                   const IpEndpointName& remoteEndpoint) {
    try {
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            auto quaternion = IMUData::Quaternion{q1, q2, q3, q4};

            pushDataToManagerBuffer(port, {double(timeTag), quaternion});

            // cout << port << ", " << timeTag << " /quaternion " << q1
            //      << ", " << q2 << ", " << q3 << ", " << q4 << endl;
        }
    } catch (Exception& e) {
        cout << "error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}
