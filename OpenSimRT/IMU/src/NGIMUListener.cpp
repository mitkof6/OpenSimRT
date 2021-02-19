#include "NGIMUListener.h"

#include "Exception.h"

using namespace std;
using namespace osc;
using namespace OpenSimRT;

NGIMUListener::NGIMUListener() { bundleReadyFlags.reset(); }

void NGIMUListener::ProcessBundle(const ReceivedBundle& b,
                                  const IpEndpointName& remoteEndpoint) {
    timeTag = b.TimeTag();
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
    // every time this function runs, it processes only one message, i.e.
    // /quaternions or /sensors, etc.
    try {
        // the 32-first bits are the #seconds since Jan 1 1900 00:00 GMT. To
        // start from 1970 (Unix epoch), remove the 70 years in seconds. Add 3
        // hours since local timezone is GMT+03:00 (Greece)
        auto time = (timeTag >> 32) +
                    double(timeTag & 0xFFFFFFFF) / PICOSECS_RESOLUTION_BIN -
                    RFC_PROTOCOL - GMT_LOCAL_TIMEZONE; // TODO find a better way

        // get /quaternions
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            data.quaternion = NGIMUData::Quaternion{
                    SimTK::Quaternion(q1, q2, q3, q4), time};
            bundleReadyFlags.set(0);
        }

        // get /sensors
        if (strcmp(m.AddressPattern(), "/sensors") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float gX, gY, gZ; // gyroscope
            float aX, aY, aZ; // acceleration
            float mX, mY, mZ; // magnetometer
            float barometer;
            args >> gX >> gY >> gZ >> aX >> aY >> aZ >> mX >> mY >> mZ >>
                    barometer >> osc::EndMessage;
            data.sensors.acceleration = SimTK::Vec3(aX, aY, aZ);
            data.sensors.gyroscope = SimTK::Vec3(gX, gY, gZ);
            data.sensors.magnetometer = SimTK::Vec3(mX, mY, mZ);
            data.sensors.barometer = SimTK::Vec1(barometer);
            data.sensors.timeStamp = time;
            bundleReadyFlags.set(1);
        }

        // get linera acceleration
        if (strcmp(m.AddressPattern(), "/linear") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float ax, ay, az; // linear acceleration
            args >> ax >> ay >> az >> osc::EndMessage;
            data.linear = NGIMUData::LinearAcceleration{SimTK::Vec3(ax, ay, az),
                                                        time};
            bundleReadyFlags.set(2);
        }

        // get altitude
        if (strcmp(m.AddressPattern(), "/altitude") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float x;
            args >> x >> osc::EndMessage;
            data.altitude = NGIMUData::Altitude{SimTK::Vec1(x), time};
            bundleReadyFlags.set(3);
        }

        // TODO add more patterns if necessary
        // ...
        //

        // stop dat transmition
        if (strcmp(m.AddressPattern(), "/button") == 0) {
            driver->stopListening();
            THROW_EXCEPTION(
                    "Destruction Button is Pressed! Goodbye cruel word!");
        }

        // when all messages are processed, push IMU bundle to buffer
        if (bundleReadyFlags.all()) { pushDataToManagerBuffer(port, data); }

    } catch (Exception& e) {
        cout << "Error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}
