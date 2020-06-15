#ifndef LISTENER_H
#define LISTENER_H

#include "Manager.h"
#include "internal/IMUExports.h"
#include "osc/OscPacketListener.h"
#include "osc/OscTypes.h"

#include <iostream>

namespace OpenSimRT {

/**
 * @brief Interface class for IMU implementations.
 *
 */
class IMU_API ListenerAdapter {
 public:
    Manager* manager; // pointer to base class manager
    int port;
    std::string ip;

    // push data to base class manager buffer
    void pushDataToManagerBuffer(const int& id, const IMUData&);

 protected:
    virtual ~ListenerAdapter(){};
};

/**
 * @brief xio NGIMU listener implementation
 *
 */
class IMU_API NGIMUListener : public osc::OscPacketListener,
                              public ListenerAdapter {
 public:
    osc::TimeTag timeTag; // TODO time is measured from 1970

 protected:
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};

} // namespace OpenSimRT
#endif
