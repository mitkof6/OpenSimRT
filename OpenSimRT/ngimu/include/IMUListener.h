#ifndef LISTENER_H
#define LISTENER_H

#include "Manager.h"
#include "internal/IMUExports.h"
#include "osc/OscPacketListener.h"
#include "osc/OscTypes.h"

#include <iostream>

namespace OpenSimRT {

class IMU_API ListenerAdapter {
 public:
    Manager* manager;
    int port;
    std::string ip;

    void pushDataToManagerBuffer(const int& id, const IMUData&);

 protected:
    virtual ~ListenerAdapter(){};
};

class IMU_API NGIMUListener : public osc::OscPacketListener,
                              public ListenerAdapter {
 public:
    osc::TimeTag timeTag;

 protected:
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};

} // namespace OpenSimRT
#endif
