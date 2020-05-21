/**
 * @file NGIMU.h
 *
 * \brief TODO
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef NGIMU_H
#define NGIMU_H

#include "CircularBuffer.h"
#include "Simulation.h"
#include "internal/NGIMUExports.h"
#include "ip/UdpSocket.h"
#include "osc/OscPacketListener.h"

#include <iostream>
#include <memory>

namespace OpenSimRT {

/**
 * \brief TODO
 */
struct NGIMU_API IMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
    };
    double t;
    Quaternion quaternion;
};

std::ostream& operator<<(std::ostream& os, const IMUData::Quaternion& q) {
    return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
}

class NGIMU_API ListenerAdapter {
 protected:
    virtual ~ListenerAdapter(){};

    CircularBuffer<1024, IMUData> buffer;
    unsigned int frameRate;
    unsigned long frameNumber{0};
};

/**
 * \brief TODO
 */
class NGIMU_API NGIMUListener : public osc::OscPacketListener,
                                public ListenerAdapter {
 protected:
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};

/**
 * @brief Interface for manager implementations for different osc libraries/imu
 * hardware
 *
 */
class NGIMU_API Manager {
 public:
    // dispatch
    inline void startListeners(const std::vector<std::string>& ips,
                               const std::vector<int> ports) {
        m_Manager->startListenersImp(ips, ports);
    }

    // inline InverseKinematics::Input getObservations() {
    //     m_Manager->getObservationsImp();
    // }

 protected:
    Manager() noexcept = default;
    Manager(const Manager&) = delete;
    Manager& operator=(const Manager&) = delete;
    virtual ~Manager() = default;

    // override for different implementations
    virtual void startListenersImp(const std::vector<std::string>& ips,
                                   const std::vector<int>& ports) = 0;
    // virtual InverseKinematics::Input getObservationsImp() = 0;
    Manager* m_Manager;
    std::vector<ListenerAdapter*> listeners;
};

class NGIMU_API NGIMUManager : public Manager {
 public:
    NGIMUManager();

    void setupIMUs (const std::vector<std::string>& remoteIPs, const std::vector<int>& remotePorts,
                    const std::string& localIP, const std::vector<int>& localPorts);
 protected:
    virtual void startListenersImp(const std::vector<std::string>& ips,
                                   const std::vector<int>& ports) override;
    // virtual InverseKinematics::Input getObservationsImp() override;

 private:
    SocketReceiveMultiplexer mux;
    std::vector<UdpSocket*> udpSockets;
    // std::vector<NGIMUListener*> listeners;
};

} // namespace OpenSimRT
#endif
