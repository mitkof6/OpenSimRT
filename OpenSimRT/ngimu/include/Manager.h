#ifndef MANAGER_H
#define MANAGER_H

/* #include "IMUListener.h" */
#include "Simulation.h"
#include "Trader.h"
#include "internal/IMUExports.h"
#include "ip/UdpSocket.h"

#include <iostream>
#include <string>
#include <vector>

namespace OpenSimRT {

class ListenerAdapter;

struct IMU_API IMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
    };
    double t;
    Quaternion quaternion;
};

/**
 * @brief Interface for manager implementations for different osc libraries/imu
 * hardware
 *
 */
class IMU_API Manager {
    friend class ListenerAdapter;

 public:
    // dispatch
    inline void startListeners() { m_Manager->startListenersImp(); }

    //* returning copy value has issues i can't explain...
    inline void getObservations(InverseKinematics::Input& input) {
        m_Manager->getObservationsImp(input);
    }

 protected:
    Manager() noexcept = default;
    Manager& operator=(const Manager&) = delete;
    Manager(const Manager&) = delete;
    virtual ~Manager() = default;

    // override for different ipmplementations
    virtual void startListenersImp() = 0;
    virtual void getObservationsImp(InverseKinematics::Input& input) = 0;

    std::vector<ListenerAdapter*> listeners;
    Manager* m_Manager;

    std::map<int, PromiseAndFuture<IMUData>> data;
};

class IMU_API NGIMUManager : public Manager {
 public:
    NGIMUManager();
    NGIMUManager(const std::vector<std::string>&, const std::vector<int>&);

    void setupListeners(const std::vector<std::string>&,
                        const std::vector<int>&);
    void setupTransmitters(const std::vector<std::string>& remoteIPs,
                           const std::vector<int>& remotePorts,
                           const std::string& localIP,
                           const std::vector<int>& localPorts);

 protected:
    virtual void startListenersImp() override;
    virtual void getObservationsImp(InverseKinematics::Input& input) override;

 private:
    SocketReceiveMultiplexer mux;
    std::vector<UdpSocket*> udpSockets;
};
} // namespace OpenSimRT
#endif
