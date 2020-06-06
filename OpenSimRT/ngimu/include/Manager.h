#ifndef MANAGER_H
#define MANAGER_H

#include "PromiseAndFuture.h"
#include "Simulation.h"
#include "internal/IMUExports.h"
#include "ip/UdpSocket.h"

#include <iostream>
#include <string>
#include <vector>

namespace OpenSimRT {

// forward declaration
class ListenerAdapter;

// IMU data container
struct IMU_API IMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
    };
    double t;
    Quaternion quaternion;
};

/*******************************************************************************/

/**
 * @brief Base class for different manager implementations.
 *
 */
class IMU_API Manager {
    friend class ListenerAdapter;

 public:
    // interface functions used in all inherited managers
    inline void startListeners() { m_Manager->startListenersImp(); }
    inline InverseKinematics::Input getObservations() {
        return m_Manager->getObservationsImp();
    }

 protected:
    Manager() noexcept = default;
    Manager& operator=(const Manager&) = delete;
    Manager(const Manager&) = delete;
    virtual ~Manager() = default;

    // override for different implementations
    virtual void startListenersImp() = 0;
    virtual InverseKinematics::Input getObservationsImp() = 0;

    // pointers to generic listeners using the adapter interface class
    std::vector<ListenerAdapter*> listeners;
    Manager* m_Manager;

    // data buffer IMU data.
    std::map<int, PromiseAndFuture<IMUData>> buffer;
};


/**
 * @brief NGIMU Manager implementation
 */
class IMU_API NGIMUManager : public Manager {
 public:
    NGIMUManager();
    NGIMUManager(const std::vector<std::string>&, const std::vector<int>&);

    // setup communication
    void setupListeners(const std::vector<std::string>&,
                        const std::vector<int>&);
    void setupTransmitters(const std::vector<std::string>& remoteIPs,
                           const std::vector<int>& remotePorts,
                           const std::string& localIP,
                           const std::vector<int>& localPorts);

 protected:
    virtual void startListenersImp() override;
    virtual InverseKinematics::Input getObservationsImp() override;

 private:
    SocketReceiveMultiplexer mux;
    std::vector<UdpSocket*> udpSockets;
};
} // namespace OpenSimRT
#endif
