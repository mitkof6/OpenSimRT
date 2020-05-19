/**
 * @file NGIMU.h
 *
 * \brief TODO
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef NGIMU_H
#define NGIMU_H

#include "internal/NGIMUExports.h"
#include "osc/OscPacketListener.h"

#include <CircularBuffer.h>
#include <InverseKinematics.h>
#include <atomic>
#include <iostream>
#include <memory>

namespace OpenSimRT {
// forward declaration
class Manager;

/**
 * \brief TODO
 */
struct NGIMU_API NGIMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
    };
    double t;
    Quaternion quaternion;
};

std::ostream& operator<<(std::ostream& os, const NGIMUData::Quaternion& q) {
    return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
}

/**
 * \brief TODO
 */
class NGIMU_API NGIMUListener : public osc::OscPacketListener {
 public:
    Manager* manager;

 protected:
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};

/**
 * \brief TODO
 */
class NGIMU_API Manager {
 public:
    std::map<int, NGIMUListener*> imus;

 public:
    Manager(std::string masterIP, std::vector<int> ports);
    // InverseKinematics::Input getObservations();
    void getMeasurement(const NGIMUListener* listener) {
        m_Manager->getMeasurementImp(listener);
    };

 protected:
    virtual void getMeasurementImp(const NGIMUListener* listener) = 0;

 private:
    void startListener(std::string ip, int port);
    std::unique_ptr<Manager> m_Manager; // singleton
};
} // namespace OpenSimRT
#endif
