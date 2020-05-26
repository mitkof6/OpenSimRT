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

#include <condition_variable>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>

namespace OpenSimRT {

class Manager;
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

template <typename T> struct PromiseAndFuture {
    std::unique_ptr<std::promise<T>> m_promise;
    std::unique_ptr<std::future<T>> m_future;

    PromiseAndFuture() { reset(); }

    void reset() {
        m_promise.reset(nullptr);
        m_future.reset(nullptr);
        m_promise = std::make_unique<std::promise<T>>();
        m_future = std::make_unique<std::future<T>>(m_promise->get_future());
    }

    bool ready() {
        std::future_status status =
                m_future->wait_for(std::chrono::milliseconds(0));
        return (status == std::future_status::ready);
    }

    T get() {
        // std::lock_guard<std::mutex> lockGuard(m_mutex);
        // if (!ready()) { return false; }
        m_future->wait();
        T ret = m_future->get();
        reset();
        return ret;
    }

    bool set(T val) {
        if (ready()) { return false; }
        m_promise->set_value(val);
        return true;
    }
};

class NGIMU_API ListenerAdapter {
 public:
    Manager* manager;
    int port;
    std::string ip;

    void pushDataToManagerBuffer(const int& id, const IMUData&);

 protected:
    virtual ~ListenerAdapter(){};
};

/**
 * \brief TODO
 */
class NGIMU_API NGIMUListener : public osc::OscPacketListener,
                                public ListenerAdapter {
 public:
    osc::TimeTag timeTag;

 protected:
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};

/**
 * @brief Interface for manager implementations for different osc libraries/imu
 * hardware
 *
 */
class NGIMU_API Manager {
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

class NGIMU_API NGIMUManager : public Manager {
 public:
    NGIMUManager(const std::vector<std::string>&, const std::vector<int>&);

    void setupIMUs(const std::vector<std::string>& remoteIPs,
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
