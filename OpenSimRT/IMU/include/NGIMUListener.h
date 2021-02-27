/**
 * @file NGIMUListener.h
 *
 * @brief Concrete implementation of the IMUListener for the NGIMU units
 * [https://x-io.co.uk/ngimu/]. The NGIMU communication is based on the OSC
 * protocol.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "IMUListener.h"
#include "NGIMUData.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscPacketListener.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <bitset>

namespace OpenSimRT {
/**
 * @brief xio NGIMU listener implementation.
 *
 */
class IMU_API NGIMUListener : public osc::OscPacketListener,
                              public ListenerAdapter<NGIMUData> {
 public:
    NGIMUListener();            // ctor
    ~NGIMUListener() = default; // dtor
    osc::uint64 timeTag;        // timeTag of the received bundle

 protected:
    /**
     * Overrides the ProcessBundle function from the oscpack library. Unpacks
     * received bundle until reaches a message.
     */
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;

    /**
     * Overrides the ProcessMessage function from the oscpack library. Processes
     * the received message and stores the data. When all values in a NGIMUData
     * object are received, the data are push to the buffer of the NGIMU driver.
     */
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;

 private:
    NGIMUData data;
    std::bitset<4> bundleReadyFlags; // number of bitset flags equal to the
                                     // message addresses required to fill a
                                     // NGIMUData object
};
} // namespace OpenSimRT
