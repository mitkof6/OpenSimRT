/**
 * @file InputDriver.h
 *
 * @brief Implements a based class for IMU data drivers. The interface includes
 * functions for starting and terminating IMU adapted listeners
 * (ListenerAdapter), and for receiving and storing data from each listener's
 * port to a safe-thread buffer.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "CircularBuffer.h"
#include "internal/IMUExports.h"

#include <map>
#include <vector>

// The size of buffer is 1, since in stream-like applications we need to pass
// the most recent data continuously from the producer to the consumer thread.
#define CIRCULAR_BUFFER_SIZE 1

namespace OpenSimRT {

// forward declaration
template <typename T> class ListenerAdapter;

/*******************************************************************************/

/**
 * @brief Base class for different IMU driver implementations.
 *
 */
template <typename T> class IMU_API InputDriver {
    friend class ListenerAdapter<T>;

 public:
    /**
     * Alias for the list of IMU data objects.
     */

    using IMUDataList = std::vector<T>;
    /**
     * Start listening to sockets.
     */
    virtual void startListening() = 0;

    /**
     * Stop listening to sockets.
     */
    virtual void stopListening() = 0;

    /**
     * Obtain IMU data from listeners.
     */
    virtual IMUDataList getData() = 0;

 protected:
    InputDriver() noexcept {};                           // ctor
    InputDriver& operator=(const InputDriver&) = delete; // deleted assign ctor
    InputDriver(const InputDriver&) = delete;            // deleted copy ctor
    virtual ~InputDriver() = default;                    // dtor

    /**
     * List of pointers to generic listeners using the adapter interface class.
     */
    std::vector<ListenerAdapter<T>*> listeners;

    /**
     * A map with thread-safe buffers to store IMU data from each port.
     */
    std::map<int, CircularBuffer<CIRCULAR_BUFFER_SIZE, T>*> buffer;
};

} // namespace OpenSimRT
