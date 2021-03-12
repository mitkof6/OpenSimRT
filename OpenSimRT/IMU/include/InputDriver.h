/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
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
    virtual IMUDataList getData() const = 0;

 protected:
    InputDriver() noexcept {};                           // ctor
    InputDriver& operator=(const InputDriver&) = delete; // deleted assign ctor
    InputDriver(const InputDriver&) = delete;            // deleted copy ctor
    virtual ~InputDriver() = default;                    // dtor

    /**
     * List of pointers to generic listeners using the adapter interface class.
     */
    std::vector<std::shared_ptr<ListenerAdapter<T>>> listeners;

    /**
     * A map with thread-safe buffers to store IMU data from each port.
     */
    mutable std::map<int,
                     std::unique_ptr<CircularBuffer<CIRCULAR_BUFFER_SIZE, T>>>
            buffer;
};

} // namespace OpenSimRT
