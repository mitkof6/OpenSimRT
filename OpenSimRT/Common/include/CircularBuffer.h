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
 * @file CircularBuffer.h
 *
 * \brief Implementation of a thread safe circular buffer.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>, Filip Konstantinos
 * <filip.k@ece.upatras.gr>
 */
#pragma once

#include "Exception.h"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

namespace OpenSimRT {
/**
 * Select mode for data retrieval from the buffer.
 *
 * - ON_ENTRY allows the user user to get data from the buffer only after
 * new data have been added.
 *
 * - CONTINUOUS mode allows the user the retrieve data from the buffer
     continuously, without waiting fot new data, until the buffer is empty.
 */
enum class DataRetrievalMode { CONTINUOUS, ON_ENTRY };

/**
 * \brief A thread safe circular buffer. It is based on a circular array with
 * size equal to `history` and elements of type defined by the template
 * parameter T. New data are appended to the buffer using the `add()` function.
 * Older data are discarded when adding new data when the buffer is full.
 * Retrieving data from the buffer is performed using the `get(M)` function,
 * where M \in [1, history] is the number of the latest (newer) elements
 * inserted in the buffer. By default, retrieval is possible only if new data
 * have been appended to the buffer for real-time processing of data. To
 * retrieve data from the buffer (and terminate the 'wait' state of the consumer
 * thread) even if no new data have been added, the user must change the
 * retrieval mode. The order of the elements in the retrieved vector can be from
 * new-to-old (default) or old-to-new.
 *
 *                         Producer Thread | Consumer Thread
 *                                         |
 *                            +------------+-------------+
 *                            |            |             |
 *                  Add ----->|          Buffer          |----> Get(M)
 *                            |            |             |
 *   setDataRetrievalMode --->+------------+-------------+
 *                            <----------history--------->
 *                                           <-----M----->
 *
 * ****************************************************************************
 * Example code:
 * ****************************************************************************
 *
 * void producerFunction() { // producer thread
 *    // adds new data to the buffer
 *    {
 *       ...
 *
 *       buffer.add(data);
 *
 *       ...
 *    }
 *
 *    // When no new data are present, change the retrieval mode to notify
 *    // and unblock the buffer. Now the buffer does not wait for new data
 *    // to be added.
 *    buffer.setDataRetrievalMode(DataRetrievalMode::CONTINUOUS);
 * }
 *
 * void consumerFunction() { // consumer thread
 *     ...
 *
 *     // Retrieve data from the buffer based on the selected mode.
 *     data = buffer.get(M);
 *
 *     ...
 * }
 *
 */
template <int history, typename T> class CircularBuffer {
 public:
    CircularBuffer() {
        current = 0;
        startOver = false;
        newValue = false;
        continuousModeFlag = false;
        buffer.resize(history);
    }

    /**
     * Determine if the buffer has at least M values.
     */
    bool isSize(int M) {
        if (startOver && history >= M) {
            return true;
        } else if (!startOver && current >= M) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Set the mode for data retrieval from the buffer. Retrieval can occur
     * continuously untill the buffer is empty, or only after new data have been
     * added to the buffer. Notifies the buffer on mode change, thus it can be
     * called at any moment.
     */
    void setDataRetrievalMode(DataRetrievalMode mode) {
        if (mode == DataRetrievalMode::CONTINUOUS)
            continuousModeFlag = true;
        else
            continuousModeFlag = false;
        bufferNotEmpty.notify_one();
    }

    /**
     * Append data in the buffer.
     */
    void add(const T& value) {
        {
            // lock
            std::lock_guard<std::mutex> lock(monitor);
            // update buffer
            buffer[current] = value;
            current++;
            newValue = true;
            if (current == history) {
                current = 0;
                startOver = true;
            }
        }
        // notify after unlocking
        bufferNotEmpty.notify_one();
    }

    /**
     * Retrieve data from buffer. To retrieve data, the buffer must have at
     * least M elements, else the consumer thread is blocked until buffer is
     * filled with M values.
     */
    std::vector<T> get(int M, bool reverseOrder = false) {
        if (M <= 0 || M > history) {
            THROW_EXCEPTION("M should be between [1, history]");
        }
        // lock
        std::unique_lock<std::mutex> lock(monitor);
        // check if data are available to proceed
        bufferNotEmpty.wait(lock, [&]() {
            return isSize(M) && (continuousModeFlag.load() || newValue);
        });
        newValue = false; // when buffer is no longer empty, condition variable
                          // is no longer in "wait" state, and the cosumer
                          // thread draws data from the buffer until the buffer
                          // is empty again.

        // if not empty get data
        std::vector<T> result;
        result.resize(M);
        int index = current - 1;
        for (int i = 0; i < M; ++i) { // order of execution matters
            if (index < 0) { index = history - 1; }
            result[i] = buffer[index];
            index--;
        }
        if (reverseOrder) { std::reverse(result.begin(), result.end()); }
        return result;
    }

 private:
    int current;
    bool startOver;
    bool newValue;
    std::atomic<bool> continuousModeFlag;
    std::vector<T> buffer;
    std::mutex monitor;
    std::condition_variable bufferNotEmpty;
};

} // namespace OpenSimRT
