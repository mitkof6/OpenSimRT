/**
 * @file CircularBuffer.h
 *
 * \brief Implementation of a thread safe circular buffer.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 * @contribution Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "Exception.h"

#include <algorithm>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

namespace OpenSimRT {

/**
 * \brief A thread safe circular buffer.
 */
template <int history, typename T> class CircularBuffer {
 public:
    CircularBuffer() {
        current = 0;
        startOver = false;
        newValue = false;
        buffer.resize(history);
    }

    bool notEmpty(int M) {
        if (startOver && history >= M) {
            return true;
        } else if (!startOver && current >= M) {
            return true;
        } else {
            return false;
        }
    }

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

    std::vector<T> get(int M, bool reverseOrder = false) {
        if (M <= 0 || M > history) {
            THROW_EXCEPTION("M should be between [1, history]");
        }
        // lock
        std::unique_lock<std::mutex> lock(monitor);
        // check if data are available to proceed
        bufferNotEmpty.wait(lock,
                            [&]() { return notEmpty(M) && newValue == true; });
        newValue = false; // when buffer is no longer empty, condition variable
                          // is no longer in "wait" state, and the cosumer
                          // thread draws data from the buffer until the buffer
                          // is empty again.

        // if not empty get data
        std::vector<T> result;
        result.resize(M);
        int index = current - 1;
        for (int i = 0; i < M; ++i) { // order of execution matters
            if (index < 0) {
                index = history - 1;
            }
            result[i] = buffer[index];
            index--;
        }
        if (reverseOrder) {
            std::reverse(result.begin(), result.end());
        }
        return result;
    }
 private:
    int current;
    bool startOver;
    bool newValue;
    std::vector<T> buffer;
    std::mutex monitor;
    std::condition_variable bufferNotEmpty;
};

} // namespace OpenSimRT

#endif
