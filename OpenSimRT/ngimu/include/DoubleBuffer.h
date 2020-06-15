#pragma once

#include <atomic>

namespace OpenSimRT {

/* Lock-free double buffer */
template <typename T> class DoubleBuffer {
 public:
    DoubleBuffer() : m_current(&m_buffers[0]), m_next(&m_buffers[1]) {
        isReady.store(false);
    }

    // set data to the next buffer and switch pointers with the current buffer
    void set(const T& val) {
        *m_next = val;
        m_next = m_current.exchange(m_next);
        isReady.store(false);
    }

    // return data from the current buffer
    T get() {
        while (isReady.exchange(true))
            ;
        return *m_current;
    }

 private:
    T m_buffers[2]; // 2 buffers
    std::atomic<T*> m_current; // buffer 1
    std::atomic<T*> m_next; // buffer 2
    std::atomic<bool> isReady; // synchronization flag
};
} // namespace OpenSimRT
