#pragma once

#include <atomic>

namespace OpenSimRT {

/* Lock-free double-buffer pattern for sharing state across threads */
template <typename T> class DoubleBuffer {
 public:
    DoubleBuffer() : m_current(&m_buffers[0]), m_next(&m_buffers[1]) {
        m_flag.store(false);
    }

    // set data to the next buffer and switch pointers with the current buffer
    void set(const T& val) {
        *m_next = val;                       // set value
        m_next = m_current.exchange(m_next); // swap pointers
        m_flag.store(false);                 // set flag
    }

    // return data from the current buffer
    T get() {
        while (m_flag.exchange(true))
            ; // wait until flag is set
        return *m_current;
    }

 private:
    T m_buffers[2];
    std::atomic<T*> m_current; // buffer 1
    std::atomic<T*> m_next;    // buffer 2
    std::atomic<bool> m_flag;  // synchronization flag
};
} // namespace OpenSimRT
