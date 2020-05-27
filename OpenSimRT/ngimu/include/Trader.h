#ifndef TRADER_H
#define TRADER_H

/* #include "internal/IMUExports.h" */

#include <chrono>
#include <future>
#include <memory>
/* #include <type_traits> */

namespace OpenSimRT {

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
} // namespace OpenSimRT
#endif
