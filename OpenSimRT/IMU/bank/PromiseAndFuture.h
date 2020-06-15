#ifndef PROMISE_AND_FUTURE_H
#define PROMISE_AND_FUTURE_H

#include <chrono>
#include <future>
#include <memory>

namespace OpenSimRT {

/**
 * @brief Alternative implementation on the producer consumer problem
 * for a single value of type T using a promise/future pair.
 */
template <typename T> struct PromiseAndFuture {
    // use unique_ptr to renew the promise and future
    std::unique_ptr<std::promise<T>> m_promise;
    std::unique_ptr<std::future<T>> m_future;
    std::mutex m_mutex;

    PromiseAndFuture() { reset(); }

    // reset promise and future
    void reset() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_promise.reset(nullptr);
        m_future.reset(nullptr);
        m_promise = std::make_unique<std::promise<T>>();
        m_future = std::make_unique<std::future<T>>(m_promise->get_future());
    }

    //  get future status
    bool ready() {
        // lock, otherwise segmentation-fault can happen during reset
        std::lock_guard<std::mutex> lock(m_mutex);
        std::future_status status =
                m_future->wait_for(std::chrono::milliseconds(0));
        return (status == std::future_status::ready);
    }

    // set promise value
    bool set(T val) {
        if (ready()) { return false; }
        m_promise->set_value(val);
        return true;
    }

    // get data from future
    T get() {
        m_future->wait();
        T ret = m_future->get();
        reset();
        return ret;
    }
};
} // namespace OpenSimRT
#endif