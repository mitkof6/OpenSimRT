/**
 * @file SlidingWindow.h
 *
 * \brief Basic implementation of a sliding window. Computes the mean value of a
 * fixed n-sized buffer.
 *
 * TODO: This implementation is very primitive and should be upgraded later on.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#ifndef SLIDING_WINDOW_H
#define SLIDING_WINDOW_H

#include "internal/CommonExports.h"

#include <SimTKcommon.h>
#include <numeric>
#include <vector>

/** @brief Basic sliding window implementation. New data are pushed in-front of
 * a fixed sized buffer, and old data are discarded. The size of the window is
 * determined from the number of elements passed in 'init()' member function or
 * by explicitely setting the size with 'setSize()' member function. The mean
 * value can be computed using the 'mean()' member function. */
template <typename T> struct SlidingWindow {
    std::vector<T> data;  // sliding window data
    std::size_t capacity; // sliding window size

    // set initial values
    void init(std::vector<T>&& aData) {
        data = std::forward<std::vector<T>>(aData);
        capacity = data.size();
    }

    // insert element
    void insert(const T& x) {
        if (data.size() == capacity) data.erase(data.begin());
        data.push_back(x);
    }

    // reserve space in memory
    void setSize(const std::size_t& size) {
        capacity = size;
        data.reserve(size);
    }

    // compute mean value of the window
    T mean() {
        return 1.0 * std::accumulate(data.begin(), data.end(), T()) /
               int(data.size());
    }
};
#endif /* SLIDING_WINDOW_H */
