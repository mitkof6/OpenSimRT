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
 * @file SlidingWindow.h
 *
 * \brief Basic implementation of a sliding window. Computes the mean value of a
 * fixed n-sized buffer.
 *
 * TODO: This implementation is very primitive and should be upgraded later on.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "internal/CommonExports.h"
#include <SimTKcommon.h>
#include <numeric>

namespace OpenSimRT {

/** @brief Basic sliding window implementation. New data are pushed in-front of
 * a fixed sized buffer, and old data are discarded. The size of the window is
 * determined from the number of elements passed in 'init()' member function or
 * by explicitely setting the size with 'setSize()' member function. The mean
 * value can be computed using the 'mean()' member function. */
template <typename T> struct SlidingWindow {
    SimTK::Array_<T> data; // sliding window data
    std::size_t capacity;  // sliding window size

    // set initial values
    void init(SimTK::Array_<T>&& aData) {
        data = std::forward<SimTK::Array_<T>>(aData);
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
    // Determine if all elements in the window are equal to input value
    bool equal(const T& x) {
        for (const auto& e : data) {
            if (e != x) return false;
        }
        return true;
    }

    // Determine if the n first elements are equal to input value.
    bool nFirstEqual(const T& x, const size_t& n) const {
        if (n > data.size()) throw std::runtime_error("Wrong input size");
        for (size_t i = 0; i < n; ++i) {
            if (data[i] != x) return false;
        }
        return true;
    }

    // Determine if the last n elements are equal to input value.
    bool nLastEqual(const T& x, const size_t& n) const {
        if (n > data.size()) throw std::runtime_error("Wrong input size");
        for (size_t i = 0; i < n; ++i) {
            if (data[data.size() - i - 1] != x) return false;
        }
        return true;
    }
};

} // namespace OpenSimRT
