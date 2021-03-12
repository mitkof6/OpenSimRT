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
 * @file Utils.h
 *
 * \brief Useful utilities.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "Exception.h"
#include "internal/CommonExports.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace OpenSimRT {

/**
 * Converts an OpenSim::Array to std container <U> (e.g., vector<double>).
 */
template <typename T, typename U>
void osimToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.getSize();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) { dstVector.at(i) = srcArray.get(i); }
}

/**
 * Converts a Simbody (e.g., Vector) to std container (e.g., vector<double>).
 */
template <typename T, typename U>
void simtkToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.size();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) { dstVector.at(i) = srcArray[i]; }
}

/**
 * Converts <T> to string and with defined precision (in case of number).
 */
template <typename T>
std::string toString(const T& value,
                     int precision = std::numeric_limits<int>::infinity()) {
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity()) {
        oss << std::setprecision(precision);
    }
    oss << value;
    return oss.str();
}

/**
 * Separates (delimiter) the values of the std container into a single line
 * string.
 */
template <typename T>
std::string dump(const T& vec, std::string delimiter,
                 int precision = std::numeric_limits<int>::infinity()) {
    std::string row = toString(vec.at(0));
    for (int i = 1; i < vec.size(); ++i) {
        row += delimiter;
        row += toString(vec.at(i));
    }
    return row;
}

/**
 * Determine if a SimTK::Vector_<T> has finite elements.
 */
template <typename T> bool isVectorFinite(const SimTK::Vector_<T>& v) {
    for (size_t i = 0; i < v.size(); ++i) {
        if (!SimTK::isFinite(v[i])) return false;
    }
    return true;
}

/**
 * Compute the projection of a point or vector on a arbitrary plane.
 */
static SimTK::Vec3
projectionOnPlane(const SimTK::Vec3& point, const SimTK::Vec3& planeOrigin,
                  const SimTK::Vec3& planeNormal = SimTK::Vec3(0, 1, 0)) {
    return point - SimTK::dot(point - planeOrigin, planeNormal) * planeNormal;
}

/**
 * Hamilton quaternion product
 */
static inline SimTK::Quaternion operator*(const SimTK::Quaternion& a,
                                          const SimTK::Quaternion& b) {
    SimTK::Quaternion c;
    c[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    c[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    c[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    c[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    return c;
}

/**
 *  Overwrite SImTK operator~ for quaternions, to compute the conjugate
 */
static inline SimTK::Quaternion operator~(const SimTK::Quaternion& q) {
    return SimTK::Quaternion(q[0], -q[1], -q[2], -q[3]);
}

/**
 * Clip a given value if exceeds the given boundaries.
 */
template <typename T> T clip(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

} // namespace OpenSimRT
