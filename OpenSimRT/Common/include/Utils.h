/**
 * @file Utils.h
 *
 * \brief Useful utilities.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <limits>
#include <sstream>
#include <iomanip>
#include "internal/CommonExports.h"


namespace OpenSimRT {

/**
 * Converts an OpenSim::Array to std container <U> (e.g., vector<double>).
 */
template<typename T, typename U>
void osimToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.getSize();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) {
        dstVector.at(i) = srcArray.get(i);
    }
}

/**
 * Converts a Simbody (e.g., Vector) to std container (e.g., vector<double>).
 */
template<typename T, typename U>
void simtkToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.size();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) {
        dstVector.at(i) = srcArray.get(i);
    }
}

/**
 * Converts <T> to string and with defined precision (in case of number).
 */
template<typename T>
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
template<typename T>
std::string dump(const T& vec, std::string delimiter,
                 int precision = std::numeric_limits<int>::infinity()) {
    std::string row = toString(vec.at(0));
    for (int i = 1; i < vec.size(); ++i) {
        row += delimiter;
        row += toString(vec.at(i));
    }
    return row;
}

} // namespace OpenSimRT

#endif
