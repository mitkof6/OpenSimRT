/**
 * @file Utils.h
 *
 * \brief Useful utilities.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef UTILS_H
#define UTILS_H

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
 * Compare two TimeSeriesTables. Number of columns and rows must match,
 * otherwise throws an exception. Comparison is performed by computing the RMSE
 * of the columns with common labels acquired from the 'query' table. If the
 * computed RMSE of any of the match columns is larger than the given threshold
 * it throws an exception.
 *
 * @param queryTable - Query table to compare.
 *
 * @param refTable - Reference table to be compare against.
 *
 * @param threshold - Threshold value (default is 1e-5).
 */
template <typename T>
void compareTables(const OpenSim::TimeSeriesTable_<T>& queryTable,
                   const OpenSim::TimeSeriesTable_<T>& refTable,
                   const double& threshold = 1e-5) {
    if (!(queryTable.getNumRows() == refTable.getNumRows()))
        THROW_EXCEPTION("Number of rows in given tables do not match.");
    if (!(queryTable.getNumColumns() == refTable.getNumColumns()))
        THROW_EXCEPTION("Number of columns in given tables do not match.");

    // create a flatten copy of the tables
    auto queryFlat = queryTable.flatten();
    auto refFlat = refTable.flatten();

    // get column labels
    auto queryLabels = queryFlat.getColumnLabels();
    auto refLabels = refFlat.getColumnLabels();

    // find and store the indexes of the column labels in tables
    std::vector<int> mapRefToQuery;
    for (const auto& label : queryLabels) {
        auto found = std::find(refLabels.begin(), refLabels.end(), label);
        mapRefToQuery.push_back(std::distance(refLabels.begin(), found));
    }

    // compute the rmse of the matched columns
    for (size_t i = 0; i < mapRefToQuery.size(); ++i) {
        if (mapRefToQuery[i] >= 0) {
            auto queryVec = queryFlat.getDependentColumnAtIndex(i);
            auto refVec = refFlat.getDependentColumnAtIndex(mapRefToQuery[i]);
            auto rmse = sqrt((queryVec - refVec).normSqr() /
                             queryFlat.getNumRows());
            std::cout << "Column '" << queryLabels[i] << "' has RMSE = " << rmse
                      << std::endl;
            SimTK_ASSERT2_ALWAYS(
                    (rmse < threshold),
                    "Column '%s' FAILED to meet accuracy of %f RMS.",
                    queryLabels[i].c_str(), threshold);
        }
    }
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
static SimTK::Vec3 projectionOnPlane(const SimTK::Vec3& point,
                                     const SimTK::Vec3& planeOrigin,
                                     const SimTK::Vec3& planeNormal) {
    return point - SimTK::dot(point - planeOrigin, planeNormal) * planeNormal;
}

} // namespace OpenSimRT
#endif
