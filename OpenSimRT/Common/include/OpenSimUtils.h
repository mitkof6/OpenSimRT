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
 * @file OpenSimUtils.h
 *
 * \brief Common OpenSim operations and functions.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include "Exception.h"
#include "internal/CommonExports.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {
// Type definitions from a moment arm function that accepts a vector and returns
// a matrix.
typedef SimTK::Matrix (*MomentArmFunctionT)(const SimTK::Vector& q);

struct Common_API OpenSimUtils {
    // Generates a unique identifier
    static int generateUID();
    // Extract model's coordinate names in multibody tree order.
    static std::vector<std::string>
    getCoordinateNamesInMultibodyTreeOrder(const OpenSim::Model& model);
    // Extract model's coordinate names in normal order.
    static std::vector<std::string>
    getCoordinateNames(const OpenSim::Model& model);
    // Extract model's muscle names.
    static std::vector<std::string> getMuscleNames(const OpenSim::Model& model);
    // Extract model's actuator names.
    static std::vector<std::string>
    getActuatorNames(const OpenSim::Model& model);
    // Disable actuators.
    static void disableActuators(OpenSim::Model& model);
    // Remove actuators.
    static void removeActuators(OpenSim::Model& model);
    // Get ordered generalized coordinates from storage.
    static OpenSim::TimeSeriesTable
    getMultibodyTreeOrderedCoordinatesFromStorage(const OpenSim::Model& model,
                                                  const std::string stoFilePath,
                                                  double samplingInterval);
    // Load moment arm from a dynamic library
    static MomentArmFunctionT
    getMomentArmFromDynamicLibrary(const OpenSim::Model& model,
                                   std::string libraryPath);

    /**
     * Update the state of the osim model by assigning the `q` and `qDot`
     * vectors, containing the generalized coordinates and generalized
     * velocities, respectively, in the model's coordinates.
     */
    static void updateState(const OpenSim::Model& model, SimTK::State& state,
                            const SimTK::Vector& q, const SimTK::Vector& qDot);

    /**
     * Compare two OpenSim::Datatables or any of the derived types (as long as
     * they share the same data types). Comparison is performed by computing the
     * RMSE of the columns with common labels acquired from the 'query' table
     * (default), or by computing the RMSE of the rows with common timestamp
     * values. The column labels of the query table must exist in the reference
     * table, otherwise it throws exception. If columns are compared, the number
     * of rows must also match. If the computed RMSE of any of the match
     * columns/rows is larger than the given threshold it throws an exception.
     *
     * @param queryTable - Query table to compare.
     *
     * @param refTable - Reference table to be compare against.
     *
     * @param threshold - Threshold value (default is 1e-5).
     *
     * @param compareColumns - Choose to compare table columns or rows.
     */
    template <template <typename, typename> class Table, typename T, typename E>
    static void compareTables(const Table<T, E>& queryTable,
                              const Table<T, E>& refTable,
                              const double& threshold = 1e-5,
                              const bool& compareColumns = true) {
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
            if (found != refLabels.end())
                mapRefToQuery.push_back(
                        std::distance(refLabels.begin(), found));
        }

        // ref table must have all the query column labels
        if (mapRefToQuery.size() != queryFlat.getNumColumns())
            THROW_EXCEPTION(
                    "Query column labels do not exist in reference table.");

        if (compareColumns) {
            // rows must match
            if (!(queryTable.getNumRows() == refTable.getNumRows()))
                THROW_EXCEPTION("Number of rows in given tables do not match.");

            // compute the rmse of the matched columns
            for (size_t i = 0; i < mapRefToQuery.size(); ++i) {
                auto queryVec = queryFlat.getDependentColumnAtIndex(i);
                auto refVec =
                        refFlat.getDependentColumnAtIndex(mapRefToQuery[i]);
                auto rmse = sqrt((queryVec - refVec).normSqr() /
                                 queryFlat.getNumRows());
                SimTK_ASSERT3_ALWAYS(
                        (rmse < threshold),
                        "Column '%s' FAILED to meet accuracy of %f "
                        "RMS with RMSE %f.",
                        queryLabels[i].c_str(), threshold, rmse);
            }
        } else {
            // compute the rmse of the matched rows
            const auto& refTime = refFlat.getIndependentColumn();
            for (int j = 0; j < queryFlat.getNumRows(); ++j) {
                auto t = queryFlat.getIndependentColumn()[j];
                auto queryRow = queryFlat.getRow(t);

                // skip missing rows
                if (std::find(refTime.begin(), refTime.end(), t) ==
                    refTime.end())
                    continue;

                // ordered copy reference row
                SimTK::RowVector refRow(queryFlat.getNumColumns());
                for (int i = 0; i < mapRefToQuery.size(); ++i)
                    refRow[i] = refFlat.getRow(t)(mapRefToQuery[i]);

                // rmse
                auto rmse = sqrt((queryRow - refRow).normSqr() /
                                 queryFlat.getNumRows());
                SimTK_ASSERT3_ALWAYS(
                        (rmse < threshold),
                        "Row at time '%f' FAILED to meet accuracy of %f "
                        "RMS with RMSE %f.",
                        t, threshold, rmse);
            }
        }
    }
};

} // namespace OpenSimRT
