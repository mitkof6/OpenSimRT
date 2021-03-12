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
     * they share the same data types). Number of columns and rows must match,
     * otherwise throws an exception. Comparison is performed by computing the
     * RMSE of the columns with common labels acquired from the 'query' table.
     * If the computed RMSE of any of the match columns is larger than the given
     * threshold it throws an exception.
     *
     * @param queryTable - Query table to compare.
     *
     * @param refTable - Reference table to be compare against.
     *
     * @param threshold - Threshold value (default is 1e-5).
     */
    template <template <typename, typename> class Table, typename T, typename E>
    static void compareTables(const Table<T, E>& queryTable,
                              const Table<T, E>& refTable,
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
                auto refVec =
                        refFlat.getDependentColumnAtIndex(mapRefToQuery[i]);
                auto rmse = sqrt((queryVec - refVec).normSqr() /
                                 queryFlat.getNumRows());
                // std::cout << "Column '" << queryLabels[i] << "' has RMSE = "
                // << rmse
                //           << std::endl;
                SimTK_ASSERT3_ALWAYS(
                        (rmse < threshold),
                        "Column '%s' FAILED to meet accuracy of %f "
                        "RMS with RMSE %f.",
                        queryLabels[i].c_str(), threshold, rmse);
            }
        }
    }
};

} // namespace OpenSimRT
