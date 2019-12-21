#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include "internal/CommonExports.h"
#include <deque>
#include <vector>
#include <string>
#include <simbody/SimTKcommon/Simmatrix.h>

namespace OpenSimRT {

/**
 * \brief A simple csv logger.
 */
class Common_API CSVLogger {
public:
    typedef std::vector<double> row_t;
    typedef std::vector<std::string> columns_t;
    columns_t columns;
    std::vector<row_t> data;
    std::string delimiter = ",";
public:
    CSVLogger(const columns_t& columns);
    void addRow(const row_t& row);
    void addRow(double t, const SimTK::Vector& vec);
    void exportToFile(std::string file);
};

} // namespace OpenSimRT

#endif
