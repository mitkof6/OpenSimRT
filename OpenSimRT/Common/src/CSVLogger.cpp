#include "CSVLogger.h"
#include "Utils.h"
#include "Exception.h"
#include <fstream>

using namespace std;
using namespace OpenSimRT;

CSVLogger::CSVLogger(const columns_t& columns) : columns(columns) {
}

void CSVLogger::addRow(const row_t& row) {
    if (row.size() != columns.size()) {
        THROW_EXCEPTION("dimensions mismatch " + toString(row.size()) +
                        " != " + toString(columns.size()));
    }
    data.push_back(row);
}

void CSVLogger::addRow(double t, const SimTK::Vector& vec) {
    row_t row;
    simtkToStd(vec, row);
    row.insert(row.begin(), t);
    addRow(row);
}

void CSVLogger::exportToFile(string file) {
    auto stream = ofstream(file, ofstream::out);
    stream << dump(columns, delimiter) << endl;
    for (auto row : data) {
        stream << dump(row, delimiter) << endl;
    }
    stream.close();
}
