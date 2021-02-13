/**
 * @file TestSyncManager.cpp
 *
 * \brief Test file for the SyncManager.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "SyncManager.h"
#include "Utils.h"

#include <vector>

using namespace std;
using namespace OpenSimRT;

void run() {
    double samplingRate = 1000.0;
    double threshold = 0.01;
    SyncManager manager(samplingRate, threshold);

    // append new entries
    manager.appendPack(
            // pair
            std::pair<double, SimTK::Vec4>(0, SimTK::Quaternion().asVec4()),
            std::pair<double, SimTK::Vec4>(0, SimTK::Quaternion().asVec4()),

            // container of pairs
            std::vector<std::pair<double, SimTK::Vec4>>{
                    {0, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()},
                    {0, SimTK::Quaternion(1, 0, 0, 0).asVec4()}},

            // pair with larger time
            std::pair<double, SimTK::Vec4>(
                    0, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()),

            // pair with container of vecs
            std::pair<double, std::vector<SimTK::Vec3>>(
                    0,
                    {SimTK::Vec3(0.3, 0.3, 0.3), SimTK::Vec3(0.2, 0.2, 0.2)}));

    // cout table
    cout << "Append: "
         << "\n";
    cout << "Time Column: "
         << SimTK::Array_<double>(manager.getTable().getIndependentColumn())
         << endl;
    cout << "Table data: " << manager.getTable().getMatrix() << endl;

    // retrieve output (removes entries from table)
    auto pack = manager.getPack();

    // output (should be empty)
    if (!pack.second.empty()) {
        cout << "Retrieve: "
             << "\n";
        cout << pack.first << " " << pack.second[0] << endl;
    }

    // append new entries
    manager.appendPack(

            // pair
            std::pair<double, SimTK::Vec4>(
                    0.4, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()),
            std::pair<double, SimTK::Vec4>(0.4, SimTK::Quaternion().asVec4()),

            // container of pairs
            std::vector<std::pair<double, SimTK::Vec4>>{
                    {0.4, SimTK::Quaternion().asVec4()},
                    {0.4, SimTK::Quaternion(0.5, 0.5, 0.5, 0.5).asVec4()}},

            // pair with larger time
            std::pair<double, SimTK::Vec4>(0.4, SimTK::Quaternion().asVec4()),

            // pair with container of vecs
            std::pair<double, std::vector<SimTK::Vec3>>(
                    0.4, {SimTK::Vec3(0), SimTK::Vec3(0)}));

    // cout table
    cout << "Append: "
         << "\n";
    cout << "Time Column: "
         << SimTK::Array_<double>(manager.getTable().getIndependentColumn())
         << endl;
    cout << "Table data: " << manager.getTable().getMatrix() << endl;

    // retrieve new output
    pack = manager.getPack();

    // output (should be not empty)
    if (!pack.second.empty()) {
        cout << "Retrieve: "
             << "\n";
        cout << pack.first << " " << pack.second[0] << endl;
    }
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
