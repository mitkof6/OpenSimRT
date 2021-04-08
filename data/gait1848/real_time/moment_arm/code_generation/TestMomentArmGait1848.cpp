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
 * @file TestMomentArm.cpp
 *
 * \brief Run-time loading of a dynamic library of the code generated muscle
 * moment arm.
 *
 * TODO: Not tested on Mac yet.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <SimTKcommon.h>
#include <iostream>

using std::cout;
using std::endl;
using std::exception;

// necessary for loading dynamic libraries
#ifdef _WIN32
#    include <winbase.h>
#    include <windows.h>
#else
#    include <dlfcn.h>
#endif

// function prototype: SimTK::Matrix calcMomentArm(const SimTK::Vector& q)
typedef SimTK::Matrix (*CalcMomentArm)(const SimTK::Vector& q);

void run() {
    // load library
    auto momentArmLibHandle =
            OpenSim::LoadOpenSimLibrary("Gait1848MomentArm", true);

    // get function pointer
#ifdef WIN32
    CalcMomentArm calcMomentArm =
            (CalcMomentArm) GetProcAddress(momentArmLibHandle, "calcMomentArm");
#else
    CalcMomentArm calcMomentArm =
            (CalcMomentArm) dlsym(momentArmLibHandle, "calcMomentArm");
#endif

    // calculate moment arm
    cout << calcMomentArm(SimTK::Vector(18, 0.0));
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
