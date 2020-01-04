/**
 * @file TestMomentArm.cpp
 *
 * \brief Run-time loading of a dynamic library of the code generated muscle
 * moment arm.
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
            OpenSim::LoadOpenSimLibrary("Gait1992MomentArm_rd", true);

    // get function pointer
#ifdef _WIN32
    CalcMomentArm GetWelcomeMessage =
            (CalcMomentArm) GetProcAddress(momentArmLibHandle, "calcMomentArm");
#else
    CalcMomentArm calcMomentArm =
            (CalcMomentArm) dlsym(momentArmLibHandle, "calcMomentArm");
#endif

    // calculate moment arm
    cout << calcMomentArm(SimTK::Vector(19, 0.0));
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
