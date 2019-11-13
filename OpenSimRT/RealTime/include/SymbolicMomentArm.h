#ifndef SYMBOLIC_MOMENT_ARM_H
#define SYMBOLIC_MOMENT_ARM_H

#include "internal/RealTimeExports.h"
#include <functional>
#include <SimTKcommon.h>

// no optimization with gcc due to large compilation time
#if __GNUG__
#define OPTIMIZATION __attribute__ ((optimize(0)))
#else
#define OPTIMIZATION
#endif

RealTime_API std::function<SimTK::Matrix(const SimTK::Vector& q)>
    momentArmSelector(std::string modelFileName);

RealTime_API SimTK::Matrix gait1018MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix gait1224MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix gait1848MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix gait2154MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix gait2392MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix rtosimMomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix laiMomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix mobl2224MomentArm(const SimTK::Vector& q) OPTIMIZATION;
RealTime_API SimTK::Matrix notremor1124MomentArm(const SimTK::Vector& q) OPTIMIZATION;

#endif
