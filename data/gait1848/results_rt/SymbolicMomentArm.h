#ifndef SYMBOLIC_MOMENT_ARM_H
#define SYMBOLIC_MOMENT_ARM_H

#include <SimTKcommon.h>

#if __GNUG__
#define OPTIMIZATION __attribute__ ((optimize(0)))
#else
#define OPTIMIZATION
#endif

SimTK::Matrix calcMomentArm(const SimTK::Vector& q) OPTIMIZATION;

#endif