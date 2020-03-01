#ifndef MOMENT_ARM_H
#define MOMENT_ARM_H

#include "MomentArmExports.h"
#include <SimTKcommon.h>

#if __GNUG__
#define OPTIMIZATION __attribute__ ((optimize(0)))
#else
#define OPTIMIZATION
#endif

#ifdef __cplusplus
extern "C" {
MomentArm_API SimTK::Matrix calcMomentArm(const SimTK::Vector& q) OPTIMIZATION;
MomentArm_API std::vector<std::string> getModelMuscleSymbolicOrder() OPTIMIZATION;
MomentArm_API std::vector<std::string> getModelCoordinateSymbolicOrder() OPTIMIZATION;
}
#endif

#endif
