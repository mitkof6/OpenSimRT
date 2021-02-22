#include "internal/RegisterPlugin.h"

#include <Actuators/Thelen2003Muscle.h>

using namespace OpenSim;

static dllObjectInstantiator instantiator;

void RegisterPlugin() { Object::RegisterType(Thelen2003Muscle()); }

dllObjectInstantiator::dllObjectInstantiator() { registerDllClasses(); }

void dllObjectInstantiator::registerDllClasses() { RegisterPlugin(); }
