/**
 * @file RegisterPlugin.h
 *
 * \brief An interface for OpenSim plugin system.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com.ch>
 */
#ifndef REGISTER_PLUGIN_H
#define REGISTER_PLUGIN_H

#include "RealTimeExports.h"
#include <OpenSim/Common/Object.h>

extern "C" {
/**
 * The purpose of this routine is to register all class types exported by
 * the plugin library.
 */
RealTime_API void RegisterPlugin();
}

class dllObjectInstantiator {
 public:
    dllObjectInstantiator();

 private:
    void registerDllClasses();
};

#endif
