/**
 * @file DynamicLibraryLoader.h
 *
 * \brief Utility for loading dynamic libraries at runtime.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef MOMENT_ARM_DYNAMIC_LIBRARY_LOADER_H
#define MOMENT_ARM_DYNAMIC_LIBRARY_LOADER_H

#include "Exception.h"

#include <OpenSim/Common/LoadOpenSimLibrary.h>

// necessary for loading dynamic libraries
#ifdef _WIN32
#    include <winbase.h>
#    include <windows.h>
#else
#    include <dlfcn.h>
#endif

namespace OpenSimRT {

/**
 * A utility that loads a function from a dynamic library (.dll, .so,
 * .dylib) in runtime.
 */
template <typename T>
T loadDynamicLibrary(std::string libraryPath, std::string functionName) {
    // load library handle
    auto handle = OpenSim::LoadOpenSimLibrary(libraryPath, true);
    if (handle == nullptr) THROW_EXCEPTION("Library cannot be found.");

	// get function pointer
#ifdef _WIN32
    T function = (T) GetProcAddress(handle, functionName);
#else
    T function = (T) dlsym(handle, functionName.c_str());
#endif

    return function;
}

} // namespace OpenSimRT

#endif
