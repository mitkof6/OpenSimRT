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
 * @file DynamicLibraryLoader.h
 *
 * \brief Utility for loading dynamic libraries at runtime.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

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
    T function = (T) GetProcAddress(handle, functionName.c_str());
#else
    T function = (T) dlsym(handle, functionName.c_str());
#endif

    return function;
}

} // namespace OpenSimRT
