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
 * @file Measure.h
 *
 * \brief Utilities for measuring time.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include <chrono>

namespace OpenSimRT {

// start measuring time
#define START_CHRONO()                                                         \
    std::chrono::high_resolution_clock::time_point t1;                         \
    t1 = std::chrono::high_resolution_clock::now();

// end measuring time
#define END_CHRONO()                                                           \
    std::chrono::high_resolution_clock::time_point t2;                         \
    t2 = std::chrono::high_resolution_clock::now();                            \
    std::cout << "Elapsed time: "                                              \
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -    \
                                                                       t1)     \
                         .count()                                              \
              << "ms" << std::endl;

} // namespace OpenSimRT
