/**
 * @file Measure.h
 *
 * \brief Utilities for measuring time.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef MEASURE_H
#define MEASURE_H

#include <chrono>

namespace OpenSimRT {

// start measuring time
#define START_CHRONO()							        \
    std::chrono::high_resolution_clock::time_point t1;  \
    t1 = std::chrono::high_resolution_clock::now();

// end measuring time
#define END_CHRONO()                                                    \
    std::chrono::high_resolution_clock::time_point t2;                  \
    t2 = std::chrono::high_resolution_clock::now();                     \
    std::cout << "Elapsed time: "                                       \
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() \
    << "ms" << std::endl;

} // namespace OpenSimRT

#endif
