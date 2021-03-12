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
 * @file Exception.h
 *
 * \brief Exception utilities.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#pragma once

#include <sstream>
#include <string>

namespace OpenSimRT {

/**
 * \brief An exception that prints the file and line number along with
 * the message.
 */
class FileLineException : public std::exception {
 public:
    FileLineException(const std::string& arg, const char* file, int line)
            : std::exception() {
        std::ostringstream o;
        o << file << ":" << line << ": " << arg;
        msg = o.str();
    }
    const char* what() const throw() { return msg.c_str(); }

 private:
    std::string msg;
};

// used as macro in order to insert __FILE__ and __LINE__
#define THROW_EXCEPTION(msg) throw FileLineException(msg, __FILE__, __LINE__)

// ensure that a value is positive
#define ENSURE_POSITIVE(i)                                                     \
    (i > 0) ? i : THROW_EXCEPTION("ENSURE_POSITIVE failed")

// ensure that i is within a range to avoid segmentation fault
#define ENSURE_BOUNDS(i, min, max)                                             \
    (i >= min && i <= max) ? i : THROW_EXCEPTION("ENSURE_BOUNDS failed")

// ensure that elements in vectors x,y are the same
#define ENSURE_ORDER_IN_VECTORS(x, y)                                          \
    if (x.size() != y.size()) {                                                \
        THROW_EXCEPTION("ENSURE_ORDER_IN_VECTOR failed! Containers must have " \
                        "the same size");                                      \
    }                                                                          \
    if (!equal(x.begin(), x.end(), y.begin())) {                               \
        THROW_EXCEPTION("ENSURE_ORDER_IN_VECTOR failed! Elements do not have " \
                        "the same order");                                     \
    }
} // namespace OpenSimRT
