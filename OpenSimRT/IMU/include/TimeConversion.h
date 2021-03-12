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
 * @file TimeConversion.h
 *
 * @brief Define functions for converting system clock time_points to NTP
 * timestamps and vice-versa.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 * @original source: https://stackoverflow.com/a/65149566
 */
#pragma once

#include "internal/IMUExports.h"

#include <chrono>
#include <string>

/**
 * Convert an system_clock::time_point to NTP timestamp
 */
IMU_API std::uint64_t tp2ntp(std::chrono::system_clock::time_point tp);

/**
 * Convert a NTP timestamp to system_clock::time_point
 */
IMU_API std::chrono::system_clock::time_point ntp2tp(std::uint64_t ntp);

/**
 * Convert a NTP timestamp to a double representing the number of seconds and
 * its fractional part from the Unix epoch.
 */
IMU_API double ntp2double(std::uint64_t ntp);

/**
 * Convert a system_clock::time_point into a human-readable string with the date
 * and time.
 */
IMU_API std::string tp2string(std::chrono::system_clock::time_point tp);
