/**
 * @file TimeConversions.h
 *
 * @brief Define functions for converting system clock time_points to NTP
 * timestamps and vice-versa.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 * @original source: https://stackoverflow.com/a/65149566
 */
#ifndef TIME_CONVERSION_H
#define TIME_CONVERSION_H

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

#endif /* TIME_CONVERSION_H */
