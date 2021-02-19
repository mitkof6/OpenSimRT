/**
 * Define constants for the system clock time and NTP conversions.
 * TODO Find library implementation if possible.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#ifndef TIMEDEFINITIONS_H
#define TIMEDEFINITIONS_H

#include <chrono>

#define PICOSECS_RESOLUTION_BIN 4294967295UL // pow(2, 32)
#define PICOSECS_RESOLUTION_DEC 1000000000   // pow(10, 9)

#ifdef WIN32
#    define RFC_PROTOCOL 0UL //

#else                                 // Unix time
#    define RFC_PROTOCOL 2208988800UL // Jan 1 1900

#endif

// GMT+03:00
#define GMT_LOCAL_TIMEZONE 10800UL

#endif /* TIMEDEFINITIONS_H */
