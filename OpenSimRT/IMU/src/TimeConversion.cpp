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
 */
#include "TimeConversion.h"
#include <chrono>
#include <iomanip>
#include <string>

using namespace std;
using namespace chrono;
using namespace literals;

using fractions = duration<int64_t, ratio<1, 0x100000000>>; // 1/(2^32)

uint64_t tp2ntp(system_clock::time_point tp) {
    // "shift" epoch from unix 1/1/1970 to ntp 1/1/1900 (70 years + 17 leap
    // days)
    tp += (70 * 365 + 17) * 24h;

    auto total = tp.time_since_epoch();
    auto secs = duration_cast<seconds>(total);
    auto fracs = duration_cast<fractions>(total - secs);

    return static_cast<uint64_t>((secs.count() << 32) | fracs.count());
}

system_clock::time_point ntp2tp(uint64_t ntp) {
    auto tp = system_clock::time_point(); // epoch

    // "shift" epoch from ntp 1/1/1900 to unix 1/1/1970 (70 years + 17 leap
    // days)
    tp -= (70 * 365 + 17) * 24h;

    tp += seconds(ntp >> 32);
    tp += duration_cast<system_clock::duration>(fractions(ntp & 0xffffffff));

    return tp;
}

double ntp2double(uint64_t ntp) {
    // "shift" epoch from ntp 1/1/1900 to unix 1/1/1970 (70 years + 17 leap
    // days)
    auto epoch = ((70 * 365 + 17) * 24h).count();

    auto secs = seconds(ntp >> 32).count();
    auto fracs = duration<double>(duration_cast<system_clock::duration>(
                                          fractions(ntp & 0xffffffff)))
                         .count();

    return secs + fracs - epoch;
}

string tp2string(system_clock::time_point tp) {
    auto tm = system_clock::to_time_t(tp);
    return string(asctime(localtime(&tm)));
}
