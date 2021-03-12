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
 * @file TestCircularBuffer.cpp
 *
 * \brief Tests the utilization of the thread safe circular buffer.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include "CircularBuffer.h"

#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSimRT;

CircularBuffer<100, double> buffer;

void producerFunction() {
    for (int i = 1; i <= 100; ++i) {
        buffer.add(i);
        cout << "ID: " << this_thread::get_id() << " Add: " << i << endl;
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

void consumerFunction(int M) {
    for (int i = 1; i <= 50; ++i) {
        auto data = buffer.get(M);
        cout << "ID: " << this_thread::get_id() << " Get: ";
        for (auto d : data) { cout << d << "\t"; }
        cout << endl;
        this_thread::sleep_for(chrono::milliseconds(2));
    }
}

void run() {
    thread producer(producerFunction);
    thread consumer1(consumerFunction, 5);
    thread consumer2(consumerFunction, 100);
    producer.join();
    consumer1.join();
    consumer2.join();
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
