/**
 * @file TestCircularBuffer.cpp
 *
 * \brief Tests the utilization of the thread safe circular buffer.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include <iostream>
#include <thread>
#include "CircularBuffer.h"

using namespace std;
using namespace OpenSimRT;

CircularBuffer<100, double> buffer;

void producerFunction() {
    for (int i = 1 ; i <= 100; ++i) {
	buffer.add(i);
	cout << "ID: " << this_thread::get_id() << " Add: " << i << endl;
	this_thread::sleep_for(chrono::milliseconds(1));
    }
}

void consumerFunction(int M) {
    for (int i = 1 ; i <= 50; ++i) {
	auto data = buffer.get(M);
	cout << "ID: " << this_thread::get_id() <<" Get: ";
	for (auto d : data) {
	    cout << d << "\t";
	}
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

int main(int argc, char *argv[]) {
    try {
        run();
    } catch (exception &e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
