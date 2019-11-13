/**
 * @file test2StagePipeline.cpp
 *
 * \brief Implements a consumer - producer system, where the producer generates
 * a noisy signal that must be consumed and filtered by the consumer through a
 * shared circular buffer.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include <thread>
#include <chrono>
// #include <atomic>
#include <random>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Signal.h>
#include "Simulation.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

// #define USE_GNUPLOT

bool consumed = false;
CircularBuffer<500, Vector> buffer;
#ifdef USE_GNUPLOT && __linux__
bool persist = false;
GNUPlot<500, 2> signalProducer(persist);
GNUPlot<500, 3> signalConsumer(persist);
GNUPlot<500, 2> D1(persist);
GNUPlot<500, 2> D2(persist);
#endif

// A sinusoidal with uniform noise.
double f(double t) {
    static default_random_engine generator;
    static uniform_real_distribution<double> distribution(0.0, .2);
    static double f = 3;
    static double fn = 10;
    //return sin(2 * M_PI * f * t) + 0.1 * sin(2 * M_PI * fn * t);
    return sin(2 * M_PI * f * t) + distribution(generator);
}

// Converts buffer data to SimTK::Matrix for easy manipulation
SimTK::Matrix stdToMatrix(const std::vector<SimTK::Vector>& data) {
    Matrix temp(data.size(), data[0].size());
    int i = 0;
    for (const auto& d : data) {
        temp[i] = d.getAsRowVector();
        i++;
    }
    return temp;
}

// Produces and adds a noisy signal to a shared buffer
void producerSignal() {
    int FPS = 100;
    double t = 0;
    double dt = 1.0 / FPS;
    while (!consumed) {
        // transmit
        Vector data(2);
        data[0] = t;
        data[1] = f(t);
        buffer.add(data);
#ifdef USE_GNUPLOT
        // visualize
        signalProducer.add(data);
        signalProducer.visualize({1}, {"lp"}, {"y_s"});
#endif
        // update state
        this_thread::sleep_for(chrono::milliseconds(1000 / FPS));
        t += dt;
    }
}

// Consumes and filters the noisy signal from a shared buffer
void consumerSignal() {
    int consume = 100;
    int FPS = 40;
    int N = 2;
    const int order = 51;
    const int M = 2 * order + 1;
    double fc = 6.0;
    for (int i = 0; i < consume; ++i) {
        // get M previous data
        // START_CHRONO();
        auto data = buffer.get(M, true); // reverse order
        Matrix mat = stdToMatrix(data);
        auto t = mat(0);
        auto yh = mat(1);
        double dt = t[1] - t[0];
        double y[M];
        Signal::LowpassFIR(order, dt, fc, M, &yh[0], &y[0]);
        // Signal::LowpassIIR(dt, fc, M, &yh[0], &y[0]); // not good enough
        // Signal::SmoothSpline(3, dt, fc, M, &t[0], &yh[0], &y[0]); // bug
        // fit splines
        GCVSpline spline(3, M, &t[0], &y[0]);
        // END_CHRONO();
#ifdef USE_GNUPLOT && __linux__
	// visualize signal
        Vector signal(3);
        signal[0] = t[0];
        signal[1] = y[0];
        signal[2] = spline.calcValue(Vector(1, t[0]));
        signalConsumer.add(signal);
        signalConsumer.visualize({1, 2},
        {"lp", "lp"},
        {"y_c", "y_h"});
        // visualize first derivative
        Vector yd(2);
        yd[0] = t[0];
        yd[1] = spline.calcDerivative({0}, Vector(1, t[1]));
        D1.add(yd);
        D1.visualize({1}, {"lp"}, {"yd"});
        // visualize second derivative
        Vector ydd(2);
        ydd[0] = t[0];
        ydd[1] = spline.calcDerivative({0, 0}, Vector(1, t[2]));
        D2.add(ydd);
        D2.visualize({1}, {"lp"}, {"ydd"});
#endif
        // update
        this_thread::sleep_for(chrono::milliseconds(1000 / FPS));
    }
    consumed = true;
}

void run() {
    thread producer(producerSignal);
    thread consumer(consumerSignal);

    producer.join();
    consumer.join();
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
