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
 * @file SignalProcessing.h
 *
 * \brief Utilities for filtering signals.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 * contribution: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "internal/CommonExports.h"

#include <SimTKcommon.h>
#include <condition_variable>
#include <mutex>

namespace OpenSimRT {

/**
 * \brief A non-casual filter that uses a low pass recursive filter for removing
 * high frequency noise and splines for calculating higher order derivatives.
 *
 * A memory buffer contains the previous values (memory parameter) of a signal
 * (one or many channels as defined by numSignals). When the current value is
 * provided, the memory buffer is left shifted and the new value is appended at
 * the end of the buffer (circular buffer). A low pass recursive filter
 * (cutoffFrequency) is applied to the signal to attenuate high frequency
 * noise. If one must compute the derivatives of the signal, generalized
 * cross-validation splines are fitted, because discontinuities are amplified by
 * numerical differentiation. The output (x, x_dot, x_ddot) is a delayed version
 * of the signal (delay parameter), so in order to use future values during
 * filtering (non-casual filter). While this may introduce an artificial delay
 * in the signal, it can greatly improve the accuracy of the first and second
 * derivatives of the signal.
 *
 * For gait kinematics we identified the following set of parameters assuming
 * that the signal is generated at 100Hz (typical for motion capture):
 *
 *    memory = 35
 *    cutoffFrequency = 6
 *    delay = 14
 *    splineOrder = 3
 *
 * The low pass filter uses a kernel (sinc and Hamming window) that multiplies
 * the signal (memory buffer). For real-time applications, the memory and delay
 * parameters affects the performance of the filter.
 */
class Common_API LowPassSmoothFilter {
 public: /* public data structures */
    struct Parameters {
        int numSignals;            // number of signals that are filtered
        int memory;                // memory buffer of the filter
        double cutoffFrequency;    // low pass cutoff frequency
        int delay;                 // sample delay to evaluate the result
        int splineOrder;           // spline order use 3
        bool calculateDerivatives; // whether to calculate derivatives
    };
    struct Input {
        double t;
        SimTK::Vector x;
    };
    struct Output {
        double t;
        SimTK::Vector x;
        SimTK::Vector xDot;
        SimTK::Vector xDDot;
        bool isValid; // requires at least # memory samples
    };

 public: /* public interface */
    LowPassSmoothFilter(const Parameters& parameters);
    Output filter(const Input& input);

 private: /* private data members */
    Parameters parameters;
    SimTK::Matrix time;
    SimTK::Matrix data;
    int initializationCounter;
};

/**
 * An thread safe implementation using mutexes and conditional variables of the
 * LowPassSmoothFilter. The filtering process is split between the producer and
 * consumer threads. The producer thread pushes new vector data as columns in
 * the 'data' and 'time' matrices with the 'updState(input)' function, whereas
 * the consumer executes the actual filtering and produces the output with the
 * 'filter()' function.
 */
class Common_API LowPassSmoothFilterTS {
 public: /* public data structures */
    struct Parameters {
        int numSignals;            // number of signals that are filtered
        int memory;                // memory buffer of the filter
        double cutoffFrequency;    // low pass cutoff frequency
        int delay;                 // sample delay to evaluate the result
        int splineOrder;           // spline order use 3
        bool calculateDerivatives; // whether to calculate derivatives
    };
    struct Input {
        double t;
        SimTK::Vector x;
    };
    struct Output {
        double t;
        SimTK::Vector x;
        SimTK::Vector xDot;
        SimTK::Vector xDDot;
    };

 public: /* public interface */
    LowPassSmoothFilterTS(const Parameters& parameters);
    ~LowPassSmoothFilterTS();
    void updState(const Input& input);
    Output filter();

 private: /* private data members */
    Parameters parameters;
    SimTK::Matrix time;
    SimTK::Matrix data;
    int initializationCounter;

    int M, D, N;
    double dt, dtPrev;
    double* xRaw;
    double* xFiltered;

    // thread synchronization parameters
    bool newDataReady = false;
    bool dataMatrixReady = false;
    std::mutex monitor;
    std::condition_variable cond;
};

/**
 * \brief A low pass filter that calculates the first and second derivative of
 * the signal.
 *
 * [rtosim] C. Pizzolato, M. Reggiani, L. Modenese & D. G. Lloyd (2016):
 * Real-time inverse kinematics and inverse dynamics for lower limb applications
 * using OpenSim, _Computer Methods in Biomechanics and Biomedical Engineering,
 * DOI: 10.1080/10255842.2016.1240789 To link to this article:
 * http://dx.doi.org/10.1080/10255842.2016.1240789
 */
class Common_API StateSpaceFilter {
 public:
    struct Parameters {
        int numSignals;         // number of signals that are filtered
        double cutoffFrequency; // low pass cutoff frequency
    };
    struct Input {
        double t;
        SimTK::Vector x;
    };
    struct Output {
        double t;
        SimTK::Vector x;
        SimTK::Vector xDot;
        SimTK::Vector xDDot;
        bool isValid;
    };
    int nc;
    double fc;
    Output state;

 public:
    StateSpaceFilter(const Parameters& parameters);
    Output filter(const Input& input);
};

/**
 * \brief A multidimensional IIR filter.
 *
 * The filter function is implemented as a direct II transposed structure for
 * high dimensional signals
 *
 *     a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
 *                           - a[1]*y[n-1] - ... - a[N]*y[n-N]
 *
 * where `M` is the degree of the numerator, `N` is the degree of the
 * denominator, and `n` is the sample number.
 *
 * For an order 2 lowpass Butterworth filter with 6Hz cutoff frequency for 60Hz
 * sampling rate:
 *
 *     a = [1.,        -1.1429805,  0.41280160]
 *     b = [0.06745527, 0.13491055, 0.06745527]
 */
class Common_API IIRFilter {
 public:
    SimTK::Matrix Y, X;
    SimTK::Vector a, b;
    int n, m;
    /* If iteration < memory return zero or signal's input value */
    enum InitialValuePolicy { Zero = 0, Signal } iv;

 public:
    IIRFilter(int n, const SimTK::Vector& a, const SimTK::Vector& b,
              InitialValuePolicy policy);
    SimTK::Vector filter(const SimTK::Vector& xn);
};

/**
 * A multidimensional digital Butterworth filter.
 */
class Common_API ButterworthFilter {
 public:
    /**
     * Chose between LowPass, HighPass, BandPass or BandCut filter type.
     */
    enum class FilterType { LowPass, HighPass, BandPass, BandCut };

    /**
     * Construct an n-dimentional Butterworth filter.
     *
     * @param [dim] - signal dimension
     * @param [filtOrder] - filter's order
     * @param [cutOffFreq] - cutoff frequency
     * @param [type] - filter type (LowPass, HighPass, BandPass, BandCut)
     * @param [policy] - use zero or signal's input value as initilization.
     */
    ButterworthFilter(int dim, int filtOrder, double cutOffFreq,
                      const FilterType& type,
                      const IIRFilter::InitialValuePolicy& policy);

    /**
     * Setup the Butterworth filter.
     *
     * @param [dim] - signal dimension
     * @param [filtOrder] - filter's order
     * @param [cutOffFreq] - cutoff frequency
     * @param [type] - filter type (LowPass, HighPass, BandPass, BandCut)
     * @param [policy] - use zero or signal's input value as initilization.
     */
    void setupFilter(int dim, int filtOrder, double cutOffFreq,
                     const FilterType& type,
                     const IIRFilter::InitialValuePolicy& policy);

    /**
     * Filter the input signal.
     */
    SimTK::Vector filter(const SimTK::Vector& xn);

 private:
    // lp
    SimTK::Vector ccof_bwlp(const int& n);
    SimTK::Vector dcof_bwlp(const int& n, const double& fcf);
    double sf_bwlp(const int& n, const double& fcf);

    // hp
    SimTK::Vector ccof_bwhp(const int& n);
    SimTK::Vector dcof_bwhp(const int& n, const double& fcf);
    double sf_bwhp(const int& n, const double& fcf);

    // bp
    // ... TODO

    // bc
    //... TODO

    // pointer to iirFilter
    SimTK::ReferencePtr<IIRFilter> iir;
};

/**
 * \brief A multidimensional FIR filter.
 *
 *     y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
 *
 * where `M` is the memory and `n` is the sample number.
 */
class Common_API FIRFilter {
 public:
    SimTK::Matrix X;
    SimTK::Vector b;
    int n, m;
    /* If iteration < memory return zero or signal's input value */
    enum InitialValuePolicy { Zero = 0, Signal } iv;

 public:
    FIRFilter(int n, const SimTK::Vector& b, InitialValuePolicy policy);
    SimTK::Vector filter(const SimTK::Vector& xn);
};

/**
 * \brief Savitzky-Golay smoothing filter.
 */
class Common_API SavitzkyGolay : public FIRFilter {
 public:
    SavitzkyGolay(int n, int m);
};

/**
 * \brief M-point numerical differentiation.
 */
class Common_API NumericalDifferentiator : public FIRFilter {
    double t;

 public:
    NumericalDifferentiator(int n, int m);
    SimTK::Vector diff(double tn, const SimTK::Vector& xn);
};

} // namespace OpenSimRT
