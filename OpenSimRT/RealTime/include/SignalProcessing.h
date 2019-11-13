/**
 * @file SignalProcessing.h
 *
 * \brief Utilities for filtering signals.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <SimTKcommon.h>
#include "internal/RealTimeExports.h"


/**
 * Shifts a SimTK::Matrix right and substitutes from input the first column of
 * the shifted matrix. An useful utility if one thinks of the matrix as a
 * multidimensional circular buffer.
 */
RealTime_API void shiftColumnsRight(const SimTK::Vector& column,
				    SimTK::Matrix& shifted);

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
class RealTime_API StateSpaceFilter {
 public:
    struct FilterState {
	double t;
	SimTK::Vector x;
	SimTK::Vector xDot;
	SimTK::Vector xDDot;
    };
    double fc;
    FilterState state;
 public:
    StateSpaceFilter(int nc, double fc);
    FilterState filter(double t, SimTK::Vector x);   
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
class RealTime_API IIRFilter {
 public:
    SimTK::Matrix Y, X;
    SimTK::Vector a, b;
    int n, m;
     /* If iteration < memory return zero or signal's input value */
    enum InitialValuePolicy {Zero=0, Signal} iv; 
 public:
    IIRFilter(int n, const SimTK::Vector& a, const SimTK::Vector& b,
	      InitialValuePolicy policy);
    SimTK::Vector filter(const SimTK::Vector& xn);
};

/**
 * \brief A multidimensional FIR filter.
 *
 *     y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
 * 
 * where `M` is the memory and `n` is the sample number.
 */
class RealTime_API FIRFilter {
 public:
    SimTK::Matrix X;
    SimTK::Vector b;
    int n, m;
    /* If iteration < memory return zero or signal's input value */
    enum InitialValuePolicy {Zero=0, Signal} iv; 
 public:
    FIRFilter(int n, const SimTK::Vector& b, InitialValuePolicy policy);
    SimTK::Vector filter(const SimTK::Vector& xn);
};

/**
 * \brief Savitzky-Golay smoothing filter.
 */
class RealTime_API  SavitzkyGolay : public FIRFilter {
 public:
    SavitzkyGolay(int n, int m);
};


/**
 * \brief M-point numerical differentiation.
 */
class RealTime_API NumericalDifferentiator : public FIRFilter {
    double t;
 public:
    NumericalDifferentiator(int n, int m);
    SimTK::Vector diff(double tn, SimTK::Vector xn);
};

#endif
