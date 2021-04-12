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
#include "SignalProcessing.h"
#include "Exception.h"
#include "Utils.h"
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorMath.h>
#define _USE_MATH_DEFINES
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Signal.h>
#include <map>
#include <math.h>

using namespace std;
using namespace SimTK;
using namespace OpenSimRT;

// Savitzky–Golay smoothing coefficients
std::map<int, std::vector<double>> SG_SMOOTHING_COEF{
        {2, {1.0, 0.0}},
        {3, {0.83333, 0.33333, -0.16667}},
        {4, {0.7, 0.4, 0.1, -0.2}},
        {5, {0.6, 0.4, 0.2, 0.0, -0.2}},
        {6, {0.52381, 0.38095, 0.2381, 0.09524, -0.04762, -0.19048}},
        {7, {0.46429, 0.35714, 0.25, 0.14286, 0.03571, -0.07143, -0.17857}}};

// Savitzky–Golay derivative coefficients
std::map<int, std::vector<double>> SG_DERIVATIVE_COEF{
        {2, {1.0, -1.0}},
        {3, {0.5, 0.0, -0.5}},
        // {3, {3.0 / 2.0, -4.0 / 2.0, 1.0 / 2.0}},
        {4, {0.3, 0.1, -0.1, -0.3}},
        // {4, {11.0 / 6.0, -18.0 / 6.0, 9.0 / 6.0, -2.0 / 6.0}},
        {5, {0.2, 0.1, 0.0, -0.1, -0.2}},
        {6, {0.14286, 0.08571, 0.02857, -0.02857, -0.08571, -0.14286}},
        {7, {0.10714, 0.07143, 0.03571, 0, -0.03571, -0.07143, -0.10714}}};

/******************************************************************************/
Vector binomial_mult(const int& n, const Vector& p) {
    Vector a(2 * n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = i; j > 0; --j) {
            a[2 * j] += p[2 * i] * a[2 * (j - 1)] -
                        p[2 * i + 1] * a[2 * (j - 1) + 1];
            a[2 * j + 1] += p[2 * i] * a[2 * (j - 1) + 1] +
                            p[2 * i + 1] * a[2 * (j - 1)];
        }
        a[0] += p[2 * i];
        a[1] += p[2 * i + 1];
    }
    return a;
}

void shiftColumnsRight(const Vector& column, Matrix& shifted) {
    if (column.size() != shifted.nrow()) {
        THROW_EXCEPTION("column vector and matrix have different dimentions" +
                        toString(column.size()) +
                        " != " + toString(shifted.nrow()));
    }
    for (int j = shifted.ncol() - 1; j > 0; --j) {
        shifted(j) = shifted(j - 1);
    }
    shifted(0) = column;
}

void shiftColumnsLeft(const Vector& column, Matrix& shifted) {
    if (column.size() != shifted.nrow()) {
        THROW_EXCEPTION("column vector and matrix have different dimentions" +
                        toString(column.size()) +
                        " != " + toString(shifted.nrow()));
    }
    for (int j = 1; j < shifted.ncol(); j++) { shifted(j - 1) = shifted(j); }
    shifted(shifted.ncol() - 1) = column;
}

/******************************************************************************/

LowPassSmoothFilter::LowPassSmoothFilter(const Parameters& parameters)
        : parameters(parameters), initializationCounter(parameters.memory - 1) {
    ENSURE_POSITIVE(parameters.numSignals);
    // at least 5 slots to define derivatives (5 - 4 > 0)
    ENSURE_POSITIVE(parameters.memory - 4);
    ENSURE_POSITIVE(parameters.cutoffFrequency);
    // if we need time derivatives then we are between [2, M - 2]
    ENSURE_BOUNDS(parameters.delay, 2, parameters.memory - 2);
    if (parameters.calculateDerivatives) {
        ENSURE_BOUNDS(parameters.splineOrder, 1, 7);
        if (parameters.splineOrder % 2 == 0) {
            THROW_EXCEPTION(
                    "spline order should be an odd number between 1 and 7");
        }
    }

    time = Matrix(1, parameters.memory, 0.0);
    data = Matrix(parameters.numSignals, parameters.memory, 0.0);
}

LowPassSmoothFilter::Output
LowPassSmoothFilter::filter(const LowPassSmoothFilter::Input& input) {
    // shift data column left and set last column as the new data
    shiftColumnsLeft(Vector(1, input.t), time);
    shiftColumnsLeft(input.x, data);

    // initialize variables
    int N = parameters.numSignals;
    int M = parameters.memory;
    int D = parameters.delay;
    double dt = time[0][M - 1] - time[0][M - 2];
    double dtPrev = time[0][M - 2] - time[0][M - 3];

    // output
    Output output;
    output.t = time[0][M - D - 1];
    output.x = Vector(N);
    output.xDot = Vector(N);
    output.xDDot = Vector(N);
    output.isValid = true;

    // check if initialized
    if (initializationCounter > 0) {
        initializationCounter--;
        output.isValid = false;
        return output;
    }

    // check if dt is consistent
    if (abs(dt - dtPrev) > 1e-5) {
        THROW_EXCEPTION("signal sampling frequency is not constant");
    }

    // filter
    for (int i = 0; i < N; i++) {
        // get raw signal
        double* xRaw = new double[M];
        double* xFiltered = new double[M];
        for (int j = 0; j < M; j++) { xRaw[j] = data[i][j]; }

        // apply a low pass filter; O = M / 2 is used for the order of the FIR
        // filter, because internally the filter performs a convolution over
        // [-O, O] = 2 M / 2 = M.
        OpenSim::Signal::LowpassFIR(parameters.memory / 2, dt,
                                    parameters.cutoffFrequency, M, xRaw,
                                    xFiltered);

        // calculate smooth splines
        if (parameters.calculateDerivatives) {
            OpenSim::GCVSpline spline(parameters.splineOrder, M, &time[0][0],
                                      xFiltered);
            output.x[i] = spline.calcValue(Vector(1, output.t));
            output.xDot[i] = spline.calcDerivative({0}, Vector(1, output.t));
            output.xDDot[i] =
                    spline.calcDerivative({0, 0}, Vector(1, output.t));
        } else {
            output.x[i] = xFiltered[M - D - 1];
        }

        // free allocated memory
        delete[] xRaw;
        delete[] xFiltered;
    }

    return output;
}

/******************************************************************************/

StateSpaceFilter::StateSpaceFilter(const Parameters& parameters)
        : fc(parameters.cutoffFrequency), nc(parameters.numSignals),
          state(Output{numeric_limits<double>::infinity(), Vector(nc, 0.0),
                       Vector(nc, 0.0), Vector(nc, 0.0), false}) {}

StateSpaceFilter::Output StateSpaceFilter::filter(const Input& input) {
    double t = input.t - 0.07; // compensate for filter lag
    Vector x(nc, &input.x[0]); // we copy because vector is transposed
    if (t < state.t) {
        state.x = x;
        state.xDot = 0.0;
        state.xDDot = 0.0;
    } else {
        double h = t - state.t;
        double a = (2 * M_PI * fc) * (2 * M_PI * fc);
        double b = sqrt(2) * 2 * M_PI * fc;
        double denom = 4 + 2 * h * b + h * h * a;
        double A = (4 + 2 * h * b - h * h * a) / denom;
        double B = 4 * h / denom;
        double C = -4 * h * a / denom;
        double D = (4 - 2 * h * b - h * h * a) / denom;
        double E = 2 * h * h * a / denom;
        double F = 4 * h * a / denom;
        Vector y = A * state.x + B * state.xDot + E * (x + state.x) / 2;
        Vector yd = C * state.x + D * state.xDot + F * (x + state.x) / 2;
        state.xDDot = (yd - state.xDot) / h;
        state.xDot = yd;
        state.x = y;
        state.isValid = true;
    }
    state.t = t;
    return state;
}

/******************************************************************************/

IIRFilter::IIRFilter(int n, const Vector& aa, const Vector& bb,
                     InitialValuePolicy policy)
        : n(n), iv(policy) {
    a = aa(1, aa.size() - 1) / aa[0];
    b = bb / aa[0];
    m = a.size();
    X = Matrix(n, b.size(), 0.0);
    Y = Matrix(n, a.size(), 0.0);
}

Vector IIRFilter::filter(const Vector& xn) {
    if (xn.size() != n) {
        THROW_EXCEPTION("input has incorrect dimensions " +
                        toString(xn.size()) + " != " + toString(n));
    }
    if (m == 0) {
        shiftColumnsRight(xn, X);
        Matrix yn = X * b - Y * a;
        shiftColumnsRight(yn(0), Y);
        return Y(0);
    } else {
        shiftColumnsRight(xn, X);
        shiftColumnsRight(xn, Y);
        m--;
        if (iv == Zero) {
            return Vector(n, 0.0);
        } else if (iv == Signal) {
            return Vector(xn.size(), &xn[0]);
        } else {
            THROW_EXCEPTION("undefined initial value policy");
        }
    }
}

ButterworthFilter::ButterworthFilter(
        int dim, int filtOrder, double cutOffFreq, const FilterType& type,
        const IIRFilter::InitialValuePolicy& policy) {
    setupFilter(dim, filtOrder, cutOffFreq, type, policy);
}

void ButterworthFilter::setupFilter(
        int dim, int filtOrder, double cutOffFreq, const FilterType& type,
        const IIRFilter::InitialValuePolicy& policy) {
    if (cutOffFreq <= 0 || cutOffFreq >= 1)
        THROW_EXCEPTION(
                "Digital filter critical frequencies must be 0 < Wn < 1");
    double sf; // scaling factor
    Vector a;  // denominator coefficients
    Vector b;  // numerator coefficients
    if (type == FilterType::LowPass) {
        sf = sf_bwlp(filtOrder, cutOffFreq);
        a = dcof_bwlp(filtOrder, cutOffFreq);
        b = ccof_bwlp(filtOrder) * sf;
    } else if (type == FilterType::HighPass) {
        sf = sf_bwhp(filtOrder, cutOffFreq);
        a = dcof_bwhp(filtOrder, cutOffFreq);
        b = ccof_bwhp(filtOrder) * sf;
    } else {
        THROW_EXCEPTION("Other filter types are not supported yet.");
    }

    // create iir filter with butter worth coefficients
    iir = new IIRFilter(dim, a, b, policy);
}

Vector ButterworthFilter::ccof_bwlp(const int& n) {
    Vector ccof(n + 1, 0.0);

    ccof[0] = 1;
    ccof[1] = n;
    for (int i = 2; i <= n / 2; ++i) {
        ccof[i] = (n - i + 1) * int(ccof[i - 1]) / i;
        ccof[n - i] = ccof[i];
    }
    ccof[n - 1] = n;
    ccof[n] = 1;
    if (!ccof.size())
        THROW_EXCEPTION("Unable to calculate numerator coefficients");
    return ccof;
}

Vector ButterworthFilter::dcof_bwlp(const int& n, const double& fcf) {
    Vector rcof(2 * n, 0.0);
    const double theta = M_PI * fcf;
    const double st = sin(theta);
    const double ct = cos(theta);

    for (int k = 0; k < n; ++k) {
        const double parg = M_PI * (double) (2 * k + 1) / (double) (2 * n);
        const double a = 1.0 + st * sin(parg);
        rcof[2 * k] = -ct / a;
        rcof[2 * k + 1] = -st * cos(parg) / a;
    }

    auto dcof = binomial_mult(n, rcof);

    dcof[1] = dcof[0];
    dcof[0] = 1.0;
    for (int k = 3; k <= n; ++k) dcof[k] = dcof[2 * k - 2];
    if (!dcof.size())
        THROW_EXCEPTION("Unable to calculate denominator coefficients");
    return dcof(0, n + 1);
}

double ButterworthFilter::sf_bwlp(const int& n, const double& fcf) {
    double omega = M_PI * fcf;
    double fomega = sin(omega);
    double parg0 = M_PI / (double) (2 * n);
    double sf = 1.0;

    for (int k = 0; k < n / 2; ++k)
        sf *= 1.0 + fomega * sin((double) (2 * k + 1) * parg0);

    fomega = sin(omega / 2.0);
    if (n % 2) sf *= fomega + cos(omega / 2.0);
    sf = pow(fomega, n) / sf;

    return sf;
}

Vector ButterworthFilter::ccof_bwhp(const int& n) {
    auto ccof = ccof_bwlp(n);

    for (int i = 0; i <= n; ++i)
        if (i % 2) ccof[i] = -ccof[i];

    return ccof;
}

Vector ButterworthFilter::dcof_bwhp(const int& n, const double& fcf) {
    return dcof_bwlp(n, fcf);
}

double ButterworthFilter::sf_bwhp(const int& n, const double& fcf) {
    double omega = M_PI * fcf;
    double fomega = sin(omega);
    double parg0 = M_PI / (double) (2 * n);
    double sf = 1.0; // scaling factor

    for (int k = 0; k < n / 2; ++k)
        sf *= 1.0 + fomega * sin((double) (2 * k + 1) * parg0);

    fomega = cos(omega / 2.0);

    if (n % 2) sf *= fomega + sin(omega / 2.0);
    sf = pow(fomega, n) / sf;

    return sf;
}

Vector ButterworthFilter::filter(const SimTK::Vector& xn) {
    return iir->filter(xn);
}

/******************************************************************************/

FIRFilter::FIRFilter(int n, const Vector& b, InitialValuePolicy policy)
        : n(n), b(b), m(b.size()), X(n, b.size(), 0.0), iv(policy) {}

Vector FIRFilter::filter(const Vector& xn) {
    if (xn.size() != n) {
        THROW_EXCEPTION("input has incorrect dimensions " +
                        toString(xn.size()) + " !=" + toString(n));
    }
    Vector x;
    if (m == 0) {
        shiftColumnsRight(xn, X);
        x = X * b;
    } else {
        shiftColumnsRight(xn, X);
        if (iv == Zero) {
            x = Vector(n, 0.0);
        } else if (iv == Signal) {
            x = xn;
        } else {
            THROW_EXCEPTION("undefined initial value policy");
        }
        m--;
    }
    return x;
}

/******************************************************************************/

SavitzkyGolay::SavitzkyGolay(int n, int m)
        : FIRFilter(n, Vector(m, &SG_SMOOTHING_COEF[ENSURE_BOUNDS(m, 2, 7)][0]),
                    Signal) {}

/******************************************************************************/

NumericalDifferentiator::NumericalDifferentiator(int n, int m)
        : FIRFilter(n,
                    Vector(m, &SG_DERIVATIVE_COEF[ENSURE_BOUNDS(m, 2, 7)][0]),
                    Zero),
          t(0.0) {}

Vector NumericalDifferentiator::diff(double tn, const Vector& xn) {
    Vector dx;
    if (t < tn) {
        dx = filter(xn) / (tn - t);
    } else {
        dx = filter(xn); // default is zero to void nan
    }
    t = tn;
    return dx;
}

/******************************************************************************/
