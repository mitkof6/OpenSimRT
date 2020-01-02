#include "SignalProcessing.h"

#include "Exception.h"
#include "Utils.h"
#include <SimTKcommon/Scalar.h>
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
            THROW_EXCEPTION("spline order should be an odd number between 1 and 7");
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

        // apply a low pass filter
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

StateSpaceFilter::StateSpaceFilter(int nc, double fc)
        : fc(fc),
          state(FilterState{numeric_limits<double>::infinity(), Vector(nc, 0.0),
                            Vector(nc, 0.0), Vector(nc, 0.0)}) {}

StateSpaceFilter::FilterState StateSpaceFilter::filter(double t,
                                                       const Vector& x) {
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
    }
    state.t = t;
    return state;
}

/******************************************************************************/

IIRFilter::IIRFilter(int n, const Vector& aa, const Vector& bb,
                     InitialValuePolicy policy)
        : n(n), a(aa), b(bb), iv(policy) {
    b = b / a[0];
    a = a / a[0];
    a = a(1, a.size() - 1);
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
            return xn;
        } else {
            THROW_EXCEPTION("undefined initial value policy");
        }
    }
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
