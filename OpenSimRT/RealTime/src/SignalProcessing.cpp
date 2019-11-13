#include "SignalProcessing.h"
#include "SimulationUtils.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace SimTK;

// Savitzky–Golay smoothing coefficients
std::map<int, std::vector<double>> SG_SMOOTHING_COEF
{
 {2, {1.0, 0.0}},
 {3, {0.83333, 0.33333, -0.16667}},
 {4, {0.7, 0.4, 0.1, -0.2}},
 {5, {0.6, 0.4, 0.2, 0.0, -0.2}},
 {6, {0.52381, 0.38095, 0.2381, 0.09524, -0.04762, -0.19048}},
 {7, {0.46429, 0.35714, 0.25, 0.14286, 0.03571, -0.07143, -0.17857}}
};

// Savitzky–Golay derivative coefficients
std::map<int, std::vector<double>> SG_DERIVATIVE_COEF
{
 {2, {1.0, -1.0}},
 {3, {0.5, 0.0, -0.5}},
 // {3, {3.0 / 2.0, -4.0 / 2.0, 1.0 / 2.0}},
 {4, {0.3, 0.1, -0.1, -0.3}},
 // {4, {11.0 / 6.0, -18.0 / 6.0, 9.0 / 6.0, -2.0 / 6.0}},
 {5, {0.2, 0.1, 0.0, -0.1, -0.2}},
 {6, {0.14286, 0.08571, 0.02857, -0.02857, -0.08571, -0.14286}},
 {7, {0.10714, 0.07143, 0.03571, 0, -0.03571, -0.07143, -0.10714}}
};


/*******************************************************************************/

void shiftColumnsRight(const Vector& column, Matrix& shifted) {
    if (column.size() != shifted.nrow()) {
        THROW_EXCEPTION("column vector and matrix have different dimentions" +
                        toString(column.size()) + " != " + toString(shifted.nrow()));
    }
    for (int j = shifted.ncol() - 1; j > 0; --j) {
        shifted(j) = shifted(j - 1);
    }
    shifted(0) = column;
}

/*******************************************************************************/

StateSpaceFilter::StateSpaceFilter(int nc, double fc)
    : fc(fc),
    state(FilterState{numeric_limits<double>::infinity(),
          Vector(nc, 0.0), Vector(nc, 0.0), Vector(nc, 0.0)}) {
}

StateSpaceFilter::FilterState StateSpaceFilter::filter(double t, Vector x) {
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

/*******************************************************************************/

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
            return  Vector(n, 0.0);
        } else if (iv == Signal) {
            return xn;
        } else {
            THROW_EXCEPTION("undefined initial value policy");
        }
    }
}

/*******************************************************************************/

FIRFilter::FIRFilter(int n, const Vector& b, InitialValuePolicy policy)
    : n(n), b(b), m(b.size()), X(n, b.size(), 0.0), iv(policy) {
}

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

/*******************************************************************************/

SavitzkyGolay::SavitzkyGolay(int n, int m)
    : FIRFilter(n, Vector(m, &SG_SMOOTHING_COEF[ENSURE_BOUNDS(m, 2, 7)][0]), Signal) {
}

/*******************************************************************************/

NumericalDifferentiator::NumericalDifferentiator(int n, int m)
    : FIRFilter(n, Vector(m, &SG_DERIVATIVE_COEF[ENSURE_BOUNDS(m, 2, 7)][0]), Zero), t(0.0) {
}

Vector NumericalDifferentiator::diff(double tn, Vector xn) {
    Vector dx;
    if (t < tn) {
        dx = filter(xn) / (tn - t);
    } else {
        dx = filter(xn); //default is zero to void nan
    }
    t = tn;
    return dx;
}

/*******************************************************************************/
