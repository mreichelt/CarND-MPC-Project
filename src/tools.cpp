#include "tools.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace Eigen;
using namespace std;
using CppAD::AD;

double deg2rad(double x) { return x * M_PI / 180; }

double rad2deg(double x) { return x * 180 / M_PI; }

// Evaluate a polynomial.
double polyeval(VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

AD<double> polyeval(VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * CppAD::pow(x, i);
    }
    return result;
}

VectorXd derivative(VectorXd coeffs) {
    if (coeffs.size() == 0) {
        return coeffs;
    }
    VectorXd derivative = VectorXd::Zero(coeffs.size() - 1);
    for (int i = 1; i < coeffs.size(); i++) {
        derivative[i - 1] = coeffs[i] * i;
    }
    return derivative;
}

VectorXd vecXd(vector<double> vec) {
    VectorXd vectorXd(vec.size());
    for (int i = 0; i < vec.size(); i++) {
        vectorXd[i] = vec[i];
    }
    return vectorXd;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

RelativeWaypoints transformToRelative(AbsoluteWaypoints absolute, Waypoint origin, double psi) {
    Waypoints result;
    for (int i = 0; i < absolute.x.size(); i++) {
        double
                delta_x = absolute.x[i] - origin.x,
                delta_y = absolute.y[i] - origin.y,
                x = cos(psi) * delta_x + sin(psi) * delta_y,
                y = -sin(psi) * delta_x + cos(psi) * delta_y;
        result.x.push_back(x);
        result.y.push_back(y);
    }
    return result;
}

double to_meters_per_second(double miles_per_hour) {
    return miles_per_hour * 0.44704;
}

double to_miles_per_hour(double meters_per_second) {
    return meters_per_second / 0.44704;
}
