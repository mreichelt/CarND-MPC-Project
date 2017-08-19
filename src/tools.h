#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace Eigen;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

const double deg2rad25 = deg2rad(25);

// Evaluate a polynomial.
double polyeval(VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
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

#endif //MPC_TOOLS_H
