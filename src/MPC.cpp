#include "MPC.h"
#include "tools.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <utility>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

typedef CPPAD_TESTVECTOR(double) Dvector;

// options for IPOPT solver
const string ipopt_options = "Integer print_level  0\n"
        "Sparse  true        forward\n"
        "Sparse  true        reverse\n"
        "Numeric max_cpu_time          0.25\n";

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

const vector<string> ipopt_status_type{
        "not_defined",
        "success",
        "maxiter_exceeded",
        "stop_at_tiny_step",
        "stop_at_acceptable_point",
        "local_infeasibility",
        "user_requested_stop",
        "feasible_point_found",
        "diverging_iterates",
        "restoration_failure",
        "error_in_step_computation",
        "invalid_number_detected",
        "too_few_degrees_of_freedom",
        "internal_error",
        "unknown"
};


class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(VectorXd coeffs) : coeffs(std::move(coeffs)) {}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // The part of the cost based on the reference state.
        const double
                weight_cte = 1,
                weight_epsi = 1,
                weight_speed = 0.0001,
                weight_minimize_steering = 0.001,
                weight_minimize_throttle = 0.001,
                weight_minimize_steering_gaps = 0.3,
                weight_minimize_throttle_gaps = 0.1;

        fg[0] = 0.0;

        for (int t = 0; t < N; t++) {
            fg[0] += weight_cte * CppAD::pow(vars[cte_start + t], 2);
            fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += weight_speed * CppAD::pow(vars[v_start + t] - ref_speed, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += weight_minimize_steering * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += weight_minimize_throttle * CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += weight_minimize_steering_gaps * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += weight_minimize_throttle_gaps * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
            AD<double>
            // state at time t + 1
                    x1 = vars[x_start + t],
                    y1 = vars[y_start + t],
                    psi1 = vars[psi_start + t],
                    v1 = vars[v_start + t],
                    cte1 = vars[cte_start + t],
                    epsi1 = vars[epsi_start + t],

            // state at time t
                    x0 = vars[x_start + t - 1],
                    y0 = vars[y_start + t - 1],
                    psi0 = vars[psi_start + t - 1],
                    v0 = vars[v_start + t - 1],
                    cte0 = vars[cte_start + t - 1],
                    epsi0 = vars[epsi_start + t - 1],
                    delta0 = vars[delta_start + t - 1],
                    a0 = vars[a_start + t - 1],

                    f_x0 = polyeval(coeffs, x0),
                    psi_des0 = CppAD::atan(polyeval(derivative(coeffs), x0));

            // model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - (f_x0 - y0 + v0 * CppAD::sin(epsi0) * dt);
            fg[1 + epsi_start + t] = epsi1 - (psi0 - psi_des0 + v0 / Lf * delta0 * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() = default;

MPC::~MPC() = default;

MPCSolution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    double
            x = state[0],
            y = state[1],
            psi = state[2],
            v = state[3],
            cte = state[4],
            epsi = state[5];

    size_t n_vars = N * 6 + (N - 1) * 2;
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);


    // Set all non-actuators upper and lower limits to the max negative and positive values.
    for (size_t i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (size_t i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -deg2rad25;
        vars_upperbound[i] = deg2rad25;
    }

    // Acceleration upper and lower limits.
    for (size_t i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // initial state lower bounds
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    // initial state upper bounds
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;


    // object that computes objective and constraints
    FG_eval fg_eval(std::move(coeffs));

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            ipopt_options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
        throw runtime_error("ipopt.solution.status == " +
                            ipopt_status_type[solution.status - CppAD::ipopt::solve_result<Dvector>::not_defined]);
    }

    // Cost
    auto cost = solution.obj_value;
//    std::cout << "Cost " << cost << std::endl;


    vector<double> predicted_x(N);
    vector<double> predicted_y(N);

    for (int i = 0; i < N; i++) {
        predicted_x[i] = solution.x[x_start + i];
        predicted_y[i] = solution.x[y_start + i];
    }

    MPCSolution mpcSolution = {
            solution.x[delta_start],
            solution.x[a_start],
            cost,
            {predicted_x, predicted_y}
    };
    return mpcSolution;
}
