#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/tools.h"
#include "../src/MPC.h"

using namespace std;

TEST_CASE("Polynomials are evaluated correctly") {
    REQUIRE(polyeval(vecXd({1.0, 1.0}), 0.0) == Approx(1.0));
    REQUIRE(polyeval(vecXd({1.0, 1.0}), 2.0) == Approx(3.0));
    REQUIRE(polyeval(vecXd({0.0, 1.0, 2.0, 3.0}), 0.0) == Approx(0.0));
    REQUIRE(polyeval(vecXd({1.0, 2.0, 3.0, 4.0}), 0.0) == Approx(1.0));
}

TEST_CASE("Derivative of nothing should be nothing") {
    VectorXd vec = vecXd({});
    REQUIRE(derivative(vec) == vec);
}

TEST_CASE("Derivative of constant should be nothing") {
    REQUIRE(derivative(vecXd({1.0})) == vecXd({}));
}

TEST_CASE("Derivative of 1st degree polynomial should be constant") {
    REQUIRE(derivative(vecXd({1.0, 2.0})) == vecXd({2.0}));
}

TEST_CASE("Derivative of 3rd degree polynomial should be 2nd degree polynomial") {
    REQUIRE(derivative(vecXd({1.0, 2.0, 3.0, 4.0})) == vecXd({2.0, 6.0, 12.0}));
}

TEST_CASE("Transform absolute to relative coordinates: origin should be (0,0)") {
    AbsoluteWaypoints points = {{1.0},
                                {2.0}};
    RelativeWaypoints expected = {{0.0},
                                  {0.0}};

    RelativeWaypoints actual = transformToRelative(points, {1.0, 2.0}, 0);
    REQUIRE(actual.x == expected.x);
    REQUIRE(actual.y == expected.y);

    // different angle should not change anything - (x,y) must still be converted to (0,0)
    actual = transformToRelative(points, {1.0, 2.0}, M_PI / 4);
    REQUIRE(actual.x == expected.x);
    REQUIRE(actual.y == expected.y);
}

TEST_CASE("Transform: point (1, 1) should become (sqrt(2), 0) with origin (0, 0) and angle pi/4") {
    AbsoluteWaypoints points = {{1.0},
                                {1.0}};
    RelativeWaypoints actual = transformToRelative(points, {0.0, 0.0}, M_PI_4);
    REQUIRE(actual.x.front() == Approx(M_SQRT2));
    REQUIRE(actual.y.front() == Approx(0.0));
}

TEST_CASE("Transform: point (1, 1) should become (-sqrt(2), 0) with origin (0, 0) and angle 5 * pi/4") {
    AbsoluteWaypoints points = {{1.0},
                                {1.0}};
    RelativeWaypoints actual = transformToRelative(points, {0.0, 0.0}, 5 * M_PI_4);
    REQUIRE(actual.x.front() == Approx(-M_SQRT2));
    REQUIRE(actual.y.front() == Approx(0.0));
}

TEST_CASE("Transform: point (1, 3) should become (1, 1) with origin (2, 2) and angle pi/2") {
    AbsoluteWaypoints points = {{1.0},
                                {3.0}};
    RelativeWaypoints actual = transformToRelative(points, {2.0, 2.0}, M_PI_2);
    REQUIRE(actual.x.front() == Approx(1.0));
    REQUIRE(actual.y.front() == Approx(1.0));
}

TEST_CASE("MPC: running on x axis with no error should continue on x axis") {
    MPC mpc;
    VectorXd state = VectorXd::Zero(6);
    double
            x = 0,
            y = 0,
            psi = 0,
            v = ref_speed,
            cte = 0,
            epsi = 0;
    state << x, y, psi, v, cte, epsi;
    auto coeffs = vecXd({0.0, 0.0});    // straight line

    const MPCSolution &solution = mpc.Solve(state, coeffs);
    REQUIRE(solution.cost == Approx(0.0));
    REQUIRE(solution.steering_delta == Approx(0.0));
    REQUIRE(solution.acceleration == Approx(0.0));

    REQUIRE(solution.waypoints.x.size() == N);
    REQUIRE(solution.waypoints.y.size() == N);

    for (int i = 0; i < N; i++) {
        // drive on the x axis beginning at (0, 0) -> just calculate driven distance
        REQUIRE(solution.waypoints.x[i] == Approx(dt * ref_speed * i));

        // car drives on x axis, therefore y stays 0
        REQUIRE(solution.waypoints.y[i] == Approx(0.0));
    }
}

TEST_CASE("MPC: driving on x axis slower than reference speed should accelerate") {
    MPC mpc;
    VectorXd state = VectorXd::Zero(6);
    double
            x = 0,
            y = 0,
            psi = 0,
            v = ref_speed * 0.9,
            cte = 0,
            epsi = 0;
    state << x, y, psi, v, cte, epsi;
    auto coeffs = vecXd({0.0, 0.0});    // straight line

    const MPCSolution &solution = mpc.Solve(state, coeffs);
    REQUIRE(solution.cost > 0.0);
    REQUIRE(solution.steering_delta == Approx(0.0));
    REQUIRE(solution.acceleration > 0.0);

    REQUIRE(solution.waypoints.x.size() == N);
    REQUIRE(solution.waypoints.y.size() == N);

    for (int i = 0; i < N; i++) {
        // just check that we continue driving on x axis
        if (i > 0) {
            REQUIRE(solution.waypoints.x[i] > 0.0);
        }
        REQUIRE(solution.waypoints.y[i] == Approx(0.0));
    }
}

TEST_CASE("MPC: driving on x axis faster than reference speed should decelerate") {
    MPC mpc;
    VectorXd state = VectorXd::Zero(6);
    double
            x = 0,
            y = 0,
            psi = 0,
            v = ref_speed * 1.1,
            cte = 0,
            epsi = 0;
    state << x, y, psi, v, cte, epsi;
    auto coeffs = vecXd({0.0, 0.0});    // straight line

    const MPCSolution &solution = mpc.Solve(state, coeffs);
    REQUIRE(solution.cost > 0.0);
    REQUIRE(solution.steering_delta == Approx(0.0));
    REQUIRE(solution.acceleration < 0.0);

    REQUIRE(solution.waypoints.x.size() == N);
    REQUIRE(solution.waypoints.y.size() == N);

    for (int i = 0; i < N; i++) {
        // just check that we continue driving on x axis
        if (i > 0) {
            REQUIRE(solution.waypoints.x[i] > 0.0);
        }
        REQUIRE(solution.waypoints.y[i] == Approx(0.0));
    }
}

TEST_CASE("MPC: driving to the right of desired lane should steer left") {
    MPC mpc;
    auto coeffs = vecXd({1.0, 0.0});    // straight line at y == 1.0
    VectorXd state = VectorXd::Zero(6);
    double
            x = 0,
            y = 0,
            psi = 0,
            v = ref_speed,
            cte = polyeval(coeffs, x),
            epsi = 0;
    state << x, y, psi, v, cte, epsi;

    const MPCSolution &solution = mpc.Solve(state, coeffs);
    REQUIRE(solution.cost > 0.0);
    REQUIRE(solution.steering_delta > 0.0);
}

TEST_CASE("MPC: driving to the left of the x axis should steer right") {
    MPC mpc;
    auto coeffs = vecXd({-1.0, 0.0});    // straight line at y == -1.0
    VectorXd state = VectorXd::Zero(6);
    double
            x = 0,
            y = 0,
            psi = 0,
            v = ref_speed,
            cte = polyeval(coeffs, x),
            epsi = 0;
    state << x, y, psi, v, cte, epsi;

    const MPCSolution &solution = mpc.Solve(state, coeffs);
    REQUIRE(solution.cost > 0.0);
    REQUIRE(solution.steering_delta < 0.0);
}
