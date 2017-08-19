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
