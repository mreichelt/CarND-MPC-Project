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
    AbsoluteWaypoints absolute = {{1.0},
                                  {2.0}};
    RelativeWaypoints expected = {{0.0},
                                  {0.0}};

    RelativeWaypoints actual = transformToRelative(absolute, {1.0, 2.0}, 0);
    REQUIRE(actual.x == expected.x);
    REQUIRE(actual.y == expected.y);

    // different angle should not change anything - (x,y) must still be converted to (0,0)
    actual = transformToRelative(absolute, {1.0, 2.0}, M_PI / 4);
    REQUIRE(actual.x == expected.x);
    REQUIRE(actual.y == expected.y);
}

TEST_CASE("Transform: point (1,1) should become (sqrt(2),0) with origin and angle pi/4") {
    AbsoluteWaypoints absolute = {{1.0},
                                  {1.0}};
    RelativeWaypoints actual = transformToRelative(absolute, {0.0, 0.0}, M_PI_4);
    REQUIRE(actual.x.front() == Approx(M_SQRT2));
    REQUIRE(actual.y.front() == Approx(0.0));
}

TEST_CASE("Transform: point (1,1) should become (-sqrt(2),0) with origin and angle 5 * pi/4") {
    AbsoluteWaypoints absolute = {{1.0},
                                  {1.0}};
    RelativeWaypoints actual = transformToRelative(absolute, {0.0, 0.0}, 5 * M_PI_4);
    REQUIRE(actual.x.front() == Approx(-M_SQRT2));
    REQUIRE(actual.y.front() == Approx(0.0));
}