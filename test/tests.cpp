#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/tools.h"
#include "../src/MPC.h"

using namespace std;

TEST_CASE("Polynomials are evaluated correctly", "[polyeval]") {
    vector<double>
            vec1 = {1.0, 1.0},
            vec2 = {0.0, 1.0, 2.0, 3.0},
            vec3 = {1.0, 2.0, 3.0, 4.0};
    vecXd(vec1);
    REQUIRE(polyeval(vecXd(vec1), 0.0) == Approx(1.0));
    REQUIRE(polyeval(vecXd(vec1), 2.0) == Approx(3.0));
    REQUIRE(polyeval(vecXd(vec2), 0.0) == Approx(0.0));
    REQUIRE(polyeval(vecXd(vec3), 0.0) == Approx(1.0));
}
