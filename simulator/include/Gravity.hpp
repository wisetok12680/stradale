#pragma once
#include "PhysicsConstants.hpp"
#include <cmath>

class Gravity {
public:
    // Computes longitudinal grade force component (in Newtons)
    // Positive Force: opposes forward motion (uphill resistance)
    // Negative Force: assists forward motion (downhill assistance)
    static double compute_force(
        double mass,  // kg
        double grade  // dimensionless slope incline
    ) {
        double sin_theta = grade / std::sqrt(1.0 + (grade * grade));
        return mass * PhysicsConstants::gravity * sin_theta; // N
    }
};
