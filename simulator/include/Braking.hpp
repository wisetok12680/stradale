#pragma once
#include "Vehicle.hpp"
#include "PhysicsConstants.hpp"
#include <algorithm>

class Braking {
public:
    // Computes longitudinal braking force opposing forward motion (always positive, in Newtons)
    // Input brake_position is clamped between 0.0 and 1.0 defensively
    // Version 1 assumption: Infinite tire grip is available (no lockup/ABS)
    static double compute_force(
        const Vehicle& vehicle,
        double brake_position // [0.0 to 1.0]
    ) {
        double clamped_pedal = std::max(0.0, std::min(brake_position, 1.0));
        double max_deceleration = vehicle.maximum_braking_g * PhysicsConstants::gravity; // m/s²
        double max_brake_force = vehicle.mass * max_deceleration; // N
        
        return max_brake_force * clamped_pedal; // N
    }
};
