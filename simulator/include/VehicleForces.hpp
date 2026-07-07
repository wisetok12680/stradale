#pragma once

struct VehicleForces {
    double tractive_force = 0.0;            // N (forward propulsion)
    double drag_force = 0.0;                // N (rearward aerodynamic drag)
    double rolling_resistance_force = 0.0;  // N (rearward mechanical resistance)
    double gravity_force = 0.0;             // N (longitudinal weight component)
    double braking_force = 0.0;             // N (rearward braking force)
};
