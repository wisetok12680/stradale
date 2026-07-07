#pragma once
#include "VehicleForces.hpp"

class ForceAccumulator {
public:
    // Accumulates all propulsive and resistive forces acting on the vehicle
    // Sign Convention: Positive resists motion (drag, rolling resistance, braking).
    // Grade gravity component has positive opposing and negative assisting.
    static double compute_net_longitudinal_force(
        const VehicleForces& forces,
        double velocity // m/s
    ) {
        // Signum direction check: ensure resistive forces act opposite to velocity direction
        double direction = (velocity >= 1e-4) ? 1.0 : ((velocity <= -1e-4) ? -1.0 : 0.0);
        
        if (direction == 0.0) {
            // Standstill: resistive forces cannot pull the vehicle backward.
            // Only propulsion (tractive force) and gravity grade components are active.
            return forces.tractive_force - forces.gravity_force;
        }
        
        // Sum forces: tractive pushing forward, other resistive forces acting against motion direction
        return forces.tractive_force 
               - (forces.drag_force * direction)
               - (forces.rolling_resistance_force * direction)
               - forces.gravity_force
               - (forces.braking_force * direction);
    }
};
