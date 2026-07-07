#pragma once
#include "Vehicle.hpp"

class RollingResistance {
public:
    // Computes retarding rolling resistance force from dynamic normal load input (always positive, in Newtons)
    // Version 1 assumption: Constant rolling resistance coefficient (C_rr)
    static double compute_force(
        const Vehicle& vehicle,
        double normal_load // N
    ) {
        return vehicle.rolling_resistance_coeff * normal_load; // N
    }
};
