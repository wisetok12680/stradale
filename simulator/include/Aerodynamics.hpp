#pragma once
#include "Vehicle.hpp"

struct AeroForces {
    double drag;              // N
    double downforce_front;   // N
    double downforce_rear;    // N
};

class Aerodynamics {
public:
    static AeroForces compute_forces(const Vehicle& vehicle, double velocity);
};
