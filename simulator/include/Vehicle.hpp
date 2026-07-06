#pragma once

#include "Engine.hpp"

struct Vehicle {
    double mass;                     // kg
    double wheelbase;                // m
    double front_track_width;        // m
    double rear_track_width;         // m
    double center_of_gravity_height; // m
    double frontal_area;             // m^2
    double drag_coefficient;         // Cd
    double rolling_resistance_coeff; // Crr
    double tyre_radius;              // m
    double final_drive_ratio;        // ratio
    
    Engine engine;
};
