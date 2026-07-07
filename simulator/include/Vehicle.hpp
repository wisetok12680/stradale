#pragma once
#include "Engine.hpp"
#include "Transmission.hpp"

struct Vehicle {
    double mass = 1300.0;                     // kg
    double front_weight_fraction = 0.46;      // dimensionless (46% front, 54% rear)
    double wheelbase = 2.70;                  // m
    double front_track_width = 1.67;          // m
    double rear_track_width = 1.65;           // m
    double cg_height = 0.32;                  // m
    double frontal_area = 1.90;               // m²
    
    // Aerodynamics
    double Cd = 0.34;                         // dimensionless
    double Cl = -1.15;                        // dimensionless (lift coefficient, negative is downforce)
    
    // Tyres & Friction
    double tyre_radius = 0.33;                // m
    double tyre_mu = 1.20;                    // dimensionless
    double rolling_resistance_coeff = 0.015;  // dimensionless
    double maximum_braking_g = 1.8;           // dimensionless (1.8 G)
    
    Engine engine;
    Transmission transmission;
};
