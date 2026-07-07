#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include "../include/Vehicle.hpp"
#include "../include/Braking.hpp"
#include "../include/PhysicsConstants.hpp"

void setup_gt3_vehicle(Vehicle& car) {
    car.mass = 1300.0;
    car.front_weight_fraction = 0.46;
    car.wheelbase = 2.70;
    car.front_track_width = 1.67;
    car.rear_track_width = 1.65;
    car.cg_height = 0.32;
    car.frontal_area = 1.90;
    car.Cd = 0.34;
    car.Cl = -1.15;
    car.tyre_radius = 0.33;
    car.tyre_mu = 1.20;
    car.rolling_resistance_coeff = 0.015;
    car.maximum_braking_g = 1.8;
}

void test_braking_validation() {
    Vehicle car;
    setup_gt3_vehicle(car);

    std::cout << "==============================================================================\n";
    std::cout << "BRAKING FORCE & EQUIVALENT DECELERATION VALIDATION\n";
    std::cout << "==============================================================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(12) << "Pedal Input"
              << std::setw(15) << "Brake Force(N)"
              << std::setw(18) << "Decel (m/s²)"
              << std::setw(15) << "Decel (G)" << "\n";

    double test_pedals[] = { 0.0, 0.25, 0.50, 1.00, -0.10, 1.20 };

    for (double pedal : test_pedals) {
        double f_brake = Braking::compute_force(car, pedal);
        double decel_ms2 = f_brake / car.mass;
        double decel_g = decel_ms2 / PhysicsConstants::gravity;

        std::cout << std::setw(11) << (pedal * 100.0) << "%"
                  << std::setw(15) << f_brake
                  << std::setw(18) << decel_ms2
                  << std::setw(15) << decel_g << "\n";

        // Assert check values (with defensive clamping checks)
        if (pedal == 0.0 || pedal == -0.10) {
            assert(std::abs(f_brake - 0.0) < 1e-3);
        }
        if (pedal == 0.25) {
            assert(std::abs(f_brake - 5736.89) < 1e-2);
        }
        if (pedal == 0.50) {
            assert(std::abs(f_brake - 11473.78) < 1e-2);
        }
        if (pedal == 1.00 || pedal == 1.20) {
            assert(std::abs(f_brake - 22947.56) < 1e-2);
            assert(std::abs(decel_g - 1.8) < 1e-3);
        }
    }
    std::cout << "==============================================================================\n";
    std::cout << "Braking Subsystem Clamping & Deceleration Check: PASSED\n";
    std::cout << "==============================================================================\n";
}

int main() {
    test_braking_validation();
    return 0;
}
