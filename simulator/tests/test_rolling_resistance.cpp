#include <iostream>
#include <iomanip>
#include <assert.h>
#include <cmath>
#include "../include/Vehicle.hpp"
#include "../include/Aerodynamics.hpp"
#include "../include/RollingResistance.hpp"
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
}

void test_rolling_resistance_validation() {
    Vehicle car;
    setup_gt3_vehicle(car);

    std::cout << "==============================================================================\n";
    std::cout << "ROLLING RESISTANCE VS AERODYNAMIC DRAG FORCE VALIDATION\n";
    std::cout << "==============================================================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(6) << "Speed"
              << std::setw(12) << "Downforce"
              << std::setw(15) << "Normal Load"
              << std::setw(15) << "F_rr (N)"
              << std::setw(15) << "F_drag (N)"
              << std::setw(12) << "Drag/F_rr" << "\n";

    double test_speeds_kmh[] = { 0.0, 100.0, 200.0, 250.0 };

    for (double speed_kmh : test_speeds_kmh) {
        double v = speed_kmh / 3.6; // m/s
        
        // 1. Aerodynamic Forces
        AeroForces aero = Aerodynamics::compute_forces(car, v);
        double total_downforce = aero.downforce_front + aero.downforce_rear;
        
        // 2. Normal Load (mg + downforce)
        double static_load = car.mass * PhysicsConstants::gravity;
        double normal_load = static_load + total_downforce;

        // 3. Rolling Resistance
        double f_rr = RollingResistance::compute_force(car, normal_load);

        double drag_to_rr_ratio = (f_rr > 0) ? (aero.drag / f_rr) : 0.0;

        std::cout << std::setw(6) << static_cast<int>(speed_kmh)
                  << std::setw(12) << total_downforce
                  << std::setw(15) << normal_load
                  << std::setw(15) << f_rr
                  << std::setw(15) << aero.drag
                  << std::setw(12) << drag_to_rr_ratio << "\n";

        // Assert check values
        if (speed_kmh == 0.0) {
            assert(std::abs(total_downforce - 0.0) < 1e-3);
            assert(std::abs(normal_load - 12748.65) < 1e-2);
            assert(std::abs(f_rr - 191.23) < 1e-2);
        }
        if (speed_kmh == 100.0) {
            assert(std::abs(total_downforce - 1032.65) < 0.1);
            assert(std::abs(f_rr - 206.72) < 0.1);
        }
        if (speed_kmh == 250.0) {
            assert(std::abs(total_downforce - 6454.05) < 0.5);
            assert(std::abs(f_rr - 288.04) < 0.1);
            assert(std::abs(aero.drag - 1908.15) < 0.5);
        }
    }
    std::cout << "==============================================================================\n";
    std::cout << "Rolling Resistance & Aero Drag Verification: PASSED\n";
    std::cout << "==============================================================================\n";
}

int main() {
    test_rolling_resistance_validation();
    return 0;
}
