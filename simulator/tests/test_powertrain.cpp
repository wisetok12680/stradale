#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include "../include/Vehicle.hpp"
#include "../include/Powertrain.hpp"
#include "../include/PhysicsConstants.hpp"

// Setup baseline GT3 engine torque curve nodes
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

    // Engine Specs
    car.engine.maximum_power = 370000.0;     // 370 kW
    car.engine.maximum_torque = 500.0;       // 500 Nm
    car.engine.idle_rpm = 1000.0;
    car.engine.redline_rpm = 8500.0;

    // Torque Curve Definition
    car.engine.torque_curve = {
        { 1000.0, 340.0 }, // Idle
        { 4000.0, 460.0 },
        { 6200.0, 500.0 }, // Peak Torque
        { 7200.0, 490.0 }, // Peak Power (~370 kW)
        { 8500.0, 410.0 }  // Redline
    };

    // Transmission Specs
    car.transmission.gear_ratios = { 3.10, 2.20, 1.65, 1.30, 1.10, 0.95 };
    car.transmission.final_drive_ratio = 4.10;
    car.transmission.drivetrain_efficiency = 0.92;
}

void test_powertrain_operating_points() {
    Vehicle car;
    setup_gt3_vehicle(car);

    std::cout << "=========================================================\n";
    std::cout << "POWERTRAIN SUBSYSTEM OPERATING POINT VALIDATION\n";
    std::cout << "=========================================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(6) << "RPM"
              << std::setw(15) << "Torque (Nm)"
              << std::setw(15) << "Power (kW)"
              << std::setw(18) << "Wheel Torque(Nm)"
              << std::setw(18) << "Tractive Force(N)" << "\n";

    double test_rpms[] = { 1000.0, 3000.0, 6200.0, 7200.0, 8500.0 };

    for (double rpm : test_rpms) {
        // Find corresponding vehicle speed in Gear 1 to produce this RPM
        // RPM = wheel_rad_s * G_gear * G_final * 60 / 2pi => wheel_rad_s = RPM / (G_gear * G_final * 60 / 2pi)
        // velocity = wheel_rad_s * tyre_radius
        double G_gear = car.transmission.gear_ratios[0]; // Gear 1
        double G_final = car.transmission.final_drive_ratio;
        double engine_angular_vel = rpm * (2.0 * PhysicsConstants::pi / 60.0);
        double wheel_angular_vel = engine_angular_vel / (G_gear * G_final);
        double velocity = wheel_angular_vel * car.tyre_radius;

        // Initialize state
        VehicleState state;
        state.velocity = velocity;
        state.current_gear = 1;
        state.throttle_position = 1.0; // full throttle

        // Compute output
        PowertrainOutput out = Powertrain::compute_forces(car, state);

        // Power = Torque * omega (rad/s)
        double omega_engine = out.engine_rpm * (2.0 * PhysicsConstants::pi / 60.0);
        double power_w = out.engine_torque * omega_engine;
        double power_kw = power_w / 1000.0;

        std::cout << std::setw(6) << static_cast<int>(out.engine_rpm)
                  << std::setw(15) << out.engine_torque
                  << std::setw(15) << power_kw
                  << std::setw(18) << out.wheel_torque
                  << std::setw(18) << out.tractive_force << "\n";

        // Assert check values
        if (std::abs(rpm - 1000.0) < 1e-3) {
            assert(std::abs(out.engine_torque - 340.0) < 1e-3);
            assert(std::abs(out.tractive_force - 12047.33) < 1.0);
        }
        if (std::abs(rpm - 6200.0) < 1e-3) {
            assert(std::abs(out.engine_torque - 500.0) < 1e-3);
        }
        if (std::abs(rpm - 7200.0) < 1e-3) {
            // Power should be close to 370 kW
            assert(std::abs(power_kw - 369.45) < 0.1);
        }
    }
    std::cout << "=========================================================\n";
    std::cout << "Powertrain Operating Points Check: PASSED\n";
    std::cout << "=========================================================\n";
}

int main() {
    test_powertrain_operating_points();
    return 0;
}
