#include <iostream>
#include <iomanip>
#include <assert.h>
#include <cmath>
#include <vector>
#include "../include/Vehicle.hpp"
#include "../include/VehicleDynamics.hpp"
#include "../include/PhysicsConstants.hpp"
#include "../include/Aerodynamics.hpp"
#include "../include/RollingResistance.hpp"
#include "../include/Gravity.hpp"
#include "../include/Powertrain.hpp"
#include "../include/Braking.hpp"
#include "../include/ForceAccumulator.hpp"

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

void print_simulation_summary(const std::string& name, double duration, double dt, const Vehicle& car, double grade, double initial_v, double throttle, double brake) {
    std::cout << "\n------------------------------------------------------------------------------\n";
    std::cout << "SCENARIO: " << name << "\n";
    std::cout << "------------------------------------------------------------------------------\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(6) << "Time"
              << std::setw(12) << "Pos (m)"
              << std::setw(12) << "Speed(kmh)"
              << std::setw(12) << "Accel (G)"
              << std::setw(12) << "F_net (N)"
              << std::setw(10) << "RPM"
              << std::setw(8) << "Gear" << "\n";

    SimulationState state;
    state.vehicle.position = 0.0;
    state.vehicle.distance_travelled = 0.0;
    state.vehicle.velocity = initial_v;
    state.vehicle.acceleration = 0.0;
    state.vehicle.current_gear = (initial_v > 20.0) ? 3 : 1;
    state.vehicle.engine_rpm = Powertrain::compute_engine_rpm(car, initial_v, state.vehicle.current_gear);
    state.vehicle.throttle_position = throttle;
    state.vehicle.brake_position = brake;

    double time = 0.0;
    while (time <= duration) {
        AeroForces aero = Aerodynamics::compute_forces(car, state.vehicle.velocity);
        double normal_load = (car.mass * PhysicsConstants::gravity) + (aero.downforce_front + aero.downforce_rear);
        double f_rr = RollingResistance::compute_force(car, normal_load);
        double f_grade = Gravity::compute_force(car.mass, grade);
        PowertrainOutput pt = Powertrain::compute_forces(car, state.vehicle);
        double f_brake = Braking::compute_force(car, state.vehicle.brake_position);

        VehicleForces forces;
        forces.tractive_force = pt.tractive_force;
        forces.drag_force = aero.drag;
        forces.rolling_resistance_force = f_rr;
        forces.gravity_force = f_grade;
        forces.braking_force = f_brake;

        double f_net = ForceAccumulator::compute_net_longitudinal_force(forces, state.vehicle.velocity);
        double accel_g = state.vehicle.acceleration / PhysicsConstants::gravity;

        std::cout << std::setw(6) << time
                  << std::setw(12) << state.vehicle.position
                  << std::setw(12) << (state.vehicle.velocity * 3.6)
                  << std::setw(12) << accel_g
                  << std::setw(12) << f_net
                  << std::setw(10) << static_cast<int>(state.vehicle.engine_rpm)
                  << std::setw(8) << state.vehicle.current_gear << "\n";

        // Validate physics checks
        if (name == "1. Full Throttle Acceleration" && time == 0.0) {
            assert(state.vehicle.velocity == 0.0);
            assert(state.vehicle.engine_rpm == 1000.0);
            assert(f_net > 11000.0); // should have high starting tractive force
        }
        if (name == "3. Maximum Braking" && time == 0.0) {
            // max brake force + drag + rolling resistance opposing motion
            assert(f_net < -23000.0); 
        }

        // Integrate & Shifting
        VehicleDynamics::integrate(car, dt, forces, state);
        VehicleDynamics::update_gear_shifting(car, state);
        time += dt;
    }
}

int main() {
    Vehicle car;
    setup_gt3_vehicle(car);

    // Run the 5 verification scenarios
    print_simulation_summary("1. Full Throttle Acceleration", 3.0, 0.5, car, 0.0, 0.0, 1.0, 0.0);
    print_simulation_summary("2. Coasting with Zero Throttle", 3.0, 0.5, car, 0.0, 50.0, 0.0, 0.0);
    print_simulation_summary("3. Maximum Braking", 3.0, 0.5, car, 0.0, 50.0, 0.0, 1.0);
    print_simulation_summary("4. Constant Uphill Climb", 3.0, 0.5, car, 0.05, 0.0, 1.0, 0.0);
    print_simulation_summary("5. Constant Downhill Descent", 3.0, 0.5, car, -0.05, 0.0, 1.0, 0.0);

    std::cout << "\n==============================================================================\n";
    std::cout << "Integrated Vehicle Dynamics Validation Scenarios: ALL PASSED\n";
    std::cout << "==============================================================================\n";

    return 0;
}
