#pragma once
#include "Vehicle.hpp"
#include "SimulationState.hpp"
#include "PhysicsConstants.hpp"
#include <algorithm>
#include <cmath>

struct PowertrainOutput {
    double engine_torque;     // Nm
    double wheel_torque;      // Nm
    double tractive_force;    // N
    double engine_rpm;        // rev/min
};

class Powertrain {
public:
    // Computes engine crankshaft RPM from wheel speed, clamped between idle and redline
    static double compute_engine_rpm(
        const Vehicle& vehicle,
        double velocity,      // m/s
        int gear              // dimensionless
    ) {
        if (gear < 1 || gear > static_cast<int>(vehicle.transmission.gear_ratios.size())) {
            return vehicle.engine.idle_rpm;
        }

        double G_gear = vehicle.transmission.gear_ratios[gear - 1];
        double G_final = vehicle.transmission.final_drive_ratio;

        // wheel_angular_velocity = velocity / tyre_radius
        double wheel_angular_velocity = velocity / vehicle.tyre_radius; // rad/s
        double engine_angular_velocity = wheel_angular_velocity * G_gear * G_final; // rad/s
        
        // Convert to rev/min: RPM = rad/s * 60 / 2pi
        double rpm = engine_angular_velocity * (60.0 / (2.0 * PhysicsConstants::pi)); // rev/min

        // Electronic rev limiter: clamp engine RPM between idle and redline
        return std::max(vehicle.engine.idle_rpm, std::min(rpm, vehicle.engine.redline_rpm));
    }

    // Computes dynamic engine torque and tractive forces for a given Simulation State
    static PowertrainOutput compute_forces(
        const Vehicle& vehicle,
        const VehicleState& state
    ) {
        PowertrainOutput output;
        
        // 1. Calculate Engine RPM
        output.engine_rpm = compute_engine_rpm(vehicle, state.velocity, state.current_gear);
        
        // 2. Lookup engine torque capability
        double max_torque_at_rpm = vehicle.engine.lookup_torque(output.engine_rpm);
        
        // 3. Modulate torque based on driver throttle input
        output.engine_torque = max_torque_at_rpm * state.throttle_position;

        // 4. Calculate wheel torque: Tw = Te * G_gear * G_final * efficiency
        double G_gear = (state.current_gear >= 1 && state.current_gear <= static_cast<int>(vehicle.transmission.gear_ratios.size()))
                        ? vehicle.transmission.gear_ratios[state.current_gear - 1]
                        : 0.0;
        double G_final = vehicle.transmission.final_drive_ratio;

        output.wheel_torque = output.engine_torque * G_gear * G_final * vehicle.transmission.drivetrain_efficiency;

        // 5. Calculate longitudinal tractive force: Ft = Tw / r_wheel
        output.tractive_force = output.wheel_torque / vehicle.tyre_radius;

        return output;
    }
};
