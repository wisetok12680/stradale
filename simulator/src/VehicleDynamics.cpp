#include "../include/VehicleDynamics.hpp"
#include "../include/PhysicsConstants.hpp"
#include "../include/Aerodynamics.hpp"
#include "../include/RollingResistance.hpp"
#include "../include/Gravity.hpp"
#include "../include/Powertrain.hpp"
#include "../include/Braking.hpp"
#include "../include/ForceAccumulator.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

void VehicleDynamics::integrate(
    const Vehicle& vehicle,
    double dt,
    const VehicleForces& forces,
    SimulationState& state
)
{
    // Accumulate forces using central sign convention
    double F_net = ForceAccumulator::compute_net_longitudinal_force(forces, state.vehicle.velocity);

    // Compute longitudinal acceleration (a = F_net / m)
    state.vehicle.acceleration = F_net / vehicle.mass;

    // Numerical Integration: Explicit Euler
    double v_new = state.vehicle.velocity + state.vehicle.acceleration * dt;

    // Clamp velocity to prevent negative speeds (due to resistive grade/drag forces)
    if (v_new < 0.0)
    {
        v_new = 0.0;
        state.vehicle.acceleration = (v_new - state.vehicle.velocity) / dt; // recalculate acceleration
    }

    state.vehicle.velocity = v_new;

    // Update positions and distances
    state.vehicle.distance_travelled += state.vehicle.velocity * dt;
    state.vehicle.position += state.vehicle.velocity * dt;

    // Update engine RPM based on new wheel angular speed
    state.vehicle.engine_rpm = Powertrain::compute_engine_rpm(vehicle, state.vehicle.velocity, state.vehicle.current_gear);
}

void VehicleDynamics::update_gear_shifting(
    const Vehicle& vehicle,
    SimulationState& state
)
{
    // Automatic Gearbox Controller:
    // Shift up if RPM climbs above 7200 RPM (peak power band)
    if (state.vehicle.engine_rpm > 7200.0 && state.vehicle.current_gear < 6)
    {
        state.vehicle.current_gear++;
    }
    // Shift down if RPM drops below 3500 RPM to keep in powerband
    else if (state.vehicle.engine_rpm < 3500.0 && state.vehicle.current_gear > 1)
    {
        state.vehicle.current_gear--;
    }
}

std::vector<SimulationStep> VehicleDynamics::run_straight_simulation(
    const Vehicle& vehicle,
    double grade,
    double duration,
    double dt
)
{
    std::vector<SimulationStep> log;
    
    // Initialize V2 Simulation State
    SimulationState state;
    state.vehicle.position = 0.0;
    state.vehicle.distance_travelled = 0.0;
    state.vehicle.velocity = 0.0;
    state.vehicle.acceleration = 0.0;
    state.vehicle.current_gear = 1;
    state.vehicle.engine_rpm = vehicle.engine.idle_rpm;
    state.vehicle.throttle_position = 1.0; // full acceleration
    state.vehicle.brake_position = 0.0;

    double time = 0.0;
    int steps = static_cast<int>(duration / dt);
    log.reserve(steps + 1);

    while (time <= duration)
    {
        // 1. Aerodynamic Forces
        AeroForces aero = Aerodynamics::compute_forces(vehicle, state.vehicle.velocity);
        
        // 2. Rolling Resistance Forces
        double normal_load = (vehicle.mass * PhysicsConstants::gravity) + (aero.downforce_front + aero.downforce_rear);
        double f_rr = RollingResistance::compute_force(vehicle, normal_load);
        
        // 3. Gravity Grade Force
        double f_grade = Gravity::compute_force(vehicle.mass, grade);
        
        // 4. Powertrain Forces
        PowertrainOutput pt = Powertrain::compute_forces(vehicle, state.vehicle);
        
        // 5. Braking Force
        double f_brake = Braking::compute_force(vehicle, state.vehicle.brake_position);

        // Populate forces accumulation struct
        VehicleForces forces;
        forces.tractive_force = pt.tractive_force;
        forces.drag_force = aero.drag;
        forces.rolling_resistance_force = f_rr;
        forces.gravity_force = f_grade;
        forces.braking_force = f_brake;

        // Log pre-integrated state (Euler standard)
        log.push_back(SimulationStep{
            time,
            state.vehicle.position,
            state.vehicle.velocity,
            state.vehicle.acceleration,
            forces.tractive_force,
            forces.drag_force,
            forces.rolling_resistance_force,
            forces.gravity_force
        });

        // 6. Integrate Equations of Motion
        integrate(vehicle, dt, forces, state);

        // 7. Manage Gear Shifting
        update_gear_shifting(vehicle, state);

        time += dt;
    }

    return log;
}

bool VehicleDynamics::export_to_json(
    const std::vector<SimulationStep>& log,
    const std::string& filepath
)
{
    std::ofstream out(filepath);
    if (!out.is_open())
    {
        return false;
    }

    out << std::fixed << std::setprecision(6);
    out << "[\n";
    for (size_t i = 0; i < log.size(); ++i)
    {
        const auto& step = log[i];
        out << "  {\n";
        out << "    \"time\": " << step.time << ",\n";
        out << "    \"position\": " << step.position << ",\n";
        out << "    \"velocity\": " << step.velocity << ",\n";
        out << "    \"acceleration\": " << step.acceleration << ",\n";
        out << "    \"F_engine\": " << step.F_engine << ",\n";
        out << "    \"F_drag\": " << step.F_drag << ",\n";
        out << "    \"F_rr\": " << step.F_rr << ",\n";
        out << "    \"F_grade\": " << step.F_grade << "\n";
        out << "  }" << (i == log.size() - 1 ? "" : ",") << "\n";
    }
    out << "]\n";

    return true;
}

std::vector<LapSimulationStep> VehicleDynamics::run_lap_simulation(
    const Vehicle& vehicle,
    const Track& track,
    double dt
)
{
    std::vector<LapSimulationStep> log;
    if (track.points.empty() || track.segments.empty())
    {
        return log;
    }

    // Initialize V2 Simulation State
    SimulationState state;
    state.vehicle.position = 0.0;
    state.vehicle.distance_travelled = 0.0;
    state.vehicle.velocity = 0.0;
    state.vehicle.acceleration = 0.0;
    state.vehicle.current_gear = 1;
    state.vehicle.engine_rpm = vehicle.engine.idle_rpm;
    state.vehicle.throttle_position = 1.0;
    state.vehicle.brake_position = 0.0;

    double time = 0.0;
    double s = 0.0; // distance along track (m)
    double total_length = track.total_length;

    // Precompute cumulative segment start distances
    std::vector<double> dist_cum;
    dist_cum.reserve(track.points.size());
    double cum_dist = 0.0;
    dist_cum.push_back(0.0);
    for (size_t i = 0; i < track.segments.size(); ++i)
    {
        cum_dist += track.segments[i].length;
        dist_cum.push_back(cum_dist);
    }

    log.reserve(2500);
    size_t current_seg_idx = 0;

    while (s < total_length)
    {
        // Find current track segment
        while (current_seg_idx < track.segments.size() - 1 && s >= dist_cum[current_seg_idx + 1])
        {
            current_seg_idx++;
        }

        double grade = track.segments[current_seg_idx].grade;
        double curvature = track.segments[current_seg_idx].curvature;
        double seg_len = track.segments[current_seg_idx].length;

        // 1. Target cornering speed limit (based on curvature & tire friction mu)
        double v_limit = 1e9;
        if (std::abs(curvature) > 0.002)
        {
            v_limit = std::sqrt((vehicle.tyre_mu * PhysicsConstants::gravity) / std::abs(curvature));
        }

        // 2. Driver Control Loop: Decide throttle and brake pedal positions
        if (state.vehicle.velocity > v_limit)
        {
            state.vehicle.throttle_position = 0.0;
            
            // Decel rate to decelerate to v_limit
            double target_decel = (v_limit - state.vehicle.velocity) / dt;
            double max_decel = -vehicle.maximum_braking_g * PhysicsConstants::gravity;
            double actual_target_decel = std::max(max_decel, target_decel); // capped at -1.8 G

            // Backward force calculation to match required deceleration
            double f_drag = Aerodynamics::compute_forces(vehicle, state.vehicle.velocity).drag;
            double normal_load = (vehicle.mass * PhysicsConstants::gravity) 
                               + (Aerodynamics::compute_forces(vehicle, state.vehicle.velocity).downforce_front 
                                  + Aerodynamics::compute_forces(vehicle, state.vehicle.velocity).downforce_rear);
            double f_rr = RollingResistance::compute_force(vehicle, normal_load);
            double f_grade = Gravity::compute_force(vehicle.mass, grade);
            
            double req_brake_force = -f_drag - f_rr - f_grade - (vehicle.mass * actual_target_decel);
            double max_brake_force = vehicle.mass * vehicle.maximum_braking_g * PhysicsConstants::gravity;
            
            state.vehicle.brake_position = std::max(0.0, std::min(req_brake_force / max_brake_force, 1.0));
        }
        else
        {
            state.vehicle.throttle_position = 1.0;
            state.vehicle.brake_position = 0.0;
        }

        // 3. Compute forces from modules
        AeroForces aero = Aerodynamics::compute_forces(vehicle, state.vehicle.velocity);
        
        double normal_load = (vehicle.mass * PhysicsConstants::gravity) 
                           + (aero.downforce_front + aero.downforce_rear);
        double f_rr = RollingResistance::compute_force(vehicle, normal_load);
        
        double f_grade = Gravity::compute_force(vehicle.mass, grade);
        
        PowertrainOutput pt = Powertrain::compute_forces(vehicle, state.vehicle);
        
        double f_brake = Braking::compute_force(vehicle, state.vehicle.brake_position);

        // 4. Populate forces struct
        VehicleForces forces;
        forces.tractive_force = pt.tractive_force;
        forces.drag_force = aero.drag;
        forces.rolling_resistance_force = f_rr;
        forces.gravity_force = f_grade;
        forces.braking_force = f_brake;

        // 5. Interpolate 2D coordinates along the track segment
        double tau = 0.0;
        if (seg_len > 1e-5)
        {
            tau = (s - dist_cum[current_seg_idx]) / seg_len;
            if (tau < 0.0) tau = 0.0;
            if (tau > 1.0) tau = 1.0;
        }

        double car_x = (1.0 - tau) * track.points[current_seg_idx].x_coordinate 
                     + tau * track.points[current_seg_idx + 1].x_coordinate;
        double car_y = (1.0 - tau) * track.points[current_seg_idx].y_coordinate 
                     + tau * track.points[current_seg_idx + 1].y_coordinate;

        // Log pre-integrated state
        log.push_back(LapSimulationStep{
            time,
            s,
            state.vehicle.velocity,
            state.vehicle.acceleration,
            car_x,
            car_y,
            forces.tractive_force,
            forces.drag_force,
            forces.rolling_resistance_force,
            forces.gravity_force,
            curvature
        });

        // Loop safety guard
        if (log.size() > 15000)
        {
            std::cout << "\n[SAFETY TRIGGERED] Simulation step limit exceeded (>15000 steps).\n";
            std::cout << "Diagnostics at crash point:\n"
                      << "  Time:               " << time << " s\n"
                      << "  Distance (s):       " << s << " m\n"
                      << "  Velocity (v):       " << state.vehicle.velocity << " m/s (" << (state.vehicle.velocity * 3.6) << " km/h)\n"
                      << "  Acceleration (a):   " << state.vehicle.acceleration << " m/s²\n"
                      << "  Current Segment:    " << current_seg_idx << " (of " << track.segments.size() << ")\n"
                      << "  Curvature:          " << curvature << " m^-1\n"
                      << "  Speed Limit (v_lim):" << v_limit << " m/s\n"
                      << "  Gear:               " << state.vehicle.current_gear << "\n"
                      << "  Engine RPM:         " << state.vehicle.engine_rpm << "\n"
                      << "  Throttle Position:  " << state.vehicle.throttle_position << "\n"
                      << "  Brake Position:     " << state.vehicle.brake_position << "\n"
                      << "  Tractive Force:     " << forces.tractive_force << " N\n"
                      << "  Braking Force:      " << forces.braking_force << " N\n"
                      << "  Drag Force:         " << forces.drag_force << " N\n"
                      << "  Rolling Resistance: " << forces.rolling_resistance_force << " N\n"
                      << "  Gravity Force:      " << forces.gravity_force << " N\n";
            break;
        }

        // 6. Integrate equations of motion using Explicit Euler
        integrate(vehicle, dt, forces, state);

        // 7. Update gear shifts
        update_gear_shifting(vehicle, state);

        // Advance track progress metrics
        s = state.vehicle.position;
        time += dt;
    }

    return log;
}

bool VehicleDynamics::export_lap_to_json(
    const std::vector<LapSimulationStep>& log,
    const std::string& filepath
)
{
    std::ofstream out(filepath);
    if (!out.is_open())
    {
        return false;
    }

    out << std::fixed << std::setprecision(6);
    out << "[\n";
    for (size_t i = 0; i < log.size(); ++i)
    {
        const auto& step = log[i];
        out << "  {\n";
        out << "    \"time\": " << step.time << ",\n";
        out << "    \"position\": " << step.position << ",\n";
        out << "    \"velocity\": " << step.velocity << ",\n";
        out << "    \"acceleration\": " << step.acceleration << ",\n";
        out << "    \"x\": " << step.x << ",\n";
        out << "    \"y\": " << step.y << ",\n";
        out << "    \"F_engine\": " << step.F_engine << ",\n";
        out << "    \"F_drag\": " << step.F_drag << ",\n";
        out << "    \"F_rr\": " << step.F_rr << ",\n";
        out << "    \"F_grade\": " << step.F_grade << ",\n";
        out << "    \"curvature\": " << step.curvature << "\n";
        out << "  }" << (i == log.size() - 1 ? "" : ",") << "\n";
    }
    out << "]\n";

    return true;
}
