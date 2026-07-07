#include "../include/VehicleDynamics.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

double VehicleDynamics::compute_drag_force(const Vehicle& vehicle, double velocity)
{
    // F_drag = 0.5 * rho * Cd * A * v^2
    const double rho = 1.225; // kg/m^3 (sea level air density)
    double drag = 0.5 * rho * vehicle.Cd * vehicle.frontal_area * velocity * velocity;
    return drag;
}

double VehicleDynamics::compute_rolling_resistance(const Vehicle& vehicle)
{
    // F_rr = Crr * m * g
    const double g = 9.80665; // m/s^2
    double rr = vehicle.rolling_resistance_coeff * vehicle.mass * g;
    return rr;
}

double VehicleDynamics::compute_grade_force(const Vehicle& vehicle, double grade)
{
    // F_grade = m * g * sin(theta) where theta = atan(grade)
    // sin(atan(g)) = g / sqrt(1 + g^2)
    const double g = 9.80665; // m/s^2
    double sin_theta = grade / std::sqrt(1.0 + grade * grade);
    double grade_force = vehicle.mass * g * sin_theta;
    return grade_force;
}

double VehicleDynamics::compute_engine_force(const Vehicle& vehicle, double velocity)
{
    // Tractive limit based on engine torque and final drive ratio
    double F_max_tractive = (vehicle.engine.maximum_torque * vehicle.transmission.final_drive_ratio) / vehicle.tyre_radius;

    if (velocity <= 1e-3)
    {
        return F_max_tractive;
    }

    // Power limit: F = P / v
    double F_power_limit = vehicle.engine.maximum_power / velocity;

    // Output is the minimum of torque limit and power limit
    double F_engine = std::min(F_max_tractive, F_power_limit);
    return F_engine;
}

std::vector<SimulationStep> VehicleDynamics::run_straight_simulation(
    const Vehicle& vehicle,
    double grade,
    double duration,
    double dt
)
{
    std::vector<SimulationStep> log;
    double time = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;

    int steps = static_cast<int>(duration / dt);
    log.reserve(steps + 1);

    while (time <= duration)
    {
        double F_drag = compute_drag_force(vehicle, velocity);
        double F_rr = compute_rolling_resistance(vehicle);
        double F_grade = compute_grade_force(vehicle, grade);
        double F_engine = compute_engine_force(vehicle, velocity);

        double F_net = F_engine - F_drag - F_rr - F_grade;
        
        acceleration = F_net / vehicle.mass;
        
        log.push_back(SimulationStep{
            time,
            position,
            velocity,
            acceleration,
            F_engine,
            F_drag,
            F_rr,
            F_grade
        });

        // Euler Integration
        velocity += acceleration * dt;
        if (velocity < 0.0) velocity = 0.0; // clamp to prevent rolling backwards from grade/resistance

        position += velocity * dt;
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

    double time = 0.0;
    double s = 0.0; // distance along track (m)
    double v = 0.0; // velocity (m/s)

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
    double total_length = track.total_length;

    log.reserve(2000);
    size_t current_seg_idx = 0;

    // Simulation runs until we complete exactly one full lap
    while (s < total_length)
    {
        // Find which segment we are on
        while (current_seg_idx < track.segments.size() - 1 && s >= dist_cum[current_seg_idx + 1])
        {
            current_seg_idx++;
        }

        double grade = track.segments[current_seg_idx].grade;
        double curvature = track.segments[current_seg_idx].curvature;
        double seg_len = track.segments[current_seg_idx].length;

        // Compute forces
        double F_drag = compute_drag_force(vehicle, v);
        double F_rr = compute_rolling_resistance(vehicle);
        double F_grade = compute_grade_force(vehicle, grade);
        double F_engine = compute_engine_force(vehicle, v);

        double F_net = F_engine - F_drag - F_rr - F_grade;
        double acceleration = F_net / vehicle.mass;

        // Integrate velocity
        double v_next = v + acceleration * dt;

        // Corner Speed Clamping
        // 1. Threshold of 0.002 m^-1 filters out GPS coordinate jitter on straightaways
        if (std::abs(curvature) > 0.002)
        {
            const double mu_lateral = 1.2;
            const double g = 9.80665;
            double v_limit = std::sqrt((mu_lateral * g) / std::abs(curvature));
            
            if (v_next > v_limit)
            {
                // 2. Limit deceleration to -1.8 G to simulate realistic progressive braking
                const double max_braking_decel = -1.8 * g; 
                double v_braked = v + max_braking_decel * dt;
                
                v_next = std::max(v_limit, v_braked);
                acceleration = (v_next - v) / dt;
            }
        }
        if (v_next < 0.0) v_next = 0.0;

        // Interpolate exact 2D Cartesian coordinates along this segment
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

        log.push_back(LapSimulationStep{
            time,
            s,
            v,
            acceleration,
            car_x,
            car_y,
            F_engine,
            F_drag,
            F_rr,
            F_grade,
            curvature
        });

        // Advance state
        v = v_next;
        s += v * dt;
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
