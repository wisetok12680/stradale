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
    double drag = 0.5 * rho * vehicle.drag_coefficient * vehicle.frontal_area * velocity * velocity;
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
    double F_max_tractive = (vehicle.engine.max_torque * vehicle.final_drive_ratio) / vehicle.tyre_radius;

    if (velocity <= 1e-3)
    {
        return F_max_tractive;
    }

    // Power limit: F = P / v
    double F_power_limit = vehicle.engine.max_power / velocity;

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
