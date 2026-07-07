#pragma once

#include "Vehicle.hpp"
#include "Track.hpp"
#include <vector>
#include <string>

struct SimulationStep {
    double time;
    double position;
    double velocity;
    double acceleration;
    double F_engine;
    double F_drag;
    double F_rr;
    double F_grade;
};

struct LapSimulationStep {
    double time;
    double position;
    double velocity;
    double acceleration;
    double x;
    double y;
    double F_engine;
    double F_drag;
    double F_rr;
    double F_grade;
    double curvature;
};

class VehicleDynamics {
public:
    // Core forces calculations
    static double compute_drag_force(const Vehicle& vehicle, double velocity);
    static double compute_rolling_resistance(const Vehicle& vehicle);
    static double compute_grade_force(const Vehicle& vehicle, double grade);
    static double compute_engine_force(const Vehicle& vehicle, double velocity);
    
    // Runs straight line acceleration run
    static std::vector<SimulationStep> run_straight_simulation(
        const Vehicle& vehicle,
        double grade,
        double duration,
        double dt
    );
    
    // Exports time-series log to JSON format
    static bool export_to_json(
        const std::vector<SimulationStep>& log,
        const std::string& filepath
    );

    // Runs a full lap simulation on a given track
    static std::vector<LapSimulationStep> run_lap_simulation(
        const Vehicle& vehicle,
        const Track& track,
        double dt
    );

    // Exports lap simulation to JSON format
    static bool export_lap_to_json(
        const std::vector<LapSimulationStep>& log,
        const std::string& filepath
    );
};
