#pragma once

#include "Vehicle.hpp"
#include "Track.hpp"
#include "SimulationState.hpp"
#include "VehicleForces.hpp"
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
    // Core state integrator using Explicit Euler (updates SimulationState by dt)
    static void integrate(
        const Vehicle& vehicle,
        double dt,
        const VehicleForces& forces,
        SimulationState& state
    );

    // Simple automatic gear shift logic
    static void update_gear_shifting(
        const Vehicle& vehicle,
        SimulationState& state
    );
    
    // Runs straight line simulation
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
