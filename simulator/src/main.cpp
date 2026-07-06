#include <iostream>
#include <iomanip>
#include "../include/TrackLoader.hpp"
#include "../include/CoordinateConverter.hpp"
#include "../include/TrackAnalyzer.hpp"
#include "../include/VehicleDynamics.hpp"

void process_track(const char* name, const char* filepath)
{
    std::cout << "\n======================================\n";
    std::cout << "PROCESSING TRACK: " << name << "\n";
    std::cout << "======================================\n";

    // 1. Load Track Points
    Track track = load_track(filepath);
    if (track.points.empty())
    {
        std::cout << "Failed to load track points from: " << filepath << "\n";
        return;
    }
    std::cout << "Successfully loaded " << track.points.size() << " raw track points.\n";

    // 2. Convert to Local Cartesian coordinates
    convert_to_cartesian(track);
    std::cout << "Converted coordinates to local Cartesian framework.\n";

    // 3. Perform Track Geometry Analysis
    analyze_track(track);

    // 4. Validate Track Geometry
    validate_track(track);

    // 5. Print Diagnostics and Statistics
    print_track_statistics(track);

    // 6. Print details of a few sample segments for verification
    if (!track.segments.empty())
    {
        std::cout << "\nSample Segments (first 5):\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << std::setw(6) << "Seg" 
                  << std::setw(15) << "Length (m)" 
                  << std::setw(15) << "Heading (rad)" 
                  << std::setw(15) << "Grade"
                  << std::setw(15) << "Curvature" << "\n";
        
        size_t print_count = std::min(track.segments.size(), static_cast<size_t>(5));
        for (size_t i = 0; i < print_count; ++i)
        {
            std::cout << std::setw(6) << i
                      << std::setw(15) << track.segments[i].length
                      << std::setw(15) << track.segments[i].heading
                      << std::setw(15) << track.segments[i].grade
                      << std::setw(15) << track.segments[i].curvature << "\n";
        }
    }
}

void run_vehicle_simulation()
{
    std::cout << "\n======================================\n";
    std::cout << "VEHICLE DYNAMICS SIMULATION RUN\n";
    std::cout << "======================================\n";

    // Initialize GT3-like sports car properties
    Vehicle car;
    car.mass = 1300.0;                    // kg
    car.wheelbase = 2.635;                // m
    car.front_track_width = 1.600;        // m
    car.rear_track_width = 1.600;         // m
    car.center_of_gravity_height = 0.400; // m
    car.frontal_area = 2.0;               // m^2
    car.drag_coefficient = 0.33;          // Cd
    car.rolling_resistance_coeff = 0.015; // Crr
    car.tyre_radius = 0.33;               // m
    car.final_drive_ratio = 4.0;
    
    car.engine.max_power = 370000.0;      // 370 kW
    car.engine.max_torque = 500.0;        // 500 Nm

    std::cout << "Vehicle Initialized:\n";
    std::cout << "  Mass: " << car.mass << " kg\n";
    std::cout << "  Max Power: " << (car.engine.max_power / 1000.0) << " kW\n";
    std::cout << "  Max Torque: " << car.engine.max_torque << " Nm\n";
    std::cout << "  Cd: " << car.drag_coefficient << ", Frontal Area: " << car.frontal_area << " m^2\n";

    double duration = 15.0; // seconds
    double dt = 0.05;       // seconds

    // 1. Flat Simulation (0% Grade)
    auto log_flat = VehicleDynamics::run_straight_simulation(car, 0.0, duration, dt);
    VehicleDynamics::export_to_json(log_flat, "simulation_log_flat.json");
    
    // 2. Uphill Simulation (+5% Grade)
    auto log_up = VehicleDynamics::run_straight_simulation(car, 0.05, duration, dt);
    VehicleDynamics::export_to_json(log_up, "simulation_log_uphill.json");
    
    // 3. Downhill Simulation (-5% Grade)
    auto log_down = VehicleDynamics::run_straight_simulation(car, -0.05, duration, dt);
    VehicleDynamics::export_to_json(log_down, "simulation_log_downhill.json");

    std::cout << "\nSimulation Logs generated successfully:\n";
    std::cout << "  - simulation_log_flat.json\n";
    std::cout << "  - simulation_log_uphill.json\n";
    std::cout << "  - simulation_log_downhill.json\n";

    // Print Telemetry Samples for Flat Track
    std::cout << "\nFlat Track (0% Grade) Telemetry Samples:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(6) << "Time" 
              << std::setw(12) << "Dist (m)" 
              << std::setw(15) << "Speed (km/h)" 
              << std::setw(12) << "Accel (G)"
              << std::setw(15) << "Engine (N)"
              << std::setw(12) << "Drag (N)" << "\n";

    for (const auto& step : log_flat)
    {
        double t = step.time;
        if (std::abs(t - 0.0) < 1e-3 || std::abs(t - 2.0) < 1e-3 || 
            std::abs(t - 5.0) < 1e-3 || std::abs(t - 10.0) < 1e-3 || 
            std::abs(t - 15.0) < 1e-3)
        {
            std::cout << std::setw(6) << step.time
                      << std::setw(12) << step.position
                      << std::setw(15) << (step.velocity * 3.6)
                      << std::setw(12) << (step.acceleration / 9.80665)
                      << std::setw(15) << step.F_engine
                      << std::setw(12) << step.F_drag << "\n";
        }
    }
}

int main()
{
    process_track("Le Mans - Circuit de la Sarthe", "data/lemans.gpx");
    process_track("Monza - Autodromo Nazionale Monza", "data/monza.gpx");
    run_vehicle_simulation();
    return 0;
}
