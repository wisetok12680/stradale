#include <iostream>
#include <iomanip>
#include "../include/TrackLoader.hpp"
#include "../include/CoordinateConverter.hpp"
#include "../include/TrackAnalyzer.hpp"
#include "../include/VehicleDynamics.hpp"
#include "../include/Aerodynamics.hpp"

Track process_track(const char* name, const char* filepath)
{
    std::cout << "\n======================================\n";
    std::cout << "PROCESSING TRACK: " << name << "\n";
    std::cout << "======================================\n";

    // 1. Load Track Points
    Track track = load_track(filepath);
    if (track.points.empty())
    {
        std::cout << "Failed to load track points from: " << filepath << "\n";
        return track;
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
    return track;
}

void run_vehicle_simulation(const Track& lemans, const Track& monza)
{
    std::cout << "\n======================================\n";
    std::cout << "VEHICLE DYNAMICS SIMULATION RUN\n";
    std::cout << "======================================\n";

    // Initialize baseline GT3 race car properties
    Vehicle car;
    car.mass = 1300.0;                              // kg
    car.front_weight_fraction = 0.46;               // dimensionless
    car.wheelbase = 2.70;                           // m
    car.front_track_width = 1.67;                   // m
    car.rear_track_width = 1.65;                    // m
    car.cg_height = 0.32;                           // m
    car.frontal_area = 1.90;                        // m²
    car.Cd = 0.34;                                  // dimensionless
    car.Cl = -1.15;                                 // dimensionless (downforce)
    car.tyre_radius = 0.33;                         // m
    car.tyre_mu = 1.20;                             // dimensionless
    car.rolling_resistance_coeff = 0.015;            // dimensionless
    
    car.engine.maximum_power = 370000.0;            // W
    car.engine.maximum_torque = 500.0;              // Nm
    car.engine.idle_rpm = 1000.0;                   // rev/min
    car.engine.redline_rpm = 8500.0;                // rev/min
    
    car.transmission.gear_ratios = { 3.10, 2.20, 1.65, 1.30, 1.10, 0.95 };
    car.transmission.final_drive_ratio = 4.10;
    car.transmission.drivetrain_efficiency = 0.92;

    std::cout << "Vehicle Initialized (GT3 Baseline):\n";
    std::cout << "  Mass: " << car.mass << " kg\n";
    std::cout << "  Max Power: " << (car.engine.maximum_power / 1000.0) << " kW\n";
    std::cout << "  Max Torque: " << car.engine.maximum_torque << " Nm\n";
    std::cout << "  Cd: " << car.Cd << ", Cl: " << car.Cl << ", Frontal Area: " << car.frontal_area << " m^2\n";

    // Aerodynamics Subsystem Validation
    std::cout << "\n========== AERODYNAMICS VALIDATION ==========\n";
    std::cout << std::fixed << std::setprecision(2);
    for (double speed_kmh : {100.0, 200.0, 250.0}) {
        double v = speed_kmh / 3.6; // convert to m/s
        auto forces = Aerodynamics::compute_forces(car, v);
        std::cout << "At " << std::setw(3) << static_cast<int>(speed_kmh) << " km/h:\n"
                  << "  Drag Force:         " << std::setw(8) << forces.drag << " N\n"
                  << "  DF Front Axle:      " << std::setw(8) << forces.downforce_front << " N\n"
                  << "  DF Rear Axle:       " << std::setw(8) << forces.downforce_rear << " N\n"
                  << "  DF Total:           " << std::setw(8) << (forces.downforce_front + forces.downforce_rear) << " N\n";
    }
    std::cout << "=============================================\n";

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

    // 4. Lap Simulations
    std::cout << "\nRunning full lap simulations...\n";
    
    auto log_lemans_lap = VehicleDynamics::run_lap_simulation(car, lemans, dt);
    VehicleDynamics::export_lap_to_json(log_lemans_lap, "lemans_lap_simulation.json");
    std::cout << "  Le Mans Lap Steps: " << log_lemans_lap.size() 
              << ", Lap Time: " << log_lemans_lap.back().time << " s\n";

    auto log_monza_lap = VehicleDynamics::run_lap_simulation(car, monza, dt);
    VehicleDynamics::export_lap_to_json(log_monza_lap, "monza_lap_simulation.json");
    std::cout << "  Monza Lap Steps: " << log_monza_lap.size() 
              << ", Lap Time: " << log_monza_lap.back().time << " s\n";

    std::cout << "\nSimulation Logs generated successfully:\n";
    std::cout << "  - simulation_log_flat.json\n";
    std::cout << "  - simulation_log_uphill.json\n";
    std::cout << "  - simulation_log_downhill.json\n";
    std::cout << "  - lemans_lap_simulation.json\n";
    std::cout << "  - monza_lap_simulation.json\n";
}

int main()
{
    Track lemans = process_track("Le Mans - Circuit de la Sarthe", "data/lemans.gpx");
    Track monza = process_track("Monza - Autodromo Nazionale Monza", "data/monza.gpx");
    run_vehicle_simulation(lemans, monza);
    return 0;
}
