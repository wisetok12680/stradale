#pragma once

struct VehicleState {
    double position = 0.0;                   // m
    double distance_travelled = 0.0;          // m
    double velocity = 0.0;                   // m/s
    double acceleration = 0.0;               // m/s²
    double heading = 0.0;                    // rad
    double yaw_angle = 0.0;                  // rad
    double yaw_rate = 0.0;                   // rad/s
    
    int current_gear = 1;                    // dimensionless (1-indexed)
    double engine_rpm = 1000.0;              // rev/min
    double wheel_angular_velocity = 0.0;      // rad/s
    
    double throttle_position = 0.0;          // dimensionless (0.0 to 1.0)
    double brake_position = 0.0;             // dimensionless (0.0 to 1.0)
};

struct SimulationState {
    VehicleState vehicle;
};
