#include "../include/Aerodynamics.hpp"
#include "../include/PhysicsConstants.hpp"
#include <cmath>

AeroForces Aerodynamics::compute_forces(const Vehicle& vehicle, double velocity) {
    AeroForces forces;
    
    // Force is symmetric regardless of forward or backward velocity direction
    double v_sq = velocity * velocity;
    
    // F_drag = 0.5 * rho * Cd * A * v^2
    forces.drag = 0.5 * PhysicsConstants::air_density * vehicle.Cd * vehicle.frontal_area * v_sq;
    
    // Total downforce = 0.5 * rho * (-Cl) * A * v^2
    double positive_cl = -vehicle.Cl;
    double total_downforce = 0.5 * PhysicsConstants::air_density * positive_cl * vehicle.frontal_area * v_sq;
    
    // Distribute downforce using static weight distribution fraction
    forces.downforce_front = total_downforce * vehicle.front_weight_fraction;
    forces.downforce_rear = total_downforce * (1.0 - vehicle.front_weight_fraction);
    
    return forces;
}
