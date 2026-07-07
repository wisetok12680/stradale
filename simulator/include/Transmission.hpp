#pragma once
#include <vector>

struct Transmission {
    std::vector<double> gear_ratios = { 3.10, 2.20, 1.65, 1.30, 1.10, 0.95 }; // dimensionless
    double final_drive_ratio = 4.10;                                          // dimensionless
    double drivetrain_efficiency = 0.92;                                      // dimensionless
};
