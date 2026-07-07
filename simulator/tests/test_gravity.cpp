#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include "../include/Gravity.hpp"
#include "../include/PhysicsConstants.hpp"

void test_gravity_validation() {
    std::cout << "==============================================================================\n";
    std::cout << "GRAVITY GRADE FORCE VS PEAK TRACTIVE FORCE VALIDATION\n";
    std::cout << "==============================================================================\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(6) << "Grade"
              << std::setw(15) << "Angle (deg)"
              << std::setw(15) << "F_grade (N)"
              << std::setw(18) << "Peak Gear 1 Tr (N)"
              << std::setw(15) << "Gravity % of Tr" << "\n";

    double mass = 1300.0; // kg
    double peak_gear_1_tractive = 17716.97; // N (Peak tractive force in gear 1)
    
    double test_grades[] = { 0.0, 0.05, -0.05, 0.10, -0.10 };

    for (double grade : test_grades) {
        double f_grade = Gravity::compute_force(mass, grade);
        double angle_deg = std::atan(grade) * (180.0 / PhysicsConstants::pi);
        double pct_tractive = (std::abs(f_grade) / peak_gear_1_tractive) * 100.0;

        std::cout << std::setw(5) << static_cast<int>(grade * 100.0) << "%"
                  << std::setw(15) << angle_deg
                  << std::setw(15) << f_grade
                  << std::setw(18) << peak_gear_1_tractive
                  << std::setw(14) << pct_tractive << "%\n";

        // Assert check values
        if (grade == 0.0) {
            assert(std::abs(f_grade - 0.0) < 1e-3);
        }
        if (grade == 0.05) {
            assert(std::abs(f_grade - 636.64) < 1e-2);
        }
        if (grade == -0.05) {
            assert(std::abs(f_grade - (-636.64)) < 1e-2);
        }
        if (grade == 0.10) {
            assert(std::abs(f_grade - 1268.54) < 1e-2);
        }
        if (grade == -0.10) {
            assert(std::abs(f_grade - (-1268.54)) < 1e-2);
        }
    }
    std::cout << "==============================================================================\n";
    std::cout << "Gravity Subsystem Validation: PASSED\n";
    std::cout << "==============================================================================\n";
}

int main() {
    test_gravity_validation();
    return 0;
}
