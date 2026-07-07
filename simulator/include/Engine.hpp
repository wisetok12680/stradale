#pragma once
#include <vector>
#include <algorithm>

struct TorquePoint {
    double rpm;        // rev/min
    double torque;     // Nm
};

struct Engine {
    double maximum_power = 370000.0;     // W (equivalent to 370 kW / ~500 hp)
    double maximum_torque = 500.0;       // Nm
    double idle_rpm = 1000.0;            // rev/min
    double redline_rpm = 8500.0;         // rev/min
    std::vector<TorquePoint> torque_curve;

    // Performs piecewise linear interpolation to look up torque at a given RPM
    double lookup_torque(double rpm) const {
        if (torque_curve.empty()) {
            return 0.0;
        }

        // Clamp RPM to physical operating limits [idle, redline]
        double clamped_rpm = std::max(idle_rpm, std::min(rpm, redline_rpm));

        // Bounding checks
        if (clamped_rpm <= torque_curve.front().rpm) {
            return torque_curve.front().torque;
        }
        if (clamped_rpm >= torque_curve.back().rpm) {
            return torque_curve.back().torque;
        }

        // Find the matching linear interval
        for (size_t i = 0; i < torque_curve.size() - 1; ++i) {
            if (clamped_rpm >= torque_curve[i].rpm && clamped_rpm <= torque_curve[i+1].rpm) {
                double rpm_low = torque_curve[i].rpm;
                double rpm_high = torque_curve[i+1].rpm;
                double t_low = torque_curve[i].torque;
                double t_high = torque_curve[i+1].torque;
                
                double factor = (clamped_rpm - rpm_low) / (rpm_high - rpm_low);
                return t_low + factor * (t_high - t_low);
            }
        }
        return 0.0;
    }
};
