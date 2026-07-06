#include "../include/TrackAnalyzer.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

double compute_track_length(const Track& track)
{
    if (track.points.size() < 2) return 0.0;
    double total_length = 0.0;
    for (size_t i = 0; i < track.points.size() - 1; ++i)
    {
        double dx = track.points[i+1].x_coordinate - track.points[i].x_coordinate;
        double dy = track.points[i+1].y_coordinate - track.points[i].y_coordinate;
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    return total_length;
}

std::vector<TrackSegment> compute_track_segments(const Track& track)
{
    if (track.points.size() < 2) return {};
    std::vector<TrackSegment> segments;
    segments.reserve(track.points.size() - 1);
    
    for (size_t i = 0; i < track.points.size() - 1; ++i)
    {
        double dx = track.points[i+1].x_coordinate - track.points[i].x_coordinate;
        double dy = track.points[i+1].y_coordinate - track.points[i].y_coordinate;
        double dz = track.points[i+1].z_coordinate - track.points[i].z_coordinate;

        double length = std::sqrt(dx*dx + dy*dy);
        double heading = std::atan2(dy, dx);
        double grade = (length > 1e-6) ? (dz / length) : 0.0;

        // Curvature is initialized to 0.0 for Phase 1
        segments.push_back(TrackSegment{length, heading, grade, 0.0});
    }
    return segments;
}

void analyze_track(Track& track)
{
    if (track.points.size() < 2) return;
    
    // Detect closed loop: distance between first and last points is small
    double dx = track.points.front().x_coordinate - track.points.back().x_coordinate;
    double dy = track.points.front().y_coordinate - track.points.back().y_coordinate;
    track.is_closed = (std::sqrt(dx*dx + dy*dy) < 0.1);

    track.segments = compute_track_segments(track);
    track.total_length = compute_track_length(track);
}

void print_track_statistics(const Track& track)
{
    std::cout << "\n========== TRACK STATISTICS ==========\n";
    std::cout << "Number of points: " << track.points.size() << "\n";
    std::cout << "Number of segments: " << track.segments.size() << "\n";
    std::cout << "Total horizontal length: " << track.total_length << " m\n";
    std::cout << "Is closed loop: " << (track.is_closed ? "Yes" : "No") << "\n";

    if (track.segments.empty())
    {
        std::cout << "======================================\n";
        return;
    }

    double max_grade = -1e9;
    double min_grade = 1e9;
    double sum_grade = 0.0;
    double max_len = -1e9;
    double min_len = 1e9;

    for (const auto& seg : track.segments)
    {
        sum_grade += seg.grade;
        if (seg.grade > max_grade) max_grade = seg.grade;
        if (seg.grade < min_grade) min_grade = seg.grade;
        if (seg.length > max_len) max_len = seg.length;
        if (seg.length < min_len) min_len = seg.length;
    }

    std::cout << "Average segment length: " << (track.total_length / track.segments.size()) << " m\n";
    std::cout << "Segment length range: [" << min_len << ", " << max_len << "] m\n";
    std::cout << "Maximum grade: " << max_grade << " (slope)\n";
    std::cout << "Minimum grade: " << min_grade << " (slope)\n";
    std::cout << "Average grade: " << (sum_grade / track.segments.size()) << "\n";
    std::cout << "======================================\n";
}

bool validate_track(const Track& track)
{
    std::cout << "\n========== TRACK VALIDATION ==========\n";
    bool is_valid = true;
    
    if (track.points.size() < 2)
    {
        std::cout << "[Validation Error] Track has fewer than 2 points.\n";
        is_valid = false;
    }
    if (track.total_length <= 0.0)
    {
        std::cout << "[Validation Error] Track has non-positive total length.\n";
        is_valid = false;
    }
    
    int degenerate_count = 0;
    int steep_count = 0;
    for (size_t i = 0; i < track.segments.size(); ++i)
    {
        if (track.segments[i].length <= 1e-4)
        {
            degenerate_count++;
        }
        if (std::abs(track.segments[i].grade) > 0.3)
        {
            steep_count++;
        }
    }
    
    if (degenerate_count > 0)
    {
        std::cout << "[Validation Warning] Found " << degenerate_count << " degenerate segments (length <= 0.1 mm).\n";
    }
    if (steep_count > 0)
    {
        std::cout << "[Validation Warning] Found " << steep_count << " segments with extremely steep grade (> 30%).\n";
    }
    
    if (is_valid)
    {
        std::cout << "Track validation PASSED. Geometry is physically sound.\n";
    }
    else
    {
        std::cout << "Track validation FAILED.\n";
    }
    std::cout << "======================================\n";
    return is_valid;
}