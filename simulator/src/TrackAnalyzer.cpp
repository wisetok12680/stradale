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
    
    size_t N = track.points.size();
    for (size_t i = 0; i < N - 1; ++i)
    {
        double dx = track.points[i+1].x_coordinate - track.points[i].x_coordinate;
        double dy = track.points[i+1].y_coordinate - track.points[i].y_coordinate;
        double dz = track.points[i+1].z_coordinate - track.points[i].z_coordinate;

        double length = std::sqrt(dx*dx + dy*dy);
        double heading = std::atan2(dy, dx);
        double grade = (length > MIN_SEGMENT_LENGTH) ? (dz / length) : 0.0;

        // Determine points for curvature calculation
        size_t prev_idx = (i == 0) ? (track.is_closed ? N - 2 : 0) : i - 1;
        size_t curr_idx = i;
        size_t next_idx = i + 1;

        double curvature = compute_curvature(
            track.points[prev_idx],
            track.points[curr_idx],
            track.points[next_idx]
        );

        segments.push_back(TrackSegment{length, heading, grade, curvature});
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
    double max_curvature = -1e9;
    double min_curvature = 1e9;
    double sum_curvature = 0.0;

    for (const auto& seg : track.segments)
    {
        sum_grade += seg.grade;
        if (seg.grade > max_grade) max_grade = seg.grade;
        if (seg.grade < min_grade) min_grade = seg.grade;
        if (seg.length > max_len) max_len = seg.length;
        if (seg.length < min_len) min_len = seg.length;

        sum_curvature += seg.curvature;
        if (seg.curvature > max_curvature) max_curvature = seg.curvature;
        if (seg.curvature < min_curvature) min_curvature = seg.curvature;
    }

    std::cout << "Average segment length: " << (track.total_length / track.segments.size()) << " m\n";
    std::cout << "Segment length range: [" << min_len << ", " << max_len << "] m\n";
    std::cout << "Maximum grade: " << max_grade << " (slope)\n";
    std::cout << "Minimum grade: " << min_grade << " (slope)\n";
    std::cout << "Average grade: " << (sum_grade / track.segments.size()) << "\n";
    std::cout << "Maximum curvature: " << max_curvature << " m^-1\n";
    std::cout << "Minimum curvature: " << min_curvature << " m^-1\n";
    std::cout << "Average curvature: " << (sum_curvature / track.segments.size()) << " m^-1\n";
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
        if (track.segments[i].length <= MIN_SEGMENT_LENGTH)
        {
            degenerate_count++;
        }
        if (std::abs(track.segments[i].grade) > 0.3)
        {
            std::cout << "[Validation Warning] Segment " << i << " has extremely steep grade: " << track.segments[i].grade << "\n";
            steep_count++;
        }
    }
    
    if (degenerate_count > 0)
    {
        std::cout << "[Validation Warning] Found " << degenerate_count << " degenerate segments (length <= " << MIN_SEGMENT_LENGTH << " m).\n";
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

double compute_curvature(
    const TrackPoint& previous,
    const TrackPoint& current,
    const TrackPoint& next
)
{
    double dx_prev = current.x_coordinate - previous.x_coordinate;
    double dy_prev = current.y_coordinate - previous.y_coordinate;
    double a = std::sqrt(dx_prev*dx_prev + dy_prev*dy_prev);

    double dx_next = next.x_coordinate - current.x_coordinate;
    double dy_next = next.y_coordinate - current.y_coordinate;
    double b = std::sqrt(dx_next*dx_next + dy_next*dy_next);

    double dx_chord = next.x_coordinate - previous.x_coordinate;
    double dy_chord = next.y_coordinate - previous.y_coordinate;
    double c = std::sqrt(dx_chord*dx_chord + dy_chord*dy_chord);

    if (a < MIN_SEGMENT_LENGTH || b < MIN_SEGMENT_LENGTH || c < MIN_SEGMENT_LENGTH)
    {
        return 0.0;
    }

    // Signed double area (2A) via vector cross product of (P_prev -> P_curr) and (P_curr -> P_next)
    double double_area_signed = dx_prev * dy_next - dy_prev * dx_next;

    // κ = 4A / (abc) which is mathematically identical to 2 * (2A_signed) / (abc)
    double curvature = (2.0 * double_area_signed) / (a * b * c);

    return curvature;
}