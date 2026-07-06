#include <iostream>
#include <iomanip>
#include "../include/TrackLoader.hpp"
#include "../include/CoordinateConverter.hpp"
#include "../include/TrackAnalyzer.hpp"

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

int main()
{
    process_track("Le Mans - Circuit de la Sarthe", "data/lemans.gpx");
    process_track("Monza - Autodromo Nazionale Monza", "data/monza.gpx");
    return 0;
}
