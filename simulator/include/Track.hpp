#pragma once

#include <vector>
#include "TrackPoint.hpp"
#include "TrackSegment.hpp"

class Track
{
public:
    std::vector<TrackPoint> points;
    std::vector<TrackSegment> segments;

    double total_length = 0.0;
    bool is_closed = false;
};
