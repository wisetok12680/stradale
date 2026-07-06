#pragma once

#include "Track.hpp"

// Computes the total horizontal length of the track from its points
double compute_track_length(const Track& track);

// Computes segments, determines if it is closed, and calculates total length
void analyze_track(Track& track);

// Generates and returns the track segments (Phase 1: length, heading, grade)
std::vector<TrackSegment> compute_track_segments(const Track& track);

// Prints geometry diagnostics and summary statistics to console
void print_track_statistics(const Track& track);

// Validates the track geometry for physical correctness (e.g. non-empty, non-zero lengths)
bool validate_track(const Track& track);