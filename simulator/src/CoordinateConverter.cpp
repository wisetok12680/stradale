#include "../include/CoordinateConverter.hpp"

#include <cmath>

const double PI = 3.14159265358979323846;

void convert_to_cartesian(Track& track)
{
    if (track.points.empty()) return;

    double origin_latitude =
        track.points[0].latitude;

    double origin_longitude =
        track.points[0].longitude;

    double origin_latitude_radians =
        origin_latitude * PI / 180.0;

    for(size_t i = 0; i < track.points.size(); i++)
    {
        double delta_latitude =
            track.points[i].latitude -
            origin_latitude;

        double delta_longitude =
            track.points[i].longitude -
            origin_longitude;

        track.points[i].x_coordinate =
            delta_longitude *
            111320.0 *
            cos(origin_latitude_radians);

        track.points[i].y_coordinate =
            delta_latitude *
            111320.0;

        track.points[i].z_coordinate =
            track.points[i].elevation;
    }
}