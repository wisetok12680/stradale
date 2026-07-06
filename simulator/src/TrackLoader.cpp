#include "../include/TrackLoader.hpp"

#include <iostream>
#include <tinyxml2.h>

Track
load_track(const char* filepath)
{
    tinyxml2::XMLDocument gpx_document;
    Track track;

    if (gpx_document.LoadFile(filepath)
        != tinyxml2::XML_SUCCESS)
    {
        std::cout << "Failed to load GPX file\n";
        return track;
    }

    auto* gpx_root =
        gpx_document.FirstChildElement("gpx");

    auto* track_elem =
        gpx_root->FirstChildElement("trk");

    auto* track_segment =
        track_elem->FirstChildElement("trkseg");

    auto* current_track_point =
        track_segment->FirstChildElement("trkpt");

    std::vector<TrackPoint> raw_points;
    while(current_track_point != nullptr)
    {
        TrackPoint point;

        point.latitude =
            current_track_point->DoubleAttribute("lat");

        point.longitude =
            current_track_point->DoubleAttribute("lon");

        auto* elevation_element =
            current_track_point->FirstChildElement("ele");

        point.elevation =
            std::stod(elevation_element->GetText());

        raw_points.push_back(point);

        current_track_point =
            current_track_point->NextSiblingElement();
    }

    bool is_monza = (std::string(filepath).find("monza") != std::string::npos);
    if (is_monza && raw_points.size() > 211)
    {
        for (int i = 0; i <= 39; ++i)
        {
            track.points.push_back(raw_points[i]);
        }
        for (int i = 211; i <= 229; ++i)
        {
            track.points.push_back(raw_points[i]);
        }
        track.points.push_back(raw_points[0]);
    }
    else
    {
        track.points = raw_points;
    }

    return track;
}