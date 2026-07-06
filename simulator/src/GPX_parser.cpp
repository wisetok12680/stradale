#include <iostream>
#include <tinyxml2.h>
#include <vector>


struct TrackPoint
{
    double latitude;
    double longitude;
    double elevation;
};
int main()
{
    tinyxml2::XMLDocument gpx_document;
    std::vector<TrackPoint> track_points;
    if (gpx_document.LoadFile("../data/lemans.gpx") != tinyxml2::XML_SUCCESS)
    {
        std::cout << "Failed to load GPX file\n";
        return 1;
    }

    auto* gpx_root =
        gpx_document.FirstChildElement("gpx");

    auto* track =
        gpx_root->FirstChildElement("trk");

    auto* track_segment =
        track->FirstChildElement("trkseg");

    auto* first_track_point =
        track_segment->FirstChildElement("trkpt");

    double latitude =
        first_track_point->DoubleAttribute("lat");

    double longitude =
        first_track_point->DoubleAttribute("lon");

    auto* elevation_element =
        first_track_point->FirstChildElement("ele");

    double elevation =
        std::stod(elevation_element->GetText());


    auto current_track_point=first_track_point;
    while(current_track_point!=nullptr){
        TrackPoint point;
        point.latitude=current_track_point->DoubleAttribute("lat");
        point.longitude=current_track_point->DoubleAttribute("lon");
        auto* elevation_element = current_track_point->FirstChildElement("ele");
        double elevation =std::stod(elevation_element->GetText());
        point.elevation=elevation;
        track_points.push_back(
            point
        );
        current_track_point=current_track_point->NextSiblingElement();
    }
    std::cout << track_points.size() << '\n';
    return 0;
}
