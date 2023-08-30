#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include "json.hpp"

namespace px4_autonomy
{

    struct Waypoint
    {
        double latitude;
        double longitude;
        double altitude;
    };

    enum WaypointUpdateStatus
    {
        NOT_REACHED,
        WAYPOINT_REACHED,
        MISSION_COMPLETE
    };

    class MissionPlanner
    {
    public:
        MissionPlanner(const std::string &filePath)
            : _currentWaypointIndex(0)
        {

            std::ifstream file(filePath);
            if (!file.is_open())
            {
                throw std::runtime_error("Could not open file.");
            }
            nlohmann::json j;
            file >> j;
            file.close();

            auto missionItems = j["mission"]["items"];
            for (const auto &item : missionItems)
            {
                if (item["command"] == 16)
                {
                    _waypoints.push_back({item["params"][4].get<double>(),
                                          item["params"][5].get<double>(),
                                          item["params"][6].get<double>()});

                    std::cout << "Waypoint: " << _waypoints.back().latitude << ", " << _waypoints.back().longitude << ", " << _waypoints.back().altitude << std::endl;
                }
            }
        }

        ~MissionPlanner() = default;

        const std::vector<Waypoint> &get_waypoints() const
        {
            return _waypoints;
        }

        int getCurrentWaypointIndex()
        {
            if (_currentWaypointIndex >= _waypoints.size())
            {
                return -1;
            }

            //std::cout << "Current waypoint index: " << _currentWaypointIndex << std::endl;

            return _currentWaypointIndex;
        }

        static double degToRad(double degrees)
        {
            return degrees * (M_PI / 180.0);
        }

        void setHomePosition(double refLatitude, double refLongitude)
        {
            _refLatitude = refLatitude;
            _refLongitude = refLongitude;
        }

        WaypointUpdateStatus updateCurrentWaypoint(const Eigen::Vector3f &currentPosition, const double acceptanceRadius, Eigen::Vector3f &local_target)
        {
            if (_currentWaypointIndex < _waypoints.size())
            {
                const auto &waypoint = _waypoints[_currentWaypointIndex];

                double waypoint_x, waypoint_y;
                latLonToLocalXY(waypoint.latitude, waypoint.longitude, waypoint_x, waypoint_y);

                local_target[0] = waypoint_y;
                local_target[1] = waypoint_x;
                local_target[2] = -waypoint.altitude;

                double distance = std::sqrt(
                    std::pow(currentPosition[0] - waypoint_y, 2) +
                    std::pow(currentPosition[1] - waypoint_x, 2));

                //std::cout << "Distance to waypoint: " << distance << std::endl;

                if (distance <= acceptanceRadius)
                {
                    _currentWaypointIndex++; // Move to the next waypoint
                    if (_currentWaypointIndex == _waypoints.size())
                    {
                        return MISSION_COMPLETE;
                    }
                    return WAYPOINT_REACHED;
                }
            }
            return NOT_REACHED;
        }

    private:
        std::vector<Waypoint> _waypoints;
        size_t _currentWaypointIndex;
        double _refLatitude{0.f};
        double _refLongitude{0.f};

        void latLonToLocalXY(double lat, double lon, double &x, double &y)
        {
            const double R = 6371000;
            lat = degToRad(lat);
            lon = degToRad(lon);
            double refLatRad = degToRad(_refLatitude);
            double refLonRad = degToRad(_refLongitude);
            x = R * (lon - refLonRad) * cos(refLatRad);
            y = R * (lat - refLatRad);
        }
    };

} // namespace px4_autonomy