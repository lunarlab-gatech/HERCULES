// Created by Sandilya Sai Garimella.
/*
This script enables a drone to load its waypoints and follow them in HERCULES simulator. Each drone must
launch an instance of this script's executable with the drone name, corresponding waypoints text file,
and an optional velocity parameter.
*/

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

using namespace msr::airlib;

void sleep_for_seconds(int seconds)
{
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

std::vector<Vector3r> loadWaypoints(const std::string &file_path, float fixed_z, bool use_waypoint_z)
{
    std::vector<Vector3r> waypoints;
    std::ifstream file(file_path);

    if (!file.is_open())
    {
        std::cerr << "Error opening waypoint file: " << file_path << std::endl;
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z)
        {
            // Recorded files store Z up-positive; AirSim expects NED (z down-negative).
            // In waypoint-z mode, clamp to >=1m above origin so ground-level points
            // recorded before takeoff don't drive the drone into the floor.
            float z_ned = use_waypoint_z ? std::min(-z, -1.0f) : fixed_z;
            waypoints.emplace_back(Vector3r(x, y, z_ned));
        }
    }

    file.close();
    return waypoints;
}

// Simulator address: HERCULES_HOST, e.g. the Windows host when the simulator runs on Windows
// and this controller in WSL; localhost otherwise.
static std::string simulatorHost()
{
    const char* host = std::getenv("HERCULES_HOST");
    return (host != nullptr && *host != '\0') ? host : "localhost";
}

int main(int argc, char *argv[])
{
    if (argc < 3 || argc > 7)
    {
        std::cerr << "Usage: " << argv[0] << " <DroneName> <WaypointFilePath> [WaypointVelocity] [FlyAltitude] [--use-waypoint-z] [--no-return-home]" << std::endl;
        return 1;
    }

    std::string drone_name = argv[1];
    std::string waypoint_file = argv[2];
    float waypoint_flight_velocity = 2.0f;
    float fly_altitude = -35.0f;
    float return_home_velocity = 3.0f;
    bool enable_return_home = true;
    bool use_waypoint_z = false;
    int positional = 0;
    for (int i = 3; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--no-return-home")
            enable_return_home = false;
        else if (arg == "--use-waypoint-z")
            use_waypoint_z = true;
        else if (positional == 0)
        {
            waypoint_flight_velocity = std::stof(arg);
            positional = 1;
        }
        else
        {
            fly_altitude = std::stof(arg);
            positional = 2;
        }
    }

    try
    {
        MultirotorRpcLibClient client(simulatorHost(), 41451, 60.0f);
        client.confirmConnection();
        client.enableApiControl(true, drone_name);
        client.armDisarm(true, drone_name);

        std::cout << "[" << drone_name << "] Taking off..." << std::endl;
        client.takeoffAsync(20.0f, drone_name)->waitOnLastTask();
        sleep_for_seconds(1);

        auto state = client.getMultirotorState(drone_name);
        if (state.landed_state == LandedState::Landed)
        {
            std::cerr << "[" << drone_name << "] Takeoff failed. Exiting..." << std::endl;
            return 1;
        }

        std::vector<Vector3r> waypoints = loadWaypoints(waypoint_file, fly_altitude, use_waypoint_z);
        if (waypoints.empty())
        {
            std::cerr << "[" << drone_name << "] No valid waypoints found. Exiting..." << std::endl;
            return 1;
        }

        // Initial climb: fixed altitude, or the first waypoint's altitude in waypoint-z mode
        float initial_z = use_waypoint_z ? waypoints.front().z() : fly_altitude;
        std::cout << "[" << drone_name << "] Moving to altitude " << -initial_z << " meters"
                  << (use_waypoint_z ? " (waypoint-z mode)" : "") << "..." << std::endl;
        client.moveToZAsync(initial_z, 2.5f, 30.0f, YawMode(true, 0), -1.0f, 1.0f, drone_name)->waitOnLastTask();

        // Skip leading waypoints within 2m (XY) of the start position: they are the
        // near-stationary points recorded during takeoff, and with ForwardOnly yaw
        // they give the controller no usable heading, causing a spin at path start.
        auto start_pos = client.getMultirotorState(drone_name).getPosition();
        size_t start_idx = 0;
        while (start_idx + 1 < waypoints.size())
        {
            float dx = waypoints[start_idx].x() - start_pos.x();
            float dy = waypoints[start_idx].y() - start_pos.y();
            if (std::sqrt(dx * dx + dy * dy) >= 2.0f)
                break;
            ++start_idx;
        }
        std::vector<Vector3r> path(waypoints.begin() + start_idx, waypoints.end());

        // Pre-aim at the first real waypoint so the path starts without a yaw flail.
        // No-op if the drone already faces its direction of travel.
        float aim_dx = path.front().x() - start_pos.x();
        float aim_dy = path.front().y() - start_pos.y();
        if (std::sqrt(aim_dx * aim_dx + aim_dy * aim_dy) > 0.5f)
        {
            float yaw_deg = std::atan2(aim_dy, aim_dx) * 180.0f / static_cast<float>(M_PI);
            client.rotateToYawAsync(yaw_deg, 10.0f, 2.0f, drone_name)->waitOnLastTask();
        }

        std::cout << "[" << drone_name << "] Flying through " << path.size()
                  << " waypoints (skipped " << start_idx << " takeoff points) at velocity "
                  << waypoint_flight_velocity << " m/s..." << std::endl;

        client.moveOnPathAsync(path, waypoint_flight_velocity, 100000.0f,
                               DrivetrainType::ForwardOnly,
                               YawMode(false, 0), 4.0f, 1.0f,
                               drone_name)
            ->waitOnLastTask();

        sleep_for_seconds(2);

        if (enable_return_home)
        {
            Vector3r home_position = waypoints.front();

            std::cout << "[" << drone_name << "] Returning to home waypoint ("
                      << home_position.x() << ", " << home_position.y() << ", " << home_position.z() << ")..." << std::endl;

            client.moveToPositionAsync(home_position.x(), home_position.y(), home_position.z(),
                                       return_home_velocity, 10000.0f,
                                       DrivetrainType::ForwardOnly,
                                       YawMode(false, 0), -1.0f, 1.0f,
                                       drone_name)
                ->waitOnLastTask();

            std::cout << "[" << drone_name << "] Hovering before landing..." << std::endl;
            client.hoverAsync(drone_name)->waitOnLastTask();
            sleep_for_seconds(5);

            client.moveToZAsync(-2.0f, 2.0f, 1000.0f, YawMode(false, 0), -1.0f, 1.0f, drone_name)->waitOnLastTask();
            sleep_for_seconds(2);

            std::cout << "[" << drone_name << "] Initiating landing..." << std::endl;
            client.landAsync(60.0f, drone_name)->waitOnLastTask();
        }
        else
        {
            std::cout << "[" << drone_name << "] Hovering in place after completing waypoints (return-home disabled)..." << std::endl;
            client.hoverAsync(drone_name)->waitOnLastTask();
            sleep_for_seconds(5);

            std::cout << "[" << drone_name << "] Descending vertically and landing..." << std::endl;
            client.moveToZAsync(-2.0f, 2.0f, 30.0f, YawMode(false, 0), -1.0f, 1.0f, drone_name)->waitOnLastTask();
            sleep_for_seconds(2);

            client.landAsync(60.0f, drone_name)->waitOnLastTask();
        }

        client.armDisarm(false, drone_name);
        client.enableApiControl(false, drone_name);

        std::cout << "[" << drone_name << "] Flight complete." << std::endl;
    }
    catch (rpc::rpc_error &e)
    {
        std::cerr << "RPC Exception occurred for drone [" << drone_name << "]: "
                  << e.get_error().as<std::string>() << std::endl;
        return 1;
    }

    return 0;
}
