// UGV_Waypoint_Following.cpp
//
// This script loads a list of waypoints from a text file and
// commands the UGV to sequentially pass through them using the
// CarRpcLibClient's moveOnPath function.
//
// Usage: UGV_Waypoint_Following <UGVName> <UGVLinearVelocity> <WaypointFilePath> [ControlHz]

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/Common.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>

using namespace msr::airlib;
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// Utility: sleep for a given number of seconds.
void sleep_for_seconds(int seconds)
{
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

// Load waypoints from a file. If the file has three values per line (x, y, z)
// they are used; if only two values are present, z is set to fixed_z.
std::vector<Vector3r> loadWaypoints(const string &file_path, float fixed_z)
{
    std::vector<Vector3r> waypoints;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        cerr << "Error opening waypoint file: " << file_path << endl;
        return waypoints;
    }

    string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z)
            waypoints.emplace_back(x, y, z);
        else if (iss >> x >> y)
            waypoints.emplace_back(x, y, fixed_z);
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
    // Accept 3 or 4 args: name, speed, file, [control loop Hz]
    if (argc < 4 || argc > 5)
    {
        cerr << "Usage: " << argv[0]
             << " <UGVName> <UGVLinearVelocity> <WaypointFilePath> [ControlHz]" << endl;
        return 1;
    }

    // Parse required args
    string ugv_name = argv[1];
    float desired_velocity = std::stof(argv[2]);
    string waypoint_file = argv[3];

    // Optional control-loop frequency (Hz), default = 10 Hz
    float control_freq = 10.0f;
    if (argc == 5)
    {
        control_freq = std::stof(argv[4]);
        if (control_freq <= 0.0f)
        {
            cerr << "Invalid ControlHz '" << argv[4]
                 << "', using default 10 Hz." << endl;
            control_freq = 10.0f;
        }
    }

    // Fixed Z for ground vehicle
    float fixed_z = 0.0f;

    // Load waypoints
    std::vector<Vector3r> waypoints = loadWaypoints(waypoint_file, fixed_z);
    if (waypoints.empty())
    {
        cerr << "[" << ugv_name << "] No valid waypoints found in file: "
             << waypoint_file << endl;
        return 1;
    }
    cout << "[" << ugv_name << "] Loaded " << waypoints.size()
         << " waypoints." << endl;

    // Create and configure client
    CarRpcLibClient client(simulatorHost(), 41452, 60);
    client.confirmConnection();
    client.enableApiControl(true, ugv_name);

    cout << "[" << ugv_name << "] Starting waypoint navigation at "
         << control_freq << " Hz control loop..." << endl;

    // Parameters for moveOnPath
    float timeout_sec = 1e6f; // max allowed time (s)
    float lookahead = 4.0f;   // lookahead distance (m)

    // Execute
    bool success = client.moveOnPath(
        waypoints,
        desired_velocity,
        timeout_sec,
        lookahead,
        ugv_name,
        control_freq);

    if (!success)
    {
        cerr << "[" << ugv_name << "] Failed to complete the waypoint mission."
             << endl;
        return 1;
    }

    cout << "[" << ugv_name << "] Waypoint mission complete." << endl;
    return 0;
}
