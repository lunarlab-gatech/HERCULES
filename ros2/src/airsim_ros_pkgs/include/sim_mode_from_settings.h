#pragma once

#include <stdexcept>
#include <string>
#include <vector>

// With interactive vehicle selection, the simulator can return settings without
// SimMode. These names match AirSimSettings::createDefaultVehicle().
inline std::string sim_mode_from_default_vehicle(const std::vector<std::string>& vehicles)
{
    if (vehicles.size() == 1) {
        const auto& vehicle = vehicles.front();
        if (vehicle == "PhysXCar")
            return "Car";
        if (vehicle == "SimpleFlight")
            return "Multirotor";
        if (vehicle == "CPHusky")
            return "SkidVehicle";
        if (vehicle == "ComputerVision")
            return "ComputerVision";
    }

    throw std::invalid_argument(
        "Simulator settings omit SimMode and the active vehicles do not identify a default mode. "
        "Set SimMode explicitly in the simulator's settings.json and restart the simulator.");
}
