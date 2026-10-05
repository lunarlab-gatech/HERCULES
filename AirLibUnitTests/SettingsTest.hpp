#ifndef msr_AirLibUnitTests_SettingsTest_hpp
#define msr_AirLibUnitTests_SettingsTest_hpp

#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"

#include <cmath>

namespace msr
{
namespace airlib
{

    class SettingsTest : public TestBase
    {
    public:
        virtual void run() override
        {
            // A heterogeneous team in the shape of ros2/settings/hero_blocks_team.json.
            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 2.0,
                "SimMode": "Hero",
                "PawnPaths": {"UGVPawn": {"PawnBP": "Class'/AirSim/VehicleAdv/SUV/UGVPawn.UGVPawn_C'"}},
                "Vehicles": {
                    "Drone1": {
                        "VehicleType": "SimpleFlight", "X": 0, "Y": 6, "Z": 0, "Yaw": 90,
                        "Cameras": {"front_center": {"X": 0.5, "Y": 0.0, "Z": -0.1, "Pitch": -10}},
                        "Sensors": {"LidarSensor1": {"SensorType": 6, "Enabled": true, "X": 0, "Y": 0, "Z": -0.8}}
                    },
                    "Husky1": {
                        "VehicleType": "PhysXCar", "PawnPath": "UGVPawn", "X": 6, "Y": 0, "Z": 0,
                        "Sensors": {"imu": {"SensorType": 2, "Enabled": true}}
                    }
                }
            })");
            auto& settings = AirSimSettings::singleton();
            settings.load(nullptr);

            testAssert(settings.simmode_name == "Hero", "SimMode was not parsed");
            testAssert(settings.pawn_paths.count("UGVPawn") == 1, "custom PawnPath missing");
            testAssert(settings.pawn_paths.count("DefaultSkidVehicle") == 1, "default PawnPaths missing");

            const auto* drone = settings.getVehicleSetting("Drone1");
            const auto* husky = settings.getVehicleSetting("Husky1");
            testAssert(drone != nullptr && husky != nullptr, "team vehicles missing");
            testAssert(drone->vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight, "drone type");
            testAssert(husky->vehicle_type == AirSimSettings::kVehicleTypePhysXCar, "husky type");
            testAssert(husky->pawn_path == "UGVPawn", "husky pawn path");
            testAssert(drone->position.y(), 6.0, "vehicle position");
            testAssert(drone->rotation.yaw, 90.0, "vehicle yaw");

            const auto& camera = drone->cameras.at("front_center");
            testAssert(near(camera.position.x(), 0.5f) && near(camera.position.z(), -0.1f),
                       "camera position");
            testAssert(camera.rotation.pitch, -10.0, "camera pitch");
            // Unset pose fields stay NaN ("keep the component default").
            testAssert(std::isnan(camera.rotation.roll), "unset camera roll stays NaN");

            const auto lidar = drone->sensors.find("LidarSensor1");
            testAssert(lidar != drone->sensors.end(), "lidar sensor missing");
            testAssert(lidar->second->sensor_type == SensorBase::SensorType::Lidar, "lidar type");
            testAssert(husky->sensors.count("imu") == 1, "husky imu missing");
        }

    private:
        static bool near(float value, float expected)
        {
            return std::fabs(value - expected) < 1e-5f;
        }
    };
}
}
#endif
