#ifndef msr_AirLibUnitTests_UrdfVisualTest_hpp
#define msr_AirLibUnitTests_UrdfVisualTest_hpp

#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "common/UrdfVisualManifest.hpp"
#include "common/VectorMath.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace msr
{
namespace airlib
{

    // Checks the reader for hercules_urdf_import's visual manifest and STL
    // meshes. With an import directory (main.cpp --urdf-import-dir DIR) it
    // also validates real importer output: every mesh loads, and every sensor
    // pose written to settings.json turns, through AirLib's own Euler
    // convention, into the FRD orientation the importer computed.
    class UrdfVisualTest : public TestBase
    {
    public:
        explicit UrdfVisualTest(std::string import_dir = "")
            : import_dir_(std::move(import_dir))
        {
        }

        virtual void run() override
        {
            testManifest();
            testManifestErrors();
            testStl();
            testSettings();
            if (!import_dir_.empty())
                testImporterOutput();
        }

    private:
        std::string import_dir_;
        int lidars_ = 0;

        static bool near(real_T a, real_T b, real_T tol = 1e-5f)
        {
            return std::abs(a - b) <= tol;
        }

        static bool nearVec(const Vector3r& a, const Vector3r& b, real_T tol = 1e-5f)
        {
            return (a - b).norm() <= tol;
        }

        static bool sameRotation(const Quaternionr& a, const Quaternionr& b, real_T tol = 1e-4f)
        {
            return std::abs(std::abs(a.dot(b)) - 1) <= tol;
        }

        void expectThrow(const std::string& json, const std::string& what)
        {
            bool threw = false;
            try {
                UrdfVisualManifest::parse(json, "");
            }
            catch (const std::runtime_error&) {
                threw = true;
            }
            testAssert(threw, "manifest with " + what + " was accepted");
        }

        void testManifest()
        {
            // FLU: 1 m forward, 2 m left, 3 m up, turned 90 degrees to the left.
            const real_T h = std::sqrt(0.5f);
            const std::string json = R"({
                "format": "hercules-urdf-visuals", "version": 1, "robot": "bot",
                "vehicle": "Bot1", "frame": "base_link", "frame_convention": "FLU",
                "units": "meters",
                "visuals": [{"name": "body", "link": "base_link", "mesh": "meshes/body.stl",
                             "position": [1, 2, 3],
                             "orientation": {"w": )" + std::to_string(h) + R"(, "x": 0, "y": 0, "z": )" + std::to_string(h) + R"(},
                             "scale": [1, 1, 2], "color": [0.1, 0.2, 1.5, 1]},
                            {"name": "wheel", "mesh": "/abs/wheel.stl"}]
            })";
            const auto manifest = UrdfVisualManifest::parse(json, "/robots/bot");
            testAssert(manifest.robot == "bot" && manifest.vehicle == "Bot1" && manifest.frame == "base_link",
                       "manifest header fields");
            testAssert(manifest.visuals.size() == 2, "manifest visual count");

            const auto& body = manifest.visuals[0];
            testAssert(body.mesh_path == "/robots/bot/meshes/body.stl", "relative mesh path resolves against the manifest");
            testAssert(manifest.visuals[1].mesh_path == "/abs/wheel.stl", "absolute mesh path kept");
            // 2 m left and 3 m up are -2 (east is right) and -3 (down) in FRD
            testAssert(nearVec(body.pose.position, Vector3r(1, -2, -3)), "FLU position converted to FRD");
            // turning left is a negative (counter-clockwise) FRD yaw
            real_T pitch, roll, yaw;
            VectorMath::toEulerianAngle(body.pose.orientation, pitch, roll, yaw);
            testAssert(near(roll, 0) && near(pitch, 0) && near(yaw, -Utils::degreesToRadians(90.0f)), "FLU yaw converted to FRD yaw");
            // and the forward axis now points left, i.e. -y in FRD
            testAssert(nearVec(VectorMath::rotateVector(Vector3r(1, 0, 0), body.pose.orientation, true),
                               Vector3r(0, -1, 0)),
                       "converted orientation rotates forward to the left");
            testAssert(nearVec(body.scale, Vector3r(1, 1, 2)), "scale read");
            testAssert(near(body.color[0], 0.1f) && near(body.color[2], 1.0f), "color read and clamped");

            const auto& wheel = manifest.visuals[1];
            testAssert(nearVec(wheel.pose.position, Vector3r::Zero()) &&
                           sameRotation(wheel.pose.orientation, Quaternionr::Identity()),
                       "missing pose defaults to identity");
            testAssert(near(wheel.color[3], 1.0f), "missing color defaults to opaque");
        }

        void testManifestErrors()
        {
            const std::string head = R"({"format": "hercules-urdf-visuals", "version": 1, )";
            expectThrow("not json", "invalid JSON");
            expectThrow(R"({"format": "other", "version": 1, "visuals": []})", "a foreign format");
            expectThrow(R"({"format": "hercules-urdf-visuals", "version": 2, "visuals": []})", "a newer version");
            expectThrow(head + R"("frame_convention": "FRD", "visuals": []})", "FRD axes");
            expectThrow(head + R"("units": "millimeters", "visuals": []})", "millimeters");
            expectThrow(head + R"("visuals": {}})", "visuals not an array");
            expectThrow(head + R"("visuals": [{"name": "a"}]})", "a visual without mesh");
            expectThrow(head + R"("visuals": [{"mesh": "a.stl", "position": [1, 2]}]})", "a short position");
            expectThrow(head + R"("visuals": [{"mesh": "a.stl", "orientation": {"w": 0}}]})", "a zero quaternion");
            expectThrow(head + R"("visuals": [{"mesh": "a.stl", "color": [1, 1, 1]}]})", "a 3-channel color");
        }

        void testStl()
        {
            namespace fs = std::filesystem;
            const fs::path dir = fs::temp_directory_path() / "airlib_urdf_visual_test";
            fs::create_directories(dir);

            // Binary STL whose 80-byte header starts with "solid", as many
            // exporters write it: must still be read as binary.
            const fs::path binary = dir / "tri_binary.stl";
            {
                std::ofstream out(binary, std::ios::binary);
                char header[80] = "solid exported by some CAD tool";
                out.write(header, sizeof(header));
                const uint32_t count = 1;
                out.write(reinterpret_cast<const char*>(&count), sizeof(count));
                const float facet[12] = { 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0 };
                out.write(reinterpret_cast<const char*>(facet), sizeof(facet));
                const uint16_t attributes = 0;
                out.write(reinterpret_cast<const char*>(&attributes), sizeof(attributes));
            }
            auto vertices = UrdfVisualManifest::loadStl(binary.string());
            testAssert(vertices.size() == 3 && nearVec(vertices[1], Vector3r(1, 0, 0)) &&
                           nearVec(vertices[2], Vector3r(0, 1, 0)),
                       "binary STL with a 'solid' header");
            testAssert(nearVec(UrdfVisualManifest::faceNormal(vertices[0], vertices[1], vertices[2]), Vector3r(0, 0, 1)),
                       "counter-clockwise face normal");

            const fs::path ascii = dir / "tri_ascii.stl";
            {
                std::ofstream out(ascii);
                out << "solid tri\n facet normal 0 0 1\n  outer loop\n"
                       "   vertex 0 0 0\n   vertex 0.5 0 0\n   vertex 0 0.25 1e-1\n"
                       "  endloop\n endfacet\nendsolid tri\n";
            }
            vertices = UrdfVisualManifest::loadStl(ascii.string());
            testAssert(vertices.size() == 3 && nearVec(vertices[2], Vector3r(0, 0.25f, 0.1f)), "ASCII STL");

            const fs::path bad = dir / "bad.stl";
            {
                std::ofstream out(bad, std::ios::binary);
                out << "garbage";
            }
            bool threw = false;
            try {
                UrdfVisualManifest::loadStl(bad.string());
            }
            catch (const std::runtime_error&) {
                threw = true;
            }
            testAssert(threw, "garbage STL rejected");
            fs::remove_all(dir);
        }

        void testSettings()
        {
            const std::string json = R"({
                "SettingsVersion": 2.0, "SimMode": "Hero",
                "Vehicles": {
                    "Bot1": {"VehicleType": "PhysXCar",
                             "Urdf": {"Visuals": "bot.visuals.json", "RobotDescription": "bot.urdf",
                                      "HideBaseMesh": true, "TwoSided": true,
                                      "SensorFrames": ["front_cam", 7, "lidar"]}},
                    "Drone1": {"VehicleType": "SimpleFlight"}
                }
            })";
            AirSimSettings::initializeSettings(json);
            auto& settings = AirSimSettings::singleton();
            settings.load(nullptr);
            const auto& bot = settings.vehicles.at("Bot1")->urdf;
            testAssert(bot.visuals == "bot.visuals.json" && bot.robot_description == "bot.urdf",
                       "Urdf paths parsed");
            testAssert(bot.hide_base_mesh && bot.two_sided, "Urdf flags parsed");
            testAssert(bot.sensor_frames == std::vector<std::string>({ "front_cam", "lidar" }),
                       "Urdf sensor frames parsed, non-strings skipped");
            const auto& drone = settings.vehicles.at("Drone1")->urdf;
            testAssert(drone.visuals.empty() && !drone.hide_base_mesh && !drone.two_sided && drone.sensor_frames.empty(),
                       "vehicles without Urdf keep the defaults");
        }

        void testImporterOutput()
        {
            namespace fs = std::filesystem;
            int manifests = 0, sensors = 0;
            for (const auto& entry : fs::directory_iterator(import_dir_)) {
                const std::string path = entry.path().string();
                const std::string suffix = ".visuals.json";
                if (path.size() <= suffix.size() || path.compare(path.size() - suffix.size(), suffix.size(), suffix) != 0)
                    continue;
                ++manifests;
                const auto manifest = UrdfVisualManifest::load(path);
                testAssert(!manifest.visuals.empty(), path + " has no visuals");
                for (const auto& visual : manifest.visuals) {
                    const auto vertices = UrdfVisualManifest::loadStl(visual.mesh_path);
                    testAssert(!vertices.empty() && vertices.size() % 3 == 0, visual.mesh_path + " is empty");
                    testAssert(std::abs(visual.pose.orientation.norm() - 1) < 1e-5f, visual.name + " orientation not unit");
                }

                // Cross-check the settings Euler angles against the importer's quaternions.
                const std::string stem = path.substr(0, path.size() - suffix.size());
                std::ifstream manifest_stream(path), settings_stream(stem + ".settings.json");
                testAssert(static_cast<bool>(settings_stream), stem + ".settings.json missing");
                const auto doc = nlohmann::json::parse(manifest_stream);
                const auto settings = nlohmann::json::parse(settings_stream);
                const auto& vehicle = settings.at("Vehicles").at(doc.at("vehicle").get<std::string>());

                // AirLib must accept the settings, and lidar beam tables must
                // agree with their channel count
                AirSimSettings::initializeSettings(settings.dump());
                AirSimSettings::singleton().load(nullptr);
                for (const auto& sensor : AirSimSettings::singleton().vehicles.at(doc.at("vehicle").get<std::string>())->sensors) {
                    if (sensor.second->sensor_type != SensorBase::SensorType::Lidar)
                        continue;
                    LidarSimpleParams lidar;
                    lidar.initializeFromSettings(*static_cast<const AirSimSettings::LidarSetting*>(sensor.second.get()));
                    testAssert(lidar.channelElevations().size() == lidar.number_of_channels,
                               "lidar " + sensor.first + " beam table does not match its channels");
                    ++lidars_;
                }
                for (const auto& sensor : doc.at("sensors")) {
                    if (sensor.at("frd_pose").is_null())
                        continue;
                    const std::string name = sensor.at("name");
                    const auto& entry_json = sensor.at("kind") == "camera" ? vehicle.at("Cameras").at(name)
                                                                           : vehicle.at("Sensors").at(name);
                    const auto& pose = sensor.at("frd_pose");
                    const auto& q = pose.at("orientation");
                    const Quaternionr expected(q.at("w").get<float>(), q.at("x").get<float>(),
                                               q.at("y").get<float>(), q.at("z").get<float>());
                    const Quaternionr from_settings = VectorMath::toQuaternion(
                        Utils::degreesToRadians(entry_json.at("Pitch").get<float>()),
                        Utils::degreesToRadians(entry_json.at("Roll").get<float>()),
                        Utils::degreesToRadians(entry_json.at("Yaw").get<float>()));
                    testAssert(sameRotation(expected, from_settings), "settings angles of " + name + " disagree with the importer");
                    const auto& p = pose.at("position");
                    testAssert(nearVec(Vector3r(p[0].get<float>(), p[1].get<float>(), p[2].get<float>()),
                                       Vector3r(entry_json.at("X").get<float>(), entry_json.at("Y").get<float>(),
                                                entry_json.at("Z").get<float>())),
                               "settings position of " + name + " disagrees with the importer");
                    ++sensors;
                }
            }
            testAssert(manifests > 0, "no *.visuals.json in " + import_dir_);
            std::cout << "UrdfVisualTest: checked " << manifests << " imported robot(s), " << sensors
                      << " sensor pose(s), " << lidars_ << " lidar(s)" << std::endl;
        }
    };
}
}
#endif
