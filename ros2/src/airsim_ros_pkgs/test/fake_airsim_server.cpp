// A stand-in for the HERCULES simulator's RPC servers, for testing the ROS 2
// wrapper without Unreal Engine.
//
// It parses a settings.json with AirLib (the same code the simulator uses)
// and answers the RPCs hercules_node makes with deterministic data on the
// wire format of the real servers (AirLib's msgpack adaptors):
//
//   * every vehicle starts at its settings pose and moves forward along its
//     own x axis at 1 m/s, so odometry is predictable;
//   * IMU/GPS/barometer/magnetometer readings are static;
//   * a lidar returns two points 5 m ahead and 5 m to the right of the
//     sensor, a distance sensor reads 2 m, both at their settings poses;
//   * images are small blank frames at the size the settings request.
//
//   fake_airsim_server --settings FILE [--multirotor-port 41451] [--car-port 41452]

// the adaptor headers expect the API types to be declared first, as in
// AirLib's own RpcLibServerBase.cpp
#include "api/RpcLibServerBase.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/AirSimSettings.hpp"
#include "common/ClockFactory.hpp"
#include "sensors/distance/DistanceSimpleParams.hpp"
#include "sensors/lidar/GPULidarSimpleParams.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "vehicles/car/api/CarRpcLibAdaptors.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibAdaptors.hpp"

#include <rpc/server.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace msr::airlib;
using Adaptors = msr::airlib_rpclib::RpcLibAdaptorsBase;
using msr::airlib_rpclib::CarRpcLibAdaptors;
using msr::airlib_rpclib::MultirotorRpcLibAdaptors;

namespace
{
std::atomic<bool> running{ true };
std::string settings_text;
const auto start_time = std::chrono::steady_clock::now();

uint64_t nowNanos()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

double elapsedSeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
}

const AirSimSettings::VehicleSetting& vehicleSetting(const std::string& name)
{
    const auto& vehicles = AirSimSettings::singleton().vehicles;
    const auto found = vehicles.find(name);
    if (found == vehicles.end())
        throw std::invalid_argument("unknown vehicle '" + name + "'");
    return *found->second;
}

real_T orZero(real_T value)
{
    return std::isnan(value) ? 0 : value;
}

// Start at the settings pose and drive forward along the body x axis at 1 m/s.
Kinematics::State kinematics(const std::string& vehicle_name)
{
    const auto& setting = vehicleSetting(vehicle_name);
    const Quaternionr orientation = VectorMath::toQuaternion(
        Utils::degreesToRadians(orZero(setting.rotation.pitch)),
        Utils::degreesToRadians(orZero(setting.rotation.roll)),
        Utils::degreesToRadians(orZero(setting.rotation.yaw)));
    const Vector3r start(orZero(setting.position.x()), orZero(setting.position.y()), orZero(setting.position.z()));
    Kinematics::State state = Kinematics::State::zero();
    state.pose.position = start + VectorMath::rotateVector(Vector3r(static_cast<real_T>(elapsedSeconds()), 0, 0),
                                                           orientation, true);
    state.pose.orientation = orientation;
    state.twist.linear = VectorMath::rotateVector(Vector3r(1, 0, 0), orientation, true);
    return state;
}

const AirSimSettings::SensorSetting& sensorSetting(const std::string& sensor_name, const std::string& vehicle_name)
{
    const auto& sensors = vehicleSetting(vehicle_name).sensors;
    const auto found = sensors.find(sensor_name);
    if (found == sensors.end())
        throw std::invalid_argument("vehicle '" + vehicle_name + "' has no sensor '" + sensor_name + "'");
    return *found->second;
}

// Two points 5 m ahead and 5 m to the right, in the sensor frame.
std::vector<real_T> twoPoints()
{
    return { 5, 0, 0, 0, 5, 0 };
}

void bindCommon(rpc::server& server)
{
    server.bind("ping", []() -> bool { return true; });
    server.bind("getServerVersion", []() -> int { return 3; });
    server.bind("getMinRequiredClientVersion", []() -> int { return 3; });
    server.bind("getSettingsString", []() -> std::string { return settings_text; });
    server.bind("listVehicles", []() -> std::vector<std::string> {
        std::vector<std::string> names;
        for (const auto& vehicle : AirSimSettings::singleton().vehicles)
            names.push_back(vehicle.first);
        return names;
    });
    server.bind("reset", []() -> void {});
    server.bind("simIsPaused", []() -> bool { return false; });
    server.bind("enableApiControl", [](bool, const std::string&) -> void {});
    server.bind("armDisarm", [](bool, const std::string&) -> bool { return true; });
    server.bind("getHomeGeoPoint", [](const std::string&) -> Adaptors::GeoPoint {
        return Adaptors::GeoPoint(GeoPoint(47.641468, -122.140165, 122));
    });
    server.bind("simListInstanceSegmentationObjects", []() -> std::vector<std::string> { return {}; });
    server.bind("simGetInstanceSegmentationColorMap", []() -> std::vector<Adaptors::Vector3r> { return {}; });
    server.bind("simListInstanceSegmentationPoses", [](bool, bool) -> std::vector<Adaptors::Pose> { return {}; });
    server.bind("simGetGroundTruthEnvironment", [](const std::string& vehicle_name) -> Adaptors::EnvironmentState {
        Environment::State state;
        state.position = kinematics(vehicle_name).pose.position;
        state.geo_point = GeoPoint(47.641468, -122.140165, 122);
        state.gravity = Vector3r(0, 0, 9.81f);
        state.air_pressure = 101325;
        state.temperature = 288.15f;
        state.air_density = 1.225f;
        return Adaptors::EnvironmentState(state);
    });

    server.bind("getImuData", [](const std::string&, const std::string& vehicle_name) -> Adaptors::ImuData {
        ImuBase::Output output;
        output.time_stamp = nowNanos();
        output.orientation = kinematics(vehicle_name).pose.orientation;
        output.angular_velocity = Vector3r::Zero();
        output.linear_acceleration = Vector3r(0, 0, -9.81f);
        return Adaptors::ImuData(output);
    });
    server.bind("getBarometerData", [](const std::string&, const std::string&) -> Adaptors::BarometerData {
        BarometerBase::Output output;
        output.time_stamp = nowNanos();
        output.altitude = 122;
        output.pressure = 101325;
        output.qnh = 1013.25f;
        return Adaptors::BarometerData(output);
    });
    server.bind("getMagnetometerData", [](const std::string&, const std::string&) -> Adaptors::MagnetometerData {
        MagnetometerBase::Output output;
        output.time_stamp = nowNanos();
        output.magnetic_field_body = Vector3r(0.2f, 0, 0.4f);
        output.magnetic_field_covariance.assign(9, 0);
        return Adaptors::MagnetometerData(output);
    });
    server.bind("getGpsData", [](const std::string&, const std::string&) -> Adaptors::GpsData {
        GpsBase::Output output;
        output.time_stamp = nowNanos();
        output.gnss.geo_point = GeoPoint(47.641468, -122.140165, 122);
        output.gnss.eph = output.gnss.epv = 0.1f;
        output.gnss.velocity = Vector3r::Zero();
        output.gnss.fix_type = GpsBase::GnssFixType::GNSS_FIX_3D_FIX;
        output.is_valid = true;
        return Adaptors::GpsData(output);
    });
    server.bind("getDistanceSensorData", [](const std::string& sensor_name, const std::string& vehicle_name) -> Adaptors::DistanceSensorData {
        DistanceSimpleParams params;
        params.initializeFromSettings(static_cast<const AirSimSettings::DistanceSetting&>(sensorSetting(sensor_name, vehicle_name)));
        DistanceSensorData data;
        data.time_stamp = nowNanos();
        data.distance = 2;
        data.min_distance = params.min_distance;
        data.max_distance = params.max_distance;
        data.relative_pose = params.relative_pose;
        return Adaptors::DistanceSensorData(data);
    });
    server.bind("getLidarData", [](const std::string& sensor_name, const std::string& vehicle_name) -> Adaptors::LidarData {
        LidarSimpleParams params;
        params.initializeFromSettings(static_cast<const AirSimSettings::LidarSetting&>(sensorSetting(sensor_name, vehicle_name)));
        LidarData data;
        data.time_stamp = nowNanos();
        data.point_cloud = twoPoints();
        data.groundtruth = { "wall", "wall" };
        data.pose = params.relative_pose;
        return Adaptors::LidarData(data);
    });
    server.bind("getGPULidarData", [](const std::string& sensor_name, const std::string& vehicle_name) -> Adaptors::GPULidarData {
        GPULidarSimpleParams params;
        params.initializeFromSettings(static_cast<const AirSimSettings::GPULidarSetting&>(sensorSetting(sensor_name, vehicle_name)));
        GPULidarData data;
        data.time_stamp = nowNanos();
        data.point_cloud = { 5, 0, 0, 0, 1, 0, 5, 0, 0, 1 }; // x, y, z, rgb, intensity
        data.pose = params.relative_pose;
        return Adaptors::GPULidarData(data);
    });
    server.bind("simGetCameraInfo", [](const std::string& camera_name, const std::string& vehicle_name) -> Adaptors::CameraInfo {
        const auto& cameras = vehicleSetting(vehicle_name).cameras;
        const auto found = cameras.find(camera_name);
        if (found == cameras.end())
            throw std::invalid_argument("no camera '" + camera_name + "'");
        CameraInfo info;
        info.pose.position = found->second.position;
        info.pose.orientation = VectorMath::toQuaternion(Utils::degreesToRadians(orZero(found->second.rotation.pitch)),
                                                         Utils::degreesToRadians(orZero(found->second.rotation.roll)),
                                                         Utils::degreesToRadians(orZero(found->second.rotation.yaw)));
        info.fov = 90;
        return Adaptors::CameraInfo(info);
    });
    server.bind("simGetImages", [](const std::vector<Adaptors::ImageRequest>& request_adaptor, const std::string& vehicle_name) -> std::vector<Adaptors::ImageResponse> {
        std::vector<ImageCaptureBase::ImageResponse> responses;
        for (const auto& request : Adaptors::ImageRequest::to(request_adaptor)) {
            ImageCaptureBase::ImageResponse response;
            const auto& camera = vehicleSetting(vehicle_name).cameras.at(request.camera_name);
            const auto& capture = camera.capture_settings.at(Utils::toNumeric(request.image_type));
            response.camera_name = request.camera_name;
            response.image_type = request.image_type;
            response.width = capture.width;
            response.height = capture.height;
            response.pixels_as_float = request.pixels_as_float;
            response.compress = false;
            response.time_stamp = nowNanos();
            if (request.pixels_as_float)
                response.image_data_float.assign(static_cast<size_t>(capture.width) * capture.height, 10.0f);
            else
                response.image_data_uint8.assign(static_cast<size_t>(capture.width) * capture.height * 3, 128);
            responses.push_back(response);
        }
        return Adaptors::ImageResponse::from(responses);
    });
    server.suppress_exceptions(true);
}

void bindMultirotor(rpc::server& server)
{
    server.bind("getMultirotorState", [](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::MultirotorState {
        MultirotorState state;
        state.kinematics_estimated = kinematics(vehicle_name);
        state.gps_location = GeoPoint(47.641468, -122.140165, 122);
        state.timestamp = nowNanos();
        state.landed_state = LandedState::Flying;
        state.ready = true;
        state.can_arm = true;
        return MultirotorRpcLibAdaptors::MultirotorState(state);
    });
    server.bind("takeoff", [](float, const std::string&) -> bool { return true; });
    server.bind("land", [](float, const std::string&) -> bool { return true; });
}

void bindCar(rpc::server& server)
{
    server.bind("getCarState", [](const std::string& vehicle_name) -> CarRpcLibAdaptors::CarState {
        return CarRpcLibAdaptors::CarState(CarApiBase::CarState(1, 1, 1000, 7000, false, kinematics(vehicle_name), nowNanos()));
    });
    server.bind("setCarControls", [](const CarRpcLibAdaptors::CarControls&, const std::string&) -> void {});
}
}

int main(int argc, char** argv)
{
    std::string settings_path;
    int multirotor_port = 0, car_port = 0;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::strcmp(argv[i], "--settings") == 0)
            settings_path = argv[i + 1];
        else if (std::strcmp(argv[i], "--multirotor-port") == 0)
            multirotor_port = std::stoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--car-port") == 0)
            car_port = std::stoi(argv[i + 1]);
    }
    if (settings_path.empty() || (multirotor_port == 0 && car_port == 0)) {
        std::cerr << "usage: fake_airsim_server --settings FILE [--multirotor-port P] [--car-port P]" << std::endl;
        return 2;
    }
    std::ifstream stream(settings_path);
    if (!stream) {
        std::cerr << "cannot read " << settings_path << std::endl;
        return 2;
    }
    std::stringstream buffer;
    buffer << stream.rdbuf();
    settings_text = buffer.str();
    AirSimSettings::initializeSettings(settings_text);
    AirSimSettings::singleton().load(nullptr);

    std::vector<std::unique_ptr<rpc::server>> servers;
    if (multirotor_port != 0) {
        servers.emplace_back(new rpc::server(static_cast<uint16_t>(multirotor_port)));
        bindCommon(*servers.back());
        bindMultirotor(*servers.back());
    }
    if (car_port != 0) {
        servers.emplace_back(new rpc::server(static_cast<uint16_t>(car_port)));
        bindCommon(*servers.back());
        bindCar(*servers.back());
    }
    for (auto& server : servers)
        server->async_run(2);
    std::cout << "fake_airsim_server ready (" << AirSimSettings::singleton().vehicles.size() << " vehicles)" << std::endl;

    std::signal(SIGINT, [](int) { running = false; });
    std::signal(SIGTERM, [](int) { running = false; });
    while (running)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (auto& server : servers)
        server->stop();
    return 0;
}
