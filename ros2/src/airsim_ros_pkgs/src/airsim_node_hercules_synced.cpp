// airsim_node_hercules_synced.cpp
//
// A synchronized sibling to hercules_node. Instead of the async wrapper's independent
// per-stream timers against a free-running sim, this node reads pose/odom, DepthPlanar,
// segmentation and LiDAR in ONE RPC render cycle and stamps every message + TF with a
// single shared timestamp. This removes the pose/image temporal skew (root cause A) and it
// requests DepthPlanar (planar Z), so the mappers need no perspective->planar conversion
// (root cause B).
//
//   sync_mode = "atomic"   : one best-effort RPC cycle per wall-timer tick (legacy async).
//   sync_mode = "lockstep" : mirrors PythonClient/hero/data_collection/
//                            hercules_multi_vehicle_data_collector.py. Pause once, then per
//                            tick simContinueForTime(dt) to advance SIM time, and BLOCK
//                            (re-request the same simGetImages / getLidarData without
//                            stepping) until every image + the lidar are non-empty. Sim time
//                            NEVER advances without complete valid data, so "sync drops" are
//                            structurally impossible.
//
// MULTI-VEHICLE: one node, ONE simContinueForTime(dt) per tick, then for EACH vehicle in
// `vehicles` (in sequence) the blocking reads (images, lidar, pose) while the sim stays
// frozen. All vehicles' data per tick share the same stamp t=step*dt. Poses are expressed
// relative to a SINGLE shared init reference (the first vehicle's first pose) so all robots
// sit at correct RELATIVE positions in one shared map frame. Per-vehicle topics/TF live
// under vehicle_name (Husky1/..., Husky2/...); one shared /cmd_vel per vehicle.
//
// It re-derives the SAME topic/TF interface hercules_node exposes so hercules_nav_bridge
// works unchanged. It does NOT touch hercules_ros_wrapper / hercules_node.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using ImageCaptureBase = msr::airlib::ImageCaptureBase;
using ImageRequest = ImageCaptureBase::ImageRequest;
using ImageResponse = ImageCaptureBase::ImageResponse;
using ImageType = ImageCaptureBase::ImageType;

// Thrown when blocking acquisition exceeds the retry ceiling. The lockstep loop catches it,
// logs it, and shuts the node down -- it must NEVER step the sim past incomplete data.
struct AcquisitionTimeout : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// --- helpers for the ported moveOnPath control law (all in native NED) ---
static inline double dist2D(double ax, double ay, double bx, double by)
{
    const double dx = ax - bx, dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}
static inline double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static inline double yawNEDFromQuat(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// All per-vehicle state. Heap-allocated (held by unique_ptr) so the std::atomic control
// fields are never copied/moved.
struct Vehicle
{
    std::string name;
    std::string odom_frame_id, lidar_frame_id, camera_body_frame_id, camera_optical_frame_id;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr seg_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scene_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr follow_path_sub;

    std::atomic<double> cmd_target_speed{0.0};   // desired speed setpoint (m/s, signed) from /cmd_vel
    std::atomic<double> cmd_steering{0.0};
    std::atomic<double> last_speed{0.0};          // car's current speed (m/s), cached each tick
    double vel_int = 0.0;                          // velocity-PID integrator (lockstep thread only)

    // moveOnPath follower state (ported from CarRpcLibClient::moveOnPath, run per lockstep tick).
    // Path is stored in GLOBAL NED (x,y) to match the native pose the control law uses.
    std::mutex follow_mtx;
    std::vector<std::pair<double, double>> follow_path;   // GLOBAL NED waypoints
    size_t follow_index = 0;                              // forward-only index (moveOnPath)
    double last_ned_x = 0.0, last_ned_y = 0.0, last_yaw_ned = 0.0;  // cached global-NED pose

    // cached LiDAR mount (queried once, ROS convention)
    bool lidar_mount_ready = false;
    double lidar_tx = 0, lidar_ty = 0, lidar_tz = 0;
    double lidar_qx = 0, lidar_qy = 0, lidar_qz = 0, lidar_qw = 1;
};

class HerculesSyncedNode : public rclcpp::Node
{
public:
    HerculesSyncedNode()
        : Node("hercules_node")
    {
        host_ip_ = declare_parameter<std::string>("host_ip", "localhost");
        host_port_ = declare_parameter<int>("host_port", 41452);
        // vehicle LIST (default two Huskies). Single-vehicle: pass ["Husky1"].
        vehicle_names_ = declare_parameter<std::vector<std::string>>(
            "vehicles", std::vector<std::string>{"Husky1", "Husky2"});
        camera_name_ = declare_parameter<std::string>("camera_name", "front_center");
        lidar_name_ = declare_parameter<std::string>("lidar_name", "LidarSensor1");
        sync_mode_ = declare_parameter<std::string>("sync_mode", "atomic");

        sync_dt_ = declare_parameter<double>("sync_dt", 0.1);
        publish_clock_ = declare_parameter<bool>("publish_clock", true);
        is_rgb_ = declare_parameter<bool>("is_vulkan", true); // Vulkan renderer -> rgb8
        fov_degrees_ = declare_parameter<double>("fov_degrees", 90.0);

        cam_x_ = declare_parameter<double>("camera_x", 0.0);
        cam_y_ = declare_parameter<double>("camera_y", -0.055);
        cam_z_ = declare_parameter<double>("camera_z", -0.35);
        cam_roll_ = declare_parameter<double>("camera_roll_deg", 0.0);
        cam_pitch_ = declare_parameter<double>("camera_pitch_deg", 0.0);
        cam_yaw_ = declare_parameter<double>("camera_yaw_deg", 0.0);
        // Drive support: apply each vehicle's /<name>/cmd_vel-derived controls before each step.
        enable_drive_ = declare_parameter<bool>("enable_drive", false);
        max_forward_speed_ = declare_parameter<double>("max_forward_speed", 2.0);
        max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.2);
        // velocity-loop gains (throttle + brake) for the slow, smooth path follower
        vel_kp_ = declare_parameter<double>("vel_kp", 1.5);
        vel_ki_ = declare_parameter<double>("vel_ki", 0.5);
        brake_kp_ = declare_parameter<double>("brake_kp", 3.0);      // over-speed brake proportionality
        drive_max_throttle_ = declare_parameter<double>("drive_max_throttle", 0.6);
        min_creep_throttle_ = declare_parameter<double>("min_creep_throttle", 0.08);  // beat static friction
        // ported moveOnPath follower gains (CarRpcLibClient::moveOnPath: Kp_steering=1, max_steer=0.5)
        follow_lookahead_ = declare_parameter<double>("follow_lookahead", 2.0);
        follow_desired_velocity_ = declare_parameter<double>("follow_desired_velocity", 0.4);
        follow_kp_steer_ = declare_parameter<double>("follow_kp_steer", 1.0);
        follow_max_steer_ = declare_parameter<double>("follow_max_steer", 0.5);
        follow_arrive_threshold_ = declare_parameter<double>("follow_arrive_threshold", 1.0);  // stop within this of goal

        include_scene_rgb_ = declare_parameter<bool>("include_scene_rgb", false);

        retry_ceiling_ = declare_parameter<int>("retry_ceiling", 2000);
        retry_timeout_s_ = declare_parameter<double>("retry_timeout_s", 10.0);
        retry_sleep_ = declare_parameter<double>("retry_sleep_s", 0.003);
        diag_interval_ = declare_parameter<int>("diag_interval_ticks", 50);

        auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto sensor = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        for (const std::string &name : vehicle_names_) {
            auto v = std::make_unique<Vehicle>();
            v->name = name;
            const std::string prefix = "/hercules_node/" + name;
            v->odom_frame_id = name + "/ground_truth/odom_local";
            v->lidar_frame_id = name + "/" + lidar_name_;
            v->camera_body_frame_id = name + "/" + camera_name_ + "_body";
            v->camera_optical_frame_id = name + "/" + camera_name_ + "_optical";

            v->odom_pub = create_publisher<nav_msgs::msg::Odometry>(prefix + "/ground_truth/odom_local", reliable);
            v->lidar_pub = create_publisher<sensor_msgs::msg::PointCloud2>(prefix + "/lidar/points/" + lidar_name_, reliable);
            v->depth_pub = create_publisher<sensor_msgs::msg::Image>(prefix + "/" + camera_name_ + "_DepthPlanar/image", sensor);
            v->depth_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(prefix + "/" + camera_name_ + "_DepthPlanar/camera_info", latched);
            v->seg_pub = create_publisher<sensor_msgs::msg::Image>(prefix + "/" + camera_name_ + "_Segmentation/image", sensor);
            if (include_scene_rgb_) {
                v->scene_pub = create_publisher<sensor_msgs::msg::Image>(prefix + "/" + camera_name_ + "_Scene/image", sensor);
            }
            vehicles_.push_back(std::move(v));
        }

        if (publish_clock_) {
            clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(rclcpp::KeepLast(1)));
        }
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        connect();
        precompute_camera_tf();

        if (enable_drive_) {
            for (auto &vp : vehicles_) {
                Vehicle *v = vp.get();
                try { client_->enableApiControl(true, v->name); } catch (...) {}
                v->cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
                    "/" + v->name + "/cmd_vel", 10,
                    [this, v](geometry_msgs::msg::Twist::SharedPtr m) {
                        // linear.x is now a SPEED SETPOINT (m/s); the velocity loop in
                        // apply_controls tracks it with throttle AND brake. angular.z is the
                        // yaw-rate command mapped to a steering fraction. NOTE the NEGATION: ROS
                        // angular.z>0 = turn LEFT (CCW), but AirSim CarControls.steering>0 = turn
                        // RIGHT. Without this the car steered away from every target (harmless when
                        // the goal was dead ahead, catastrophic for goals to the side/behind).
                        double s = -m->angular.z / std::max(1e-6, max_yaw_rate_);
                        v->cmd_target_speed.store(m->linear.x);
                        v->cmd_steering.store(std::max(-1.0, std::min(1.0, s)));
                    });
                // Committed path to FOLLOW via the ported moveOnPath law (in native NED, run per
                // lockstep tick). Poses arrive in the shared `map` frame -> convert to global NED.
                // A non-empty path takes priority over /cmd_vel (which then only does reverse/stop).
                v->follow_path_sub = create_subscription<nav_msgs::msg::Path>(
                    // LATCHED (transient-local, reliable) to match the executor: the follower must
                    // always see the LAST path -- especially the empty STOP path -- even across a
                    // discovery/matching gap. A volatile once-per-event message could be dropped and
                    // leave the car driving a stale path (the follower has no self-stop).
                    "/" + v->name + "/follow_path",
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                    [this, v](nav_msgs::msg::Path::SharedPtr m) {
                        std::vector<std::pair<double, double>> pts;
                        if (shared_init_received_) {
                            pts.reserve(m->poses.size());
                            for (const auto &ps : m->poses) {
                                // map (world-NWU, origin=first vehicle spawn) -> global NED
                                double gx = ps.pose.position.x + shared_init_px_;
                                double gy = -(shared_init_py_ + ps.pose.position.y);
                                pts.emplace_back(gx, gy);
                            }
                        }
                        std::lock_guard<std::mutex> lk(v->follow_mtx);
                        v->follow_path = std::move(pts);
                        v->follow_index = 0;
                    });
            }
            RCLCPP_INFO(get_logger(), "hercules_synced_node: drive enabled -- moveOnPath follower on "
                                      "/<vehicle>/follow_path, /cmd_vel for reverse/stop.");
        }

        {
            std::string names;
            for (const auto &vp : vehicles_) names += (names.empty() ? "" : ",") + vp->name;
            if (sync_mode_ == "lockstep") {
                RCLCPP_WARN(get_logger(),
                            "hercules_synced_node: LOCKSTEP (collector pattern), vehicles=[%s]. ONE "
                            "simContinueForTime(dt=%.4f) then BLOCK per vehicle until every image+lidar "
                            "is valid; sim time never advances past incomplete data.", names.c_str(), sync_dt_);
                client_->simPause(true);
                lockstep_thread_ = std::thread(&HerculesSyncedNode::lockstep_loop, this);
            } else {
                RCLCPP_INFO(get_logger(), "hercules_synced_node: ATOMIC mode at %.1f Hz, vehicles=[%s].",
                            1.0 / sync_dt_, names.c_str());
                timer_ = create_wall_timer(
                    std::chrono::duration<double>(sync_dt_),
                    std::bind(&HerculesSyncedNode::atomic_cycle, this));
            }
        }
    }

    ~HerculesSyncedNode() override
    {
        running_ = false;
        if (lockstep_thread_.joinable()) {
            lockstep_thread_.join();
        }
        try {
            if (client_ && enable_drive_) {
                msr::airlib::CarApiBase::CarControls stop;
                stop.throttle = 0.0;
                stop.brake = 1.0;
                for (auto &vp : vehicles_) {
                    client_->setCarControls(stop, vp->name);
                    client_->enableApiControl(false, vp->name);
                }
            }
            if (client_ && sync_mode_ == "lockstep") {
                client_->simPause(false);
            }
        } catch (...) {
        }
    }

    void apply_controls(Vehicle &v)
    {
        if (!enable_drive_) {
            return;
        }
        const double speed = v.last_speed.load();
        msr::airlib::CarApiBase::CarControls ctrl;

        // --- PATH MODE: follow the committed path via the ported moveOnPath control law, run ONCE
        // per lockstep tick in native NED (steps mirror CarRpcLibClient::moveOnPath lines 169-219:
        // forward index stepping + distance-based lookahead + pure-pursuit). Throttle uses the
        // velocity+brake loop so the brakeless car actually holds the slow setpoint. ---
        double tx = 0.0, ty = 0.0;
        bool have_path = false;
        {
            std::lock_guard<std::mutex> lk(v.follow_mtx);
            if (!v.follow_path.empty()) {
                auto &path = v.follow_path;
                const double px = v.last_ned_x, py = v.last_ned_y;
                // TERMINAL (moveOnPath lines 153-157): reached the path end -> stop and CLEAR the
                // path so the car stays stopped. Safety net if the executor is late/dead -- without
                // it the follower (which has no self-stop) would orbit the last waypoint forever.
                if (dist2D(px, py, path.back().first, path.back().second) < follow_arrive_threshold_) {
                    path.clear();
                    v.follow_index = 0;
                    // have_path stays false -> falls through to velocity-mode STOP (brake) below.
                }
            }
            if (!v.follow_path.empty()) {
                have_path = true;
                const auto &path = v.follow_path;
                const double px = v.last_ned_x, py = v.last_ned_y;
                // forward-only index stepping: advance while closer to the NEXT waypoint
                while (v.follow_index + 1 < path.size()) {
                    const double d_cur = dist2D(px, py, path[v.follow_index].first, path[v.follow_index].second);
                    const double d_next = dist2D(px, py, path[v.follow_index + 1].first, path[v.follow_index + 1].second);
                    if (d_next < d_cur) v.follow_index++;
                    else break;
                }
                // distance-based lookahead: first waypoint at least follow_lookahead_ away
                size_t tgt = path.size() - 1;
                for (size_t i = v.follow_index + 1; i < path.size(); ++i) {
                    if (dist2D(px, py, path[i].first, path[i].second) >= follow_lookahead_) { tgt = i; break; }
                }
                tx = path[tgt].first; ty = path[tgt].second;
            }
        }
        if (have_path) {
            const double desired_heading = std::atan2(ty - v.last_ned_y, tx - v.last_ned_x);
            ctrl.steering = std::clamp(follow_kp_steer_ * normalizeAngle(desired_heading - v.last_yaw_ned),
                                       -follow_max_steer_, follow_max_steer_);
            const double err = follow_desired_velocity_ - speed;
            if (err > 0.0) {
                v.vel_int = std::max(-1.0, std::min(1.0, v.vel_int + err * sync_dt_));
                double thr = vel_kp_ * err + vel_ki_ * v.vel_int;
                ctrl.throttle = std::max(min_creep_throttle_, std::min(drive_max_throttle_, thr));
                ctrl.brake = 0.0;
            } else {
                ctrl.throttle = 0.0; ctrl.brake = std::min(1.0, brake_kp_ * (-err)); v.vel_int = 0.0;
            }
            try { client_->setCarControls(ctrl, v.name); } catch (...) {}
            return;
        }

        // --- VELOCITY MODE (no path): /cmd_vel setpoint for reverse (un-stick) or stop, with brake. ---
        ctrl.steering = std::max(-1.0, std::min(1.0, v.cmd_steering.load()));
        const double target = v.cmd_target_speed.load();   // desired m/s (signed)
        if (target < -0.02) {                               // reverse
            ctrl.is_manual_gear = true; ctrl.manual_gear = -1;
            ctrl.throttle = std::min(0.5, -target / std::max(1e-6, max_forward_speed_));
            ctrl.brake = 0.0; v.vel_int = 0.0;
        } else {                                            // stop -> brake to a full halt
            ctrl.throttle = 0.0;
            ctrl.brake = (speed > 0.03) ? 1.0 : 0.0;
            v.vel_int = 0.0;
        }
        try { client_->setCarControls(ctrl, v.name); } catch (...) {}
    }

private:
    void connect()
    {
        client_ = std::make_unique<msr::airlib::CarRpcLibClient>(host_ip_, host_port_);
        try {
            client_->confirmConnection();
            RCLCPP_INFO(get_logger(), "hercules_synced_node connected to AirSim at %s:%d.",
                        host_ip_.c_str(), host_port_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "hercules_synced_node failed to connect: %s", e.what());
        }
    }

    // ---- request vector: [Scene?, DepthPlanar(float), Segmentation], one render frame ----
    std::vector<ImageRequest> image_requests() const
    {
        std::vector<ImageRequest> reqs;
        if (include_scene_rgb_) {
            reqs.emplace_back(camera_name_, ImageType::Scene, false, false);
        }
        reqs.emplace_back(camera_name_, ImageType::DepthPlanar, true, false); // pixels_as_float
        reqs.emplace_back(camera_name_, ImageType::Segmentation, false, false);
        return reqs;
    }

    static bool image_valid(const ImageResponse &r)
    {
        if (r.width == 0 || r.height == 0) {
            return false;
        }
        return r.pixels_as_float ? !r.image_data_float.empty() : !r.image_data_uint8.empty();
    }

    // Returns the name of the first empty image in the response set, "" if all valid.
    std::string first_missing_image(const std::vector<ImageResponse> &responses, size_t expected) const
    {
        if (responses.size() != expected) {
            return "response-count-mismatch";
        }
        size_t idx = 0;
        if (include_scene_rgb_) {
            if (!image_valid(responses[idx])) return "Scene";
            idx++;
        }
        if (!image_valid(responses[idx])) return "DepthPlanar";
        idx++;
        if (!image_valid(responses[idx])) return "Segmentation";
        return "";
    }

    // Collector-style get_nonempty_images: ONE request vector, re-request WITHOUT stepping the
    // sim until every image is non-empty. Rendering continues while paused, so this converges.
    std::vector<ImageResponse> acquire_images_blocking(Vehicle &v, int &out_retries)
    {
        const auto reqs = image_requests();
        const auto t0 = std::chrono::steady_clock::now();
        for (int attempt = 0; running_ && rclcpp::ok(); ++attempt) {
            std::vector<ImageResponse> responses = client_->simGetImages(reqs, v.name);
            const std::string missing = first_missing_image(responses, reqs.size());
            if (missing.empty()) {
                out_retries = attempt;
                return responses;
            }
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (attempt + 1 >= retry_ceiling_ || elapsed >= retry_timeout_s_) {
                throw AcquisitionTimeout(
                    v.name + " image type '" + missing + "' still empty after " + std::to_string(attempt + 1)
                    + " retries / " + std::to_string(elapsed) + " s -- render never produced a valid "
                    "frame. Aborting rather than stepping the sim past incomplete data.");
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_sleep_));
        }
        throw AcquisitionTimeout("image acquisition interrupted before a valid frame arrived.");
    }

    // Collector-style get_nonempty_lidar: re-request WITHOUT stepping until non-empty.
    msr::airlib::LidarData acquire_lidar_blocking(Vehicle &v, int &out_retries)
    {
        const auto t0 = std::chrono::steady_clock::now();
        for (int attempt = 0; running_ && rclcpp::ok(); ++attempt) {
            msr::airlib::LidarData ld = client_->getLidarData(lidar_name_, v.name);
            if (ld.point_cloud.size() >= 3) {
                out_retries = attempt;
                return ld;
            }
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (attempt + 1 >= retry_ceiling_ || elapsed >= retry_timeout_s_) {
                throw AcquisitionTimeout(
                    v.name + " LiDAR '" + lidar_name_ + "' empty after " + std::to_string(attempt + 1)
                    + " retries / " + std::to_string(elapsed) + " s. Aborting rather than stepping "
                    "the sim past incomplete data.");
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_sleep_));
        }
        throw AcquisitionTimeout("lidar acquisition interrupted before a valid sweep arrived.");
    }

    // ---- lockstep: ONE step advances the whole sim; then read+publish EVERY vehicle ----
    void lockstep_loop()
    {
        uint64_t step = 0;
        while (running_ && rclcpp::ok()) {
            const auto tick_t0 = std::chrono::steady_clock::now();
            try {
                for (auto &vp : vehicles_) apply_controls(*vp);
                client_->simContinueForTime(sync_dt_); // advance SIM by dt, then frozen
                step++;
                const int64_t t_ns = static_cast<int64_t>(static_cast<double>(step) * sync_dt_ * 1e9);
                const rclcpp::Time stamp(t_ns);

                if (publish_clock_) {
                    rosgraph_msgs::msg::Clock clk;
                    clk.clock = stamp;
                    clock_pub_->publish(clk);
                }

                int tick_retries = 0;
                for (auto &vp : vehicles_) {
                    Vehicle &v = *vp;
                    const auto veh_t0 = std::chrono::steady_clock::now();
                    int img_retries = 0;
                    std::vector<ImageResponse> images = acquire_images_blocking(v, img_retries);
                    int lidar_retries = 0;
                    msr::airlib::LidarData lidar = acquire_lidar_blocking(v, lidar_retries);
                    const auto car_state = client_->getCarState(v.name);
                    tick_retries += img_retries + lidar_retries;

                    publish_odom_and_tf(v, car_state, stamp);
                    publish_images(v, images, stamp);
                    publish_lidar(v, lidar, stamp);

                    const double veh_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - veh_t0).count();
                    RCLCPP_DEBUG(get_logger(), "  [%s] read+publish %.0f ms (retries %d)",
                                 v.name.c_str(), veh_ms, img_retries + lidar_retries);
                }

                const double tick_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - tick_t0).count();
                record_tick_diag(step, tick_retries, tick_ms);
            } catch (const AcquisitionTimeout &e) {
                RCLCPP_FATAL(get_logger(),
                             "hercules_synced_node: %s Shutting down.", e.what());
                rclcpp::shutdown();
                return;
            } catch (rpc::rpc_error &e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RPC error: %s",
                                      e.get_error().as<std::string>().c_str());
            }
        }
    }

    void record_tick_diag(uint64_t step, int retries, double tick_ms)
    {
        tick_ms_window_.push_back(tick_ms);
        total_retries_ += retries;
        if (static_cast<int>(tick_ms_window_.size()) < std::max(1, diag_interval_)) {
            return;
        }
        std::vector<double> w = tick_ms_window_;
        std::sort(w.begin(), w.end());
        const double med = w[w.size() / 2];
        const double p95 = w[std::min(w.size() - 1, static_cast<size_t>(w.size() * 0.95))];
        double wall_ms = 0.0;
        for (double v : w) wall_ms += v;
        const double sim_ms = static_cast<double>(w.size()) * sync_dt_ * 1000.0;
        const double rtf = wall_ms > 0.0 ? sim_ms / wall_ms : 0.0;
        RCLCPP_INFO(get_logger(),
                    "lockstep tick %lu | %zu vehicles | wall/tick median %.0f ms p95 %.0f ms | %d retries "
                    "over %d ticks | real-time factor %.2fx",
                    static_cast<unsigned long>(step), vehicles_.size(), med, p95, total_retries_,
                    static_cast<int>(w.size()), rtf);
        tick_ms_window_.clear();
        total_retries_ = 0;
    }

    // ---- atomic: legacy async one best-effort cycle per wall-timer tick (all vehicles) ----
    void atomic_cycle()
    {
        try {
            for (auto &vp : vehicles_) apply_controls(*vp);
            rclcpp::Time stamp;
            bool stamped = false;
            for (auto &vp : vehicles_) {
                Vehicle &v = *vp;
                const auto car_state = client_->getCarState(v.name);
                if (!stamped) {
                    stamp = rclcpp::Time(static_cast<int64_t>(car_state.timestamp));
                    stamped = true;
                    if (publish_clock_) {
                        rosgraph_msgs::msg::Clock clk;
                        clk.clock = stamp;
                        clock_pub_->publish(clk);
                    }
                }
                std::vector<ImageResponse> images = client_->simGetImages(image_requests(), v.name);
                msr::airlib::LidarData lidar;
                try { lidar = client_->getLidarData(lidar_name_, v.name); } catch (...) {}
                publish_odom_and_tf(v, car_state, stamp);
                publish_images(v, images, stamp);
                publish_lidar(v, lidar, stamp);
            }
        } catch (rpc::rpc_error &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RPC error: %s",
                                  e.get_error().as<std::string>().c_str());
        }
    }

    void publish_odom_and_tf(Vehicle &v, const msr::airlib::CarApiBase::CarState &car_state,
                             const rclcpp::Time &stamp)
    {
        const auto &k = car_state.kinematics_estimated;
        v.last_speed.store(std::abs(car_state.speed));   // feed the velocity controller (next tick)
        // getCarState/kinematics POSITION is relative to THIS vehicle's OWN spawn point, so
        // every vehicle collapses onto the same map origin -- the per-vehicle spawn offsets
        // (settings.json X/Y/Z) are lost and the maps overlap/misalign. simGetObjectPose
        // returns the GLOBAL (world) position, so all vehicles share ONE true world frame and
        // sit at their correct relative offsets. Orientation is already global in kinematics,
        // so only position is swapped here; twist stays from kinematics. Fall back to the
        // kinematics position if the object-pose query ever fails.
        auto gpos = k.pose.position;
        try {
            auto op = client_->simGetObjectPose(v.name);
            if (!std::isnan(op.position.x())) gpos = op.position;
        } catch (...) {}
        // Cache the GLOBAL NED pose (raw, un-negated) for the moveOnPath follower next tick.
        v.last_ned_x = gpos.x();
        v.last_ned_y = gpos.y();
        v.last_yaw_ned = yawNEDFromQuat(k.pose.orientation.x(), k.pose.orientation.y(),
                                        k.pose.orientation.z(), k.pose.orientation.w());
        // AirSim NED -> ROS NWU (negate y, z), matching hercules_ros_wrapper.
        double px = gpos.x();
        double py = -gpos.y();
        double pz = -gpos.z();
        double qx = k.pose.orientation.x();
        double qy = -k.pose.orientation.y();
        double qz = -k.pose.orientation.z();
        double qw = k.pose.orientation.w();

        // Poses are relative to a SINGLE SHARED init reference (first vehicle's first pose), so
        // all robots share ONE map frame at correct relative positions (not each at origin).
        if (!shared_init_received_) {
            shared_init_px_ = px; shared_init_py_ = py; shared_init_pz_ = pz;
            shared_init_q_ = tf2::Quaternion(qx, qy, qz, qw);
            shared_init_received_ = true;
        }
        px -= shared_init_px_; py -= shared_init_py_; pz -= shared_init_pz_;
        tf2::Quaternion q_rel = shared_init_q_.inverse() * tf2::Quaternion(qx, qy, qz, qw);
        q_rel.normalize();
        qx = q_rel.x(); qy = q_rel.y(); qz = q_rel.z(); qw = q_rel.w();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = v.name;
        odom.child_frame_id = v.odom_frame_id;
        odom.pose.pose.position.x = px;
        odom.pose.pose.position.y = py;
        odom.pose.pose.position.z = pz;
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;
        odom.twist.twist.linear.x = k.twist.linear.x();
        odom.twist.twist.linear.y = -k.twist.linear.y();
        odom.twist.twist.linear.z = -k.twist.linear.z();
        odom.twist.twist.angular.x = k.twist.angular.x();
        odom.twist.twist.angular.y = -k.twist.angular.y();
        odom.twist.twist.angular.z = -k.twist.angular.z();
        v.odom_pub->publish(odom);

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = stamp;
        odom_tf.header.frame_id = v.name;
        odom_tf.child_frame_id = v.odom_frame_id;
        odom_tf.transform.translation.x = px;
        odom_tf.transform.translation.y = py;
        odom_tf.transform.translation.z = pz;
        odom_tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_tf);

        publish_sensor_mount_tfs(v, stamp);
    }

    void publish_sensor_mount_tfs(Vehicle &v, const rclcpp::Time &stamp)
    {
        if (!v.lidar_mount_ready) {
            try {
                const auto ld = client_->getLidarData(lidar_name_, v.name);
                v.lidar_tx = ld.pose.position.x();
                v.lidar_ty = -ld.pose.position.y();
                v.lidar_tz = -ld.pose.position.z();
                tf2::Quaternion lq(ld.pose.orientation.x(), -ld.pose.orientation.y(),
                                   -ld.pose.orientation.z(), ld.pose.orientation.w());
                v.lidar_qx = lq.x(); v.lidar_qy = lq.y(); v.lidar_qz = lq.z(); v.lidar_qw = lq.w();
                v.lidar_mount_ready = true;
            } catch (...) {
                return;
            }
        }
        std::vector<geometry_msgs::msg::TransformStamped> tfs;
        tfs.push_back(make_tf(stamp, v.odom_frame_id, v.lidar_frame_id,
                              v.lidar_tx, v.lidar_ty, v.lidar_tz,
                              v.lidar_qx, v.lidar_qy, v.lidar_qz, v.lidar_qw));
        tfs.push_back(make_tf(stamp, v.odom_frame_id, v.camera_body_frame_id,
                              cam_body_tx_, cam_body_ty_, cam_body_tz_,
                              cam_body_qx_, cam_body_qy_, cam_body_qz_, cam_body_qw_));
        tfs.push_back(make_tf(stamp, v.camera_body_frame_id, v.camera_optical_frame_id,
                              0.0, 0.0, 0.0,
                              cam_opt_qx_, cam_opt_qy_, cam_opt_qz_, cam_opt_qw_));
        tf_broadcaster_->sendTransform(tfs);
    }

    static geometry_msgs::msg::TransformStamped make_tf(
        const rclcpp::Time &stamp, const std::string &parent, const std::string &child,
        double tx, double ty, double tz, double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = tx;
        tf.transform.translation.y = ty;
        tf.transform.translation.z = tz;
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;
        return tf;
    }

    void precompute_camera_tf()
    {
        // body frame: settings pose (NED) -> ROS (negate y,z of translation + rotation).
        tf2::Quaternion q_body;
        q_body.setRPY(cam_roll_ * M_PI / 180.0, cam_pitch_ * M_PI / 180.0, cam_yaw_ * M_PI / 180.0);
        q_body.normalize();
        cam_body_tx_ = cam_x_;
        cam_body_ty_ = -cam_y_;
        cam_body_tz_ = -cam_z_;
        cam_body_qx_ = q_body.x();
        cam_body_qy_ = -q_body.y();
        cam_body_qz_ = -q_body.z();
        cam_body_qw_ = q_body.w();
        // optical frame: body_ros * (0.5,-0.5,0.5,-0.5).
        tf2::Quaternion q_body_ros(cam_body_qx_, cam_body_qy_, cam_body_qz_, cam_body_qw_);
        tf2::Quaternion q_b2o(0.5, -0.5, 0.5, -0.5);
        q_b2o.normalize();
        tf2::Quaternion q_opt = q_body_ros * q_b2o;
        q_opt.normalize();
        cam_opt_qx_ = q_opt.x(); cam_opt_qy_ = q_opt.y(); cam_opt_qz_ = q_opt.z(); cam_opt_qw_ = q_opt.w();
    }

    // Publish images from a pre-acquired response vector (order: [Scene?, Depth, Seg]).
    void publish_images(Vehicle &v, const std::vector<ImageResponse> &responses, const rclcpp::Time &stamp)
    {
        size_t idx = 0;
        if (include_scene_rgb_) {
            if (idx < responses.size()) {
                publish_scene(v, responses[idx], stamp);
            }
            idx++;
        }
        if (idx < responses.size()) {
            publish_depth(v, responses[idx], stamp);
        }
        idx++;
        if (idx < responses.size()) {
            publish_seg(v, responses[idx], stamp);
        }
    }

    void publish_depth(Vehicle &v, const ImageResponse &r, const rclcpp::Time &stamp)
    {
        if (r.width == 0 || r.height == 0 || r.image_data_float.empty()) {
            return;
        }
        sensor_msgs::msg::Image depth;
        depth.header.stamp = stamp;
        depth.header.frame_id = v.camera_optical_frame_id;
        depth.height = r.height;
        depth.width = r.width;
        depth.encoding = "32FC1";
        depth.is_bigendian = 0;
        depth.step = r.width * sizeof(float);
        depth.data.resize(r.image_data_float.size() * sizeof(float));
        std::memcpy(depth.data.data(), r.image_data_float.data(),
                    r.image_data_float.size() * sizeof(float));
        v.depth_pub->publish(depth);
        publish_camera_info(v, stamp, r.width, r.height);
    }

    void publish_seg(Vehicle &v, const ImageResponse &r, const rclcpp::Time &stamp)
    {
        if (r.width == 0 || r.height == 0 || r.image_data_uint8.empty()) {
            return;
        }
        sensor_msgs::msg::Image seg;
        seg.header.stamp = stamp;
        seg.header.frame_id = v.camera_optical_frame_id;
        seg.height = r.height;
        seg.width = r.width;
        seg.encoding = is_rgb_ ? "rgb8" : "bgr8";
        seg.is_bigendian = 0;
        seg.step = r.width * 3;
        seg.data = r.image_data_uint8;
        v.seg_pub->publish(seg);
    }

    void publish_scene(Vehicle &v, const ImageResponse &r, const rclcpp::Time &stamp)
    {
        if (!v.scene_pub || r.width == 0 || r.height == 0 || r.image_data_uint8.empty()) {
            return;
        }
        sensor_msgs::msg::Image rgb;
        rgb.header.stamp = stamp;
        rgb.header.frame_id = v.camera_optical_frame_id;
        rgb.height = r.height;
        rgb.width = r.width;
        rgb.encoding = is_rgb_ ? "rgb8" : "bgr8";
        rgb.is_bigendian = 0;
        rgb.step = r.width * 3;
        rgb.data = r.image_data_uint8;
        v.scene_pub->publish(rgb);
    }

    void publish_camera_info(Vehicle &v, const rclcpp::Time &stamp, int width, int height)
    {
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = stamp;
        info.header.frame_id = v.camera_optical_frame_id;
        info.width = width;
        info.height = height;
        const double fx = (width / 2.0) / std::tan(fov_degrees_ * M_PI / 180.0 / 2.0);
        info.k = {fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0};
        info.p = {fx, 0.0, width / 2.0, 0.0, 0.0, fx, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
        info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        info.distortion_model = "plumb_bob";
        v.depth_info_pub->publish(info);
    }

    void publish_lidar(Vehicle &v, const msr::airlib::LidarData &ld, const rclcpp::Time &stamp)
    {
        if (ld.point_cloud.size() < 3) {
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = v.lidar_frame_id;
        const uint32_t n = ld.point_cloud.size() / 3;
        cloud.height = 1;
        cloud.width = n;
        cloud.fields.resize(3);
        const char *names[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i) {
            cloud.fields[i].name = names[i];
            cloud.fields[i].offset = i * 4;
            cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[i].count = 1;
        }
        cloud.is_bigendian = false;
        cloud.point_step = 12;
        cloud.row_step = cloud.point_step * n;
        cloud.is_dense = true;
        cloud.data.resize(static_cast<size_t>(n) * 12);
        float *out = reinterpret_cast<float *>(cloud.data.data());
        for (uint32_t i = 0; i < n; ++i) {
            // NED -> NWU: negate y and z, matching fixPointCloud in the async wrapper.
            out[i * 3 + 0] = ld.point_cloud[i * 3 + 0];
            out[i * 3 + 1] = -ld.point_cloud[i * 3 + 1];
            out[i * 3 + 2] = -ld.point_cloud[i * 3 + 2];
        }
        v.lidar_pub->publish(cloud);
    }

    // params
    std::string host_ip_, camera_name_, lidar_name_, sync_mode_;
    std::vector<std::string> vehicle_names_;
    int host_port_;
    double sync_dt_, fov_degrees_;
    bool publish_clock_, is_rgb_;
    double cam_x_, cam_y_, cam_z_, cam_roll_, cam_pitch_, cam_yaw_;
    bool enable_drive_ = false;
    double max_forward_speed_ = 2.0, max_yaw_rate_ = 1.2;
    double vel_kp_ = 1.5, vel_ki_ = 0.5, brake_kp_ = 3.0, drive_max_throttle_ = 0.6, min_creep_throttle_ = 0.08;
    double follow_lookahead_ = 2.0, follow_desired_velocity_ = 0.4, follow_kp_steer_ = 1.0, follow_max_steer_ = 0.5;
    double follow_arrive_threshold_ = 1.0;
    bool include_scene_rgb_ = false;
    int retry_ceiling_ = 2000;
    double retry_timeout_s_ = 10.0;
    double retry_sleep_ = 0.003;
    int diag_interval_ = 50;

    // diagnostics (not a control mechanism)
    std::vector<double> tick_ms_window_;
    int total_retries_ = 0;

    // shared camera mount transform (identical mount on both vehicles)
    double cam_body_tx_ = 0, cam_body_ty_ = 0, cam_body_tz_ = 0;
    double cam_body_qx_ = 0, cam_body_qy_ = 0, cam_body_qz_ = 0, cam_body_qw_ = 1;
    double cam_opt_qx_ = 0, cam_opt_qy_ = 0, cam_opt_qz_ = 0, cam_opt_qw_ = 1;

    // SHARED odom relative-to-init reference (so all robots share one map frame)
    // atomic: written on the lockstep thread (first tick), read on the ROS thread in the
    // follow_path callback -- the release/acquire ensures shared_init_px_/py_ are visible before
    // the callback converts a path with them (they are set once, before received_ becomes true).
    std::atomic<bool> shared_init_received_{false};
    double shared_init_px_ = 0, shared_init_py_ = 0, shared_init_pz_ = 0;
    tf2::Quaternion shared_init_q_{0, 0, 0, 1};

    std::unique_ptr<msr::airlib::CarRpcLibClient> client_;
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread lockstep_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HerculesSyncedNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
