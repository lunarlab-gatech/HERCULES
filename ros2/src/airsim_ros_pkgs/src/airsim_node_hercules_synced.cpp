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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
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

    // FALL-THROUGH RESCUE state: last pose observed while clearly grounded (see lockstep_loop)
    msr::airlib::Pose last_good_pose;
    bool has_good_pose = false;

    // STUCK-NUDGE state: rolling displacement anchor (see lockstep_loop)
    float stuck_ax = 0.f, stuck_ay = 0.f;
    double stuck_at = -1.0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr seg_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scene_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr follow_path_sub;

    std::atomic<double> cmd_target_speed{0.0};   // desired speed setpoint (m/s, signed) from /cmd_vel
    std::atomic<double> terrain_max_speed{1e9};  // per-class cap from /<vehicle>/max_speed
    // PROXIMITY GOVERNOR state (2026-10-12): speed cap from the nearest other vehicle ahead, computed
    // once per tick from the cached global poses (see proximity_governor); 1e9 = no vehicle near.
    double peer_max_speed = 1e9;
    int peer_idx = -1;                 // index of the vehicle being yielded to (-1 none)
    double yield_since = -1.0;         // sim time the current yield began (-1 not yielding)
    double yield_total_s = 0.0;        // diagnostics: cumulative yield time this run
    int yield_events = 0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr max_speed_sub;
    std::atomic<double> cmd_steering{0.0};
    std::atomic<double> last_speed{0.0};          // car's current speed (m/s), cached each tick
    double vel_int = 0.0;                          // velocity-PID integrator (lockstep thread only)

    // Path follower state, run per lockstep tick in GLOBAL NED (x,y).
    std::mutex follow_mtx;
    std::vector<std::pair<double, double>> follow_path;   // GLOBAL NED waypoints
    size_t follow_index = 0;                              // legacy (kept for the stuck-nudge check)
    double last_ned_x = 0.0, last_ned_y = 0.0, last_yaw_ned = 0.0;  // cached global-NED pose
    // --- tube follower (2026-09-27) ---
    // fmode: 0 TRACK (pure pursuit on the projection), 1 ALIGN_FWD, 2 ALIGN_REV, 3 ALIGN_BRAKE.
    // The ALIGN states are the K-turn: alternating forward-full-lock / reverse-full-lock segments
    // that rotate a car-like robot essentially in place (measured 2026-09-27 on the UGVPawn:
    // 176 deg in 6 segments, 0.14 m net displacement, <=0.55 m excursion).
    int fmode = 0;
    size_t proj_seg = 0;                                  // polyline segment of the current projection
    double seg_moved = 0.0, seg_last_x = 0.0, seg_last_y = 0.0;
    int align_segments = 0;                               // segments in the current align episode
    int align_steer_sign = 0;                             // latched per segment
    double align_target_yaw = 0.0;                        // FROZEN at align entry (see follow_step)
    double xte = 0.0, alpha = 0.0;                        // diagnostics: cross-track error, heading error
    double path_max_xte = 0.0, path_sum_xte = 0.0; int path_n = 0; int path_align_total = 0;
    int diag_tick = 0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub;

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
        // (retuned 2026-09-27 for the PhysXCar/UGVPawn engine, see speed_loop: small gains, no
        // standing floor, coast band before braking)
        vel_kp_ = declare_parameter<double>("vel_kp", 0.25);
        vel_ki_ = declare_parameter<double>("vel_ki", 0.05);
        vel_int_max_ = declare_parameter<double>("vel_int_max", 2.0);     // ki * this = max integrator throttle (full lock needs ~0.1-0.2)
        brake_kp_ = declare_parameter<double>("brake_kp", 0.5);      // over-speed brake proportionality (beyond the deadband); harness 2026-09-27: kp 0.25 / brake 0.5 holds 0.40 m/s flat (std 0.08)
        brake_deadband_ = declare_parameter<double>("brake_deadband", 0.05);  // m/s over target before the brake acts
        // FALL-THROUGH RESCUE: a hard prop collision can punch a vehicle through the terrain's
        // one-sided collision mesh (unbounded depenetration impulse); it then free-falls under
        // the world (and the dangling pawn used to crash the editor). NED z grows DOWNWARD:
        // z above fall_rescue_z means the vehicle is far below any drivable ground -> teleport
        // it back to its last grounded pose and stop it for a tick.
        fall_rescue_ = declare_parameter<bool>("fall_rescue", true);
        fall_rescue_z_ = declare_parameter<double>("fall_rescue_z", 10.0);
        // STUCK-NUDGE (last resort under the executor's stall-reverse): a vehicle with an
        // ACTIVE follow path that moves < stuck_min_motion over stuck_window_s of sim time is
        // physically wedged on something the map calls drivable (seen live: a physics-solid
        // grass tree pinned a Husky for hours). Teleport it nudge_back_m straight backward.
        stuck_nudge_ = declare_parameter<bool>("stuck_nudge", true);
        stuck_window_s_ = declare_parameter<double>("stuck_window_s", 60.0);
        stuck_min_motion_ = declare_parameter<double>("stuck_min_motion", 0.3);
        nudge_back_m_ = declare_parameter<double>("nudge_back_m", 1.5);
        drive_max_throttle_ = declare_parameter<double>("drive_max_throttle", 0.6);
        min_creep_throttle_ = declare_parameter<double>("min_creep_throttle", 0.03);  // beat static friction (applied only while starting from rest)
        // follower cruise speed and arrival radius (legacy names kept for the launch files)
        follow_lookahead_ = declare_parameter<double>("follow_lookahead", 2.0);     // unused by the tube follower
        follow_desired_velocity_ = declare_parameter<double>("follow_desired_velocity", 0.4);
        follow_kp_steer_ = declare_parameter<double>("follow_kp_steer", 1.0);         // unused by the tube follower
        follow_max_steer_ = declare_parameter<double>("follow_max_steer", 0.5);       // unused by the tube follower
        follow_arrive_threshold_ = declare_parameter<double>("follow_arrive_threshold", 1.0);  // stop within this of goal
        // --- TUBE FOLLOWER (2026-09-27). Why: the moveOnPath port capped steering at half lock
        // (R ~ 2.8 m measured) and held cruise speed regardless of heading error, so a path that
        // starts 100-150 deg off the robot's heading was executed as a 4-7 m arc -- next to a 2 m
        // river channel that arc IS the river (pilot 2: planned paths crossed 5 m of water, driven
        // paths 20 m). The tube follower (a) projects the robot onto the path and pursues a short
        // lookahead with FULL steering authority (R ~ 0.85 m at full lock), (b) slows for heading
        // error, cross-track error and upcoming corners, and (c) re-orients IN PLACE with a K-turn
        // (alternating forward/reverse full-lock segments) whenever the heading error exceeds
        // follow_align_enter_deg, instead of arcing. The planner's kinematics are otherwise
        // unchanged: the executed trajectory now stays within a fraction of a metre of the plan.
        follow_align_enter_deg_ = declare_parameter<double>("follow_align_enter_deg", 75.0);   // 90 deg grid corners: full-lock pursuit at creep; K-turn only beyond
        follow_align_exit_deg_ = declare_parameter<double>("follow_align_exit_deg", 12.0);
        follow_align_speed_ = declare_parameter<double>("follow_align_speed", 0.3);          // legacy (loop target); unused when align_throttle > 0
        // K-turn segments run at CONSTANT throttle: this Chaos car barely turns below ~0.2 m/s (live
        // 2026-09-27: full lock at ~0.1 m/s gave R ~ 2-3 m vs 0.85 m at 0.75 m/s), so the slow
        // speed-loop segments turned only 4-14 deg each; the probe's constant-throttle segments
        // turned ~30 deg. A speed guard brakes the segment if it runs away.
        follow_align_throttle_ = declare_parameter<double>("follow_align_throttle", 0.05);
        follow_align_vmax_ = declare_parameter<double>("follow_align_vmax", 1.0);   // legs brake above this (0.9 m/s legs overshot and drifted 20 deg while braking)
        follow_align_tangent_xte_ = declare_parameter<double>("follow_align_tangent_xte", 0.0);  // within this of the path: align to the path direction, not the bearing
        follow_track_min_speed_ = declare_parameter<double>("follow_track_min_speed", 0.3);     // never creep below the good-turning regime
        follow_softstart_v_ = declare_parameter<double>("follow_softstart_v", 0.15);            // below this speed: throttle capped, integrator frozen
        follow_softstart_throttle_ = declare_parameter<double>("follow_softstart_throttle", 0.08);
        follow_softstart_steer_ = declare_parameter<double>("follow_softstart_steer", 0.6);        // max |steer| while starting from rest
        follow_align_seg_m_ = declare_parameter<double>("follow_align_seg_m", 0.35);
        follow_align_max_segments_ = declare_parameter<int>("follow_align_max_segments", 12);
        follow_align_reverse_first_ = declare_parameter<bool>("follow_align_reverse_first", false);  // v4 K-turn is the validated default (2026-09-27 harness: reverse-first + asymmetric legs + 0.5 m/s leg cap crawled/jammed, 2.5x path); the options stay for later work
        follow_align_seg_fwd_m_ = declare_parameter<double>("follow_align_seg_fwd_m", 0.0);   // asymmetric legs: footprint drifts backward
        follow_align_seg_rev_m_ = declare_parameter<double>("follow_align_seg_rev_m", 0.0);   // (<= 0 -> follow_align_seg_m for both)
        follow_lookahead_min_ = declare_parameter<double>("follow_lookahead_min", 0.5);
        follow_lookahead_max_ = declare_parameter<double>("follow_lookahead_max", 1.2);
        follow_lookahead_gain_ = declare_parameter<double>("follow_lookahead_gain", 1.2);   // + gain * v
        follow_wheelbase_ = declare_parameter<double>("follow_wheelbase", 1.0);            // R_fulllock*tan(50deg) = 0.85*1.19
        follow_max_steer_deg_ = declare_parameter<double>("follow_max_steer_deg", 50.0);    // UGVWheelFront max_steer_angle
        follow_xte_slow_ = declare_parameter<double>("follow_xte_slow", 0.3);              // above this: half speed
        follow_corner_slow_deg_ = declare_parameter<double>("follow_corner_slow_deg", 50.0); // turn within lookahead -> 0.35 speed
        follow_diag_every_ = declare_parameter<int>("follow_diag_every", 5);                 // ticks between /follow_diag msgs
        // PROXIMITY GOVERNOR (2026-10-12): the UGVPawn mesh has no collision with other vehicles
        // (PHYSICS_ONLY, ignores the Vehicle/Pawn channels), so robots drive through each other.
        // Pure speed governor on the GROUND-TRUTH poses this node already caches each tick -- no
        // map, planner or coordinator change: for each vehicle, the nearest OTHER vehicle inside
        // the forward cone (|bearing| < peer_cone_deg) and closer than peer_slow_m scales the
        // follower's speed cap linearly to 0 at peer_stop_m. Head-on: the vehicle listed FIRST
        // in `vehicles` keeps driving (fixed priority), the other yields; after peer_yield_timeout_s
        // of continuous yielding the yielder takes priority so two vehicles never wait forever.
        // Behind a vehicle that is itself stopped the same timeout applies. Off by default: the
        // 54 results runs were recorded without it; any run with it on is a separate population.
        peer_avoid_ = declare_parameter<bool>("peer_avoid", false);
        peer_slow_m_ = declare_parameter<double>("peer_slow_m", 4.0);
        peer_stop_m_ = declare_parameter<double>("peer_stop_m", 1.5);
        peer_cone_deg_ = declare_parameter<double>("peer_cone_deg", 70.0);
        peer_yield_timeout_s_ = declare_parameter<double>("peer_yield_timeout_s", 8.0);
        peer_lane_half_m_ = declare_parameter<double>("peer_lane_half_m", 1.2);   // |cross-track| below this = in my path (Husky ~1 m wide)

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
                // terrain governor: per-class achievable-speed cap (physical referent of
                // the semantic cost vector); the follower clamps its cruise setpoint to it
                v->max_speed_sub = create_subscription<std_msgs::msg::Float64>(
                    "/" + v->name + "/max_speed", 10,
                    [v](std_msgs::msg::Float64::SharedPtr m) {
                        v->terrain_max_speed.store(std::max(0.05, m->data));
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
                        if (!v->follow_path.empty() && v->path_n > 0) {
                            RCLCPP_INFO(get_logger(), "[%s] path replaced: max_xte=%.2f mean_xte=%.2f align_segments=%d (%d ticks)",
                                        v->name.c_str(), v->path_max_xte, v->path_sum_xte / v->path_n,
                                        v->path_align_total, v->path_n);
                        }
                        v->follow_path = std::move(pts);
                        v->follow_index = 0;
                        v->proj_seg = 0; v->fmode = 0; v->align_segments = 0; v->align_steer_sign = 0;
                        v->seg_moved = 0.0; v->path_max_xte = 0.0; v->path_sum_xte = 0.0; v->path_n = 0;
                        v->path_align_total = 0;
                    });
                v->diag_pub = create_publisher<std_msgs::msg::String>("/" + v->name + "/follow_diag", 10);
            }
            RCLCPP_INFO(get_logger(), "hercules_synced_node: drive enabled -- TUBE follower v5 on /<vehicle>/follow_path "
                                      "(align_enter=%.0f deg, exit=%.0f deg, align throttle=%.2f legs fwd %.2f / rev %.2f m %s-first, track floor %.2f m/s, lookahead %.1f-%.1f m, "
                                      "wheelbase %.2f m, max steer %.0f deg; speed loop kp=%.2f ki=%.2f creep=%.2f brake kp=%.1f deadband=%.2f), /cmd_vel for reverse/stop.",
                        follow_align_enter_deg_, follow_align_exit_deg_, follow_align_throttle_,
                        follow_align_seg_fwd_m_ > 0.0 ? follow_align_seg_fwd_m_ : follow_align_seg_m_,
                        follow_align_seg_rev_m_ > 0.0 ? follow_align_seg_rev_m_ : follow_align_seg_m_,
                        follow_align_reverse_first_ ? "reverse" : "forward", follow_track_min_speed_,
                        follow_lookahead_min_, follow_lookahead_max_, follow_wheelbase_, follow_max_steer_deg_,
                        vel_kp_, vel_ki_, min_creep_throttle_, brake_kp_, brake_deadband_);
        }

        {
            std::string names;
            RCLCPP_INFO(get_logger(), "hercules_synced_node: PROXIMITY GOVERNOR %s (slow %.1f m, stop %.1f m, cone %.0f deg, lane half-width %.1f m, yield timeout %.0f s, priority = vehicle order; hard stop for anything in my path)",
                        peer_avoid_ ? "ON" : "off", peer_slow_m_, peer_stop_m_, peer_cone_deg_, peer_lane_half_m_, peer_yield_timeout_s_);
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

    // velocity loop shared by TRACK and ALIGN: |speed| toward v_des; reverse = manual gear -1 with
    // positive throttle (how the executor's /cmd_vel reverse has always driven the car).
    void speed_loop(Vehicle &v, msr::airlib::CarApiBase::CarControls &ctrl, double speed_abs, double v_des, bool reverse)
    {
        if (reverse) { ctrl.is_manual_gear = true; ctrl.manual_gear = -1; }   // forward: caller decides the gear
        const double err = v_des - speed_abs;
        if (err > -brake_deadband_) {
            // THROTTLE regime (including a small over-speed band where the car just coasts).
            // This engine is strong at the bottom of the pedal (probe: 0.03 -> 1 m/s within a
            // metre), so the loop is PI with SMALL gains and NO standing throttle floor once the
            // car rolls: the old floor of 0.08 whenever err > 0 made every tick either a hard
            // launch (0.08 -> 0.55 m/s in 0.5 s) or a hard brake -- a 0.06/0.58 m/s sawtooth
            // (pilot diag 2026-09-27). SOFT START: below follow_softstart_v the throttle is
            // capped (full lock + high throttle from rest jams the car), a creep floor beats
            // static friction, and the integrator winds at half rate so a car that does not move
            // (slope, rough ground) works its way up to the cap instead of sitting on the floor.
            // Engine map (probe 2026-09-27, fresh PIE): throttle 0.012 -> no motion at all, 0.020 ->
            // 0.5 m/s^2 with NO steady state below ~3 m/s. So no throttle holds a low speed; the
            // loop pulses a small throttle below target and COASTS (throttle 0) in the band above
            // it -- kp 0.25 lifts a 0.1 m/s error over the 0.012 dead zone; validated on the harness.
            const bool starting = speed_abs < follow_softstart_v_;
            const double int_rate = starting ? 0.5 : 1.0;
            v.vel_int = std::max(0.0, std::min(vel_int_max_, v.vel_int + err * sync_dt_ * int_rate));
            double thr = (err > 0.0) ? vel_kp_ * err + vel_ki_ * v.vel_int : 0.0;
            const double cap = starting ? follow_softstart_throttle_ : drive_max_throttle_;
            const double floor = (starting && err > 0.0) ? min_creep_throttle_ : 0.0;
            ctrl.throttle = std::max(floor, std::min(cap, thr));
            ctrl.brake = 0.0;
        } else {
            ctrl.throttle = 0.0; ctrl.brake = std::min(1.0, brake_kp_ * (-err - brake_deadband_)); v.vel_int = 0.0;
        }
    }

    // Closest point of p on segment a->b; returns t in [0,1] and writes the point.
    static inline double projectOnSeg(double ax, double ay, double bx, double by, double px, double py,
                                      double &qx, double &qy)
    {
        const double vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy;
        double t = 0.0;
        if (L2 > 1e-12) t = std::max(0.0, std::min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2));
        qx = ax + t * vx; qy = ay + t * vy;
        return t;
    }

    // TUBE follower. Returns true when a path is being followed (ctrl filled), false when there is
    // no path (caller falls through to /cmd_vel velocity mode).
    bool follow_step(Vehicle &v, msr::airlib::CarApiBase::CarControls &ctrl, double speed_abs)
    {
        std::lock_guard<std::mutex> lk(v.follow_mtx);
        auto &path = v.follow_path;
        if (path.empty()) return false;
        const double px = v.last_ned_x, py = v.last_ned_y, yaw = v.last_yaw_ned;
        // TERMINAL: within the arrival radius of the path end -> stop and CLEAR the path so the
        // car stays stopped even if the executor is late/dead.
        if (dist2D(px, py, path.back().first, path.back().second) < follow_arrive_threshold_) {
            RCLCPP_INFO(get_logger(), "[%s] path done: max_xte=%.2f mean_xte=%.2f align_segments=%d (%d ticks)",
                        v.name.c_str(), v.path_max_xte, v.path_n ? v.path_sum_xte / v.path_n : 0.0,
                        v.path_align_total, v.path_n);
            path.clear(); v.follow_index = 0; v.fmode = 0;
            return false;
        }
        const size_t n = path.size();
        if (n < 2) {                                   // single point: drive straight at it
            const double a = normalizeAngle(std::atan2(path[0].second - py, path[0].first - px) - yaw);
            ctrl.steering = std::clamp(a * 2.0, -1.0, 1.0);
            speed_loop(v, ctrl, speed_abs, std::min({follow_align_speed_, v.terrain_max_speed.load(), v.peer_max_speed}), false);
            return true;
        }
        // 1. PROJECTION onto the polyline, searched in a window around the current segment so a
        //    hairpin that passes near the robot cannot steal the projection.
        size_t s0 = v.proj_seg > 3 ? v.proj_seg - 3 : 0;
        size_t s1 = std::min(n - 1, v.proj_seg + 40);   // 40 segments ~ 8 m of 0.2 m cells
        double best = 1e18, bt = 0.0, bqx = px, bqy = py; size_t bseg = v.proj_seg;
        for (size_t i = s0; i + 1 <= s1 && i + 1 < n; ++i) {
            double qx, qy;
            const double t = projectOnSeg(path[i].first, path[i].second, path[i + 1].first, path[i + 1].second, px, py, qx, qy);
            const double d = dist2D(px, py, qx, qy);
            if (d < best - 1e-9) { best = d; bt = t; bqx = qx; bqy = qy; bseg = i; }
        }
        v.proj_seg = bseg;
        v.xte = best;
        // path tangent at the projection (skip zero-length segments)
        double tang = yaw;
        for (size_t i = bseg; i + 1 < n; ++i) {
            const double dx = path[i + 1].first - path[i].first, dy = path[i + 1].second - path[i].second;
            if (dx * dx + dy * dy > 1e-8) { tang = std::atan2(dy, dx); break; }
        }
        // 2. LOOKAHEAD point at arc length L ahead of the projection (L shrinks in ALIGN so the
        //    target heading is the local path direction, not a far-away point).
        const double L = (v.fmode == 0)
            ? std::clamp(follow_lookahead_min_ + follow_lookahead_gain_ * speed_abs, follow_lookahead_min_, follow_lookahead_max_)
            : follow_lookahead_min_;
        double tx = bqx, ty = bqy, acc = 0.0; size_t i = bseg;
        {
            // remainder of the projection segment first
            double cx = bqx, cy = bqy;
            while (i + 1 < n) {
                const double nx = path[i + 1].first, ny = path[i + 1].second;
                const double seg = dist2D(cx, cy, nx, ny);
                if (acc + seg >= L) {
                    const double f = seg > 1e-9 ? (L - acc) / seg : 0.0;
                    tx = cx + f * (nx - cx); ty = cy + f * (ny - cy); acc = L; break;
                }
                acc += seg; cx = nx; cy = ny; tx = nx; ty = ny; ++i;
            }
        }
        // sharpest turn within the lookahead window (corner anticipation)
        double turn_ahead = 0.0;
        {
            double prev = tang; double walked = 0.0;
            for (size_t j = bseg; j + 1 < n && walked < follow_lookahead_max_; ++j) {
                const double dx = path[j + 1].first - path[j].first, dy = path[j + 1].second - path[j].second;
                const double l = std::sqrt(dx * dx + dy * dy);
                if (l < 1e-6) continue;
                const double d = std::atan2(dy, dx);
                turn_ahead = std::max(turn_ahead, std::fabs(normalizeAngle(d - prev)));
                prev = d; walked += l;
            }
        }
        const double alpha = normalizeAngle(std::atan2(ty - py, tx - px) - yaw);   // heading error to the lookahead point
        v.alpha = alpha;
        v.path_max_xte = std::max(v.path_max_xte, best); v.path_sum_xte += best; v.path_n++;
        const double enter = follow_align_enter_deg_ * M_PI / 180.0, exit = follow_align_exit_deg_ * M_PI / 180.0;
        const double vcap = std::min({follow_desired_velocity_, v.terrain_max_speed.load(), v.peer_max_speed});
        // YIELD: a vehicle whose peer cap is (near) zero brakes and holds, whatever follower mode it is in
        // -- a K-turn in progress must not swing into the other vehicle either. It resumes from TRACK
        // (fmode 0) or from ALIGN_BRAKE (3), which re-evaluates the heading, never mid-leg.
        if (peer_avoid_ && v.peer_max_speed < 0.02) {
            ctrl.throttle = 0.0; ctrl.brake = 1.0; ctrl.steering = 0.0; v.vel_int = 0.0;
            if (v.fmode == 1 || v.fmode == 2) v.fmode = 3;
            return true;
        }
        // 3. MODE transitions
        if (v.fmode == 0 && std::fabs(alpha) > enter) {
            v.fmode = 3;                                 // brake first, then start the K-turn
            v.align_segments = 0; v.align_steer_sign = 0;
            // The target heading is FROZEN for the whole episode. Recomputing it from a lookahead
            // point while the robot shuffles through a K-turn let the point hop around a corner,
            // flipped the sign of the error mid-segment and made the turn oscillate (live 2026-09-27:
            // +56 -> -67 deg, 12 segments, gave up). The robot barely moves during a K-turn, so the
            // direction to rejoin the path is a constant; TRACK absorbs the residual afterwards.
            // NEAR the path (a path that starts at the robot and runs behind it -- every planner
            // path with a >75 deg initial turn) the target is the PATH DIRECTION at the lookahead,
            // not the bearing to the lookahead point: with reverse-first legs the robot moves ALONG
            // such a path during the K-turn, the projection advances past the old lookahead and the
            // bearing target flips sign on exit (harness v5A 2026-09-27: +80 -> -155 deg chatter,
            // 2.2x path length). Far from the path the bearing is still the right thing to face.
            double tang_la = tang;
            if (i + 1 < n) {
                const double dx = path[i + 1].first - path[i].first, dy = path[i + 1].second - path[i].second;
                if (dx * dx + dy * dy > 1e-8) tang_la = std::atan2(dy, dx);
            }
            const bool near_path = best < follow_align_tangent_xte_;
            v.align_target_yaw = near_path ? tang_la : std::atan2(ty - py, tx - px);
            RCLCPP_INFO(get_logger(), "[%s] ALIGN: heading error %.0f deg (xte %.2f m) -> K-turn in place toward %.0f deg (%s)",
                        v.name.c_str(), alpha * 180.0 / M_PI, best, v.align_target_yaw * 180.0 / M_PI,
                        near_path ? "path tangent" : "bearing to lookahead");
        }
        // error against the frozen target while aligning; against the live lookahead otherwise
        const double a_err = (v.fmode == 0) ? alpha : normalizeAngle(v.align_target_yaw - yaw);
        if (v.fmode == 0) {
            // ---- TRACK: pure pursuit with full steering authority, gated speed ----
            const double kappa = 2.0 * std::sin(alpha) / std::max(L, 0.2);
            const double delta = std::atan(kappa * follow_wheelbase_);               // bicycle model
            ctrl.steering = std::clamp(delta / (follow_max_steer_deg_ * M_PI / 180.0), -1.0, 1.0);
            // FULL LOCK FROM A STANDSTILL JAMS THIS CAR (live 2026-09-27: steer 1.0 + any throttle
            // from rest = 0.00 m, engine idle; 0.6 lock moves freely; lock applied while braked and
            // THEN throttled also moves -- the K-turn's brake phase does exactly that). TRACK caps
            // the lock until the car is rolling.
            if (speed_abs < follow_softstart_v_)
                ctrl.steering = static_cast<float>(std::clamp(static_cast<double>(ctrl.steering),
                                                              -follow_softstart_steer_, follow_softstart_steer_));
            double sf = 1.0;
            const double a_abs = std::fabs(alpha);
            if (a_abs > 15.0 * M_PI / 180.0)           // 15 deg .. enter: 1.0 -> 0.35
                sf = std::max(0.35, 1.0 - (a_abs - 15.0 * M_PI / 180.0) / std::max(enter - 15.0 * M_PI / 180.0, 1e-3) * 0.65);
            if (best > follow_xte_slow_) sf = std::min(sf, 0.5);
            if (turn_ahead > follow_corner_slow_deg_ * M_PI / 180.0) sf = std::min(sf, 0.35);
            ctrl.is_manual_gear = true; ctrl.manual_gear = 1;      // explicit forward gear (see ALIGN)
            speed_loop(v, ctrl, speed_abs, std::max(std::min(follow_track_min_speed_, vcap), vcap * sf), false);
        } else if (v.fmode == 3) {
            // ---- ALIGN_BRAKE: come to rest, then decide ----
            ctrl.throttle = 0.0; ctrl.brake = 1.0; v.vel_int = 0.0;
            ctrl.steering = (v.align_steer_sign != 0) ? -static_cast<double>(v.align_steer_sign) : 0.0;   // pre-set the NEXT segment's lock (alternates)
            if (speed_abs < 0.06) {
                if (std::fabs(a_err) < exit || v.align_segments >= follow_align_max_segments_) {
                    if (v.align_segments >= follow_align_max_segments_ && std::fabs(a_err) >= exit)
                        RCLCPP_WARN(get_logger(), "[%s] ALIGN gave up after %d segments (heading error %.0f deg) -> TRACK",
                                    v.name.c_str(), v.align_segments, a_err * 180.0 / M_PI);
                    else
                        RCLCPP_INFO(get_logger(), "[%s] ALIGNED after %d segments (residual %.0f deg to target, %.0f deg to lookahead, xte %.2f m) -> TRACK",
                                    v.name.c_str(), v.align_segments, a_err * 180.0 / M_PI, alpha * 180.0 / M_PI, best);
                    v.fmode = 0;
                } else {
                    // next K-turn segment: alternate REV / FWD; steer sign latched so a segment
                    // never chatters. FWD full lock toward the target; REV with the OPPOSITE lock
                    // turns the heading the same way (measured: rev steer -1 -> yaw rate +56 deg/s).
                    // REVERSE FIRST: a K-turn happens at a path start, where the robot stands at
                    // the end of its previous path facing that path's goal -- typically a frontier
                    // at a bank or unknown ground -- while the ground BEHIND it is its own track.
                    // Pilot 2026-09-27 (Husky3 r1): a forward-first K-turn at a bank swept 0.7 m
                    // into water before the first reverse leg; reverse-first keeps the footprint
                    // on driven ground.
                    const bool fwd = follow_align_reverse_first_ ? (v.align_segments % 2 == 1) : (v.align_segments % 2 == 0);
                    const int sgn = a_err >= 0.0 ? 1 : -1;
                    v.align_steer_sign = fwd ? sgn : -sgn;
                    v.fmode = fwd ? 1 : 2;
                    v.seg_moved = 0.0; v.seg_last_x = px; v.seg_last_y = py;
                    v.align_segments++; v.path_align_total++;
                }
            }
        } else {
            // ---- ALIGN_FWD (1) / ALIGN_REV (2): one full-lock segment of follow_align_seg_m ----
            v.seg_moved += dist2D(px, py, v.seg_last_x, v.seg_last_y);
            v.seg_last_x = px; v.seg_last_y = py;
            ctrl.steering = static_cast<double>(v.align_steer_sign);
            // gears are set EXPLICITLY (gear_immediate): letting the automatic box shift back from
            // the reverse leg cost most of a 0.35 m forward segment (auto-box latency), which is
            // why the first live K-turns turned only 4-14 deg per segment.
            ctrl.is_manual_gear = true; ctrl.manual_gear = (v.fmode == 2) ? -1 : 1;
            if (follow_align_throttle_ > 0.0) {
                if (speed_abs < follow_align_vmax_) { ctrl.throttle = follow_align_throttle_; ctrl.brake = 0.0; }
                else { ctrl.throttle = 0.0; ctrl.brake = 0.5; }
                v.vel_int = 0.0;
            } else {
                speed_loop(v, ctrl, speed_abs, follow_align_speed_, v.fmode == 2);
            }
            // ASYMMETRIC legs (v5): short forward / long reverse legs bias the K-turn footprint
            // BACKWARD, away from whatever the robot was facing when its path ended (a bank, in
            // 3 of the 4 wet K-turns of the 2026-09-27 pilot the FIRST wet leg was a forward one).
            const double seg_len = (v.fmode == 2) ? (follow_align_seg_rev_m_ > 0.0 ? follow_align_seg_rev_m_ : follow_align_seg_m_)
                                                  : (follow_align_seg_fwd_m_ > 0.0 ? follow_align_seg_fwd_m_ : follow_align_seg_m_);
            if (v.seg_moved >= seg_len || std::fabs(a_err) < exit) {
                v.fmode = 3;                             // brake, then re-evaluate
            }
        }
        // 4. diagnostics
        if (v.diag_pub && (++v.diag_tick % std::max(1, follow_diag_every_)) == 0) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "{\"mode\":%d,\"xte\":%.3f,\"alpha_deg\":%.1f,\"aerr_deg\":%.1f,\"v\":%.3f,\"steer\":%.2f,\"seg\":%zu,\"L\":%.2f,\"turn_ahead_deg\":%.0f,\"align_segs\":%d,\"brake\":%.2f,\"thr\":%.2f,\"gear\":%d}",
                          v.fmode, best, alpha * 180.0 / M_PI, a_err * 180.0 / M_PI, speed_abs, ctrl.steering, bseg, L,
                          turn_ahead * 180.0 / M_PI, v.align_segments, ctrl.brake, ctrl.throttle, ctrl.is_manual_gear ? ctrl.manual_gear : 0);
            std_msgs::msg::String m; m.data = buf; v.diag_pub->publish(m);
        }
        return true;
    }

    // PROXIMITY GOVERNOR: once per tick, before the controls, from the poses cached by the previous
    // tick's publish_odom_and_tf (global NED). Writes v.peer_max_speed for every vehicle.
    void proximity_governor(double sim_now)
    {
        const size_t n = vehicles_.size();
        const double cone = peer_cone_deg_ * M_PI / 180.0;
        for (size_t i = 0; i < n; ++i) {
            Vehicle &v = *vehicles_[i];
            double cap = 1e9; int who = -1; double dmin = 1e9, dbear = 0.0;
            for (size_t j = 0; j < n; ++j) {
                if (j == i) continue;
                const Vehicle &o = *vehicles_[j];
                const double dx = o.last_ned_x - v.last_ned_x, dy = o.last_ned_y - v.last_ned_y;
                const double d = std::sqrt(dx * dx + dy * dy);
                if (d >= peer_slow_m_ || d >= dmin) continue;
                const double bearing = normalizeAngle(std::atan2(dy, dx) - v.last_yaw_ned);
                // body-frame position of the other vehicle: along-track (ahead of me) and cross-track
                const double along = dx * std::cos(v.last_yaw_ned) + dy * std::sin(v.last_yaw_ned);
                const double cross = -dx * std::sin(v.last_yaw_ned) + dy * std::cos(v.last_yaw_ned);
                const bool in_path = along > 0.0 && std::fabs(cross) < peer_lane_half_m_;
                const bool other_stopped = o.last_speed.load() < 0.05;
                if (std::fabs(bearing) > cone && !in_path) continue;   // only vehicles AHEAD slow us down
                // HARD RULE (crossing test 2026-10-12: the priority vehicle drove straight through the
                // stopped yielder on a shared line): a vehicle IN MY PATH that is stopped, or inside the
                // stop distance, caps me whatever the priority -- nobody drives into anybody. Priority
                // only decides who slows first in the approach band. A yielder parked BESIDE my line
                // (|cross| >= peer_lane_half_m, the usual crossing case) does not hold me.
                const bool hard = in_path && (other_stopped || d < peer_stop_m_);
                if (!hard) {
                    // PRIORITY: head-on (the other also sees us in its cone) -> the vehicle listed first
                    // keeps driving unless it has been yielding past the timeout; a vehicle that is
                    // only behind/beside us never makes us yield.
                    const double ob = normalizeAngle(std::atan2(-dy, -dx) - o.last_yaw_ned);
                    const bool head_on = std::fabs(ob) <= cone;
                    const bool i_has_priority = (i < j);
                    const bool my_timeout = v.yield_since >= 0.0 && (sim_now - v.yield_since) > peer_yield_timeout_s_;
                    const bool other_timeout = o.yield_since >= 0.0 && (sim_now - o.yield_since) > peer_yield_timeout_s_ && o.peer_idx == static_cast<int>(i);
                    if (head_on && i_has_priority && !other_timeout) continue;   // the other one yields
                    if (my_timeout && !other_timeout) continue;                  // waited long enough: go
                }
                dmin = d; dbear = bearing; who = static_cast<int>(j);
                cap = std::clamp((d - peer_stop_m_) / std::max(peer_slow_m_ - peer_stop_m_, 1e-3), 0.0, 1.0) * follow_desired_velocity_;
            }
            const bool yielding_now = who >= 0;
            const bool was_yielding = v.yield_since >= 0.0;
            if (yielding_now && !was_yielding) {
                v.yield_since = sim_now; v.yield_events++;
                RCLCPP_INFO(get_logger(), "[%s] YIELD to %s: %.1f m at %.0f deg -> cap %.2f m/s (sim t %.1f)",
                            v.name.c_str(), vehicles_[who]->name.c_str(), dmin, dbear * 180.0 / M_PI, cap, sim_now);
            } else if (!yielding_now && was_yielding) {
                v.yield_total_s += sim_now - v.yield_since;
                RCLCPP_INFO(get_logger(), "[%s] YIELD over after %.1f s (total %.1f s, %d events)",
                            v.name.c_str(), sim_now - v.yield_since, v.yield_total_s, v.yield_events);
                v.yield_since = -1.0;
            }
            v.peer_idx = who; v.peer_max_speed = cap;
        }
    }

    void apply_controls(Vehicle &v)
    {
        if (!enable_drive_) {
            return;
        }
        const double speed = v.last_speed.load();
        msr::airlib::CarApiBase::CarControls ctrl;

        // --- PATH MODE: the TUBE follower (see the parameter block for the rationale). Runs once
        // per lockstep tick in native NED. Throttle uses the velocity+brake loop so the brakeless
        // car holds the slow setpoints. ---
        if (follow_step(v, ctrl, speed)) {
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
                // governor only once every vehicle has a cached pose (tick 1 found all four at the
                // origin and logged four phantom yields on the 2026-10-12 demo)
                if (peer_avoid_ && step > 0) proximity_governor(static_cast<double>(step) * sync_dt_);
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
                    auto car_state = client_->getCarState(v.name);
                    const float z_ned = car_state.kinematics_estimated.pose.position.z();
                    if (fall_rescue_ && v.has_good_pose && z_ned > fall_rescue_z_) {
                        RCLCPP_ERROR(get_logger(),
                                     "[%s] BELOW WORLD (z_ned=%.1f > %.1f) -> rescue teleport to "
                                     "last grounded pose", v.name.c_str(), z_ned, fall_rescue_z_);
                        try {
                            msr::airlib::Pose up = v.last_good_pose;
                            up.position.z() -= 0.3f;   // slight lift to avoid re-penetration
                            client_->simSetVehiclePose(up, true, v.name);
                            msr::airlib::CarApiBase::CarControls stop;
                            stop.brake = 1.0;
                            client_->setCarControls(stop, v.name);
                            car_state = client_->getCarState(v.name);
                        } catch (...) {
                            RCLCPP_ERROR(get_logger(), "[%s] rescue teleport failed", v.name.c_str());
                        }
                    } else if (z_ned < fall_rescue_z_ * 0.5) {
                        v.last_good_pose = car_state.kinematics_estimated.pose;
                        v.has_good_pose = true;
                    }
                    // stuck-nudge: displacement over a sliding sim-time window while a follow
                    // path is active; wedged -> teleport straight backward along the heading
                    if (stuck_nudge_) {
                        const double sim_now = static_cast<double>(step) * sync_dt_;
                        const auto &pp = car_state.kinematics_estimated.pose.position;
                        // a YIELDING vehicle is meant to stand still: keep re-anchoring so the
                        // stuck detector never teleports it backward into whatever it yielded to
                        if (v.stuck_at < 0.0 || v.follow_path.empty() || (peer_avoid_ && v.yield_since >= 0.0)) {
                            v.stuck_ax = pp.x(); v.stuck_ay = pp.y(); v.stuck_at = sim_now;
                        } else if (sim_now - v.stuck_at >= stuck_window_s_) {
                            const double moved = std::hypot(pp.x() - v.stuck_ax,
                                                            pp.y() - v.stuck_ay);
                            if (moved < stuck_min_motion_) {
                                auto pose = car_state.kinematics_estimated.pose;
                                const auto q = pose.orientation;
                                const double yaw = std::atan2(
                                    2.0 * (q.w() * q.z() + q.x() * q.y()),
                                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
                                pose.position.x() -= static_cast<float>(nudge_back_m_ * std::cos(yaw));
                                pose.position.y() -= static_cast<float>(nudge_back_m_ * std::sin(yaw));
                                pose.position.z() -= 0.15f;
                                RCLCPP_WARN(get_logger(),
                                            "[%s] STUCK (%.2fm in %.0fs with active path) -> "
                                            "nudge %.1fm backward", v.name.c_str(), moved,
                                            stuck_window_s_, nudge_back_m_);
                                try {
                                    client_->simSetVehiclePose(pose, true, v.name);
                                } catch (...) {
                                    RCLCPP_ERROR(get_logger(), "[%s] nudge failed", v.name.c_str());
                                }
                            }
                            v.stuck_ax = pp.x(); v.stuck_ay = pp.y(); v.stuck_at = sim_now;
                        }
                    }
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
                consecutive_timeouts_ = 0;
            } catch (const AcquisitionTimeout &e) {
                RCLCPP_FATAL(get_logger(),
                             "hercules_synced_node: %s Shutting down.", e.what());
                rclcpp::shutdown();
                return;
            } catch (rpc::rpc_error &e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RPC error: %s",
                                      e.get_error().as<std::string>().c_str());
            } catch (rpc::timeout &e) {
                // The editor can hitch or crash mid-tick (seen live: a PawnSimApi pawn-deref
                // crash hung simContinueForTime past the 60s RPC timeout). rpc::timeout is NOT
                // an rpc_error, so it used to escape this loop, leave the thread, and SIGABRT
                // the whole bridge. Ride it out: reconnect and retry the tick; give up only
                // after enough consecutive dead ticks that the sim is clearly gone.
                ++consecutive_timeouts_;
                RCLCPP_ERROR(get_logger(),
                             "lockstep tick RPC timeout (%d consecutive): %s -- reconnecting",
                             consecutive_timeouts_, e.what());
                if (consecutive_timeouts_ >= 5) {
                    RCLCPP_FATAL(get_logger(),
                                 "%d consecutive tick timeouts -- sim unreachable. Shutting down.",
                                 consecutive_timeouts_);
                    rclcpp::shutdown();
                    return;
                }
                try {
                    client_ = std::make_unique<msr::airlib::CarRpcLibClient>(host_ip_, host_port_);
                    client_->confirmConnection();
                    for (auto &vp : vehicles_) {
                        try { client_->enableApiControl(true, vp->name); } catch (...) {}
                    }
                    RCLCPP_WARN(get_logger(), "reconnected after tick timeout");
                } catch (...) {
                    RCLCPP_ERROR(get_logger(), "reconnect failed; retrying next tick");
                }
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
    double vel_kp_ = 0.25, vel_ki_ = 0.05, vel_int_max_ = 2.0, brake_kp_ = 0.5, brake_deadband_ = 0.05;
    double drive_max_throttle_ = 0.6, min_creep_throttle_ = 0.03;
    double follow_lookahead_ = 2.0, follow_desired_velocity_ = 0.4, follow_kp_steer_ = 1.0, follow_max_steer_ = 0.5;
    double follow_arrive_threshold_ = 1.0;
    double follow_align_enter_deg_ = 75.0, follow_align_exit_deg_ = 12.0, follow_align_speed_ = 0.3;
    double follow_align_seg_m_ = 0.35, follow_align_seg_fwd_m_ = 0.0, follow_align_seg_rev_m_ = 0.0, follow_align_tangent_xte_ = 0.0;
    int follow_align_max_segments_ = 12; bool follow_align_reverse_first_ = false;
    double follow_align_throttle_ = 0.05, follow_align_vmax_ = 1.0, follow_track_min_speed_ = 0.3;
    double follow_softstart_v_ = 0.15, follow_softstart_throttle_ = 0.08, follow_softstart_steer_ = 0.6;
    double follow_lookahead_min_ = 0.5, follow_lookahead_max_ = 1.2, follow_lookahead_gain_ = 1.2;
    bool peer_avoid_ = false; double peer_slow_m_ = 4.0, peer_stop_m_ = 1.5, peer_cone_deg_ = 70.0, peer_yield_timeout_s_ = 8.0, peer_lane_half_m_ = 1.2;
    double follow_wheelbase_ = 1.0, follow_max_steer_deg_ = 50.0, follow_xte_slow_ = 0.3, follow_corner_slow_deg_ = 50.0;
    int follow_diag_every_ = 5;
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
    int consecutive_timeouts_ = 0;
    bool fall_rescue_ = true;
    double fall_rescue_z_ = 10.0;
    bool stuck_nudge_ = true;
    double stuck_window_s_ = 60.0, stuck_min_motion_ = 0.3, nudge_back_m_ = 1.5;
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
