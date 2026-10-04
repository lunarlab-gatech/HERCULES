#include <chrono>
#include <memory>
#include <random>
#include <vector>
#include <cmath>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <functional>
#include <utility>
#include <filesystem>
#include <fstream>
#include <string>
#include <iostream>
#include <queue>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Use  ``sudo apt-get install nlohmann-json3-dev`` to install this
#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std::chrono_literals;

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner"),
                          occupancy_grid_received_ground_(false),
                          occupancy_grid_received_drone_(false),
                          rng_(rd_())
    {
        // Declare parameters (default values for ground planning).
        this->declare_parameter("z_height", -0.25);
        this->declare_parameter("trajectory_length", 200.0); // meters
        this->declare_parameter("square_size", 800.0);       // planning area side (meters)
        this->declare_parameter("max_linear_velocity", 1.5); // default for UGV
        this->declare_parameter("robot_name", "Husky1");

        // Starting point parameters.
        this->declare_parameter("start_x", 0.0);
        this->declare_parameter("start_y", 0.0);
        this->declare_parameter("start_z", -0.25); // default same as z_height

        // Unicycle-like constraints.
        this->declare_parameter("start_yaw", 0.0);           // initial heading in degrees
        this->declare_parameter("max_turn_angle_deg", 45.0); // default for UGV

        // Declare UGV and drone specific parameters.
        this->declare_parameter("ugv_max_linear_velocity", 1.5);
        this->declare_parameter("ugv_max_turn_angle_deg", 45.0);
        this->declare_parameter("drone_max_linear_velocity", 1.5);
        this->declare_parameter("drone_max_turn_angle_deg", 105.0);
        this->declare_parameter("max_linear_acceleration", 2.5);

        // New parameter for obstacle inflation (meters).
        this->declare_parameter("ugv_inflation_radius", 1.0);
        this->declare_parameter("uav_inflation_radius", 2.0);
        this->declare_parameter("uav_astar_inflation_radius", 4.0);

        // Settings file and trajectory inflation parameter.
        // this->declare_parameter("settings_file", "/home/sgarimella34/Documents/AirSim/settings_trajectory_planning.json");
        this->declare_parameter("settings_file", "/home/dellg16ssg/Documents/AirSim/settings_trajectory_planning.json");
        std::string settings_file_;
        this->declare_parameter("trajectory_exploration_radius", 5.0);
        // this->declare_parameter("drone_altitude", 35.0); // meters
        this->declare_parameter("drone_altitude", 10.0); // meters
        this->declare_parameter("use_k_rrt_for_checkpoints", false);

        this->declare_parameter<int>("planning_horizon_steps", 50);
        // FlightPattern for drones whose settings entry has none: "RandomExplore" or "Convoy".
        this->declare_parameter("default_flight_pattern", "RandomExplore");
        // Save each trajectory relative to its vehicle's settings X/Y. AirSim reports and
        // commands vehicle positions relative to each vehicle's spawn point, so set this when
        // the files are replayed by the waypoint controllers.
        this->declare_parameter("trajectories_relative_to_start", false);
        // Shut down once every trajectory is saved, instead of spinning until interrupted.
        this->declare_parameter("exit_when_done", false);

        // Retrieve parameters.
        this->get_parameter("use_k_rrt_for_checkpoints", use_k_rrt_for_checkpoints_);
        this->get_parameter("trajectory_exploration_radius", trajectory_exploration_radius_);
        this->get_parameter("settings_file", settings_file_);
        this->get_parameter("z_height", z_height_);
        this->get_parameter("trajectory_length", trajectory_length_);
        this->get_parameter("square_size", square_size_);
        this->get_parameter("max_linear_velocity", max_linear_velocity_);
        this->get_parameter("start_x", start_x_);
        this->get_parameter("start_y", start_y_);
        this->get_parameter("start_z", start_z_);
        this->get_parameter("start_yaw", start_yaw_deg_);
        this->get_parameter("max_turn_angle_deg", max_turn_angle_deg_);
        this->get_parameter("ugv_inflation_radius", ugv_inflation_radius_);
        this->get_parameter("uav_inflation_radius", uav_inflation_radius_);
        this->get_parameter("uav_astar_inflation_radius", uav_astar_inflation_radius_);

        this->get_parameter("robot_name", robot_name_);
        this->get_parameter("drone_altitude", drone_altitude_);

        this->get_parameter("ugv_max_linear_velocity", ugv_max_linear_velocity_);
        this->get_parameter("ugv_max_turn_angle_deg", ugv_max_turn_angle_deg_);
        this->get_parameter("drone_max_linear_velocity", drone_max_linear_velocity_);
        this->get_parameter("drone_max_turn_angle_deg", drone_max_turn_angle_deg_);
        this->get_parameter("max_linear_acceleration", max_linear_acceleration_);

        planning_horizon_steps_ =
            this->get_parameter("planning_horizon_steps").as_int();
        this->get_parameter("default_flight_pattern", default_flight_pattern_);
        this->get_parameter("trajectories_relative_to_start", trajectories_relative_to_start_);
        this->get_parameter("exit_when_done", exit_when_done_);

        // Declare and assign default value
        output_folder_string_ = this->declare_parameter<std::string>(
            "output_folder",
            "/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_random_explore/");

        // for CSLAM, random explore motion
        // output_folder_string_ = "/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/";
        // output_folder_string_ = "/home/dellg16ssg/multi-robot-coordination/trajectory_data/CSLAM_random_explore/";

        // for BEVP, random explore motion
        // output_folder_string_ = "/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_random_explore/";
        // output_folder_string_ = "/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_random_explore/";

        // // for BEVP, convoy motion
        // output_folder_string_ = "/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_convoy/";
        // output_folder_string_ = "/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_convoy/";

        ground_map_topic_ = this->declare_parameter<std::string>(
            "ground_map_topic",
            "Ausenv_0mAlt_OGM_0p5m");

        drone_map_topic_ = this->declare_parameter<std::string>(
            "drone_map_topic",
            "Ausenv_10mAlt_OGM_0p5m");

        // Convert degrees to radians.
        start_yaw_ = start_yaw_deg_ * M_PI / 180.0;
        max_turn_angle_rad_ = max_turn_angle_deg_ * M_PI / 180.0;

        // Define the planning area.
        grid_resolution_ = 0.25; // meters per cell
        grid_origin_x_ = -square_size_ / 2.0;
        grid_origin_y_ = -square_size_ / 2.0;
        grid_width_ = static_cast<int>(square_size_ / grid_resolution_);
        grid_height_ = static_cast<int>(square_size_ / grid_resolution_);
        // These occupancy grids will be filled from the callbacks.
        occupancy_grid_ground_.assign(grid_width_ * grid_height_, 0);
        occupancy_grid_drone_.assign(grid_width_ * grid_height_, 0);

        // Load vehicle parameters from the JSON settings file.
        std::ifstream settings_ifs(settings_file_);
        if (!settings_ifs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open settings file: %s", settings_file_.c_str());
        }
        else
        {
            json settings_json;
            try
            {
                settings_ifs >> settings_json;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error parsing settings JSON: %s", e.what());
            }
            settings_ifs.close();

            if (settings_json.contains("Vehicles"))
            {
                for (auto &[veh_name, veh_data] : settings_json["Vehicles"].items())
                {
                    VehicleInfo info;
                    info.name = veh_name;

                    // X and Y swapped to adjust for coordinate transforms between hercules and ros2
                    // info.start_x = veh_data.value("X", 0.0);
                    // info.start_y = veh_data.value("Y", 0.0);
                    info.start_x = veh_data.value("Y", 0.0);
                    info.start_y = veh_data.value("X", 0.0);

                    info.start_z = veh_data.value("Z", 0.0);
                    info.start_yaw = veh_data.value("Yaw", 0.0);
                    info.trajectory_length = veh_data.value("TrajectoryLength", trajectory_length_);
                    if (veh_data.contains("Checkpoints") && veh_data["Checkpoints"].is_array())
                    {
                        for (const auto &checkpoint : veh_data["Checkpoints"])
                        {
                            double cx = checkpoint.value("x", 0.0);
                            double cy = checkpoint.value("y", 0.0);
                            double cz = checkpoint.value("z", 0.0);
                            info.checkpoints.push_back(std::make_tuple(cx, cy, cz));
                        }
                    }

                    info.reach_checkpoints_first = veh_data.value("ReachCheckPointsFirst", true);

                    if (veh_data.contains("FlightPattern"))
                    {
                        info.flight_pattern = veh_data.value("FlightPattern", "RandomExplore");
                    }
                    else
                    {
                        info.flight_pattern = default_flight_pattern_;
                    }
                    vehicles_.push_back(info);
                }
                // Sort vehicles by numeric order extracted from their names.
                std::sort(vehicles_.begin(), vehicles_.end(), [](const VehicleInfo &a, const VehicleInfo &b)
                          {
          auto extractNumber = [](const std::string &s) -> int {
            std::string num;
            for (char c : s)
            {
              if (std::isdigit(c))
                num.push_back(c);
            }
            return num.empty() ? 0 : std::stoi(num);
          };
          return extractNumber(a.name) < extractNumber(b.name); });
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No Vehicles section found in settings file");
            }
        }

        // Subscribe to the two occupancy grid topics.
        // Ground OGM (altitude 0.0) used for UGV planning.
        occupancy_grid_sub_ground_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            ground_map_topic_, 10,
            std::bind(&TrajectoryPlanner::occupancy_grid_ground_callback, this, std::placeholders::_1));

        occupancy_grid_sub_drone_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            drone_map_topic_, 10,
            std::bind(&TrajectoryPlanner::occupancy_grid_drone_callback, this, std::placeholders::_1));

        // Publisher for updated occupancy grid (can be used for both types).
        updated_ogm_pub_ground_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("updated_ground_occupancy_grid", 10);
        updated_ogm_pub_drone_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("updated_drone_occupancy_grid", 10);

        // Timer.
        timer_ = this->create_wall_timer(1s, std::bind(&TrajectoryPlanner::timer_callback, this));

        // Initialize random generators.
        x_dist_ = std::uniform_real_distribution<double>(grid_origin_x_, grid_origin_x_ + square_size_);
        y_dist_ = std::uniform_real_distribution<double>(grid_origin_y_, grid_origin_y_ + square_size_);

        RCLCPP_INFO(this->get_logger(), "Trajectory Planner Node Initialized");
    }

private:
    // -------------------- Vehicle Info Structure --------------------
    struct VehicleInfo
    {
        std::string name;
        double trajectory_length;
        double start_x, start_y, start_z, start_yaw;
        std::vector<std::tuple<double, double, double>> checkpoints;
        bool reach_checkpoints_first;
        std::string flight_pattern; // "RandomExplore" (default) or "Convoy" for drones
    };
    std::vector<VehicleInfo> vehicles_;

    // -------------------- Occupancy Grids and Flags --------------------
    // Ground occupancy grid (from the ground-specific OGM topic).
    std::vector<int8_t> occupancy_grid_ground_;
    bool occupancy_grid_received_ground_ = false;
    // Drone occupancy grid (from the drone-specific OGM topic).
    std::vector<int8_t> occupancy_grid_drone_;
    bool occupancy_grid_received_drone_ = false;
    // This dynamic occupancy grid (used during planning) will be set to either one.
    std::vector<int8_t> dynamic_occupancy_grid_;

    // For UGV planning – holds the ground OGM updated with UGV trajectory exploration.
    std::vector<int8_t> dynamic_ground_grid_;
    // For drone planning – holds the drone OGM updated with both UGV and drone exploration.
    std::vector<int8_t> dynamic_drone_grid_;

    // For ground OGM:
    int original_width_ground_ = 0;
    int original_height_ground_ = 0;
    double original_origin_x_ground_ = 0.0;
    double original_origin_y_ground_ = 0.0;
    std::vector<int8_t> original_ogm_ground_;

    // For drone OGM:
    int original_width_drone_ = 0;
    int original_height_drone_ = 0;
    double original_origin_x_drone_ = 0.0;
    double original_origin_y_drone_ = 0.0;
    std::vector<int8_t> original_ogm_drone_;

    // -------------------- Parameters and Variables --------------------
    double z_height_;
    double trajectory_length_;
    double square_size_;
    int num_waypoints_;
    double max_linear_velocity_;
    double max_linear_acceleration_;
    double grid_resolution_;
    double grid_origin_x_, grid_origin_y_;
    int grid_width_, grid_height_;
    double trajectory_exploration_radius_;
    std::string default_flight_pattern_ = "RandomExplore";
    bool trajectories_relative_to_start_ = false;
    bool exit_when_done_ = false;
    double inflation_radius_;
    double ugv_inflation_radius_, uav_inflation_radius_;
    double uav_astar_inflation_radius_;

    int planning_horizon_steps_;

    std::string output_file_path_;
    std::string output_folder_string_;
    std::string robot_name_;
    bool use_k_rrt_for_checkpoints_ = false;
    bool current_reach_checkpoints_first_ = true;

    std::string ground_map_topic_;
    std::string drone_map_topic_;

    // Starting point and orientation.
    double start_x_;
    double start_y_;
    double start_z_;
    double start_yaw_deg_;
    double start_yaw_;
    double drone_altitude_;

    // Turning constraints.
    double max_turn_angle_deg_;
    double max_turn_angle_rad_;

    double ugv_max_linear_velocity_, ugv_max_turn_angle_deg_;
    double drone_max_linear_velocity_, drone_max_turn_angle_deg_;

    // Waypoints provided from settings.
    std::vector<std::tuple<double, double, double>> provided_waypoints_;

    // ROS interfaces.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_ground_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_drone_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_ogm_pub_ground_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_ogm_pub_drone_;

    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> trajectory_publishers_;
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_publishers_;

    // Trajectory storage.
    std::map<std::string, std::vector<std::tuple<double, double, double, double>>> plannedTrajectories_;
    std::vector<std::tuple<double, double, double, double>> trajectory_;

    // Random generators.
    std::random_device rd_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> x_dist_;
    std::uniform_real_distribution<double> y_dist_;

    // -------------------- Helper Functions --------------------
    std::vector<int8_t> inflate_obstacles(const std::vector<int8_t> &grid, double inflation_radius)
    {
        std::vector<int8_t> inflated_grid = grid;
        int inflation_cells = static_cast<int>(std::ceil(inflation_radius / grid_resolution_));

        for (int y = 0; y < grid_height_; ++y)
        {
            for (int x = 0; x < grid_width_; ++x)
            {
                int idx = y * grid_width_ + x;
                if (grid[idx] == 100 || grid[idx] == 50)
                {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
                    {
                        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                                continue;
                            double dist = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
                            if (dist <= inflation_radius)
                            {
                                int n_idx = ny * grid_width_ + nx;
                                if (inflated_grid[n_idx] != 100) // Avoid overwriting existing hard obstacles
                                    inflated_grid[n_idx] = 50;   // Mark inflated cells as soft obstacles
                            }
                        }
                    }
                }
            }
        }
        return inflated_grid;
    }

    // For simplicity, assume names containing "Husky" are UGVs and those containing "Drone" are drones.
    bool isUGV(const VehicleInfo &veh)
    {
        return (veh.name.find("Husky") != std::string::npos);
    }
    bool isDrone(const VehicleInfo &veh)
    {
        return (veh.name.find("Drone") != std::string::npos);
    }

    // -------------------- Occupancy Grid Callbacks --------------------
    // Ground occupancy grid callback (used for UGV planning).
    void occupancy_grid_ground_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (occupancy_grid_received_ground_)
        {
            RCLCPP_INFO(this->get_logger(), "Ground occupancy grid already received; ignoring new message.");
            return;
        }
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty ground occupancy grid; waiting for valid data.");
            return;
        }

        // *** Store original map info for publishing ***
        original_width_ground_ = msg->info.width;
        original_height_ground_ = msg->info.height;
        original_origin_x_ground_ = msg->info.origin.position.x;
        original_origin_y_ground_ = msg->info.origin.position.y;
        original_ogm_ground_ = msg->data;

        // Use the message's resolution for both planning and publishing.
        grid_resolution_ = msg->info.resolution;

        // *** Compute planning grid parameters based on square_size_ and the original map center ***
        double map_center_x = original_origin_x_ground_ + (original_width_ground_ * grid_resolution_) / 2.0;
        double map_center_y = original_origin_y_ground_ + (original_height_ground_ * grid_resolution_) / 2.0;
        grid_width_ = static_cast<int>(square_size_ / grid_resolution_);
        grid_height_ = static_cast<int>(square_size_ / grid_resolution_);
        grid_origin_x_ = map_center_x - square_size_ / 2.0;
        grid_origin_y_ = map_center_y - square_size_ / 2.0;

        // Build planning grid (using your current logic but with the planning grid parameters)
        occupancy_grid_ground_.resize(grid_width_ * grid_height_, 0);
        for (int j = 0; j < grid_height_; j++)
        {
            for (int i = 0; i < grid_width_; i++)
            {
                double world_x = grid_origin_x_ + (i + 0.5) * grid_resolution_;
                double world_y = grid_origin_y_ + (j + 0.5) * grid_resolution_;
                int cell_x = static_cast<int>(std::floor((world_x - msg->info.origin.position.x) / msg->info.resolution));
                int cell_y = static_cast<int>(std::floor((world_y - msg->info.origin.position.y) / msg->info.resolution));
                int idx = j * grid_width_ + i;
                if (cell_x >= 0 && cell_x < static_cast<int>(msg->info.width) &&
                    cell_y >= 0 && cell_y < static_cast<int>(msg->info.height))
                {
                    int msg_index = cell_y * msg->info.width + cell_x;
                    int8_t val = msg->data[msg_index];
                    // Preserve obstacles (100), treat everything else (0 or -1) as unknown for planning.
                    if (val == 100)
                        occupancy_grid_ground_[idx] = 100;
                    else
                        occupancy_grid_ground_[idx] = -1;
                }
                else
                {
                    occupancy_grid_ground_[idx] = -1; // unknown
                }
            }
        }
        occupancy_grid_received_ground_ = true;
        RCLCPP_INFO(this->get_logger(), "Ground occupancy grid updated for planning.");
    }

    // Drone occupancy grid callback (used for drone planning).
    void occupancy_grid_drone_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (occupancy_grid_received_drone_)
        {
            RCLCPP_INFO(this->get_logger(), "Drone occupancy grid already received; ignoring new message.");
            return;
        }
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty drone occupancy grid; waiting for valid data.");
            return;
        }

        // --- Store original drone map info for publishing ---
        original_width_drone_ = msg->info.width;
        original_height_drone_ = msg->info.height;
        original_origin_x_drone_ = msg->info.origin.position.x;
        original_origin_y_drone_ = msg->info.origin.position.y;
        original_ogm_drone_ = msg->data;

        // --- Use the message's resolution for both planning and publishing ---
        grid_resolution_ = msg->info.resolution;

        // --- Compute planning grid parameters based on square_size_ and the original map center ---
        double map_center_x = original_origin_x_drone_ + (original_width_drone_ * grid_resolution_) / 2.0;
        double map_center_y = original_origin_y_drone_ + (original_height_drone_ * grid_resolution_) / 2.0;
        grid_width_ = static_cast<int>(square_size_ / grid_resolution_);
        grid_height_ = static_cast<int>(square_size_ / grid_resolution_);
        grid_origin_x_ = map_center_x - square_size_ / 2.0;
        grid_origin_y_ = map_center_y - square_size_ / 2.0;

        // --- Build the planning grid from the original drone OGM ---
        occupancy_grid_drone_.resize(grid_width_ * grid_height_, 0);
        for (int j = 0; j < grid_height_; j++)
        {
            for (int i = 0; i < grid_width_; i++)
            {
                double world_x = grid_origin_x_ + (i + 0.5) * grid_resolution_;
                double world_y = grid_origin_y_ + (j + 0.5) * grid_resolution_;
                int cell_x = static_cast<int>(std::floor((world_x - msg->info.origin.position.x) / msg->info.resolution));
                int cell_y = static_cast<int>(std::floor((world_y - msg->info.origin.position.y) / msg->info.resolution));
                int idx = j * grid_width_ + i;
                if (cell_x >= 0 && cell_x < static_cast<int>(msg->info.width) &&
                    cell_y >= 0 && cell_y < static_cast<int>(msg->info.height))
                {
                    int msg_index = cell_y * msg->info.width + cell_x;
                    int8_t val = msg->data[msg_index];
                    // Preserve obstacles (100), treat everything else (0 or -1) as unknown for planning.
                    if (val == 100)
                        occupancy_grid_drone_[idx] = 100;
                    else
                        occupancy_grid_drone_[idx] = -1;
                }
                else
                {
                    occupancy_grid_drone_[idx] = -1; // Mark cell as unknown if out-of-bounds.
                }
            }
        }
        occupancy_grid_received_drone_ = true;
        RCLCPP_INFO(this->get_logger(), "Drone occupancy grid updated for planning.");
    }

    // -------------------- Bresenham Algorithm --------------------
    std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1)
    {
        std::vector<std::pair<int, int>> cells;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            cells.push_back(std::make_pair(x0, y0));
            if (x0 == x1 && y0 == y1)
                break;
            int e2 = 2 * err;
            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }
        }
        return cells;
    }

    bool check_line_free_bresenham(const std::tuple<double, double, double, double> &p1,
                                   const std::tuple<double, double, double, double> &p2,
                                   double infl)
    {
        int x0 = static_cast<int>((std::get<0>(p1) - grid_origin_x_) / grid_resolution_);
        int y0 = static_cast<int>((std::get<1>(p1) - grid_origin_y_) / grid_resolution_);
        int x1 = static_cast<int>((std::get<0>(p2) - grid_origin_x_) / grid_resolution_);
        int y1 = static_cast<int>((std::get<1>(p2) - grid_origin_y_) / grid_resolution_);
        auto line_cells = bresenham(x0, y0, x1, y1);
        double effective_infl = infl + grid_resolution_ / 2.0;
        int infl_cells = static_cast<int>(std::ceil(effective_infl / grid_resolution_));
        for (auto cell : line_cells)
        {
            int cx = cell.first, cy = cell.second;
            for (int dx = -infl_cells; dx <= infl_cells; ++dx)
            {
                for (int dy = -infl_cells; dy <= infl_cells; ++dy)
                {
                    double dist = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
                    if (dist > effective_infl)
                    {
                        continue;
                    }
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                    {
                        continue;
                    }

                    int cell_val = dynamic_occupancy_grid_[ny * grid_width_ + nx];
                    if (cell_val == 100 || cell_val == 50)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    bool check_line_free_bresenham(const std::tuple<double, double, double, double> &p1,
                                   const std::tuple<double, double, double, double> &p2)
    {
        return check_line_free_bresenham(p1, p2, inflation_radius_);
    }

    // -------------------- A* Algorithm Implementation --------------------
    std::vector<std::pair<int, int>> a_star(int start_x, int start_y, int goal_x, int goal_y)
    {
        struct PQItem
        {
            double f;
            int x;
            int y;
        };
        struct cmp
        {
            bool operator()(const PQItem &a, const PQItem &b) { return a.f > b.f; }
        };
        std::priority_queue<PQItem, std::vector<PQItem>, cmp> open;
        auto index = [this](int x, int y)
        { return y * grid_width_ + x; };
        const double INF = 1e9;
        std::vector<bool> closed(grid_width_ * grid_height_, false);
        std::vector<double> g_cost(grid_width_ * grid_height_, INF);
        std::vector<int> parent_x(grid_width_ * grid_height_, -1);
        std::vector<int> parent_y(grid_width_ * grid_height_, -1);
        auto heuristic = [goal_x, goal_y](int x, int y) -> double
        {
            return std::sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y));
        };
        int start_idx = index(start_x, start_y);
        g_cost[start_idx] = 0.0;
        open.push({heuristic(start_x, start_y), start_x, start_y});
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        bool found = false;
        while (!open.empty())
        {
            auto current = open.top();
            open.pop();
            int cx = current.x, cy = current.y;
            int c_idx = index(cx, cy);
            if (closed[c_idx])
            {
                continue;
            }
            closed[c_idx] = true;
            if (cx == goal_x && cy == goal_y)
            {
                found = true;
                break;
            }
            for (auto d : directions)
            {
                int nx = cx + d.first, ny = cy + d.second;
                if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                {
                    continue;
                }
                int n_idx = index(nx, ny);
                if (dynamic_occupancy_grid_[n_idx] == 100 || dynamic_occupancy_grid_[n_idx] == 50)
                {
                    continue;
                }
                if (closed[n_idx])
                {
                    continue;
                }
                double step_cost = (d.first != 0 && d.second != 0) ? std::sqrt(2.0) : 1.0;
                double tentative_g = g_cost[c_idx] + step_cost;
                if (tentative_g < g_cost[n_idx])
                {
                    g_cost[n_idx] = tentative_g;
                    parent_x[n_idx] = cx;
                    parent_y[n_idx] = cy;
                    double f = tentative_g + heuristic(nx, ny);
                    open.push({f, nx, ny});
                }
            }
        }
        std::vector<std::pair<int, int>> path;
        if (!found)
            return path;
        int cx = goal_x, cy = goal_y;
        while (!(cx == start_x && cy == start_y))
        {
            path.push_back({cx, cy});
            int idx_val = index(cx, cy);
            int px = parent_x[idx_val];
            int py = parent_y[idx_val];
            cx = px;
            cy = py;
        }
        path.push_back({start_x, start_y});
        std::reverse(path.begin(), path.end());
        return path;
    }

    // -------------------- Trajectory Planning --------------------
    // Returns a pair: first element is the "dense" trajectory used for updating the occupancy grid,
    // and second element is the "sparse" trajectory for publishing and saving.
    std::pair<std::vector<std::tuple<double, double, double, double>>,
              std::vector<std::tuple<double, double, double, double>>>
    plan_trajectory()
    {
        using TrajVec = std::vector<std::tuple<double, double, double, double>>;
        TrajVec dense_traj, sparse_traj;
        // Add initial state.
        dense_traj.push_back(std::make_tuple(start_x_, start_y_, start_z_, 0.0));
        sparse_traj.push_back(std::make_tuple(start_x_, start_y_, start_z_, 0.0));
        double curr_x = start_x_, curr_y = start_y_, curr_time = 0.0, curr_theta = start_yaw_;

        // Lambda to compute time increment along a segment.
        auto compute_dt = [this](double x0, double y0,
                                 double x1, double y1,
                                 double current_theta) -> double
        {
            // 1) Linear distance
            double dx = x1 - x0;
            double dy = y1 - y0;
            double distance = std::sqrt(dx * dx + dy * dy);

            // 2) Trapezoidal motion profile: accel → coast → decel
            double v0 = 0.0; // assume start‐at‐rest each segment
            double vmax = max_linear_velocity_;
            double a = max_linear_acceleration_;

            // time to accelerate to vmax
            double t_acc = (vmax - v0) / a;
            double d_acc = 0.5 * (v0 + vmax) * t_acc;
            double t_dec = t_acc; // symmetric decel back to stop
            double d_dec = d_acc;

            double d_remain = distance - (d_acc + d_dec);
            double t_coast = d_remain > 0.0 ? d_remain / vmax : 0.0;

            double t_linear;
            if (d_remain > 0.0)
            {
                // can reach vmax
                t_linear = t_acc + t_coast + t_dec;
            }
            else
            {
                // too short to hit vmax → triangular profile
                t_linear = 2.0 * std::sqrt(distance / a);
            }

            // 3) Yaw‐change time (kept proportional to the linear time as before)
            double new_heading = std::atan2(dy, dx);
            double dtheta = std::fabs(new_heading - current_theta);
            if (dtheta > M_PI)
                dtheta = 2.0 * M_PI - dtheta;
            double t_angular = (dtheta / max_turn_angle_rad_) * t_linear;

            return t_linear + t_angular;
        };

        // Lambda: plan a segment from current state to a given goal.
        auto plan_segment =
            [this, &curr_x, &curr_y, &curr_time, &curr_theta, &compute_dt](double goal_x, double goal_y, double goal_z) -> std::pair<TrajVec, TrajVec>
        {
            if (!use_k_rrt_for_checkpoints_)
            {
                // A* based planning segment.
                TrajVec seg_dense, seg_sparse;
                seg_dense.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));
                seg_sparse.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));

                // Build a raw path using A*.
                std::vector<std::pair<double, double>> raw_path;
                raw_path.push_back({curr_x, curr_y});
                int start_cell_x = static_cast<int>(std::floor((curr_x - grid_origin_x_) / grid_resolution_));
                int start_cell_y = static_cast<int>(std::floor((curr_y - grid_origin_y_) / grid_resolution_));
                int goal_cell_x = static_cast<int>(std::floor((goal_x - grid_origin_x_) / grid_resolution_));
                int goal_cell_y = static_cast<int>(std::floor((goal_y - grid_origin_y_) / grid_resolution_));

                double effective_inflation = inflation_radius_;
                if (robot_name_.find("Drone") != std::string::npos && !use_k_rrt_for_checkpoints_)
                {
                    effective_inflation = uav_astar_inflation_radius_;
                }
                std::vector<int8_t> inflated_grid = inflate_obstacles(dynamic_occupancy_grid_, effective_inflation);

                std::swap(dynamic_occupancy_grid_, inflated_grid);
                auto path_cells = a_star(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y);
                std::swap(dynamic_occupancy_grid_, inflated_grid); // restore original grid

                for (size_t i = 1; i < path_cells.size(); ++i)
                {
                    int cell_x = path_cells[i].first;
                    int cell_y = path_cells[i].second;
                    double wx = grid_origin_x_ + (cell_x + 0.5) * grid_resolution_;
                    double wy = grid_origin_y_ + (cell_y + 0.5) * grid_resolution_;
                    raw_path.push_back({wx, wy});
                }

                // Smooth the raw path.
                auto smooth_path_result = smoothPath(raw_path, 50, 0.1);
                double turn_threshold = 1.0;
                auto refined_path = smooth_path_result;
                for (int iter = 0; iter < 5; ++iter)
                {
                    refined_path = refineSharpTurns(refined_path, turn_threshold);
                }

                // ───────────────────────────────────────────────────────────────────
                // Inserted block: clamp the very first turn so it never exceeds max_turn_angle_rad_.
                if (refined_path.size() > 1)
                {
                    // Current robot position:
                    double x0 = curr_x;
                    double y0 = curr_y;
                    // First waypoint from the refined path:
                    double x1 = refined_path[1].first;
                    double y1 = refined_path[1].second;

                    // Angle from (x0,y0) → (x1,y1):
                    double phi = std::atan2(y1 - y0, x1 - x0);

                    // Compute Δθ = phi − curr_theta, wrapped into [−π, +π]:
                    double dtheta = phi - curr_theta;
                    while (dtheta > M_PI)
                        dtheta -= 2.0 * M_PI;
                    while (dtheta < -M_PI)
                        dtheta += 2.0 * M_PI;

                    // If the initial turn exceeds max_turn_angle_rad_, clamp it:
                    if (std::fabs(dtheta) > max_turn_angle_rad_)
                    {
                        // Allow only ±max_turn_angle_rad_ at t = 0
                        double clamped_theta = curr_theta + ((dtheta > 0) ? max_turn_angle_rad_ : -max_turn_angle_rad_);

                        // Keep the same straight‐line distance but rotated to clamped_theta:
                        double dist01 = std::hypot(x1 - x0, y1 - y0);
                        double new_x1 = x0 + dist01 * std::cos(clamped_theta);
                        double new_y1 = y0 + dist01 * std::sin(clamped_theta);

                        // Overwrite the first waypoint in refined_path:
                        refined_path[1].first = new_x1;
                        refined_path[1].second = new_y1;
                    }
                }
                // ───────────────────────────────────────────────────────────────────

                // Build dense segment using (possibly) “clamped” refined_path:
                {
                    double prev_x = refined_path.front().first;
                    double prev_y = refined_path.front().second;
                    double time_acc = curr_time;
                    double local_theta = curr_theta;

                    for (size_t i = 1; i < refined_path.size(); ++i)
                    {
                        double wx = refined_path[i].first;
                        double wy = refined_path[i].second;
                        double dt_val = compute_dt(prev_x, prev_y, wx, wy, local_theta);
                        time_acc += dt_val;
                        seg_dense.push_back(std::make_tuple(wx, wy, start_z_, time_acc));
                        local_theta = std::atan2(wy - prev_y, wx - prev_x);
                        prev_x = wx;
                        prev_y = wy;
                    }
                }

                // Build sparse segment by sparsification (unchanged):
                {
                    auto sparsified_path = sparsifyPath(refined_path, 0.05);
                    double prev_x = sparsified_path.front().first;
                    double prev_y = sparsified_path.front().second;
                    double time_acc = curr_time;
                    double local_theta = curr_theta;

                    for (size_t i = 1; i < sparsified_path.size(); ++i)
                    {
                        double wx = sparsified_path[i].first;
                        double wy = sparsified_path[i].second;
                        double dt_val = compute_dt(prev_x, prev_y, wx, wy, local_theta);
                        time_acc += dt_val;
                        seg_sparse.push_back(std::make_tuple(wx, wy, start_z_, time_acc));
                        local_theta = std::atan2(wy - prev_y, wx - prev_x);
                        prev_x = wx;
                        prev_y = wy;
                    }
                }

                return std::make_pair(seg_dense, seg_sparse);
            }
            else
            {
                // Kinodynamic-RRT based planning segment (unchanged).
                using TrajVec = std::vector<std::tuple<double, double, double, double>>;
                struct Node
                {
                    double x, y, theta, time, cost;
                    int parent;
                };
                std::vector<Node> tree;
                Node root = {curr_x, curr_y, curr_theta, curr_time, 0.0, -1};
                tree.push_back(root);
                int max_iterations = 100000;
                bool reached = false;
                int goal_index = -1;
                double dt_val = 2.0;
                double v = max_linear_velocity_;
                double goal_threshold = 1.0;

                for (int iter = 0; iter < max_iterations; iter++)
                {
                    double sample_choice = std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
                    double x_rand, y_rand;
                    double bias_probability = 0.2;
                    if (sample_choice < bias_probability)
                    {
                        x_rand = goal_x;
                        y_rand = goal_y;
                    }
                    else
                    {
                        x_rand = x_dist_(rng_);
                        y_rand = y_dist_(rng_);
                    }
                    // Find nearest node…
                    int nearest_index = 0;
                    double min_dist = std::numeric_limits<double>::max();
                    for (int i = 0; i < tree.size(); i++)
                    {
                        double dx = tree[i].x - x_rand;
                        double dy = tree[i].y - y_rand;
                        double dist = std::hypot(dx, dy);
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            nearest_index = i;
                        }
                    }
                    Node nearest = tree[nearest_index];
                    double theta_des = std::atan2(y_rand - nearest.y, x_rand - nearest.x);
                    double dtheta = theta_des - nearest.theta;
                    while (dtheta > M_PI)
                        dtheta -= 2.0 * M_PI;
                    while (dtheta < -M_PI)
                        dtheta += 2.0 * M_PI;
                    double new_theta = nearest.theta + dtheta;
                    double step = v * dt_val;
                    double new_x = nearest.x + step * std::cos(new_theta);
                    double new_y = nearest.y + step * std::sin(new_theta);
                    double new_time = nearest.time + dt_val;
                    double distance_cost = nearest.cost + step;

                    if (new_x < grid_origin_x_ || new_x > grid_origin_x_ + square_size_ ||
                        new_y < grid_origin_y_ || new_y > grid_origin_y_ + square_size_)
                    {
                        continue;
                    }
                    if (!check_line_free_bresenham(
                            std::make_tuple(nearest.x, nearest.y, start_z_, 0.0),
                            std::make_tuple(new_x, new_y, start_z_, 0.0),
                            inflation_radius_))
                    {
                        continue;
                    }
                    Node new_node = {new_x, new_y, new_theta, new_time, distance_cost, nearest_index};
                    tree.push_back(new_node);

                    double dx_goal = new_x - goal_x;
                    double dy_goal = new_y - goal_y;
                    if (std::hypot(dx_goal, dy_goal) < goal_threshold)
                    {
                        reached = true;
                        goal_index = tree.size() - 1;
                        break;
                    }
                }

                TrajVec seg_dense_rrt, seg_sparse_rrt;
                if (!reached)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Kinodynamic RRT for checkpoint failed after %d iterations", max_iterations);
                    seg_dense_rrt.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));
                    seg_sparse_rrt.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));
                    return std::make_pair(seg_dense_rrt, seg_sparse_rrt);
                }

                // Reconstruct path…
                std::vector<Node> path;
                int idx = goal_index;
                while (idx != -1)
                {
                    path.push_back(tree[idx]);
                    idx = tree[idx].parent;
                }
                std::reverse(path.begin(), path.end());
                for (const auto &node : path)
                {
                    seg_dense_rrt.push_back(std::make_tuple(node.x, node.y, start_z_, node.time));
                }
                // Create sparse version by sparsification…
                std::vector<std::pair<double, double>> densePoints;
                for (const auto &pt : seg_dense_rrt)
                {
                    densePoints.push_back({std::get<0>(pt), std::get<1>(pt)});
                }
                auto sparsified_points = sparsifyPath(densePoints, 0.05);
                double prev_x = sparsified_points.front().first;
                double prev_y = sparsified_points.front().second;
                double time_acc = std::get<3>(seg_dense_rrt.front());
                double local_theta = curr_theta;
                seg_sparse_rrt.push_back(std::make_tuple(prev_x, prev_y, start_z_, time_acc));
                for (size_t i = 1; i < sparsified_points.size(); i++)
                {
                    double wx = sparsified_points[i].first;
                    double wy = sparsified_points[i].second;
                    double dt_new = compute_dt(prev_x, prev_y, wx, wy, local_theta);
                    time_acc += dt_new;
                    seg_sparse_rrt.push_back(std::make_tuple(wx, wy, start_z_, time_acc));
                    local_theta = std::atan2(wy - prev_y, wx - prev_x);
                    prev_x = wx;
                    prev_y = wy;
                }

                return std::make_pair(seg_dense_rrt, seg_sparse_rrt);
            }
        }; // end of plan_segment lambda

        // Lambda: plan a random segment for additional trajectory if needed.
        auto plan_random_segment = [this, &curr_x, &curr_y, &curr_time, &curr_theta, &compute_dt](double remaining_length) -> std::pair<TrajVec, TrajVec>
        {
            struct Node
            {
                double x, y, theta, time, cost;
                int parent;
            };
            std::vector<Node> tree;
            Node root = {curr_x, curr_y, curr_theta, curr_time, 0.0, -1};
            tree.push_back(root);
            int max_iterations = 100000;
            bool reached = false;
            int goal_index = -1;
            double dt_val = 2.0;
            double v = max_linear_velocity_;

            // Precompute unknown cell centers.
            std::vector<std::pair<double, double>> unknown_cells;
            for (int j = 0; j < grid_height_; j++)
            {
                for (int i = 0; i < grid_width_; i++)
                {
                    int idx = j * grid_width_ + i;
                    if (dynamic_occupancy_grid_[idx] == -1)
                    {
                        double wx = grid_origin_x_ + (i + 0.5) * grid_resolution_;
                        double wy = grid_origin_y_ + (j + 0.5) * grid_resolution_;
                        unknown_cells.push_back({wx, wy});
                    }
                }
            }

            for (int iter = 0; iter < max_iterations; iter++)
            {
                double bias_probability = 0.7;
                double sample_choice = std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
                double x_rand, y_rand;
                if (sample_choice < bias_probability && !unknown_cells.empty())
                {
                    int rand_idx = std::uniform_int_distribution<int>(0, unknown_cells.size() - 1)(rng_);
                    x_rand = unknown_cells[rand_idx].first;
                    y_rand = unknown_cells[rand_idx].second;
                }
                else
                {
                    x_rand = x_dist_(rng_);
                    y_rand = y_dist_(rng_);
                }

                int nearest_index = 0;
                double min_dist = std::numeric_limits<double>::max();
                for (int i = 0; i < tree.size(); i++)
                {
                    double dx = tree[i].x - x_rand;
                    double dy = tree[i].y - y_rand;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        nearest_index = i;
                    }
                }
                Node nearest = tree[nearest_index];
                double theta_des = std::atan2(y_rand - nearest.y, x_rand - nearest.x);
                double dtheta = theta_des - nearest.theta;
                while (dtheta > M_PI)
                    dtheta -= 2 * M_PI;
                while (dtheta < -M_PI)
                    dtheta += 2 * M_PI;
                // Here we use the turning limits for random segment planning.
                if (dtheta > max_turn_angle_rad_)
                    dtheta = max_turn_angle_rad_;
                if (dtheta < -max_turn_angle_rad_)
                    dtheta = -max_turn_angle_rad_;
                double new_theta = nearest.theta + dtheta;
                double step = v * dt_val;
                double new_x = nearest.x + step * std::cos(new_theta);
                double new_y = nearest.y + step * std::sin(new_theta);
                double new_time = nearest.time + dt_val;
                double distance_cost = nearest.cost + step;
                if (new_x < grid_origin_x_ || new_x > grid_origin_x_ + square_size_ ||
                    new_y < grid_origin_y_ || new_y > grid_origin_y_ + square_size_)
                {
                    continue;
                }
                if (!check_line_free_bresenham(std::make_tuple(nearest.x, nearest.y, start_z_, 0.0),
                                               std::make_tuple(new_x, new_y, start_z_, 0.0), inflation_radius_))
                {
                    continue;
                }
                Node new_node = {new_x, new_y, new_theta, new_time, distance_cost, nearest_index};
                tree.push_back(new_node);
                if (distance_cost >= remaining_length)
                {
                    reached = true;
                    goal_index = tree.size() - 1;
                    break;
                }
            }

            TrajVec dense_rand, sparse_rand;
            if (!reached && tree.size() > 1)
            {
                // Keep the longest branch rather than nothing.
                goal_index = static_cast<int>(std::max_element(tree.begin(), tree.end(), [](const Node &a, const Node &b)
                                                               { return a.cost < b.cost; }) -
                                              tree.begin());
                RCLCPP_WARN(this->get_logger(), "Kinodynamic RRT reached %.1f of %.1f m after %d iterations; using that branch",
                            tree[goal_index].cost, remaining_length, max_iterations);
                reached = true;
            }
            if (!reached)
            {
                RCLCPP_WARN(this->get_logger(), "Kinodynamic RRT found no free step from (%.1f, %.1f)", curr_x, curr_y);
                dense_rand.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));
                sparse_rand.push_back(std::make_tuple(curr_x, curr_y, start_z_, curr_time));
                return std::make_pair(dense_rand, sparse_rand);
            }
            std::vector<Node> path;
            int idx = goal_index;
            while (idx != -1)
            {
                path.push_back(tree[idx]);
                idx = tree[idx].parent;
            }
            std::reverse(path.begin(), path.end());
            for (const auto &node : path)
            {
                dense_rand.push_back(std::make_tuple(node.x, node.y, start_z_, node.time));
            }
            std::vector<std::pair<double, double>> densePoints;
            for (const auto &pt : dense_rand)
            {
                densePoints.push_back({std::get<0>(pt), std::get<1>(pt)});
            }
            auto sparsified_points = sparsifyPath(densePoints, 0.05);
            double prev_x = sparsified_points.front().first;
            double prev_y = sparsified_points.front().second;
            double time_acc = std::get<3>(dense_rand.front());
            double local_theta = curr_theta;
            sparse_rand.push_back(std::make_tuple(prev_x, prev_y, start_z_, time_acc));
            for (size_t i = 1; i < sparsified_points.size(); i++)
            {
                double wx = sparsified_points[i].first;
                double wy = sparsified_points[i].second;
                double dt_new = compute_dt(prev_x, prev_y, wx, wy, local_theta);
                time_acc += dt_new;
                sparse_rand.push_back(std::make_tuple(wx, wy, start_z_, time_acc));
                local_theta = std::atan2(wy - prev_y, wx - prev_x);
                prev_x = wx;
                prev_y = wy;
            }
            return std::make_pair(dense_rand, sparse_rand);
        };

        // Now build the overall trajectory.
        TrajVec dense_full_traj = dense_traj;
        TrajVec sparse_full_traj = sparse_traj;

        // Branch based on the flag: if true, plan checkpoint segments first.
        if (current_reach_checkpoints_first_ || provided_waypoints_.empty())
        {
            for (const auto &pt : provided_waypoints_)
            {
                double goal_x = std::get<0>(pt);
                double goal_y = std::get<1>(pt);
                double goal_z = std::get<2>(pt);
                auto seg_pair = plan_segment(goal_x, goal_y, goal_z);
                // Append segments (avoiding duplicate starting state).
                dense_full_traj.insert(dense_full_traj.end(), seg_pair.first.begin() + 1, seg_pair.first.end());
                sparse_full_traj.insert(sparse_full_traj.end(), seg_pair.second.begin() + 1, seg_pair.second.end());
                curr_x = goal_x;
                curr_y = goal_y;
                curr_time = std::get<3>(seg_pair.second.back());
                if (seg_pair.second.size() >= 2)
                {
                    double prev_x = std::get<0>(seg_pair.second[seg_pair.second.size() - 2]);
                    double prev_y = std::get<1>(seg_pair.second[seg_pair.second.size() - 2]);
                    curr_theta = std::atan2(goal_y - prev_y, goal_x - prev_x);
                }
            }
        }
        else
        {
            // Plan only the random segment with the specified trajectory_length_
            auto rand_seg_pair = plan_random_segment(trajectory_length_);
            dense_full_traj = rand_seg_pair.first;
            sparse_full_traj = rand_seg_pair.second;

            // Update current state to end of random segment
            curr_x = std::get<0>(rand_seg_pair.second.back());
            curr_y = std::get<1>(rand_seg_pair.second.back());
            curr_time = std::get<3>(rand_seg_pair.second.back());
            if (rand_seg_pair.second.size() >= 2)
            {
                double prev_x = std::get<0>(rand_seg_pair.second[rand_seg_pair.second.size() - 2]);
                double prev_y = std::get<1>(rand_seg_pair.second[rand_seg_pair.second.size() - 2]);
                curr_theta = std::atan2(curr_y - prev_y, curr_x - prev_x);
            }

            // Plan checkpoint segments sequentially (length-unbounded)
            for (const auto &pt : provided_waypoints_)
            {
                double goal_x = std::get<0>(pt);
                double goal_y = std::get<1>(pt);
                double goal_z = std::get<2>(pt);
                auto seg_pair = plan_segment(goal_x, goal_y, goal_z);
                dense_full_traj.insert(dense_full_traj.end(), seg_pair.first.begin() + 1, seg_pair.first.end());
                sparse_full_traj.insert(sparse_full_traj.end(), seg_pair.second.begin() + 1, seg_pair.second.end());
                curr_x = goal_x;
                curr_y = goal_y;
                curr_time = std::get<3>(seg_pair.second.back());
                if (seg_pair.second.size() >= 2)
                {
                    double prev_x = std::get<0>(seg_pair.second[seg_pair.second.size() - 2]);
                    double prev_y = std::get<1>(seg_pair.second[seg_pair.second.size() - 2]);
                    curr_theta = std::atan2(goal_y - prev_y, goal_x - prev_x);
                }
            }
        }

        // If the total planned length is less than desired, add an extra random segment.
        double total_length = 0.0;
        for (size_t i = 1; i < sparse_full_traj.size(); i++)
        {
            double dx = std::get<0>(sparse_full_traj[i]) - std::get<0>(sparse_full_traj[i - 1]);
            double dy = std::get<1>(sparse_full_traj[i]) - std::get<1>(sparse_full_traj[i - 1]);
            total_length += std::sqrt(dx * dx + dy * dy);
        }
        // An RRT branch seldom gets much longer than the planning area is wide, so cover the
        // remaining length with a chain of segments of at most half the area's side.
        const double max_segment_length = std::max(square_size_ / 2.0, 1.0);
        struct SegmentStart
        {
            size_t dense_size, sparse_size;
            double length, x, y, time, theta;
        };
        std::vector<SegmentStart> segment_starts;
        int dead_end_retries = 5;
        while (total_length < trajectory_length_ - 1.0)
        {
            double remaining_length = std::min(trajectory_length_ - total_length, max_segment_length);
            auto extra_seg = plan_random_segment(remaining_length);
            if (extra_seg.second.size() < 2)
            {
                // Dead end: the turn limit allows no free step from here. Plan the previous
                // segment again, which most likely ends elsewhere.
                if (segment_starts.empty() || dead_end_retries-- <= 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s: stuck after %.1f of %.1f m; its trajectory ends there",
                                 robot_name_.c_str(), total_length, trajectory_length_);
                    break;
                }
                const SegmentStart previous = segment_starts.back();
                segment_starts.pop_back();
                RCLCPP_WARN(this->get_logger(), "Dead end at (%.1f, %.1f); planning the previous segment again", curr_x, curr_y);
                dense_full_traj.resize(previous.dense_size);
                sparse_full_traj.resize(previous.sparse_size);
                total_length = previous.length;
                curr_x = previous.x;
                curr_y = previous.y;
                curr_time = previous.time;
                curr_theta = previous.theta;
                continue;
            }
            segment_starts.push_back({dense_full_traj.size(), sparse_full_traj.size(), total_length, curr_x, curr_y, curr_time, curr_theta});
            dense_full_traj.insert(dense_full_traj.end(), extra_seg.first.begin() + 1, extra_seg.first.end());
            sparse_full_traj.insert(sparse_full_traj.end(), extra_seg.second.begin() + 1, extra_seg.second.end());
            for (size_t i = 1; i < extra_seg.second.size(); i++)
            {
                total_length += std::hypot(std::get<0>(extra_seg.second[i]) - std::get<0>(extra_seg.second[i - 1]),
                                           std::get<1>(extra_seg.second[i]) - std::get<1>(extra_seg.second[i - 1]));
            }
            // Continue from the segment's end, steering the next one towards unexplored cells.
            const auto &last = extra_seg.second.back();
            const auto &before_last = extra_seg.second[extra_seg.second.size() - 2];
            curr_x = std::get<0>(last);
            curr_y = std::get<1>(last);
            curr_time = std::get<3>(last);
            curr_theta = std::atan2(curr_y - std::get<1>(before_last), curr_x - std::get<0>(before_last));
            update_dynamic_occupancy_grid(extra_seg.first, dynamic_occupancy_grid_);
        }

        // -------------------- Apply planning horizon in discrete steps (if enabled) --------------------
        // planning_horizon_steps_ is interpreted as a limit on the number of *segments*
        // of the sparse trajectory. We therefore keep at most planning_horizon_steps_ + 1
        // sparse waypoints (including the initial state), and we truncate the dense
        // trajectory to have approximately the same *arc length*.
        if (planning_horizon_steps_ > 0)
        {
            int max_points = planning_horizon_steps_ + 1; // include start state
            if (static_cast<int>(sparse_full_traj.size()) > max_points)
            {
                // 1) Compute horizon length along the sparse trajectory.
                double horizon_length = 0.0;
                for (int i = 1; i < max_points; ++i)
                {
                    double dx = std::get<0>(sparse_full_traj[i]) - std::get<0>(sparse_full_traj[i - 1]);
                    double dy = std::get<1>(sparse_full_traj[i]) - std::get<1>(sparse_full_traj[i - 1]);
                    horizon_length += std::sqrt(dx * dx + dy * dy);
                }

                // 2) Truncate sparse trajectory to that many waypoints.
                sparse_full_traj.resize(static_cast<std::size_t>(max_points));

                // 3) Truncate dense trajectory so its arc length does not exceed horizon_length.
                std::vector<std::tuple<double, double, double, double>> dense_truncated;
                dense_truncated.reserve(dense_full_traj.size());
                dense_truncated.push_back(dense_full_traj.front());

                double dense_length = 0.0;
                const double eps = 1e-6;

                for (std::size_t i = 1; i < dense_full_traj.size(); ++i)
                {
                    double dx = std::get<0>(dense_full_traj[i]) - std::get<0>(dense_full_traj[i - 1]);
                    double dy = std::get<1>(dense_full_traj[i]) - std::get<1>(dense_full_traj[i - 1]);
                    double seg_len = std::sqrt(dx * dx + dy * dy);

                    if (dense_length + seg_len > horizon_length + eps)
                    {
                        break;
                    }

                    dense_length += seg_len;
                    dense_truncated.push_back(dense_full_traj[i]);
                }

                dense_full_traj.swap(dense_truncated);
            }
        }
        // -------------------------------------------------------------------------

        double final_length = 0.0;
        for (size_t i = 1; i < sparse_full_traj.size(); i++)
        {
            double dx = std::get<0>(sparse_full_traj[i]) - std::get<0>(sparse_full_traj[i - 1]);
            double dy = std::get<1>(sparse_full_traj[i]) - std::get<1>(sparse_full_traj[i - 1]);
            final_length += std::sqrt(dx * dx + dy * dy);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Planned trajectory length: %.2f m with %zu waypoints",
                    final_length, sparse_full_traj.size());
        return std::make_pair(dense_full_traj, sparse_full_traj);
    }

    // -------------------- Smoothing, Refinement, and Sparsification --------------------
    std::vector<std::pair<double, double>> smoothPath(const std::vector<std::pair<double, double>> &path,
                                                      int iterations = 50, double alpha = 0.1)
    {
        std::vector<std::pair<double, double>> smoothed = path;
        for (int iter = 0; iter < iterations; ++iter)
        {
            for (size_t i = 1; i < smoothed.size() - 1; ++i)
            {
                double new_x = smoothed[i].first + alpha * (smoothed[i - 1].first + smoothed[i + 1].first - 2.0 * smoothed[i].first);
                double new_y = smoothed[i].second + alpha * (smoothed[i - 1].second + smoothed[i + 1].second - 2.0 * smoothed[i].second);
                smoothed[i] = {new_x, new_y};
            }
        }
        return smoothed;
    }
    std::vector<std::pair<double, double>> refineSharpTurns(const std::vector<std::pair<double, double>> &path,
                                                            double turn_threshold)
    {
        std::vector<std::pair<double, double>> refined = path;
        for (size_t i = 1; i < refined.size() - 1; ++i)
        {
            double dx1 = refined[i].first - refined[i - 1].first;
            double dy1 = refined[i].second - refined[i - 1].second;
            double dx2 = refined[i + 1].first - refined[i].first;
            double dy2 = refined[i + 1].second - refined[i].second;
            double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (mag1 < 1e-6 || mag2 < 1e-6)
            {
                continue;
            }
            double dot = dx1 * dx2 + dy1 * dy2;
            double angle = std::acos(std::clamp(dot / (mag1 * mag2), -1.0, 1.0));
            if (angle > turn_threshold)
            {
                refined[i].first = (refined[i - 1].first + refined[i + 1].first) / 2.0;
                refined[i].second = (refined[i - 1].second + refined[i + 1].second) / 2.0;
            }
        }
        return refined;
    }
    std::vector<std::pair<double, double>> sparsifyPath(const std::vector<std::pair<double, double>> &path, double angle_threshold = 0.05)
    {
        if (path.size() < 3)
            return path;
        std::vector<std::pair<double, double>> sparsified;
        sparsified.push_back(path.front());
        for (size_t i = 1; i < path.size() - 1; ++i)
        {
            const auto &prev = sparsified.back();
            const auto &curr = path[i];
            const auto &next = path[i + 1];
            double dx1 = curr.first - prev.first;
            double dy1 = curr.second - prev.second;
            double dx2 = next.first - curr.first;
            double dy2 = next.second - curr.second;
            double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (mag1 < 1e-6 || mag2 < 1e-6)
            {
                sparsified.push_back(curr);
                continue;
            }
            double dot = dx1 * dx2 + dy1 * dy2;
            double angle = std::acos(std::clamp(dot / (mag1 * mag2), -1.0, 1.0));
            if (angle > angle_threshold)
                sparsified.push_back(curr);
        }
        sparsified.push_back(path.back());
        return sparsified;
    }

    // -------------------- Occupancy Grid Update --------------------
    // This function updates dynamic_occupancy_grid_ by inflating the area around each waypoint in the trajectory.
    // (For UGV planning, these inflated cells are obstacles; for drone planning they mark the drone's own path.)
    void update_dynamic_occupancy_grid(
        const std::vector<std::tuple<double, double, double, double>> &traj,
        std::vector<int8_t> &grid)
    {
        // Use the exploration radius parameter (set via "trajectory_exploration_radius")
        double radius = trajectory_exploration_radius_;
        if (radius <= 0.0)
            return;

        // Determine how many cells this radius corresponds to.
        int cell_radius = static_cast<int>(std::ceil(radius / grid_resolution_));

        // For each waypoint in the trajectory...
        for (const auto &pt : traj)
        {
            // Get the waypoint coordinates.
            double wx = std::get<0>(pt);
            double wy = std::get<1>(pt);
            // Convert world coordinates to grid indices.
            int cx = static_cast<int>(std::floor((wx - grid_origin_x_) / grid_resolution_));
            int cy = static_cast<int>(std::floor((wy - grid_origin_y_) / grid_resolution_));

            // Loop over the neighboring cells within cell_radius.
            for (int dx = -cell_radius; dx <= cell_radius; ++dx)
            {
                for (int dy = -cell_radius; dy <= cell_radius; ++dy)
                {
                    int nx = cx + dx;
                    int ny = cy + dy;
                    // Make sure we’re inside the grid.
                    if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_)
                        continue;
                    // Compute the Euclidean distance from the cell center to the waypoint.
                    double dist = std::sqrt((dx * grid_resolution_) * (dx * grid_resolution_) +
                                            (dy * grid_resolution_) * (dy * grid_resolution_));
                    if (dist <= radius)
                    {
                        int index = ny * grid_width_ + nx;
                        // Only update cells that are still unknown (-1) to 0 (explored)
                        if (grid[index] == -1)
                        {
                            grid[index] = 0;
                        }
                    }
                }
            }
        }
    }

    // Publishes the passed–in occupancy grid.
    void publish_updated_occupancy_grid(const std::vector<int8_t> &planning_grid, bool isDrone = false)
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.info.resolution = grid_resolution_;

        int orig_width = 0;
        int orig_height = 0;
        double orig_origin_x = 0.0;
        double orig_origin_y = 0.0;
        std::vector<int8_t> full_grid;

        if (isDrone)
        {
            orig_width = original_width_drone_;
            orig_height = original_height_drone_;
            orig_origin_x = original_origin_x_drone_;
            orig_origin_y = original_origin_y_drone_;
            // Start with a grid that only preserves static obstacles from the original OGM.
            full_grid.assign(original_ogm_drone_.size(), -1);
            for (size_t i = 0; i < original_ogm_drone_.size(); ++i)
            {
                if (original_ogm_drone_[i] == 100)
                    full_grid[i] = 100;
            }
            msg.info.origin.position.z = drone_altitude_;
        }
        else
        {
            orig_width = original_width_ground_;
            orig_height = original_height_ground_;
            orig_origin_x = original_origin_x_ground_;
            orig_origin_y = original_origin_y_ground_;
            // Start with a grid that only preserves static obstacles from the original OGM.
            full_grid.assign(original_ogm_ground_.size(), -1);
            for (size_t i = 0; i < original_ogm_ground_.size(); ++i)
            {
                if (original_ogm_ground_[i] == 100)
                    full_grid[i] = 100;
            }
            msg.info.origin.position.z = 0.0;
        }

        msg.info.width = orig_width;
        msg.info.height = orig_height;
        msg.info.origin.position.x = orig_origin_x;
        msg.info.origin.position.y = orig_origin_y;

        // Determine the offset of the planning grid relative to the original grid.
        int offset_x = static_cast<int>((grid_origin_x_ - orig_origin_x) / grid_resolution_);
        int offset_y = static_cast<int>((grid_origin_y_ - orig_origin_y) / grid_resolution_);

        // Overlay the planning (dynamic) grid into the original grid.
        for (int j = 0; j < grid_height_; j++)
        {
            for (int i = 0; i < grid_width_; i++)
            {
                int orig_x = i + offset_x;
                int orig_y = j + offset_y;
                if (orig_x >= 0 && orig_x < orig_width &&
                    orig_y >= 0 && orig_y < orig_height)
                {
                    int orig_idx = orig_y * orig_width + orig_x;
                    int planning_idx = j * grid_width_ + i;
                    full_grid[orig_idx] = planning_grid[planning_idx];
                }
            }
        }
        msg.data = full_grid;

        if (isDrone)
        {
            updated_ogm_pub_drone_->publish(msg);
        }
        else
        {
            updated_ogm_pub_ground_->publish(msg);
        }
    }

    void publish_trajectory_for_robot(const std::string &robot,
                                      const std::vector<std::tuple<double, double, double, double>> &traj)
    {
        auto now = this->now();
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now;
        path_msg.header.frame_id = "map";

        for (const auto &pt : traj)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = std::get<0>(pt);
            pose.pose.position.y = std::get<1>(pt);
            // If the vehicle is a drone, override the z value with the drone altitude parameter.
            if (robot.find("Drone") != std::string::npos)
            {
                pose.pose.position.z = drone_altitude_; // drone_altitude_ is retrieved from parameter
            }
            else
            {
                pose.pose.position.z = std::get<2>(pt);
            }
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        if (trajectory_publishers_.find(robot) == trajectory_publishers_.end())
        {
            trajectory_publishers_[robot] = this->create_publisher<nav_msgs::msg::Path>(robot + "_trajectory", 10);
        }
        trajectory_publishers_[robot]->publish(path_msg);
    }

    void publish_checkpoints_for_robot(const std::string &robot,
                                       const std::vector<std::tuple<double, double, double>> &checkpoints)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = robot + "_checkpoints";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (const auto &pt : checkpoints)
        {
            geometry_msgs::msg::Point p;
            p.x = std::get<0>(pt);
            p.y = std::get<1>(pt);
            p.z = std::get<2>(pt);
            marker.points.push_back(p);
        }
        if (marker_publishers_.find(robot) == marker_publishers_.end())
        {
            marker_publishers_[robot] = this->create_publisher<visualization_msgs::msg::Marker>(robot + "_checkpoints", 10);
        }
        marker_publishers_[robot]->publish(marker);
    }

    // -------------------- Trajectory File Saving --------------------
    void save_trajectory_to_file(const std::vector<std::tuple<double, double, double, double>> &traj)
    {
        std::error_code mkdir_error;
        std::filesystem::create_directories(std::filesystem::path(output_file_path_).parent_path(), mkdir_error);
        std::ofstream ofs(output_file_path_);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", output_file_path_.c_str());
            return;
        }
        // start_x_/start_y_ hold the vehicle being saved; they are in the same swapped frame.
        const double off_x = trajectories_relative_to_start_ ? start_x_ : 0.0;
        const double off_y = trajectories_relative_to_start_ ? start_y_ : 0.0;
        for (const auto &pt : traj)
        {
            ofs << std::get<1>(pt) - off_y << " " // Y first
                << std::get<0>(pt) - off_x << " " // X second
                << std::get<2>(pt) << " "         // Z remains
                << std::get<3>(pt) << "\n";       // Timestamp
        }
        ofs.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s", output_file_path_.c_str());
    }

    // Computes the time increment to traverse from (x0, y0) to (x1, y1)
    // given the current heading (current_theta). It uses the member variables
    // max_linear_velocity_ and max_turn_angle_rad_.
    double compute_dt_segment(double x0, double y0,
                              double x1, double y1,
                              double current_theta)
    {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double distance = std::sqrt(dx * dx + dy * dy);

        double v0 = 0.0;
        double vmax = max_linear_velocity_;
        double a = max_linear_acceleration_;

        double t_acc = (vmax - v0) / a;
        double d_acc = 0.5 * (v0 + vmax) * t_acc;
        double t_dec = t_acc;
        double d_dec = d_acc;

        double d_remain = distance - (d_acc + d_dec);
        double t_coast = d_remain > 0.0 ? d_remain / vmax : 0.0;

        double t_linear;
        if (d_remain > 0.0)
        {
            t_linear = t_acc + t_coast + t_dec;
        }
        else
        {
            t_linear = 2.0 * std::sqrt(distance / a);
        }

        double new_heading = std::atan2(dy, dx);
        double dtheta = std::fabs(new_heading - current_theta);
        if (dtheta > M_PI)
            dtheta = 2.0 * M_PI - dtheta;
        double t_angular = (dtheta / max_turn_angle_rad_) * t_linear;

        return t_linear + t_angular;
    }

    // This function replicates the logic from plan_segment lambda,
    // but as a standalone member function. It returns a pair of trajectories:
    // the dense version (for updating the occupancy grid) and the sparse version
    // (for publishing/saving). The planning is done from the initial state provided
    // (init_x, init_y, init_z, init_time, init_theta) to the goal (goal_x, goal_y, goal_z).
    std::pair<std::vector<std::tuple<double, double, double, double>>,
              std::vector<std::tuple<double, double, double, double>>>
    plan_segment_to_goal(double init_x, double init_y, double init_z, double init_time, double init_theta,
                         double goal_x, double goal_y, double goal_z)
    {
        using TrajVec = std::vector<std::tuple<double, double, double, double>>;
        TrajVec seg_dense, seg_sparse;
        // Start with the initial state.
        seg_dense.push_back(std::make_tuple(init_x, init_y, init_z, init_time));
        seg_sparse.push_back(std::make_tuple(init_x, init_y, init_z, init_time));

        if (!use_k_rrt_for_checkpoints_)
        {
            // A* based planning segment.
            std::vector<std::pair<double, double>> raw_path;
            raw_path.push_back({init_x, init_y});
            int start_cell_x = static_cast<int>(std::floor((init_x - grid_origin_x_) / grid_resolution_));
            int start_cell_y = static_cast<int>(std::floor((init_y - grid_origin_y_) / grid_resolution_));
            int goal_cell_x = static_cast<int>(std::floor((goal_x - grid_origin_x_) / grid_resolution_));
            int goal_cell_y = static_cast<int>(std::floor((goal_y - grid_origin_y_) / grid_resolution_));

            double effective_inflation = inflation_radius_;
            if (robot_name_.find("Drone") != std::string::npos && !use_k_rrt_for_checkpoints_)
            {
                effective_inflation = uav_astar_inflation_radius_;
            }
            std::vector<int8_t> inflated_grid = inflate_obstacles(dynamic_occupancy_grid_, effective_inflation);

            std::swap(dynamic_occupancy_grid_, inflated_grid);
            auto path_cells = a_star(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y);
            std::swap(dynamic_occupancy_grid_, inflated_grid); // restore original grid

            for (size_t i = 1; i < path_cells.size(); ++i)
            {
                int cell_x = path_cells[i].first;
                int cell_y = path_cells[i].second;
                double wx = grid_origin_x_ + (cell_x + 0.5) * grid_resolution_;
                double wy = grid_origin_y_ + (cell_y + 0.5) * grid_resolution_;
                raw_path.push_back({wx, wy});
            }
            // Smooth the raw path.
            auto smooth_path_result = smoothPath(raw_path, 50, 0.1);
            double turn_threshold = 1.0;
            auto refined_path = smooth_path_result;
            for (int iter = 0; iter < 5; ++iter)
            {
                refined_path = refineSharpTurns(refined_path, turn_threshold);
            }
            // Build dense segment.
            double prev_x = refined_path.front().first;
            double prev_y = refined_path.front().second;
            double time_acc = init_time;
            double local_theta = init_theta;
            for (size_t i = 1; i < refined_path.size(); ++i)
            {
                double wx = refined_path[i].first;
                double wy = refined_path[i].second;
                double dt_val = compute_dt_segment(prev_x, prev_y, wx, wy, local_theta);
                time_acc += dt_val;
                seg_dense.push_back(std::make_tuple(wx, wy, init_z, time_acc));
                local_theta = std::atan2(wy - prev_y, wx - prev_x);
                prev_x = wx;
                prev_y = wy;
            }
            // Build sparse segment via sparsification.
            auto sparsified_path = sparsifyPath(refined_path, 0.05);
            prev_x = sparsified_path.front().first;
            prev_y = sparsified_path.front().second;
            time_acc = init_time;
            local_theta = init_theta;
            for (size_t i = 1; i < sparsified_path.size(); ++i)
            {
                double wx = sparsified_path[i].first;
                double wy = sparsified_path[i].second;
                double dt_val = compute_dt_segment(prev_x, prev_y, wx, wy, local_theta);
                time_acc += dt_val;
                seg_sparse.push_back(std::make_tuple(wx, wy, init_z, time_acc));
                local_theta = std::atan2(wy - prev_y, wx - prev_x);
                prev_x = wx;
                prev_y = wy;
            }
            return std::make_pair(seg_dense, seg_sparse);
        }
        else
        {
            // Kinodynamic-RRT based planning segment.
            struct Node
            {
                double x, y, theta, time, cost;
                int parent;
            };
            std::vector<Node> tree;
            Node root = {init_x, init_y, init_theta, init_time, 0.0, -1};
            tree.push_back(root);
            int max_iterations = 100000;
            bool reached = false;
            int goal_index = -1;
            double dt_val = 2.0; // Time increment per step.
            double v = max_linear_velocity_;
            double goal_threshold = 1.0; // Distance threshold.

            for (int iter = 0; iter < max_iterations; iter++)
            {
                double sample_choice = std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
                double x_rand, y_rand;
                double bias_probability = 0.2; // 20% chance to sample the goal directly.
                if (sample_choice < bias_probability)
                {
                    x_rand = goal_x;
                    y_rand = goal_y;
                }
                else
                {
                    x_rand = x_dist_(rng_);
                    y_rand = y_dist_(rng_);
                }
                // Find the nearest node.
                int nearest_index = 0;
                double min_dist = std::numeric_limits<double>::max();
                for (int i = 0; i < tree.size(); i++)
                {
                    double dx = tree[i].x - x_rand;
                    double dy = tree[i].y - y_rand;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        nearest_index = i;
                    }
                }
                Node nearest = tree[nearest_index];
                double theta_des = std::atan2(y_rand - nearest.y, x_rand - nearest.x);
                double dtheta = theta_des - nearest.theta;
                while (dtheta > M_PI)
                    dtheta -= 2 * M_PI;
                while (dtheta < -M_PI)
                    dtheta += 2 * M_PI;
                double new_theta = nearest.theta + dtheta;
                double step = v * dt_val;
                double new_x = nearest.x + step * std::cos(new_theta);
                double new_y = nearest.y + step * std::sin(new_theta);
                double new_time = nearest.time + dt_val;
                double distance_cost = nearest.cost + step;
                if (new_x < grid_origin_x_ || new_x > grid_origin_x_ + square_size_ ||
                    new_y < grid_origin_y_ || new_y > grid_origin_y_ + square_size_)
                {
                    continue;
                }
                if (!check_line_free_bresenham(std::make_tuple(nearest.x, nearest.y, init_z, 0.0),
                                               std::make_tuple(new_x, new_y, init_z, 0.0), inflation_radius_))
                {
                    continue;
                }
                Node new_node = {new_x, new_y, new_theta, new_time, distance_cost, nearest_index};
                tree.push_back(new_node);
                double dx_goal = new_x - goal_x;
                double dy_goal = new_y - goal_y;
                if (std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal) < goal_threshold)
                {
                    reached = true;
                    goal_index = tree.size() - 1;
                    break;
                }
            }

            TrajVec seg_dense_rrt, seg_sparse_rrt;
            if (!reached)
            {
                RCLCPP_ERROR(this->get_logger(), "Kinodynamic RRT for checkpoint failed after %d iterations", max_iterations);
                seg_dense_rrt.push_back(std::make_tuple(init_x, init_y, init_z, init_time));
                seg_sparse_rrt.push_back(std::make_tuple(init_x, init_y, init_z, init_time));
                return std::make_pair(seg_dense_rrt, seg_sparse_rrt);
            }

            std::vector<Node> path;
            int idx = goal_index;
            while (idx != -1)
            {
                path.push_back(tree[idx]);
                idx = tree[idx].parent;
            }
            std::reverse(path.begin(), path.end());
            for (const auto &node : path)
            {
                seg_dense_rrt.push_back(std::make_tuple(node.x, node.y, init_z, node.time));
            }
            std::vector<std::pair<double, double>> densePoints;
            for (const auto &pt : seg_dense_rrt)
            {
                densePoints.push_back({std::get<0>(pt), std::get<1>(pt)});
            }
            auto sparsified_points = sparsifyPath(densePoints, 0.05);
            double prev_x = sparsified_points.front().first;
            double prev_y = sparsified_points.front().second;
            double time_acc = std::get<3>(seg_dense_rrt.front());
            double local_theta = init_theta;
            seg_sparse_rrt.push_back(std::make_tuple(prev_x, prev_y, init_z, time_acc));
            for (size_t i = 1; i < sparsified_points.size(); i++)
            {
                double wx = sparsified_points[i].first;
                double wy = sparsified_points[i].second;
                double dt_new = compute_dt_segment(prev_x, prev_y, wx, wy, local_theta);
                time_acc += dt_new;
                seg_sparse_rrt.push_back(std::make_tuple(wx, wy, init_z, time_acc));
                local_theta = std::atan2(wy - prev_y, wx - prev_x);
                prev_x = wx;
                prev_y = wy;
            }
            return std::make_pair(seg_dense_rrt, seg_sparse_rrt);
        }
    }

    // member function to plan a convoy trajectory.
    //    This function uses the UGV’s sparse trajectory (ugv_traj) as a reference.
    //    For each waypoint in ugv_traj (except the first), we plan a segment
    //    from the drone’s current state to the UGV waypoint (using the drone’s OGM and with goal altitude set to drone_altitude_).
    //    Then, we rescale the computed time stamps so that the segment finishes at the same time as the UGV’s waypoint.
    std::pair<std::vector<std::tuple<double, double, double, double>>,
              std::vector<std::tuple<double, double, double, double>>>
    plan_convoy_trajectory(const std::vector<std::tuple<double, double, double, double>> &ugv_traj)
    {
        using TrajVec = std::vector<std::tuple<double, double, double, double>>;
        TrajVec dense_traj, sparse_traj;
        // Start at the drone’s starting state (with drone_altitude_)
        dense_traj.push_back(std::make_tuple(start_x_, start_y_, drone_altitude_, 0.0));
        sparse_traj.push_back(std::make_tuple(start_x_, start_y_, drone_altitude_, 0.0));
        double curr_x = start_x_, curr_y = start_y_, curr_time = 0.0, curr_theta = start_yaw_;

        // For each subsequent waypoint in the companion UGV’s trajectory…
        for (size_t i = 1; i < ugv_traj.size(); i++)
        {
            double target_x = std::get<0>(ugv_traj[i]);
            double target_y = std::get<1>(ugv_traj[i]);
            double target_time = std::get<3>(ugv_traj[i]); // desired arrival time

            // Plan a segment from current state to the target at drone altitude.
            // (This code reuses your existing plan_segment logic.
            // For clarity you might refactor that lambda into a helper function, e.g. plan_segment_to_goal.)
            auto seg_pair = plan_segment_to_goal(curr_x, curr_y, drone_altitude_, curr_time, curr_theta, target_x, target_y, drone_altitude_);

            // Rescale the time stamps in seg_pair so that the segment ends at target_time.
            double seg_start_time = curr_time;
            double seg_end_time = std::get<3>(seg_pair.second.back());
            double scaling = (target_time - seg_start_time) / (seg_end_time - seg_start_time);
            for (size_t j = 0; j < seg_pair.second.size(); j++)
            {
                double orig_time = std::get<3>(seg_pair.second[j]);
                double new_time = seg_start_time + (orig_time - seg_start_time) * scaling;
                std::get<3>(seg_pair.second[j]) = new_time;
                std::get<3>(seg_pair.first[j]) = new_time;
            }
            // Append the new segment (skipping the duplicate starting point)
            dense_traj.insert(dense_traj.end(), seg_pair.first.begin() + 1, seg_pair.first.end());
            sparse_traj.insert(sparse_traj.end(), seg_pair.second.begin() + 1, seg_pair.second.end());

            // Update current state for next segment.
            curr_x = target_x;
            curr_y = target_y;
            curr_time = target_time;
            if (seg_pair.second.size() >= 2)
            {
                double prev_x = std::get<0>(seg_pair.second[seg_pair.second.size() - 2]);
                double prev_y = std::get<1>(seg_pair.second[seg_pair.second.size() - 2]);
                curr_theta = std::atan2(target_y - prev_y, target_x - prev_x);
            }
        }

        // -------------------- Apply planning horizon in discrete steps (if enabled) --------------------
        // Same idea as in plan_trajectory(): limit to planning_horizon_steps_ sparse
        // segments and truncate the dense convoy trajectory by *arc length*.
        if (planning_horizon_steps_ > 0)
        {
            int max_points = planning_horizon_steps_ + 1; // include start state
            if (static_cast<int>(sparse_traj.size()) > max_points)
            {
                // 1) Horizon length along the sparse convoy trajectory.
                double horizon_length = 0.0;
                for (int i = 1; i < max_points; ++i)
                {
                    double dx = std::get<0>(sparse_traj[i]) - std::get<0>(sparse_traj[i - 1]);
                    double dy = std::get<1>(sparse_traj[i]) - std::get<1>(sparse_traj[i - 1]);
                    horizon_length += std::sqrt(dx * dx + dy * dy);
                }

                // 2) Truncate sparse trajectory.
                sparse_traj.resize(static_cast<std::size_t>(max_points));

                // 3) Truncate dense trajectory by length.
                std::vector<std::tuple<double, double, double, double>> dense_truncated;
                dense_truncated.reserve(dense_traj.size());
                dense_truncated.push_back(dense_traj.front());

                double dense_length = 0.0;
                const double eps = 1e-6;

                for (std::size_t i = 1; i < dense_traj.size(); ++i)
                {
                    double dx = std::get<0>(dense_traj[i]) - std::get<0>(dense_traj[i - 1]);
                    double dy = std::get<1>(dense_traj[i]) - std::get<1>(dense_traj[i - 1]);
                    double seg_len = std::sqrt(dx * dx + dy * dy);

                    if (dense_length + seg_len > horizon_length + eps)
                    {
                        break;
                    }

                    dense_length += seg_len;
                    dense_truncated.push_back(dense_traj[i]);
                }

                dense_traj.swap(dense_truncated);
            }
        }
        // -------------------------------------------------------------------------

        RCLCPP_INFO(this->get_logger(), "Planned convoy trajectory (time-synced with companion UGV).");
        return std::make_pair(dense_traj, sparse_traj);
    }

    // -------------------- Main Planning Routine --------------------
    // This function first plans for all UGVs (using the ground OGM) and then for all drones (using the drone OGM).
    void plan_all_trajectories()
    {
        // ----- UGV Planning -----
        if (!occupancy_grid_received_ground_)
        {
            RCLCPP_WARN(this->get_logger(), "Ground occupancy grid not received yet; cannot plan UGV trajectories.");
            return;
        }
        // The map callbacks centre the planning area on the received map; sample within it.
        x_dist_ = std::uniform_real_distribution<double>(grid_origin_x_, grid_origin_x_ + square_size_);
        y_dist_ = std::uniform_real_distribution<double>(grid_origin_y_, grid_origin_y_ + square_size_);

        // Copy the ground occupancy grid into the UGV dynamic grid.
        dynamic_ground_grid_ = occupancy_grid_ground_;

        for (const auto &veh : vehicles_)
        {
            if (isUGV(veh))
            {
                // Set UGV-specific parameters.
                max_linear_velocity_ = ugv_max_linear_velocity_;
                max_turn_angle_deg_ = ugv_max_turn_angle_deg_;
                max_turn_angle_rad_ = max_turn_angle_deg_ * M_PI / 180.0;

                // Set starting state from vehicle info.
                robot_name_ = veh.name;
                trajectory_length_ = veh.trajectory_length;
                start_x_ = veh.start_x;
                start_y_ = veh.start_y;
                start_z_ = veh.start_z;
                start_yaw_deg_ = veh.start_yaw;
                start_yaw_ = start_yaw_deg_ * M_PI / 180.0;
                provided_waypoints_ = veh.checkpoints;
                current_reach_checkpoints_first_ = veh.reach_checkpoints_first;

                // Use the UGV dynamic grid for planning.
                dynamic_occupancy_grid_ = dynamic_ground_grid_;
                trajectory_.clear();
                inflation_radius_ = ugv_inflation_radius_; // setting obstacle inflation specific to ugv
                auto seg_pair = plan_trajectory();
                update_dynamic_occupancy_grid(seg_pair.first, dynamic_ground_grid_);
                publish_updated_occupancy_grid(dynamic_ground_grid_, false);

                output_file_path_ = output_folder_string_ + veh.name + "_trajectory.txt";
                save_trajectory_to_file(seg_pair.second);
                plannedTrajectories_[veh.name] = seg_pair.second;
                publish_trajectory_for_robot(veh.name, seg_pair.second);
                publish_checkpoints_for_robot(veh.name, veh.checkpoints);
            }
        }

        // ----- Drone Planning -----
        if (!occupancy_grid_received_drone_)
        {
            RCLCPP_WARN(this->get_logger(), "Drone occupancy grid not received yet; skipping drone planning.");
            return;
        }

        // Copy the drone occupancy grid into the drone dynamic grid.
        dynamic_drone_grid_ = occupancy_grid_drone_;

        // Merge only the *unknown* drone cells that UGV has now explored:
        if (dynamic_ground_grid_.size() == dynamic_drone_grid_.size())
        {
            for (size_t i = 0; i < dynamic_drone_grid_.size(); i++)
            {
                // if UGV says “free” AND drone still thought it was unknown (-1), now mark explored (0)
                if (dynamic_ground_grid_[i] == 0 && dynamic_drone_grid_[i] == -1)
                {
                    dynamic_drone_grid_[i] = 0;
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "UGV and Drone grid sizes differ; cannot merge explored cells.");
        }

        // Build a mapping for Convoy mode: assign each convoy drone a unique UGV companion based on starting positions.
        std::map<std::string, std::string> drone_to_ugv;
        std::vector<VehicleInfo> ugv_list;
        for (const auto &veh : vehicles_)
        {
            if (isUGV(veh))
                ugv_list.push_back(veh);
        }
        for (const auto &veh : vehicles_)
        {
            if (isDrone(veh) && veh.flight_pattern == "Convoy")
            {
                double min_dist = std::numeric_limits<double>::max();
                std::string selected_ugv;
                for (const auto &ugv : ugv_list)
                {
                    // Only consider UGVs not already paired.
                    bool already_assigned = false;
                    for (const auto &pair : drone_to_ugv)
                    {
                        if (pair.second == ugv.name)
                        {
                            already_assigned = true;
                            break;
                        }
                    }
                    if (already_assigned)
                        continue;
                    double dx = veh.start_x - ugv.start_x;
                    double dy = veh.start_y - ugv.start_y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        selected_ugv = ugv.name;
                    }
                }
                if (!selected_ugv.empty())
                    drone_to_ugv[veh.name] = selected_ugv;
                else if (!ugv_list.empty())
                    drone_to_ugv[veh.name] = ugv_list[0].name; // fallback assignment
            }
        }

        // Process each drone.
        for (const auto &veh : vehicles_)
        {
            if (isDrone(veh))
            {
                // Set drone-specific parameters.
                max_linear_velocity_ = drone_max_linear_velocity_;
                max_turn_angle_deg_ = drone_max_turn_angle_deg_;
                max_turn_angle_rad_ = max_turn_angle_deg_ * M_PI / 180.0;

                // Set starting state from vehicle info (override z to drone_altitude_).
                robot_name_ = veh.name;
                trajectory_length_ = veh.trajectory_length;
                start_x_ = veh.start_x;
                start_y_ = veh.start_y;
                start_z_ = drone_altitude_;
                start_yaw_deg_ = veh.start_yaw;
                start_yaw_ = start_yaw_deg_ * M_PI / 180.0;
                provided_waypoints_ = veh.checkpoints;
                current_reach_checkpoints_first_ = veh.reach_checkpoints_first;

                dynamic_occupancy_grid_ = dynamic_drone_grid_;
                trajectory_.clear();

                if (veh.flight_pattern == "Convoy")
                {
                    // For Convoy mode, select the companion UGV's trajectory.
                    std::string companion = drone_to_ugv[veh.name];
                    auto ugv_traj_it = plannedTrajectories_.find(companion);
                    if (ugv_traj_it == plannedTrajectories_.end())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Companion UGV trajectory for %s not found", companion.c_str());
                        // Fallback to default (RandomExplore) planning.

                        inflation_radius_ = uav_inflation_radius_;

                        auto seg_pair = plan_trajectory();
                        update_dynamic_occupancy_grid(seg_pair.first, dynamic_drone_grid_);
                        publish_updated_occupancy_grid(dynamic_drone_grid_, true);
                        output_file_path_ = output_folder_string_ + veh.name + "_trajectory.txt";

                        save_trajectory_to_file(seg_pair.second);
                        plannedTrajectories_[veh.name] = seg_pair.second;
                        publish_trajectory_for_robot(veh.name, seg_pair.second);
                        publish_checkpoints_for_robot(veh.name, veh.checkpoints);
                    }
                    else
                    {
                        // Plan the convoy trajectory by tracking the companion UGV's sparse trajectory.
                        inflation_radius_ = uav_inflation_radius_;
                        auto seg_pair = plan_convoy_trajectory(ugv_traj_it->second);
                        update_dynamic_occupancy_grid(seg_pair.first, dynamic_drone_grid_);
                        publish_updated_occupancy_grid(dynamic_drone_grid_, true);
                        output_file_path_ = output_folder_string_ + veh.name + "_trajectory.txt";
                        save_trajectory_to_file(seg_pair.second);
                        plannedTrajectories_[veh.name] = seg_pair.second;
                        publish_trajectory_for_robot(veh.name, seg_pair.second);
                        publish_checkpoints_for_robot(veh.name, veh.checkpoints);
                    }
                }
                else
                { // "RandomExplore" mode (default behavior)

                    inflation_radius_ = uav_inflation_radius_;

                    auto seg_pair = plan_trajectory();
                    update_dynamic_occupancy_grid(seg_pair.first, dynamic_drone_grid_);
                    publish_updated_occupancy_grid(dynamic_drone_grid_, true);
                    output_file_path_ = output_folder_string_ + veh.name + "_trajectory.txt";
                    save_trajectory_to_file(seg_pair.second);
                    plannedTrajectories_[veh.name] = seg_pair.second;
                    publish_trajectory_for_robot(veh.name, seg_pair.second);
                    publish_checkpoints_for_robot(veh.name, veh.checkpoints);
                }
            }
        }
    }

    // -------------------- Timer Callback --------------------
    void timer_callback()
    {
        if (!occupancy_grid_received_ground_)
        {
            RCLCPP_WARN(this->get_logger(), "Ground occupancy grid not received; skipping planning.");
            return;
        }
        // Planning runs once, so wait for the drone map too rather than skipping the drones.
        const bool has_drones = std::any_of(vehicles_.begin(), vehicles_.end(),
                                            [this](const VehicleInfo &v) { return isDrone(v); });
        if (has_drones && !occupancy_grid_received_drone_)
        {
            RCLCPP_WARN(this->get_logger(), "Drone occupancy grid not received; waiting before planning.");
            return;
        }
        // Plan both UGV and (if available) drone trajectories sequentially.
        plan_all_trajectories();
        // Optionally cancel timer if planning only once.
        timer_->cancel();
        if (exit_when_done_)
        {
            RCLCPP_INFO(this->get_logger(), "Planning finished; exiting.");
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}