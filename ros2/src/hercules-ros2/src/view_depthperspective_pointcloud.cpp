#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Vector3.hpp>

using namespace std::chrono_literals;

class RGBDToPointCloud : public rclcpp::Node
{
public:
    RGBDToPointCloud()
        : Node("rgbd_to_pointcloud_cpp")
    {
        // parameters
        declare_parameter<std::string>("robot_name", "Drone1");
        declare_parameter<double>("max_depth", 1000.0);
        declare_parameter<int>("decimation", 2);

        robot_name_ = get_parameter("robot_name").as_string();
        max_depth_ = get_parameter("max_depth").as_double();
        decimation_ = get_parameter("decimation").as_int();

        // topics
        std::string base = "/hercules_node/" + robot_name_;
        std::string depth_topic = base + "/front_center_DepthPerspective/image";
        std::string info_topic = base + "/front_center_DepthPerspective/camera_info";
        std::string color_topic = base + "/front_center_Scene/image";
        std::string output_topic = base + "/colored_pointcloud";

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // publisher
        pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        // sync subs
#ifdef USE_OLD_RMW_QOS_MESSAGE_FILTERS
        // Jazzy and older: message_filters only accepts rmw_qos_profile_t.
        const auto qos = rclcpp::QoS(10).get_rmw_qos_profile();
#else
        const auto qos = rclcpp::QoS(10);
#endif
        depth_sub_.subscribe(this, depth_topic, qos);
        color_sub_.subscribe(this, color_topic, qos);
        info_sub_.subscribe(this, info_topic, qos);
        sync_ = std::make_shared<Sync>(Sync(10), depth_sub_, color_sub_, info_sub_);
        sync_->registerCallback(
            std::bind(&RGBDToPointCloud::callback, this, std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));

        RCLCPP_INFO(get_logger(),
                    "[%s] depth=%s  color=%s  info=%s -> publish=%s  max_depth=%.1f  decimation=%d",
                    robot_name_.c_str(),
                    depth_topic.c_str(),
                    color_topic.c_str(),
                    info_topic.c_str(),
                    output_topic.c_str(),
                    max_depth_,
                    decimation_);
    }

private:
    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        // 1) check encoding
        if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
        {
            RCLCPP_WARN(get_logger(),
                        "Depth encoding '%s' not '32FC1'. Enable float depths in AirSimSettings.json.",
                        depth_msg->encoding.c_str());
            return;
        }

        // 2) convert to cv::Mat
        auto dptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        auto cptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        const cv::Mat &depth = dptr->image;
        const cv::Mat &color = cptr->image;
        int h = depth.rows, w = depth.cols;

        // 3) intrinsics
        double fx = info_msg->k[0], fy = info_msg->k[4];
        double cx = info_msg->k[2], cy = info_msg->k[5];

        // 4) lookup TF
        std::string cam_frame = depth_msg->header.frame_id;
        std::string odom_frame = robot_name_ + "/odom_local";
        geometry_msgs::msg::TransformStamped tfst;
        try
        {
            tfst = tf_buffer_->lookupTransform(
                odom_frame, cam_frame,
                depth_msg->header.stamp,
                rclcpp::Duration(0, 500000000));
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
            return;
        }
        tf2::Quaternion q(
            tfst.transform.rotation.x,
            tfst.transform.rotation.y,
            tfst.transform.rotation.z,
            tfst.transform.rotation.w);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 T(
            tfst.transform.translation.x,
            tfst.transform.translation.y,
            tfst.transform.translation.z);

        // 5) prepare PointCloud2
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = depth_msg->header.stamp;
        cloud.header.frame_id = odom_frame;
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = 16; // x,y,z (3×4) + rgb (4)

        // estimate max points
        size_t max_pts = ((h + decimation_ - 1) / decimation_) * ((w + decimation_ - 1) / decimation_);
        cloud.width = 0;
        cloud.row_step = 0;
        cloud.fields.resize(4);
        cloud.fields[0] = make_field("x", 0);
        cloud.fields[1] = make_field("y", 4);
        cloud.fields[2] = make_field("z", 8);
        cloud.fields[3] = make_field("rgb", 12);

        // allocate
        cloud.data.resize(max_pts * cloud.point_step);
        uint8_t *ptr = cloud.data.data();
        size_t cnt = 0;

        // 6) fill in points
        for (int v = 0; v < h; v += decimation_)
        {
            const float *drow = depth.ptr<float>(v);
            const cv::Vec3b *crow = color.ptr<cv::Vec3b>(v);
            // precompute per-row direction
            float ydir = ((float)v - (float)cy) / (float)fy;

            for (int u = 0; u < w; u += decimation_)
            {
                float r = drow[u];
                // convert slant-range→forward depth Zc
                float xdir = ((float)u - (float)cx) / (float)fx;
                float ray_norm = std::sqrt(xdir * xdir + ydir * ydir + 1.0f);
                float zc = r / ray_norm;

                if (zc <= 0.0f || std::isnan(zc) || zc > max_depth_)
                {
                    continue;
                }

                // camera-space XYZ
                float x_cam = xdir * zc;
                float y_cam = ydir * zc;

                // color→RGB32
                const auto &pix = crow[u];
                uint32_t ri = (uint32_t(pix[2]) << 16) | (uint32_t(pix[1]) << 8) | pix[0];
                float rgbf;
                std::memcpy(&rgbf, &ri, sizeof(rgbf));

                // transform into map
                tf2::Vector3 P_cam(x_cam, y_cam, zc), P_map = R * P_cam + T;
                float xm = (float)P_map.x(), ym = (float)P_map.y(), zm = (float)P_map.z();

                // write to buffer
                std::memcpy(ptr + 0, &xm, 4);
                std::memcpy(ptr + 4, &ym, 4);
                std::memcpy(ptr + 8, &zm, 4);
                std::memcpy(ptr + 12, &rgbf, 4);
                ptr += cloud.point_step;
                ++cnt;
            }
        }

        // finalize
        cloud.width = cnt;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cnt * cloud.point_step);

        pc_pub_->publish(cloud);
    }

    // helper to make a PointField
    inline sensor_msgs::msg::PointField make_field(
        const std::string &name, uint32_t offset)
    {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count = 1;
        return f;
    }

    // members
    std::string robot_name_;
    double max_depth_;
    int decimation_;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    using Sync = message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo>>;
    std::shared_ptr<Sync> sync_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
