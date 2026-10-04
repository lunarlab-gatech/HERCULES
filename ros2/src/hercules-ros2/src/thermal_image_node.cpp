#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <set>
#include <algorithm> // for std::clamp

using std::placeholders::_1;

class ThermalImageNode : public rclcpp::Node
{
public:
    ThermalImageNode()
        : Node("thermal_image_node"),
          initialized_(false)
    {
        // --- Topics ---
        declare_parameter("scene_topic",
                          std::string("/hercules_node/Drone1/front_center_Scene/image"));
        declare_parameter("seg_topic",
                          std::string("/hercules_node/Drone1/front_center_Segmentation/image"));
        declare_parameter("thermal_topic",
                          std::string("/hercules_node/Drone1/front_center_ThermalIR/image"));
        get_parameter("scene_topic", scene_topic_);
        get_parameter("seg_topic", seg_topic_);
        get_parameter("thermal_topic", thermal_topic_);

        // --- Thermal sim parameters (Kelvin + emissivity) ---
        declare_parameter("temp_min", 285.0);
        declare_parameter("temp_max", 310.0);
        declare_parameter("eps_min", 0.85);
        declare_parameter("eps_max", 0.98);
        get_parameter("temp_min", temp_min_);
        get_parameter("temp_max", temp_max_);
        get_parameter("eps_min", eps_min_);
        get_parameter("eps_max", eps_max_);

        // --- Blending weight & colormap ---
        declare_parameter("thermal_weight", 0.25);
        declare_parameter("use_colormap", false);
        get_parameter("thermal_weight", alpha_);
        get_parameter("use_colormap", use_cmap_);

        // --- View mode: thermal | nightvision | flir ---
        declare_parameter("view_mode", std::string("nightvision"));
        get_parameter("view_mode", view_mode_);

        // --- NVG gain (multiplier for night-vision intensifier) ---
        declare_parameter("nvg_gain", 1.0);
        get_parameter("nvg_gain", nvg_gain_);

        // --- ROS interfaces ---
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            thermal_topic_, 10);
        scene_sub_ = create_subscription<sensor_msgs::msg::Image>(
            scene_topic_, 10,
            std::bind(&ThermalImageNode::sceneCallback, this, _1));
        seg_sub_ = create_subscription<sensor_msgs::msg::Image>(
            seg_topic_, 10,
            std::bind(&ThermalImageNode::segCallback, this, _1));

        RCLCPP_INFO(get_logger(),
                    "ThermalImageNode:\n"
                    " scene:      '%s'\n"
                    " segmentation:'%s'\n"
                    " output:     '%s'\n"
                    " T-range:    [%.1f,%.1f] K\n"
                    " eps-range:  [%.2f,%.2f]\n"
                    " blend:      %.2f\n"
                    " colormap:   %s\n"
                    " view_mode:  %s",
                    scene_topic_.c_str(),
                    seg_topic_.c_str(),
                    thermal_topic_.c_str(),
                    temp_min_, temp_max_,
                    eps_min_, eps_max_,
                    alpha_, use_cmap_ ? "ON" : "OFF",
                    view_mode_.c_str());
    }

private:
    void sceneCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Keep last grayscale scene for blending
        auto cvb = cv_bridge::toCvShare(msg, "bgr8");
        cv::cvtColor(cvb->image, last_gray_, cv::COLOR_BGR2GRAY);
    }

    void segCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 1) segmentation mask
        cv::Mat seg = cv_bridge::toCvShare(msg, "mono8")->image;

        // 2) init LUT on first frame
        if (!initialized_)
        {
            initLUT(seg);
            initialized_ = true;
            RCLCPP_INFO(
                get_logger(),
                "LUT initialized for %zu labels", lut_.size());
        }

        // 3) build flat thermal map [0–255]
        cv::Mat thermo(seg.size(), CV_8UC1);
        for (int y = 0; y < seg.rows; ++y)
        {
            for (int x = 0; x < seg.cols; ++x)
            {
                uint8_t lbl = seg.at<uint8_t>(y, x);
                auto it = lut_.find(lbl);
                thermo.at<uint8_t>(y, x) = (it != lut_.end() ? it->second : 0);
            }
        }

        // 4) blend with last_gray_
        cv::Mat blended;
        if (!last_gray_.empty() && last_gray_.size() == thermo.size())
        {
            cv::addWeighted(thermo, alpha_,
                            last_gray_, 1.0 - alpha_,
                            0.0, blended);
        }
        else
        {
            blended = thermo;
        }

        // 5) visualization
        cv::Mat output;
        std::string encoding;

        if (view_mode_ == "nightvision")
        {
            // 0) base luminance from real scene
            cv::Mat luminance;
            if (!last_gray_.empty() && last_gray_.size() == seg.size())
            {
                luminance = last_gray_;
            }
            else
            {
                // fallback to blended if scene is not available
                luminance = blended;
            }

            // Optional: small thermal fusion (for "fused" goggles)
            const double thermal_blend = 0.15; // 0 = pure scene, 1 = pure thermal
            cv::Mat fused;
            cv::addWeighted(luminance, 1.0 - thermal_blend,
                            blended, thermal_blend,
                            0.0, fused);

            // 1) automatic gain control based on histogram percentiles
            // map [p_low, p_high] to [0, 255]
            cv::Mat agc_in = fused.clone();
            int histSize = 256;
            float range[] = {0.f, 256.f};
            const float *ranges[] = {range};
            cv::Mat hist;
            cv::calcHist(&agc_in, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);

            int total = agc_in.rows * agc_in.cols;
            int low_count = static_cast<int>(0.02 * total);
            int high_count = static_cast<int>(0.98 * total);

            int cumsum = 0;
            int p_low = 0, p_high = 255;
            for (int i = 0; i < 256; ++i)
            {
                cumsum += static_cast<int>(hist.at<float>(i));
                if (cumsum >= low_count)
                {
                    p_low = i;
                    break;
                }
            }
            cumsum = 0;
            for (int i = 255; i >= 0; --i)
            {
                cumsum += static_cast<int>(hist.at<float>(i));
                if (cumsum >= total - high_count)
                {
                    p_high = i;
                    break;
                }
            }
            if (p_high <= p_low)
                p_high = p_low + 1; // avoid divide by zero

            double alpha_gain = 255.0 / (p_high - p_low);
            double beta_offset = -alpha_gain * p_low;

            cv::Mat agc;
            agc_in.convertTo(agc, CV_8U, alpha_gain, beta_offset);

            // 2) CLAHE for local contrast (behaves more like intensifier)
            cv::Mat clahe_out;
            {
                auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
                clahe->apply(agc, clahe_out);
            }

            // 3) gain-dependent shot noise (darker scenes produce more grain)
            double mean_val = cv::mean(agc_in)[0];
            // rough proxy for "gain": darker mean -> higher gain
            double norm_mean = std::max(1.0, mean_val);
            // darker mean -> higher base gain, scaled by nvg_gain_
            double gain_factor = std::clamp((80.0 / norm_mean) * nvg_gain_, 0.5, 4.0);

            double noise_sigma = 3.0 * gain_factor; // tune
            cv::Mat noise(clahe_out.size(), CV_16SC1);
            cv::randn(noise, 0.0, noise_sigma);

            cv::Mat tmp16;
            clahe_out.convertTo(tmp16, CV_16SC1);
            tmp16 += noise;
            cv::Mat noisy;
            tmp16.convertTo(noisy, CV_8U); // values are saturated to [0,255]

            // 4) highlight bloom/halo around the brightest pixels
            cv::Mat bright_mask;
            cv::threshold(noisy, bright_mask, 220, 255, cv::THRESH_BINARY);
            cv::Mat glow;
            cv::GaussianBlur(bright_mask, glow, cv::Size(0, 0), 5.0, 5.0);

            // scale glow so it adds a soft halo, not full white
            // cv::normalize(glow, glow, 0, 120, cv::NORM_MINMAX);
            cv::normalize(glow, glow, 0, 40, cv::NORM_MINMAX);
            cv::Mat halo_src;
            noisy.copyTo(halo_src);
            halo_src += glow;

            // 5) vignette (reuse your existing logic)
            cv::Mat vignet = halo_src.clone();
            {
                int w = vignet.cols, h = vignet.rows;
                cv::Mat mask(h, w, CV_32F);
                cv::Point2f c(w / 2.f, h / 2.f);
                float maxD = std::hypot(w / 2.f, h / 2.f);
                for (int y = 0; y < h; ++y)
                {
                    float *pm = mask.ptr<float>(y);
                    for (int x = 0; x < w; ++x)
                    {
                        float d = std::hypot(x - c.x, y - c.y) / maxD;
                        float v = 1.0f - d * d;
                        pm[x] = std::clamp(v, 0.35f, 1.0f); // a bit stronger vignette
                    }
                }
                cv::Mat v32;
                vignet.convertTo(v32, CV_32F);
                cv::multiply(v32, mask, v32);
                v32.convertTo(vignet, CV_8U);
            }

            // 6) green phosphor conversion
            cv::Mat nv_bgr;
            cv::cvtColor(vignet, nv_bgr, cv::COLOR_GRAY2BGR);
            std::vector<cv::Mat> ch(3);
            cv::split(nv_bgr, ch);

            // a bit less saturated, slightly less blue bleed
            ch[1] = ch[1] * 0.8f + 15;                          // main green channel
            ch[0] = ch[1] * 0.03f;                              // tiny blue bleed
            ch[2] = cv::Mat::zeros(ch[2].size(), ch[2].type()); // no red

            cv::merge(ch, nv_bgr);

            output = nv_bgr;
            encoding = "rgb8";
        }

        else if (view_mode_ == "flir")
        {
            cv::applyColorMap(blended, output, cv::COLORMAP_INFERNO);
            encoding = "rgb8";
        }
        else // "thermal"
        {
            if (use_cmap_)
            {
                cv::applyColorMap(blended, output, cv::COLORMAP_INFERNO);
                encoding = "rgb8";
            }
            else
            {
                output = blended;
                encoding = "mono8";
            }
        }

        // 6) publish
        auto out_msg = cv_bridge::CvImage(
                           msg->header, encoding, output)
                           .toImageMsg();
        image_pub_->publish(*out_msg);
    }

    // Build LUT: segID → thermal count
    void initLUT(const cv::Mat &seg)
    {
        std::set<uint8_t> labels;
        for (int y = 0; y < seg.rows; ++y)
            for (int x = 0; x < seg.cols; ++x)
                labels.insert(seg.at<uint8_t>(y, x));

        // synthetic camera response (µ=11,σ=1)
        const double mu = 11.0, sigma = 1.0;
        std::vector<double> band, resp;
        for (double lambda = 8.0; lambda < 14.0; lambda += 0.01)
        {
            band.push_back(lambda);
            resp.push_back(std::exp(-0.5 * std::pow((lambda - mu) / sigma, 2)));
        }

        const double c1 = 1.19104e8, c2 = 1.43879e4;
        double max_rad = 0.0;
        std::map<uint8_t, double> rads;

        std::mt19937_64 rng(std::random_device{}());
        std::uniform_real_distribution<double> dT(temp_min_, temp_max_);
        std::uniform_real_distribution<double> dE(eps_min_, eps_max_);

        // integrate per-label
        for (auto L : labels)
        {
            double T = dT(rng);
            double eps = dE(rng);
            double sum = 0.0;
            for (size_t i = 0; i < band.size(); ++i)
            {
                double Ld = eps * resp[i] *
                            (c1 / (std::pow(band[i], 5) *
                                   (std::exp(c2 / (band[i] * T)) - 1.0)));
                sum += Ld * 0.01;
            }
            rads[L] = sum;
            max_rad = std::max(max_rad, sum);
        }
        // normalize
        for (auto &kv : rads)
        {
            uint8_t cnt = static_cast<uint8_t>(
                std::round((kv.second / max_rad) * 255.0));
            lut_[kv.first] = cnt;
        }
    }

    // state
    bool initialized_;
    std::map<uint8_t, uint8_t> lut_;
    cv::Mat last_gray_;

    // params & ROS
    std::string scene_topic_, seg_topic_, thermal_topic_;
    double temp_min_, temp_max_, eps_min_, eps_max_, alpha_, nvg_gain_;
    bool use_cmap_;
    std::string view_mode_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        scene_sub_,
        seg_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalImageNode>());
    rclcpp::shutdown();
    return 0;
}
