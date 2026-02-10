#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <random>
#include <set>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>

using std::placeholders::_1;

struct ThermalProfile
{
    double base_temp_K; // nominal surface temperature
    double emissivity;  // 0..1
    bool is_animal;     // warm body (kangaroo, deer, human, etc.)
    bool is_fire;       // true for fire / flame emitters
    bool is_kangaroo;   // specifically a kangaroo
};

// Pack RGB into a 32-bit key: 0x00RRGGBB
static inline uint32_t makeColorKey(uint8_t r, uint8_t g, uint8_t b)
{
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

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
        declare_parameter("depth_topic",
                          std::string("/hercules_node/Drone1/front_center_DepthPlanar/image"));
        declare_parameter("thermal_topic",
                          std::string("/hercules_node/Drone1/front_center_ThermalIR/image"));

        declare_parameter("label_map_csv",
                          std::string("/home/sgarimella34/multi-robot-coordination/HERCULES/PythonClient/segmentation/label_color_map_ausenvkangaroos.csv"));

        get_parameter("scene_topic", scene_topic_);
        get_parameter("seg_topic", seg_topic_);
        get_parameter("depth_topic", depth_topic_);
        get_parameter("thermal_topic", thermal_topic_);
        get_parameter("label_map_csv", label_map_csv_);

        // --- Thermal sim parameters (global floor/ceiling, mainly for sanity) ---
        declare_parameter("temp_min", 280.0);
        declare_parameter("temp_max", 1300.0); // allow fire-like objects
        declare_parameter("eps_min", 0.80);
        declare_parameter("eps_max", 0.99);
        get_parameter("temp_min", temp_min_);
        get_parameter("temp_max", temp_max_);
        get_parameter("eps_min", eps_min_);
        get_parameter("eps_max", eps_max_);

        // Depth attenuation factor (how fast intensity decays with distance)
        declare_parameter("depth_attenuation", 0.01); // 1 / (1 + k * d)
        get_parameter("depth_attenuation", depth_attenuation_);

        // --- Blending weight & colormap (still used for NVG) ---
        declare_parameter("thermal_weight", 0.25);
        // Outside FLIR, we still want color by default
        declare_parameter("use_colormap", true);
        get_parameter("thermal_weight", alpha_);
        get_parameter("use_colormap", use_cmap_);

        // --- View mode: thermal | nightvision | flir ---
        // Default to FLIR
        declare_parameter("view_mode", std::string("flir"));
        get_parameter("view_mode", view_mode_);

        // --- NVG gain (multiplier for night-vision intensifier) ---
        declare_parameter("nvg_gain", 1.0);
        get_parameter("nvg_gain", nvg_gain_);

        // --- Fire-from-RGB toggle ---
        declare_parameter("use_fire_rgb", false);
        get_parameter("use_fire_rgb", use_fire_rgb_);

        // --- Precompute spectral response and load label map ---
        precomputeSpectralResponse();
        loadLabelMap(label_map_csv_);

        // --- ROS interfaces ---
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            thermal_topic_, 10);

        scene_sub_ = create_subscription<sensor_msgs::msg::Image>(
            scene_topic_, 10,
            std::bind(&ThermalImageNode::sceneCallback, this, _1));

        seg_sub_ = create_subscription<sensor_msgs::msg::Image>(
            seg_topic_, 10,
            std::bind(&ThermalImageNode::segCallback, this, _1));

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10,
            std::bind(&ThermalImageNode::depthCallback, this, _1));

        RCLCPP_INFO(get_logger(),
                    "ThermalImageNode:\n"
                    " scene:      '%s'\n"
                    " segmentation:'%s'\n"
                    " depth:      '%s'\n"
                    " output:     '%s'\n"
                    " T-range:    [%.1f,%.1f] K\n"
                    " eps-range:  [%.2f,%.2f]\n"
                    " depth_att:  %.3f\n"
                    " view_mode:  %s\n"
                    " use_fire_rgb: %s",
                    scene_topic_.c_str(),
                    seg_topic_.c_str(),
                    depth_topic_.c_str(),
                    thermal_topic_.c_str(),
                    temp_min_, temp_max_,
                    eps_min_, eps_max_,
                    depth_attenuation_,
                    view_mode_.c_str(),
                    use_fire_rgb_ ? "true" : "false");
    }

private:
    // Small helper: lowercase copy of a string
    static std::string toLower(const std::string &s)
    {
        std::string out = s;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });
        return out;
    }

    // Decide emissivity and base temperature from label + object_name
    ThermalProfile classifyLabel(const std::string &label,
                                 const std::string &object_name) const
    {
        // Combine both fields, since BP_Kangaroo23 is usually in object_name
        std::string combined = label + " " + object_name;
        std::string l = toLower(combined);

        ThermalProfile p;
        // Default neutral
        p.base_temp_K = 295.0; // ~22 °C
        p.emissivity = 0.90;
        p.is_animal = false;
        p.is_fire = false;
        p.is_kangaroo = false;

        if (l.find("tree") != std::string::npos ||
            l.find("bush") != std::string::npos ||
            l.find("grass") != std::string::npos ||
            l.find("plant") != std::string::npos ||
            l.find("vegetation") != std::string::npos)
        {
            // Vegetation, slightly cooler
            p.base_temp_K = 293.0; // ~20 °C
            p.emissivity = 0.97;
        }
        else if (l.find("road") != std::string::npos ||
                 l.find("ground") != std::string::npos ||
                 l.find("dirt") != std::string::npos ||
                 l.find("rock") != std::string::npos ||
                 l.find("soil") != std::string::npos)
        {
            // Soil / asphalt / rock
            p.base_temp_K = 300.0; // ~27 °C
            p.emissivity = 0.93;
        }
        else if (l.find("car") != std::string::npos ||
                 l.find("truck") != std::string::npos ||
                 l.find("vehicle") != std::string::npos ||
                 l.find("husky") != std::string::npos)
        {
            // Vehicles / metal, warmed by engine
            p.base_temp_K = 305.0; // ~32 °C
            p.emissivity = 0.90;
        }
        else if (l.find("fire") != std::string::npos ||
                 l.find("flame") != std::string::npos ||
                 l.find("torch") != std::string::npos)
        {
            // Fire source (very hot, high emissivity)
            p.base_temp_K = 1000.0;
            p.emissivity = 0.98;
            p.is_fire = true;
        }
        else if (l.find("animal") != std::string::npos ||
                 l.find("kangaroo") != std::string::npos ||
                 l.find("deer") != std::string::npos ||
                 l.find("human") != std::string::npos ||
                 l.find("person") != std::string::npos)
        {
            // Warm body: kangaroo, etc.
            p.base_temp_K = 315.0; // ~42 °C
            p.emissivity = 0.98;
            p.is_animal = true;

            if (l.find("kangaroo") != std::string::npos)
                p.is_kangaroo = true;
        }

        // Clamp to global min/max in case you extend these later
        p.base_temp_K = std::clamp(p.base_temp_K, temp_min_, temp_max_);
        p.emissivity = std::clamp(p.emissivity, eps_min_, eps_max_);
        return p;
    }

    // Load CSV: Label, ObjectName, SegmentationID, R, G, B
    // Build mapping: (R,G,B) -> ThermalProfile
    void loadLabelMap(const std::string &csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_WARN(get_logger(),
                        "Could not open label map CSV: %s", csv_path.c_str());
            return;
        }

        std::string line;
        // skip header
        std::getline(file, line);

        size_t count = 0;
        while (std::getline(file, line))
        {
            if (line.empty())
                continue;

            std::stringstream ss(line);
            std::string label, object_name, seg_id_str, r_str, g_str, b_str;

            if (!std::getline(ss, label, ','))
                continue;
            if (!std::getline(ss, object_name, ','))
                continue;
            if (!std::getline(ss, seg_id_str, ','))
                continue;
            if (!std::getline(ss, r_str, ','))
                continue;
            if (!std::getline(ss, g_str, ','))
                continue;
            if (!std::getline(ss, b_str, ','))
                continue;

            int r_i = 0, g_i = 0, b_i = 0;
            try
            {
                r_i = std::stoi(r_str);
                g_i = std::stoi(g_str);
                b_i = std::stoi(b_str);
            }
            catch (...)
            {
                continue;
            }

            uint8_t r = static_cast<uint8_t>(std::clamp(r_i, 0, 255));
            uint8_t g = static_cast<uint8_t>(std::clamp(g_i, 0, 255));
            uint8_t b = static_cast<uint8_t>(std::clamp(b_i, 0, 255));

            // Classify based on label + object_name (this is where "kangaroo" is detected)
            ThermalProfile prof = classifyLabel(label, object_name);

            uint32_t key = makeColorKey(r, g, b);
            color_to_profile_[key] = prof;
            ++count;
        }

        RCLCPP_INFO(get_logger(),
                    "Loaded %zu thermal profiles (color-based) from %s",
                    count, csv_path.c_str());
    }

    // Precompute spectral band and sensor response for LWIR 8–14 µm
    void precomputeSpectralResponse()
    {
        const double mu = 11.0;   // center wavelength
        const double sigma = 1.0; // spread

        band_.clear();
        resp_.clear();

        for (double lambda = 8.0; lambda <= 14.0; lambda += 0.01)
        {
            band_.push_back(lambda);
            resp_.push_back(std::exp(-0.5 * std::pow((lambda - mu) / sigma, 2)));
        }
    }

    void sceneCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto cvb = cv_bridge::toCvShare(msg, "bgr8");
            last_scene_bgr_ = cvb->image.clone(); // full-color scene
            cv::cvtColor(cvb->image, last_gray_, cv::COLOR_BGR2GRAY);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "sceneCallback cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto cvb = cv_bridge::toCvShare(msg, msg->encoding);

            if (msg->encoding == "32FC1")
            {
                last_depth_ = cvb->image.clone(); // meters
            }
            else if (msg->encoding == "16UC1")
            {
                // e.g. millimeters -> meters
                cv::Mat tmp;
                cvb->image.convertTo(tmp, CV_32F, 0.001);
                last_depth_ = tmp;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Unexpected depth encoding: %s", msg->encoding.c_str());
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "depthCallback cv_bridge exception: %s", e.what());
        }
    }

    void segCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Treat segmentation as a color image whose RGB values match the CSV
        cv::Mat seg_bgr;
        try
        {
            seg_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "segCallback cv_bridge exception: %s", e.what());
            return;
        }

        // 1) init LUT on first frame (per-color radiance from CSV + physics)
        if (!initialized_)
        {
            initLUT(seg_bgr);
            initialized_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Color-based LUT initialized for %zu entries", lut_.size());
        }

        // 2) build flat thermal map [0–255] with depth attenuation
        cv::Mat thermo(seg_bgr.size(), CV_8UC1);
        cv::Mat kangaroo_mask(seg_bgr.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat fire_mask(seg_bgr.size(), CV_8UC1, cv::Scalar(0));

        bool use_depth =
            !last_depth_.empty() &&
            last_depth_.size() == seg_bgr.size() &&
            last_depth_.type() == CV_32FC1;

        // Scene HSV for fire detection (Niagara flames) when enabled
        cv::Mat scene_hsv;
        bool have_scene_for_fire =
            use_fire_rgb_ &&
            !last_scene_bgr_.empty() &&
            last_scene_bgr_.size() == seg_bgr.size();

        if (have_scene_for_fire)
        {
            cv::cvtColor(last_scene_bgr_, scene_hsv, cv::COLOR_BGR2HSV);
        }

        int kangaroo_pixel_count = 0;
        int fire_pixel_count_raw = 0;

        for (int y = 0; y < seg_bgr.rows; ++y)
        {
            const cv::Vec3b *seg_row = seg_bgr.ptr<cv::Vec3b>(y);
            uint8_t *t_row = thermo.ptr<uint8_t>(y);
            uint8_t *kang_row = kangaroo_mask.ptr<uint8_t>(y);
            uint8_t *fire_row = fire_mask.ptr<uint8_t>(y);

            const float *d_row = use_depth ? last_depth_.ptr<float>(y) : nullptr;
            const cv::Vec3b *hsv_row = have_scene_for_fire ? scene_hsv.ptr<cv::Vec3b>(y) : nullptr;

            for (int x = 0; x < seg_bgr.cols; ++x)
            {
                const cv::Vec3b &pix = seg_row[x];
                uint8_t b = pix[0];
                uint8_t g = pix[1];
                uint8_t r = pix[2];

                uint32_t key = makeColorKey(r, g, b);

                auto it = lut_.find(key);
                uint8_t base_cnt = (it != lut_.end() ? it->second : 0);

                // Kangaroo detection via CSV profiles
                bool is_kangaroo = false;
                auto itp = color_to_profile_.find(key);
                if (itp != color_to_profile_.end())
                {
                    if (itp->second.is_kangaroo)
                        is_kangaroo = true;
                }

                if (is_kangaroo)
                {
                    // Extra boost before colormap so they sit near the very hot end
                    if (base_cnt < 245)
                        base_cnt = 245;
                    kang_row[x] = 255;
                    ++kangaroo_pixel_count;
                }

                // Fire detection from RGB scene using HSV, only if enabled
                bool is_fire_here = false;
                if (hsv_row != nullptr)
                {
                    const cv::Vec3b &hsv = hsv_row[x];
                    uint8_t H = hsv[0]; // 0..179
                    uint8_t S = hsv[1]; // 0..255
                    uint8_t V = hsv[2]; // 0..255

                    // Heuristic for flames: bright, saturated, red/orange
                    if (V > 180 && S > 120 &&
                        ((H <= 20) || (H >= 160)))
                    {
                        is_fire_here = true;
                    }
                }

                if (is_fire_here)
                {
                    base_cnt = 255; // max thermal intensity for raw fire pixels
                    fire_row[x] = 255;
                    ++fire_pixel_count_raw;
                }

                // Depth attenuation (you could skip attenuation for fire if you prefer)
                if (use_depth)
                {
                    float d = d_row[x];
                    if (std::isfinite(d) && d > 0.0f)
                    {
                        double atten = 1.0 / (1.0 + depth_attenuation_ * d);
                        double val = base_cnt * atten;
                        t_row[x] = static_cast<uint8_t>(std::clamp(val, 0.0, 255.0));
                    }
                    else
                    {
                        t_row[x] = base_cnt;
                    }
                }
                else
                {
                    t_row[x] = base_cnt;
                }
            }
        }

        // --- Post-process fire mask to make it a thick, filled blob ---
        int fire_pixel_count = 0;
        if (use_fire_rgb_ && fire_pixel_count_raw > 0)
        {
            // 1) Make the mask thicker and close gaps
            // Increase kernel size and iterations compared to before
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(21, 21)); // was 11x11

            // Close small holes and connect nearby bits
            cv::morphologyEx(fire_mask, fire_mask, cv::MORPH_CLOSE,
                             kernel, cv::Point(-1, -1), 3); // was 2

            // Grow a bit more
            cv::dilate(fire_mask, fire_mask, kernel,
                       cv::Point(-1, -1), 2); // was 1

            // 2) Fill any remaining interior holes
            cv::Mat ff = fire_mask.clone();
            // Flood-fill from outside; mark all external background as 255
            cv::floodFill(ff, cv::Point(0, 0), 255);
            // Invert: now "holes" become 255, outside becomes 0
            cv::bitwise_not(ff, ff);
            // Union with original ring to get a solid blob
            cv::bitwise_or(fire_mask, ff, fire_mask);

            fire_pixel_count = cv::countNonZero(fire_mask);

            // Force thermal map to maximum wherever we consider it fire
            for (int y = 0; y < thermo.rows; ++y)
            {
                uint8_t *t_row = thermo.ptr<uint8_t>(y);
                uint8_t *fire_row = fire_mask.ptr<uint8_t>(y);
                for (int x = 0; x < thermo.cols; ++x)
                {
                    if (fire_row[x])
                        t_row[x] = 255;
                }
            }
        }

        // Estimate number of distinct kangaroos via connected components
        int kangaroo_count = 0;
        if (kangaroo_pixel_count > 0)
        {
            cv::Mat labels;
            int n_labels = cv::connectedComponents(kangaroo_mask, labels, 8, CV_32S);
            kangaroo_count = std::max(0, n_labels - 1);
        }

        if (kangaroo_pixel_count == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "segCallback: no kangaroo pixels detected this frame. "
                "Check CSV colors and label strings for 'kangaroo'.");
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "segCallback: detected %d kangaroos (approx), %d kangaroo pixels.",
                kangaroo_count, kangaroo_pixel_count);
        }

        if (use_fire_rgb_ && fire_pixel_count > 0)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "segCallback: fire mask ~%d pixels (after fill).",
                fire_pixel_count);
        }

        // 3) build "blended" only for NVG; for FLIR we use pure thermo
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

        // 4) visualization
        cv::Mat output;
        std::string encoding;

        if (view_mode_ == "nightvision")
        {
            cv::Mat nv_bgr;
            cv::cvtColor(blended, nv_bgr, cv::COLOR_GRAY2BGR);
            std::vector<cv::Mat> ch(3);
            cv::split(nv_bgr, ch);
            ch[1] = ch[1] * 0.9f + 15;                          // main green channel
            ch[0] = ch[1] * 0.05f;                              // tiny blue bleed
            ch[2] = cv::Mat::zeros(ch[2].size(), ch[2].type()); // no red
            cv::merge(ch, nv_bgr);
            output = nv_bgr;
            encoding = "bgr8";
        }
        else if (view_mode_ == "flir")
        {
            // FLIR-style view: pure thermal map -> Inferno colormap
            cv::Mat flir_bgr;
            cv::applyColorMap(thermo, flir_bgr, cv::COLORMAP_INFERNO);

            cv::Mat flir_out = flir_bgr.clone();
            const cv::Vec3b kangaroo_bgr(0, 165, 255); // bright orange in BGR
            const cv::Vec3b fire_bgr(0, 255, 255);     // bright yellow

            if (kangaroo_pixel_count > 0 || fire_pixel_count > 0)
            {
                for (int y = 0; y < flir_out.rows; ++y)
                {
                    cv::Vec3b *row = flir_out.ptr<cv::Vec3b>(y);
                    const uint8_t *krow = kangaroo_mask.ptr<uint8_t>(y);
                    const uint8_t *frow = fire_mask.ptr<uint8_t>(y);

                    for (int x = 0; x < flir_out.cols; ++x)
                    {
                        if (frow[x])
                        {
                            cv::Vec3b orig = row[x];
                            // Fire: mostly bright yellow, keep a bit of Inferno
                            row[x][0] = static_cast<uchar>(0.2 * orig[0] + 0.8 * fire_bgr[0]);
                            row[x][1] = static_cast<uchar>(0.2 * orig[1] + 0.8 * fire_bgr[1]);
                            row[x][2] = static_cast<uchar>(0.2 * orig[2] + 0.8 * fire_bgr[2]);
                        }
                        else if (krow[x])
                        {
                            cv::Vec3b orig = row[x];
                            // Kangaroo: mostly orange, keep some Inferno
                            row[x][0] = static_cast<uchar>(0.2 * orig[0] + 0.8 * kangaroo_bgr[0]);
                            row[x][1] = static_cast<uchar>(0.2 * orig[1] + 0.8 * kangaroo_bgr[1]);
                            row[x][2] = static_cast<uchar>(0.2 * orig[2] + 0.8 * kangaroo_bgr[2]);
                        }
                    }
                }
            }

            output = flir_out;
            encoding = "bgr8";
        }
        else // "thermal" but still colorized
        {
            cv::Mat col;
            cv::applyColorMap(thermo, col, cv::COLORMAP_INFERNO);
            output = col;
            encoding = "bgr8";
        }

        // 5) publish
        auto out_msg = cv_bridge::CvImage(
                           msg->header, encoding, output)
                           .toImageMsg();
        image_pub_->publish(*out_msg);
    }

    // Build LUT: color_key -> thermal count [0..255] using CSV + Planck integral
    void initLUT(const cv::Mat &seg_bgr)
    {
        std::set<uint32_t> colors;
        for (int y = 0; y < seg_bgr.rows; ++y)
        {
            const cv::Vec3b *row = seg_bgr.ptr<cv::Vec3b>(y);
            for (int x = 0; x < seg_bgr.cols; ++x)
            {
                uint8_t b = row[x][0];
                uint8_t g = row[x][1];
                uint8_t r = row[x][2];
                uint32_t key = makeColorKey(r, g, b);
                colors.insert(key);
            }
        }

        const double c1 = 1.19104e8;
        const double c2 = 1.43879e4;

        double max_rad = 0.0;
        std::unordered_map<uint32_t, double> rads;

        // First pass: compute radiance per color key
        for (uint32_t key : colors)
        {
            ThermalProfile prof;

            auto itp = color_to_profile_.find(key);
            if (itp != color_to_profile_.end())
            {
                prof = itp->second;
            }
            else
            {
                // default neutral if not in CSV
                prof.base_temp_K = 295.0;
                prof.emissivity = 0.90;
                prof.is_animal = false;
                prof.is_fire = false;
                prof.is_kangaroo = false;
            }

            double T = prof.base_temp_K;
            double eps = prof.emissivity;

            // integrate LWIR radiance over [8,14] µm
            double sum = 0.0;
            for (size_t i = 0; i < band_.size(); ++i)
            {
                double lambda = band_[i];
                double sensor = resp_[i];

                double Ld = eps * sensor *
                            (c1 / (std::pow(lambda, 5) *
                                   (std::exp(c2 / (lambda * T)) - 1.0)));

                sum += Ld * 0.01; // Δλ = 0.01 µm
            }

            // Optionally clamp fire radiance so it does not completely dominate
            if (prof.is_fire)
            {
                const double max_fire_factor = 3.0;
                sum = std::min(sum, max_fire_factor * 1e6); // arbitrary large scale
            }

            rads[key] = sum;
            max_rad = std::max(max_rad, sum);
        }

        if (max_rad <= 0.0)
            max_rad = 1.0;

        // Second pass: normalize to [0,255] and boost animals
        for (auto &kv : rads)
        {
            uint32_t key = kv.first;
            double rad = kv.second;

            uint8_t cnt = static_cast<uint8_t>(
                std::round((rad / max_rad) * 255.0));

            auto itp = color_to_profile_.find(key);
            if (itp != color_to_profile_.end())
            {
                const ThermalProfile &prof = itp->second;

                // Ensure animals (kangaroos, etc.) live near the top of the brightness range
                if (prof.is_animal)
                {
                    if (cnt < 220)
                        cnt = 220;
                }

                if (prof.is_fire)
                {
                    cnt = 255;
                }
            }

            lut_[key] = cnt;
        }

        // Debug log for a few entries
        RCLCPP_INFO(get_logger(), "Color-based LUT entries:");
        int printed = 0;
        for (auto &kv : lut_)
        {
            if (printed > 50)
                break; // avoid spamming logs

            uint32_t key = kv.first;
            uint8_t gray = kv.second;
            uint8_t r = static_cast<uint8_t>((key >> 16) & 0xFF);
            uint8_t g = static_cast<uint8_t>((key >> 8) & 0xFF);
            uint8_t b = static_cast<uint8_t>(key & 0xFF);

            auto itp = color_to_profile_.find(key);
            bool is_animal = (itp != color_to_profile_.end()) && itp->second.is_animal;
            bool is_fire = (itp != color_to_profile_.end()) && itp->second.is_fire;
            bool is_kangaroo = (itp != color_to_profile_.end()) && itp->second.is_kangaroo;

            RCLCPP_INFO(get_logger(),
                        "  color (R=%u,G=%u,B=%u) -> gray %u (animal=%s, kangaroo=%s, fire=%s)",
                        r, g, b, gray,
                        is_animal ? "true" : "false",
                        is_kangaroo ? "true" : "false",
                        is_fire ? "true" : "false");
            ++printed;
        }
    }

    // state
    bool initialized_;
    // color_key (0x00RRGGBB) -> grayscale
    std::unordered_map<uint32_t, uint8_t> lut_;
    // color_key (0x00RRGGBB) -> thermal profile (used to detect kangaroo etc.)
    std::unordered_map<uint32_t, ThermalProfile> color_to_profile_;

    cv::Mat last_scene_bgr_; // full scene for fire detection
    cv::Mat last_gray_;
    cv::Mat last_depth_;
    std::vector<double> band_;
    std::vector<double> resp_;

    // params & ROS
    std::string scene_topic_, seg_topic_, depth_topic_, thermal_topic_;
    std::string label_map_csv_;
    double temp_min_, temp_max_, eps_min_, eps_max_;
    double alpha_, nvg_gain_;
    double depth_attenuation_;
    bool use_cmap_;
    std::string view_mode_;
    bool use_fire_rgb_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        scene_sub_,
        seg_sub_,
        depth_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalImageNode>());
    rclcpp::shutdown();
    return 0;
}
