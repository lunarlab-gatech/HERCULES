// Licensed under the MIT License.

#ifndef msr_airlib_UrdfVisualManifest_hpp
#define msr_airlib_UrdfVisualManifest_hpp

// Reader for the visual manifest written by the hercules_urdf_import tool
// (ros2/src/hercules_urdf_import) and for the binary/ASCII STL meshes it
// references. The manifest gives every URDF visual as a mesh file plus a
// pose relative to the vehicle body, in ROS FLU axes (x forward, y left,
// z up) and meters; this header converts poses to the NED/FRD convention the
// rest of AirLib uses, so a simulator can place the meshes with its usual
// NED -> engine transforms. Mesh vertices stay in their FLU geometry frame:
// convert each one with fluToFrd() before handing it to such a transform.

#include "common/Common.hpp"
#include "common/common_utils/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{

    class UrdfVisualManifest
    {
    public:
        struct Visual
        {
            std::string name;
            std::string link;
            std::string mesh_path; // absolute, or as given if base_dir was empty
            Pose pose; // mesh frame relative to the vehicle body, NED/FRD
            Vector3r scale = Vector3r(1, 1, 1); // along the mesh's own axes
            float color[4] = { 0.7f, 0.7f, 0.7f, 1.0f }; // linear RGBA, 0..1
        };

        static constexpr const char* kFormat = "hercules-urdf-visuals";
        static constexpr int kVersion = 1;

        std::string robot;
        std::string vehicle;
        std::string frame;
        std::vector<Visual> visuals;

        static UrdfVisualManifest load(const std::string& path)
        {
            std::ifstream stream(path);
            if (!stream)
                throw std::runtime_error("cannot open URDF visual manifest '" + path + "'");
            std::stringstream text;
            text << stream.rdbuf();
            return parse(text.str(), directoryOf(path));
        }

        // Parse manifest JSON; relative mesh paths are resolved against base_dir.
        static UrdfVisualManifest parse(const std::string& json_text, const std::string& base_dir)
        {
            nlohmann::json doc;
            try {
                doc = nlohmann::json::parse(json_text);
            }
            catch (const std::exception& e) {
                throw std::runtime_error(std::string("URDF visual manifest is not valid JSON: ") + e.what());
            }
            if (!doc.is_object() || doc.value("format", "") != kFormat)
                throw std::runtime_error(std::string("not a ") + kFormat + " manifest");
            if (doc.value("version", 0) != kVersion)
                throw std::runtime_error("unsupported URDF visual manifest version " +
                                         std::to_string(doc.value("version", 0)));
            if (doc.value("frame_convention", "FLU") != "FLU" || doc.value("units", "meters") != "meters")
                throw std::runtime_error("URDF visual manifest must use FLU axes and meters");

            UrdfVisualManifest manifest;
            manifest.robot = doc.value("robot", "");
            manifest.vehicle = doc.value("vehicle", "");
            manifest.frame = doc.value("frame", "");
            if (!doc.contains("visuals") || !doc["visuals"].is_array())
                throw std::runtime_error("URDF visual manifest has no visuals array");

            for (const auto& item : doc["visuals"]) {
                Visual visual;
                visual.name = item.value("name", "");
                visual.link = item.value("link", "");
                const std::string mesh = item.value("mesh", "");
                if (mesh.empty())
                    throw std::runtime_error("URDF visual '" + visual.name + "' has no mesh");
                visual.mesh_path = resolvePath(mesh, base_dir);

                const Vector3r position = readVector(item, "position", Vector3r::Zero(), visual.name);
                Quaternionr orientation = Quaternionr::Identity();
                if (item.contains("orientation")) {
                    const auto& q = item["orientation"];
                    if (!q.is_object())
                        throw std::runtime_error("URDF visual '" + visual.name + "' orientation must be {w,x,y,z}");
                    orientation = Quaternionr(q.value("w", 1.0f), q.value("x", 0.0f),
                                              q.value("y", 0.0f), q.value("z", 0.0f));
                    if (!(orientation.norm() > 1e-6f))
                        throw std::runtime_error("URDF visual '" + visual.name + "' has a zero quaternion");
                    orientation.normalize();
                }
                visual.pose = Pose(fluToFrd(position), fluToFrd(orientation));
                visual.scale = readVector(item, "scale", Vector3r(1, 1, 1), visual.name);

                if (item.contains("color")) {
                    const auto& color = item["color"];
                    if (!color.is_array() || color.size() != 4)
                        throw std::runtime_error("URDF visual '" + visual.name + "' color must be [r,g,b,a]");
                    for (int i = 0; i < 4; ++i)
                        visual.color[i] = std::min(1.0f, std::max(0.0f, color[i].get<float>()));
                }
                manifest.visuals.push_back(visual);
            }
            return manifest;
        }

        // ROS FLU axes -> AirLib FRD axes (a 180 degree turn about x).
        static Vector3r fluToFrd(const Vector3r& v)
        {
            return Vector3r(v.x(), -v.y(), -v.z());
        }

        // Rotation R expressed in FRD axes: C * R * C with C = diag(1, -1, -1).
        static Quaternionr fluToFrd(const Quaternionr& q)
        {
            return Quaternionr(q.w(), q.x(), -q.y(), -q.z());
        }

        // Load an STL mesh as a flat triangle list (three vertices per
        // triangle, file units, file axes). Binary files are recognized by
        // their exact size, so binary files whose header starts with "solid"
        // still load as binary.
        static std::vector<Vector3r> loadStl(const std::string& path)
        {
            std::ifstream stream(path, std::ios::binary);
            if (!stream)
                throw std::runtime_error("cannot open mesh '" + path + "'");
            const std::vector<char> data{ std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };

            std::vector<Vector3r> vertices;
            if (data.size() >= 84) {
                uint32_t count = 0;
                std::memcpy(&count, data.data() + 80, sizeof(count)); // little-endian on all targets
                if (data.size() == 84 + 50ull * count) {
                    vertices.reserve(3ull * count);
                    for (uint32_t i = 0; i < count; ++i) {
                        const char* facet = data.data() + 84 + 50ull * i + 12; // skip the normal
                        for (int corner = 0; corner < 3; ++corner) {
                            float xyz[3];
                            std::memcpy(xyz, facet + 12 * corner, sizeof(xyz));
                            vertices.emplace_back(xyz[0], xyz[1], xyz[2]);
                        }
                    }
                    checkFinite(vertices, path);
                    return vertices;
                }
            }

            std::string text(data.begin(), data.end());
            std::istringstream lines(text);
            std::string word;
            if (!(lines >> word) || word != "solid")
                throw std::runtime_error("'" + path + "' is neither binary nor ASCII STL");
            while (lines >> word) {
                if (word == "vertex") {
                    float x, y, z;
                    if (!(lines >> x >> y >> z))
                        throw std::runtime_error("bad vertex in '" + path + "'");
                    vertices.emplace_back(x, y, z);
                }
            }
            if (vertices.size() % 3 != 0)
                throw std::runtime_error("vertex count of '" + path + "' is not a multiple of 3");
            checkFinite(vertices, path);
            return vertices;
        }

        // Unit normal of a triangle wound counter-clockwise in a right-handed
        // frame; zero for a degenerate triangle.
        static Vector3r faceNormal(const Vector3r& a, const Vector3r& b, const Vector3r& c)
        {
            const Vector3r n = (b - a).cross(c - a);
            const real_T length = n.norm();
            return length > 0 ? Vector3r(n / length) : Vector3r(Vector3r::Zero());
        }

        static std::string resolvePath(const std::string& path, const std::string& base_dir)
        {
            if (path.empty() || base_dir.empty() || isAbsolute(path))
                return path;
            const char last = base_dir.back();
            return (last == '/' || last == '\\') ? base_dir + path : base_dir + "/" + path;
        }

        static std::string directoryOf(const std::string& path)
        {
            const auto slash = path.find_last_of("/\\");
            return slash == std::string::npos ? std::string() : path.substr(0, slash);
        }

    private:
        static bool isAbsolute(const std::string& path)
        {
            return path[0] == '/' || path[0] == '\\' || (path.size() > 1 && path[1] == ':');
        }

        static Vector3r readVector(const nlohmann::json& item, const char* key, const Vector3r& fallback,
                                   const std::string& name)
        {
            if (!item.contains(key))
                return fallback;
            const auto& v = item[key];
            if (!v.is_array() || v.size() != 3)
                throw std::runtime_error("URDF visual '" + name + "' " + key + " must have 3 numbers");
            Vector3r out(v[0].get<float>(), v[1].get<float>(), v[2].get<float>());
            if (!std::isfinite(out.x()) || !std::isfinite(out.y()) || !std::isfinite(out.z()))
                throw std::runtime_error("URDF visual '" + name + "' " + key + " is not finite");
            return out;
        }

        static void checkFinite(const std::vector<Vector3r>& vertices, const std::string& path)
        {
            for (const auto& v : vertices)
                if (!std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z()))
                    throw std::runtime_error("mesh '" + path + "' has a non-finite vertex");
        }
    };
}
} //namespace
#endif
