// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef AIRLIB_HEADER_ONLY
#ifndef AIRLIB_NO_RPC

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
#include <algorithm> // for std::clamp, std::min
#include <limits>    // for std::numeric_limits
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif

#ifdef nil
#undef nil
#endif

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"
#include "vehicles/car/api/CarRpcLibAdaptors.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif

    namespace msr
{
    namespace airlib
    {

        typedef msr::airlib_rpclib::CarRpcLibAdaptors CarRpcLibAdaptors;

        CarRpcLibClient::CarRpcLibClient(const string &ip_address, uint16_t port, float timeout_sec)
            : RpcLibClientBase(ip_address, port, timeout_sec) {}

        CarRpcLibClient::~CarRpcLibClient() {}

        void CarRpcLibClient::setCarControls(const CarApiBase::CarControls &controls, const std::string &vehicle_name)
        {
            static_cast<rpc::client *>(getClient())->call("setCarControls", CarRpcLibAdaptors::CarControls(controls), vehicle_name);
        }

        CarApiBase::CarState CarRpcLibClient::getCarState(const std::string &vehicle_name)
        {
            return static_cast<rpc::client *>(getClient())->call("getCarState", vehicle_name).as<CarRpcLibAdaptors::CarState>().to();
        }

        CarApiBase::CarControls CarRpcLibClient::getCarControls(const std::string &vehicle_name)
        {
            return static_cast<rpc::client *>(getClient())->call("getCarControls", vehicle_name).as<CarRpcLibAdaptors::CarControls>().to();
        }

        static float normalizeAngle(float angle)
        {
            constexpr float pi = static_cast<float>(M_PI);
            while (angle > pi)
                angle -= 2 * pi;
            while (angle < -pi)
                angle += 2 * pi;
            return angle;
        }

        static float distance2D(const Vector3r &a, const Vector3r &b)
        {
            return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2));
        }

        static Vector3r getLookaheadPoint(const Vector3r &current,
                                          const vector<Vector3r> &path,
                                          float lookahead_distance,
                                          size_t start_index)
        {
            for (size_t i = start_index; i + 1 < path.size(); ++i)
            {
                const Vector3r &A = path[i], &B = path[i + 1];
                Vector3r seg = B - A;
                float seg_len = seg.norm();
                if (seg_len < 1e-6f)
                    continue;

                Vector3r dir = seg / seg_len;
                float t = (current - A).dot(dir);
                t = std::clamp(t, 0.0f, seg_len);
                Vector3r proj = A + dir * t;
                float d = distance2D(current, proj);

                if (d <= lookahead_distance && (lookahead_distance - d) <= (seg_len - t))
                {
                    float offset = lookahead_distance - d;
                    return proj + dir * offset;
                }
            }
            // fallback to next waypoint
            return (start_index + 1 < path.size()) ? path[start_index + 1] : path.back();
        }

        static float getYawFromQuaternion(const Quaternionr &q)
        {
            return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                              1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        }

        bool CarRpcLibClient::moveOnPath(const vector<Vector3r> &path,
                                         float desired_velocity,
                                         float timeout_sec,
                                         float lookahead,
                                         const std::string &vehicle_name, float control_hz)
        {
            if (path.empty())
            {
                std::cerr << "Error: Provided path is empty." << std::endl;
                return false;
            }

            const float control_period = 1.0f / control_hz;
            const float Kp_steering = 1.0f;
            const float max_steer = 0.5f;
            const float throttle_gain = 0.5f;
            const float threshold = 1.0f; // only still used for terminal checks

            auto start_time = std::chrono::steady_clock::now();
            bool closed_loop = (distance2D(path.front(), path.back()) < 1e-3);
            bool passed_start = false;
            size_t current_index = 0;

            while (true)
            {
                // 1) timeout
                auto now = std::chrono::steady_clock::now();
                float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - start_time).count();
                if (elapsed > timeout_sec)
                {
                    std::cerr << "Timeout reached while following the path." << std::endl;
                    break;
                }

                // 2) state
                auto car_state = getCarState(vehicle_name);
                Vector3r pos = car_state.kinematics_estimated.pose.position;
                float speed = car_state.speed;

                // 3) terminal
                if (!closed_loop && distance2D(pos, path.back()) < threshold)
                {
                    std::cout << "Destination reached." << std::endl;
                    break;
                }
                if (closed_loop)
                {
                    if (!passed_start && distance2D(pos, path.front()) > threshold * 2)
                        passed_start = true;
                    if (passed_start && distance2D(pos, path.front()) < threshold)
                    {
                        std::cout << "Closed-loop: Returned to start." << std::endl;
                        break;
                    }
                }

                // 4) **forward-only index stepping**:
                //    keep advancing current_index while you are closer to the next WP than this one
                while (current_index + 1 < path.size())
                {
                    float d_cur = distance2D(pos, path[current_index]);
                    float d_next = distance2D(pos, path[current_index + 1]);
                    if (d_next < d_cur)
                    {
                        current_index++;
                    }
                    else
                    {
                        break;
                    }
                }

                // 5) **distance‐based lookahead**:
                //    find the first waypoint ≥ lookahead meters away
                size_t target_index = path.size() - 1; // fallback = final WP
                for (size_t i = current_index + 1; i < path.size(); ++i)
                {
                    if (distance2D(pos, path[i]) >= lookahead)
                    {
                        target_index = i;
                        break;
                    }
                }
                Vector3r target = path[target_index];

                /* std::cout << "[DEBUG] cur_idx=" << current_index
                          << ", tgt_idx=" << target_index
                          << ", d_cur=" << distance2D(pos, path[current_index])
                          << ", d_tgt=" << distance2D(pos, target)
                          << std::endl; */

                // 6) pure‐pursuit to that target
                float desired_heading = std::atan2(target.y() - pos.y(), target.x() - pos.x());
                float current_heading = getYawFromQuaternion(car_state.kinematics_estimated.pose.orientation);
                float steer = std::clamp(
                    Kp_steering * normalizeAngle(desired_heading - current_heading),
                    -max_steer, max_steer);
                float thr = (speed < desired_velocity)
                                ? std::min(throttle_gain * (desired_velocity - speed), 1.0f)
                                : 0.0f;

                CarApiBase::CarControls ctrl;
                ctrl.steering = steer;
                ctrl.throttle = thr;
                ctrl.handbrake = false;
                ctrl.is_manual_gear = false;
                setCarControls(ctrl, vehicle_name);

                std::this_thread::sleep_for(std::chrono::milliseconds(int(control_period * 1000)));
            }

            // stop
            CarApiBase::CarControls stop;
            stop.throttle = 0;
            stop.steering = 0;
            stop.handbrake = true;
            setCarControls(stop, vehicle_name);

            return true;
        }

        bool CarRpcLibClient::moveOnPathNumericalVelocity(
            const vector<Vector3r> &path,
            float desired_velocity,
            float timeout_sec,
            float lookahead,
            const std::string &vehicle_name,
            float control_hz)
        {
            if (path.empty())
            {
                std::cerr << "Error: Provided path is empty." << std::endl;
                return false;
            }

            // timings
            const float control_period = 1.0f / control_hz;
            const float threshold = 1.0f;

            // pure-pursuit gains
            const float Kp_steer = 1.0f;
            const float max_steer = 0.5f;

            // throttle PID gains
            const float Kp_thr = 0.5f, Ki_thr = 0.2f, Kd_thr = 0.05f;
            float integral = 0.0f, prev_error = 0.0f;

            // rate limiter: cap +Δthrottle, allow full -Δthrottle in one tick
            const float max_dthr_up = 0.04f;   // your chosen rise limiter
            const float max_dthr_down = 0.6f; // drop limiter ≥1.0 → instant drop
            float throttle_prev = 0.0f;

            // speed filter (one-pole)
            const float alpha_speed = 0.6f;
            float speed_filtered = desired_velocity;

            // pure-pursuit indexing
            bool closed_loop = (distance2D(path.front(), path.back()) < 1e-3f);
            bool passed_start = false;
            size_t current_index = 0;

            // initial sim state
            auto state = getCarState(vehicle_name);
            Vector3r last_pos = state.kinematics_estimated.pose.position;
            uint64_t last_ts = state.timestamp; // nanoseconds

            // real-time timeout
            auto t_start = std::chrono::steady_clock::now();

            while (true)
            {
                // 1) real-time timeout
                auto t_now = std::chrono::steady_clock::now();
                if (std::chrono::duration<float>(t_now - t_start).count() > timeout_sec)
                {
                    std::cerr << "Timeout reached while following the path." << std::endl;
                    break;
                }

                // 2) pull new sim state
                state = getCarState(vehicle_name);
                Vector3r pos = state.kinematics_estimated.pose.position;
                uint64_t ts = state.timestamp;

                // 3) sim-dt
                float dt_sim = float(ts - last_ts) * 1e-9f;
                if (dt_sim <= 0)
                    dt_sim = control_period;

                // 4) numeric speed + smoothing
                float speed_est = distance2D(pos, last_pos) / dt_sim;
                last_pos = pos;
                last_ts = ts;
                speed_filtered = alpha_speed * speed_est + (1 - alpha_speed) * speed_filtered;

                // 5) terminal checks
                if (!closed_loop && distance2D(pos, path.back()) < threshold)
                {
                    std::cout << "Destination reached." << std::endl;
                    break;
                }
                if (closed_loop)
                {
                    if (!passed_start && distance2D(pos, path.front()) > 2 * threshold)
                        passed_start = true;
                    if (passed_start && distance2D(pos, path.front()) < threshold)
                    {
                        std::cout << "Closed-loop: Returned to start." << std::endl;
                        break;
                    }
                }

                // 6) forward-only index stepping
                while (current_index + 1 < path.size())
                {
                    if (distance2D(pos, path[current_index + 1]) < distance2D(pos, path[current_index]))
                        ++current_index;
                    else
                        break;
                }

                // 7) lookahead target
                size_t target_index = path.size() - 1;
                for (size_t i = current_index + 1; i < path.size(); ++i)
                    if (distance2D(pos, path[i]) >= lookahead)
                    {
                        target_index = i;
                        break;
                    }
                Vector3r target = path[target_index];

                // 8) pure-pursuit steering
                float desired_heading = std::atan2(target.y() - pos.y(), target.x() - pos.x());
                float current_heading = getYawFromQuaternion(state.kinematics_estimated.pose.orientation);
                float steer = std::clamp(
                    Kp_steer * normalizeAngle(desired_heading - current_heading),
                    -max_steer, max_steer);

                // 9) PID throttle on filtered speed
                float error = desired_velocity - speed_filtered;
                integral = std::clamp(integral + error * dt_sim, -1.0f, 1.0f);
                float derivative = (error - prev_error) / dt_sim;
                prev_error = error;
                float raw_thr = std::clamp(
                    Kp_thr * error + Ki_thr * integral + Kd_thr * derivative,
                    0.0f, 1.0f);

                // 10) **asymmetric rate limiter**
                float delta = raw_thr - throttle_prev;
                if (delta > max_dthr_up)
                    delta = max_dthr_up;
                else if (delta < -max_dthr_down)
                    delta = -max_dthr_down;
                float thr = throttle_prev + delta;
                throttle_prev = thr;

                // 11) send controls
                CarApiBase::CarControls ctrl;
                ctrl.steering = steer;
                ctrl.throttle = thr;
                ctrl.handbrake = false;
                ctrl.is_manual_gear = false;
                setCarControls(ctrl, vehicle_name);

                // 12) sleep real-time
                std::this_thread::sleep_for(
                    std::chrono::duration<float>(control_period));
            }

            // stop at the end
            CarApiBase::CarControls stop;
            stop.throttle = 0;
            stop.steering = 0;
            stop.handbrake = true;
            stop.is_manual_gear = false;
            setCarControls(stop, vehicle_name);

            return true;
        }

    } // namespace airlib
} // namespace msr

#endif
#endif
