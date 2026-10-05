// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "sensors/lidar/LidarSimple.hpp"
#include "NedTransform.h"
#include <deque>
#include <memory>
#include <mutex>

struct FHerculesGpuHit;

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal.
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealLidarSensor : public msr::airlib::LidarSimple
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
                      AActor* actor, const NedTransform* ned_transform);

protected:
    virtual bool getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final) override;

	virtual void pause(const bool is_paused);

    virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

    virtual void getLocalPose(msr::airlib::Pose& sensor_pose);

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void createLasers();
    bool shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        const uint32 channel, const float horizontal_angle, const float vertical_angle, 
        const msr::airlib::LidarSimpleParams &params, const float noise_sample, Vector3r &point, std::string &label, FVector& raw_point);
    FVector Vector3rToFVector(const Vector3r& input_vector);

    // Casting on the GPU (FHerculesGpuRayCaster, airsim.Lidar.GpuRayTracing): the rays of a tick are cast
    // a few frames later and their results land, on the game thread, in the sweep they belong to; a
    // sweep is published once all its rays are back. Results follow the CPU traces' rules: components
    // the visibility channel passes through are passed through, labels name the hit actor, and rays
    // whose GPU result can't be matched to those rules are traced on the CPU instead.
    struct GpuSweep
    {
        msr::airlib::vector<msr::airlib::real_T> points;
        msr::airlib::vector<std::string> labels;
        int outstanding = 0; // batches in flight
        bool closed = false; // all its rays have been cast
    };
    struct GpuComponent
    {
        std::string label;
        bool pass_through = false; // the visibility channel ignores it
        bool trace_on_cpu = false; // has a LiDAR-ignoring physical material, which only the CPU traces resolve per face
    };
    struct GpuState
    {
        std::mutex mutex;
        std::shared_ptr<GpuSweep> sweep; // being cast
        std::deque<std::shared_ptr<GpuSweep>> complete;
        TArray<uint32> pass_through; // component IDs the rays pass through
        // game thread only
        TMap<uint32, GpuComponent> components;
        TSet<uint32> unidentified;
        double last_scan_seconds = -1.0e9;
    };
    struct GpuBatch
    {
        msr::airlib::Pose lidar_pose;
        msr::airlib::Pose vehicle_pose;
        FVector start;
        TArray<FVector3f> directions;
        TArray<float> max_distances;
        TArray<uint32> point_indices;
        TArray<float> noise_samples;
    };
    bool castOnGpu(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose, const msr::airlib::LidarSimpleParams& params,
        const TArray<TPair<uint32, uint32>>& scan_angles, bool sweep_complete, uint32 total_points,
        msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final);
    void collectGpuHits(GpuState& state, GpuSweep& sweep, const GpuBatch& batch, bool cast, const TArray<FHerculesGpuHit>& hits);
    const GpuComponent* identifyGpuComponent(GpuState& state, uint32 component_id, const FVector& hit_point, const FVector& direction);
    bool traceOnCpu(const GpuBatch& batch, int32 ray, Vector3r& point, std::string& label, FVector& raw_point);
    Vector3r toLidarFrame(const GpuBatch& batch, const FVector& impact_point) const;

private:
    AActor* actor_;
    const NedTransform* ned_transform_;
	float saved_clockspeed_ = 1;
    msr::airlib::vector<msr::airlib::real_T> laser_angles_;
    msr::airlib::vector<msr::airlib::real_T> laser_azimuth_offsets_; // degrees, clockwise, per laser
    msr::airlib::vector<FVector> point_cloud_draw_;
	uint32 current_horizontal_angle_index_ = 0;
	TArray<float> horizontal_angles_;
	std::mt19937 gen_;
	std::normal_distribution<float> dist_;
    const msr::airlib::LidarSimpleParams sensor_params_;
    msr::airlib::Pose sensor_reference_frame_;
    const float draw_time_;
    const bool external_;
    std::shared_ptr<GpuState> gpu_ = std::make_shared<GpuState>();
    bool logged_caster_ = false;
};