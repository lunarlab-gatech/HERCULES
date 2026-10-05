// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "Async/ParallelFor.h"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "HerculesGpuRayCaster.h"
#include "HAL/IConsoleManager.h"
#include "UObject/UObjectIterator.h"
#include <random>

static TAutoConsoleVariable<int32> CVarLidarGpuRayTracing(
	TEXT("airsim.Lidar.GpuRayTracing"),
	1,
	TEXT("1: cast LiDAR rays on the GPU with hardware ray tracing where supported; 0: always trace them on the CPU"),
	ECVF_Default);

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting &setting,
									 AActor *actor, const NedTransform *ned_transform)
	: LidarSimple(setting), actor_(actor), ned_transform_(ned_transform),
	  sensor_params_(getParams()),
	  draw_time_(1.05f / sensor_params_.horizontal_rotation_frequency),
	  external_(getParams().external)
{
	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, getParams().min_noise_standard_deviation);
	point_cloud_draw_.clear();
	createLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{

	msr::airlib::LidarSimpleParams params = getParams();

	const auto number_of_lasers = params.number_of_channels;

	float horizontal_delta = (params.horizontal_FOV_end - params.horizontal_FOV_start) / float(params.measurement_per_cycle - 1);
	for (uint32 i = 0; i < params.measurement_per_cycle; i++)
	{
		horizontal_angles_.Add(params.horizontal_FOV_start + i * horizontal_delta);
	}

	if (number_of_lasers <= 0)
		return;

	// elevation and azimuth offset of each laser: evenly spread over the
	// vertical FOV, or a calibrated per-channel table (VerticalAngles /
	// AzimuthOffsets, e.g. from an Ouster sensor's metadata)
	laser_angles_ = params.channelElevations();
	laser_azimuth_offsets_.clear();
	for (auto i = 0u; i < number_of_lasers; ++i)
		laser_azimuth_offsets_.emplace_back(params.channelAzimuthOffset(i));

	current_horizontal_angle_index_ = horizontal_angles_.Num() - 1;
}

// Set echo object in correct pose in physical world
void UnrealLidarSensor::updatePose(const msr::airlib::Pose &sensor_pose, const msr::airlib::Pose &vehicle_pose)
{
	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
	// DRAW DEBUG
	if (sensor_params_.draw_sensor)
	{
		FVector sensor_position;
		if (external_)
		{
			sensor_position = ned_transform_->toFVector(sensor_reference_frame_.position, 100, true);
		}
		else
		{
			sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		}
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, draw_time_);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_, 10);
	}
}

// Get echo pose in Local NED
void UnrealLidarSensor::getLocalPose(msr::airlib::Pose &sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
	;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
}

// Pause Unreal simulation
void UnrealLidarSensor::pause(const bool is_paused)
{
	if (is_paused)
	{
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else
	{
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
bool UnrealLidarSensor::getPointCloud(const msr::airlib::Pose &lidar_pose,
									  const msr::airlib::Pose &vehicle_pose,
									  const msr::airlib::TTimeDelta delta_time,
									  msr::airlib::vector<msr::airlib::real_T> &point_cloud,
									  msr::airlib::vector<std::string> &groundtruth,
									  msr::airlib::vector<msr::airlib::real_T> &point_cloud_final,
									  msr::airlib::vector<std::string> &groundtruth_final)
{
	updatePose(lidar_pose, vehicle_pose);

	bool refresh = false;
	msr::airlib::LidarSimpleParams params = getParams();
	const uint32 number_of_lasers = params.number_of_channels;
	const uint32 total_points = number_of_lasers * params.measurement_per_cycle;

	if (point_cloud.size() == 0)
	{
		point_cloud.assign(total_points * 3, 0);
		groundtruth.assign(total_points, "out_of_range");
	}

	// --- synthesize dt when paused (or nearly paused) and use it everywhere ---
	float dt = static_cast<float>(delta_time);
	if (dt <= 1e-6f)
	{
		// exactly one revolution period at the current horizontal rotation frequency
		dt = 1.0f / params.horizontal_rotation_frequency; // e.g., 0.1f for 10 Hz
	}

	// How much azimuth we advance this tick
	const float angle_distance_of_tick = params.horizontal_rotation_frequency * 360.0f * dt;
	const double angle_distance_per_measure = 360.0 / params.measurement_per_cycle;

	// Number of horizontal samples to cast this tick (per laser/channel)
	uint32 points_to_scan_with_one_laser_temp =
		FMath::RoundHalfFromZero(angle_distance_of_tick / angle_distance_per_measure);

	// Never drop to 0 when paused
	if (points_to_scan_with_one_laser_temp == 0)
	{
		points_to_scan_with_one_laser_temp = 1;
	}

	// Cap so a single sweep can fit (optional; or disable limit_points in settings)
	const uint32 max_points_full_sweep = params.measurement_per_cycle * number_of_lasers;
	if (params.limit_points && points_to_scan_with_one_laser_temp * number_of_lasers > max_points_full_sweep)
	{
		points_to_scan_with_one_laser_temp = max_points_full_sweep / number_of_lasers;
	}

	const uint32 points_to_scan_with_one_laser = points_to_scan_with_one_laser_temp;

	// normalize FOV start/end
	float laser_start = std::fmod(360.0f + params.horizontal_FOV_start, 360.0f);
	float laser_end = std::fmod(360.0f + params.horizontal_FOV_end, 360.0f);

	float previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];

	if (sensor_params_.draw_debug_points)
	{
		point_cloud_draw_.clear();
		point_cloud_draw_.assign(points_to_scan_with_one_laser * number_of_lasers, FVector());
	}

	// the horizontal angles to scan this tick, in order, up to the end of the sweep
	TArray<TPair<uint32, uint32>> scan_angles; // angle index, step (1-based)
	scan_angles.Reserve(points_to_scan_with_one_laser);
	bool sweep_complete = false;
	for (uint32 i = 1; i <= points_to_scan_with_one_laser; ++i)
	{
		if (current_horizontal_angle_index_ == horizontal_angles_.Num() - 1)
			current_horizontal_angle_index_ = 0;
		else
			current_horizontal_angle_index_ += 1;

		float horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];

		// wrap → full sweep completed: publish once this tick's points are in, and stop here
		if ((previous_horizontal_angle > horizontal_angle) && (point_cloud.size() != 0))
		{
			sweep_complete = true;
			break;
		}

		// Avoid duplicate angle
		if ((horizontal_angle - previous_horizontal_angle) <= 0.00005f && (horizontal_angle - 0) >= 0.00005f)
		{
			previous_horizontal_angle = horizontal_angle;
			continue;
		}

		// Skip outside horizontal FOV
		if (!VectorMath::isAngleBetweenAngles(horizontal_angle, laser_start, laser_end))
		{
			previous_horizontal_angle = horizontal_angle;
			continue;
		}

		scan_angles.Emplace(current_horizontal_angle_index_, i);
		previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];
	}

	const bool use_gpu = CVarLidarGpuRayTracing.GetValueOnAnyThread() != 0 && FHerculesGpuRayCaster::IsSupported() && actor_->GetWorld() != nullptr;
	if (!logged_caster_)
	{
		UE_LOG(LogTemp, Log, TEXT("LiDAR on %s: rays cast on the %s"), *actor_->GetName(), use_gpu ? TEXT("GPU (hardware ray tracing)") : TEXT("CPU"));
		logged_caster_ = true;
	}
	if (use_gpu)
		return castOnGpu(lidar_pose, vehicle_pose, params, scan_angles, sweep_complete, total_points, point_cloud_final, groundtruth_final);

	// shoot the lasers of all those angles in one parallel batch: one batch per angle (one ray per
	// channel) spent more time handing out and waiting for the work than tracing. Noise is drawn
	// here, as the generator can't be shared between threads.
	const int32 ray_count = scan_angles.Num() * number_of_lasers;
	TArray<float> noise_samples;
	if (params.generate_noise)
	{
		noise_samples.SetNumUninitialized(ray_count);
		for (float &sample : noise_samples)
			sample = dist_(gen_);
	}
	ParallelFor(ray_count, [&](int32 ray)
				{
const uint32 laser = ray % number_of_lasers;
const uint32 angle_index = scan_angles[ray / number_of_lasers].Key;
const uint32 i = scan_angles[ray / number_of_lasers].Value;
float  vertical_angle      = laser_angles_[laser];
uint32 current_point_index = number_of_lasers * angle_index + laser;
uint32 draw_index          = number_of_lasers * (i - 1) + laser; // fixed: i-1 to stay in-bounds

Vector3r point;
FVector  draw_point;
std::string label;

if (shootLaser(lidar_pose, vehicle_pose, laser, horizontal_angles_[angle_index] + laser_azimuth_offsets_[laser], vertical_angle,
params, params.generate_noise ? noise_samples[ray] : 0.0f, point, label, draw_point))
{
point_cloud[current_point_index * 3    ] = point.x();
point_cloud[current_point_index * 3 + 1] = point.y();
point_cloud[current_point_index * 3 + 2] = point.z();
groundtruth[current_point_index] = label;

if (sensor_params_.draw_debug_points)
point_cloud_draw_[draw_index] = draw_point;
} });

	if (sweep_complete)
	{
		if ((((int)point_cloud.size() / 3) != (int)total_points) ||
			(groundtruth.size() != total_points))
		{
			UE_LOG(LogTemp, Warning, TEXT("Pointcloud or labels incorrect size! points:%i labels:%i"),
				   (int)(point_cloud.size() / 3), groundtruth.size());
		}

		point_cloud_final = point_cloud;
		groundtruth_final = groundtruth;

		// prepare buffers for the next sweep
		point_cloud.assign(total_points * 3, 0);
		groundtruth.assign(total_points, "out_of_range");

		refresh = true;
	}

	if (sensor_params_.draw_debug_points)
	{
		for (uint32 j = 0; j < point_cloud_draw_.size(); j++)
		{
			UAirBlueprintLib::DrawPoint(
				actor_->GetWorld(),
				point_cloud_draw_[j],
				5,
				FColor::Green,
				false,
				(1.0f / (sensor_params_.horizontal_rotation_frequency * 2.0f)));
		}
	}

	return refresh;
}

FVector UnrealLidarSensor::Vector3rToFVector(const Vector3r &input_vector)
{
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose &lidar_pose, const msr::airlib::Pose &vehicle_pose,
								   const uint32 laser, const float horizontal_angle, const float vertical_angle,
								   const msr::airlib::LidarSimpleParams &params, const float noise_sample, Vector3r &point, std::string &label, FVector &raw_point)
{
	// start position
	Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

	// We need to compose rotations here rather than rotate a vector by a quaternion
	// Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

	// get ray quaternion in lidar frame (angles must be in radians)
	msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
		msr::airlib::Utils::degreesToRadians(vertical_angle),	 // pitch - rotation around Y axis
		0,														 // roll  - rotation around X axis
		msr::airlib::Utils::degreesToRadians(horizontal_angle)); // yaw   - rotation around Z axis

	// get ray quaternion in body frame
	msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);

	// get ray quaternion in world frame
	msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

	// get ray vector (end position)
	Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;

	FHitResult hit_result = FHitResult(ForceInit);
	TArray<AActor *> actorArray;
	// actorArray.Add(actor_);
	bool is_hit;
	if (params.external)
	{
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->toFVector(start, 100, true), ned_transform_->toFVector(end, 100, true), hit_result, actorArray, ECC_Visibility, true, true);
	}
	else
	{
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actorArray, ECC_Visibility, true, true);
	}
	bool ignoreMaterial = false;
	if (hit_result.PhysMaterial != nullptr)
	{
		if (hit_result.PhysMaterial.Get()->GetFName().ToString().Contains("Lidar_Ignore_PhysicalMaterial"))
			ignoreMaterial = true;
	}
	if (is_hit && !ignoreMaterial)
	{

		FVector impact_point = hit_result.ImpactPoint;

		// Store the name the hit object.
		auto hitActor = hit_result.GetActor();
		if (hitActor != nullptr)
		{
			label = TCHAR_TO_UTF8(*hitActor->GetName());
		}

		raw_point = impact_point;

		// if (label.empty())
		//{
		//	UE_LOG(LogTemp, Warning, TEXT("Empty label!"));
		// }
		//  If enabled add range noise
		if (params.generate_noise)
		{
			// Add noise based on normal distribution taking into account scaling of noise with distance
			float distance_noise = noise_sample * (1 + ((hit_result.Distance / 100) / params.range) * (params.noise_distance_scale - 1));

			Vector3r impact_point_local = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * ((hit_result.Distance / 100) + distance_noise) + start;
			if (params.external)
			{
				impact_point = ned_transform_->fromRelativeNed(impact_point_local);
			}
			else
			{
				impact_point = ned_transform_->fromLocalNed(impact_point_local);
			}
		}

		raw_point = impact_point;

		Vector3r point_v_i;
		if (params.external)
		{
			point_v_i = ned_transform_->toVector3r(impact_point, 0.01, true);
		}
		else
		{
			point_v_i = ned_transform_->toLocalNed(impact_point);
		}

		// tranform to lidar frame
		point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);

		return true;
	}
	else
	{
		return false;
	}
}

// The LiDAR's position in Unreal coordinates, and a point there converted to the LiDAR's frame, as the
// CPU traces do
static FVector lidarStart(const NedTransform* ned_transform, bool external, const msr::airlib::Vector3r& start)
{
	return external ? ned_transform->toFVector(start, 100, true) : ned_transform->fromLocalNed(start);
}

UnrealLidarSensor::Vector3r UnrealLidarSensor::toLidarFrame(const GpuBatch& batch, const FVector& impact_point) const
{
	const Vector3r point_v_i = external_ ? ned_transform_->toVector3r(impact_point, 0.01, true) : ned_transform_->toLocalNed(impact_point);
	return VectorMath::transformToBodyFrame(point_v_i, batch.lidar_pose + batch.vehicle_pose, true);
}

bool UnrealLidarSensor::castOnGpu(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose, const msr::airlib::LidarSimpleParams& params,
								  const TArray<TPair<uint32, uint32>>& scan_angles, bool sweep_complete, uint32 total_points,
								  msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final)
{
	const uint32 number_of_lasers = params.number_of_channels;
	const Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

	// The same rays as shootLaser, from the LiDAR's position at this tick
	TSharedRef<GpuBatch> batch = MakeShared<GpuBatch>();
	batch->lidar_pose = lidar_pose;
	batch->vehicle_pose = vehicle_pose;
	batch->start = lidarStart(ned_transform_, external_, start);
	const int32 ray_count = scan_angles.Num() * number_of_lasers;
	TArray<FHerculesGpuRay> rays;
	rays.Reserve(ray_count);
	batch->directions.Reserve(ray_count);
	batch->max_distances.Reserve(ray_count);
	batch->point_indices.Reserve(ray_count);
	for (const TPair<uint32, uint32>& angle : scan_angles)
	{
		for (uint32 laser = 0; laser < number_of_lasers; ++laser)
		{
			const msr::airlib::Quaternionr ray_q_l = VectorMath::toQuaternion(
				msr::airlib::Utils::degreesToRadians(laser_angles_[laser]), 0,
				msr::airlib::Utils::degreesToRadians(horizontal_angles_[angle.Key] + laser_azimuth_offsets_[laser]));
			const msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation), vehicle_pose.orientation);
			const Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;
			const FVector segment = lidarStart(ned_transform_, external_, end) - batch->start;
			const float length = segment.Size();
			FHerculesGpuRay& ray = rays.AddDefaulted_GetRef();
			ray.Direction = FVector3f(segment / FMath::Max(length, UE_SMALL_NUMBER));
			ray.MaxDistance = length;
			batch->directions.Add(ray.Direction);
			batch->max_distances.Add(length);
			batch->point_indices.Add(number_of_lasers * angle.Key + laser);
		}
	}
	if (params.generate_noise)
	{
		batch->noise_samples.SetNumUninitialized(ray_count);
		for (float& sample : batch->noise_samples)
			sample = dist_(gen_);
	}

	std::shared_ptr<GpuSweep> sweep;
	TArray<uint32> pass_through;
	{
		std::lock_guard<std::mutex> lock(gpu_->mutex);
		auto new_sweep = [total_points]() {
			auto created = std::make_shared<GpuSweep>();
			created->points.assign(total_points * 3, 0);
			created->labels.assign(total_points, "out_of_range");
			return created;
		};
		if (!gpu_->sweep)
			gpu_->sweep = new_sweep();
		sweep = gpu_->sweep;
		if (ray_count > 0)
			++sweep->outstanding;
		if (sweep_complete)
		{
			sweep->closed = true;
			gpu_->sweep = new_sweep();
			if (sweep->outstanding == 0)
				gpu_->complete.push_back(sweep);
		}
		pass_through = gpu_->pass_through;
	}

	if (ray_count > 0)
	{
		std::weak_ptr<GpuState> weak_state = gpu_;
		const FVector origin = batch->start;
		FHerculesGpuRayCaster::Submit(*actor_->GetWorld(), origin, MoveTemp(rays), MoveTemp(pass_through),
			[this, weak_state, sweep, batch](bool cast, TArray<FHerculesGpuHit>&& hits) {
				std::shared_ptr<GpuState> state = weak_state.lock();
				if (!state)
					return; // the sensor is gone
				collectGpuHits(*state, *sweep, *batch, cast, hits);
				std::lock_guard<std::mutex> lock(state->mutex);
				if (--sweep->outstanding == 0 && sweep->closed)
					state->complete.push_back(sweep);
			});
	}

	// Publish the newest sweep whose rays are all back (older ones would be overwritten anyway)
	std::lock_guard<std::mutex> lock(gpu_->mutex);
	if (gpu_->complete.empty())
		return false;
	GpuSweep& newest = *gpu_->complete.back();
	point_cloud_final = std::move(newest.points);
	groundtruth_final = std::move(newest.labels);
	gpu_->complete.clear();
	return true;
}

void UnrealLidarSensor::collectGpuHits(GpuState& state, GpuSweep& sweep, const GpuBatch& batch, bool cast, const TArray<FHerculesGpuHit>& hits)
{
	const msr::airlib::LidarSimpleParams& params = sensor_params_;
	for (int32 ray = 0; ray < batch.point_indices.Num(); ++ray)
	{
		Vector3r point;
		std::string label;
		FVector raw_point;
		bool hit = false;
		bool on_cpu = !cast;
		if (cast && hits[ray].IsHit())
		{
			const FVector direction(batch.directions[ray]);
			const GpuComponent* component = identifyGpuComponent(state, hits[ray].PrimitiveComponentId, batch.start + direction * hits[ray].Distance, direction);
			if (component == nullptr || component->pass_through || component->trace_on_cpu)
			{
				on_cpu = true;
			}
			else
			{
				float distance = hits[ray].Distance;
				if (params.generate_noise)
					distance += 100.0f * batch.noise_samples[ray] * (1 + ((hits[ray].Distance / 100) / params.range) * (params.noise_distance_scale - 1));
				raw_point = batch.start + direction * distance;
				point = toLidarFrame(batch, raw_point);
				label = component->label;
				hit = true;
			}
		}
		if (on_cpu)
			hit = traceOnCpu(batch, ray, point, label, raw_point);
		if (!hit)
			continue;
		const uint32 index = batch.point_indices[ray];
		sweep.points[index * 3] = point.x();
		sweep.points[index * 3 + 1] = point.y();
		sweep.points[index * 3 + 2] = point.z();
		sweep.labels[index] = label;
		if (params.draw_debug_points)
			UAirBlueprintLib::DrawPoint(actor_->GetWorld(), raw_point, 5, FColor::Green, false, 1.0f / (params.horizontal_rotation_frequency * 2.0f));
	}
}

// The component a GPU ray hit, with what the CPU traces would make of it, or null if it can't be told yet
// (the ray is then traced on the CPU). Game thread.
const UnrealLidarSensor::GpuComponent* UnrealLidarSensor::identifyGpuComponent(GpuState& state, uint32 component_id, const FVector& hit_point, const FVector& direction)
{
	if (const GpuComponent* known = state.components.Find(component_id))
		return known;

	auto describe = [&state, component_id](const UPrimitiveComponent& component) -> const GpuComponent* {
		GpuComponent& info = state.components.Add(component_id);
		info.label = component.GetOwner() != nullptr ? std::string(TCHAR_TO_UTF8(*component.GetOwner()->GetName())) : std::string();
		info.pass_through = !(component.IsQueryCollisionEnabled() && component.GetCollisionResponseToChannel(ECC_Visibility) == ECR_Block);
		auto ignores_lidar = [](const UPhysicalMaterial* material) {
			return material != nullptr && material->GetFName().ToString().Contains("Lidar_Ignore_PhysicalMaterial");
		};
		info.trace_on_cpu = ignores_lidar(component.BodyInstance.GetSimplePhysicalMaterial());
		for (int32 index = 0; index < component.GetNumMaterials() && !info.trace_on_cpu; ++index)
		{
			const UMaterialInterface* material = component.GetMaterial(index);
			info.trace_on_cpu = material != nullptr && ignores_lidar(material->GetPhysicalMaterial());
		}
		if (info.pass_through)
		{
			std::lock_guard<std::mutex> lock(state.mutex);
			state.pass_through.AddUnique(component_id);
		}
		state.unidentified.Remove(component_id);
		return &info;
	};

	// A short trace across the hit finds the component when the CPU traces would hit it as well
	UWorld* world = actor_->GetWorld();
	FHitResult probe;
	FCollisionQueryParams query(SCENE_QUERY_STAT(LidarGpuIdentify), true);
	if (world->LineTraceSingleByChannel(probe, hit_point - direction * 20.0, hit_point + direction * 20.0, ECC_Visibility, query)
		&& probe.GetComponent() != nullptr && probe.GetComponent()->GetPrimitiveSceneId().PrimIDValue == component_id)
		return describe(*probe.GetComponent());

	// Otherwise look it up among the world's components, at most every 2 s
	state.unidentified.Add(component_id);
	const double now = FPlatformTime::Seconds();
	if (now - state.last_scan_seconds < 2.0)
		return nullptr;
	state.last_scan_seconds = now;
	for (TObjectIterator<UPrimitiveComponent> it; it; ++it)
	{
		if (it->GetWorld() != world || !it->IsRegistered())
			continue;
		const uint32 id = it->GetPrimitiveSceneId().PrimIDValue;
		if (state.unidentified.Contains(id))
			describe(**it);
	}
	return state.components.Find(component_id);
}

// One ray exactly as the CPU traces cast it (see shootLaser)
bool UnrealLidarSensor::traceOnCpu(const GpuBatch& batch, int32 ray, Vector3r& point, std::string& label, FVector& raw_point)
{
	const msr::airlib::LidarSimpleParams& params = sensor_params_;
	const FVector direction(batch.directions[ray]);
	FHitResult hit_result(ForceInit);
	TArray<AActor*> ignore;
	const bool is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, batch.start, batch.start + direction * batch.max_distances[ray], hit_result, ignore, ECC_Visibility, true, true);
	if (!is_hit)
		return false;
	if (hit_result.PhysMaterial != nullptr && hit_result.PhysMaterial.Get()->GetFName().ToString().Contains("Lidar_Ignore_PhysicalMaterial"))
		return false;
	if (AActor* hit_actor = hit_result.GetActor())
		label = TCHAR_TO_UTF8(*hit_actor->GetName());
	float distance = hit_result.Distance;
	if (params.generate_noise)
		distance += 100.0f * batch.noise_samples[ray] * (1 + ((hit_result.Distance / 100) / params.range) * (params.noise_distance_scale - 1));
	raw_point = params.generate_noise ? batch.start + direction * distance : FVector(hit_result.ImpactPoint);
	point = toLidarFrame(batch, raw_point);
	return true;
}
