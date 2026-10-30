// Unitree Go2 pawn for HERCULES (2026-10).
//
// The dog is an AirSim ComputerVision vehicle: Unreal does no locomotion physics for it. An external
// process (PythonClient/go2/go2_mujoco_bridge.py) simulates the Go2 in MuJoCo, runs the walking policy
// and streams body pose + 12 joint angles over a local TCP socket (wire protocol below, compatible with
// the 80-byte packet of L. Trujillo's S2R_Go2Sim receiver). This pawn poses a static-mesh rig built
// from the menagerie Go2 link meshes (/AirSim/Go2/Meshes) to match, so every AirSim camera / sensor
// attached to the vehicle (cameras, lidar, IMU, GPS, ground-truth kinematics) works as for any other
// HERCULES vehicle, and the ROS 2 bridge / Python client see it as a normal vehicle.
//
// Two-way link (optional, negotiated by a HELLO packet from the bridge):
//   * the pawn samples the Unreal ground around the dog with line traces and sends it back as a
//     heightfield window, so MuJoCo feels the real terrain (ramps, obstacle tops, cliffs at walls);
//   * it also reports Unreal sim time, so the bridge can run in lockstep with the simulator.
//
// Frames: MuJoCo is right-handed (x fwd, y left, z up, metres); Unreal is left-handed (x fwd, y right,
// z up, cm). Mapping: p_ue = (100 x, -100 y, 100 z); q_ue = (w, -x, y, -z); a joint rotation by theta
// about MuJoCo x/z becomes -theta about Unreal x/z, about y it is unchanged. The MuJoCo world frame is
// anchored at the pawn's placed pose (first pose packet): position deltas are rotated by the placed yaw.
#pragma once

#include "CoreMinimal.h"
#include "Vehicles/ComputerVision/ComputerVisionPawn.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "CollisionQueryParams.h"
#include "Go2Pawn.generated.h"

class FSocket;

#pragma pack(push, 1)
// bridge -> pawn, 80 bytes, little-endian ("GO2\0" protocol of S2R_Go2Sim)
struct FGo2PosePacket
{
    uint32 magic; // 0x474F3200
    float body_x, body_y, body_z; // m, MuJoCo world frame
    float qw, qx, qy, qz; // body orientation, MuJoCo frame
    float joints[12]; // rad, FL FR RL RR x (hip, thigh, calf)
};
static_assert(sizeof(FGo2PosePacket) == 80, "FGo2PosePacket must be 80 bytes");

// bridge -> pawn, 16 bytes: capability handshake (HERCULES extension; Lucas's bridge never sends it)
struct FGo2HelloPacket
{
    uint32 magic; // 0x474F3202
    uint32 flags; // bit0: MuJoCo already stands on the UE heightfield (do NOT add the ground trace height)
                  // bit1: send feedback packets   bit2: include the terrain grid in feedback
    float grid_half_m; // half extent of the terrain window (m)
    float grid_cell_m; // cell size (m)
};
static_assert(sizeof(FGo2HelloPacket) == 16, "FGo2HelloPacket must be 16 bytes");

// pawn -> bridge, 32-byte header followed by grid_n*grid_n floats (row = y, col = x, MuJoCo frame,
// heights in m relative to the MuJoCo ground reference z=0)
struct FGo2FeedbackHeader
{
    uint32 magic; // 0x474F3201
    uint32 grid_n; // 0 = no grid in this packet
    float sim_time; // Unreal world time (s), stops when the sim is paused
    float ground_z; // ground height under the body (m, MuJoCo frame)
    float cell; // grid cell (m)
    float origin_x, origin_y; // MuJoCo xy of grid[0][0]
    float reserved;
};
static_assert(sizeof(FGo2FeedbackHeader) == 32, "FGo2FeedbackHeader must be 32 bytes");
#pragma pack(pop)

UCLASS()
class AGo2Pawn : public AComputerVisionPawn
{
    GENERATED_BODY()

public:
    AGo2Pawn();

    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    // APawn::GetVelocity() reads the movement component (we have none) -> AirSim's kinematics/IMU saw zero
    // linear velocity; report the finite-difference velocity of the posed body instead.
    virtual FVector GetVelocity() const override { return velocity_; }

    // TCP port the pawn listens on. Overridden by env GO2_PORT_<VehicleName>, then GO2_PORT.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    int32 ListenPort = 12345;

    // Pose/joint smoothing rate (1/s) toward the latest packet; <= 0 snaps. Env GO2_LERP.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    float LerpRate = 30.0f;

    // Probe height above the body for terrain/ground traces (cm). Anything whose top is higher than this
    // above the body is "overhead" (passable); a trace that starts inside geometry is a wall (cliff).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    float TerrainProbeUpCm = 150.0f;

    // Terrain sweep is spread over this many ticks (rows per tick = grid_n / this).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    int32 TerrainSweepTicks = 5;

    // Standing height of the body origin above ground used for the initial placement (cm).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    float StandHeightCm = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Go2")
    bool bDrawDebug = false;

    // Apply joint angles (rad, MuJoCo order) to the rig.
    void setJointAngles(const float* joints_rad);
    bool isBridgeConnected() const { return client_ != nullptr; }

private:
    // ---- rig ----
    UPROPERTY()
    USceneComponent* body_ = nullptr;
    UPROPERTY()
    TArray<UStaticMeshComponent*> base_meshes_;
    UPROPERTY()
    USceneComponent* hip_[4] = { nullptr, nullptr, nullptr, nullptr };
    UPROPERTY()
    USceneComponent* thigh_[4] = { nullptr, nullptr, nullptr, nullptr };
    UPROPERTY()
    USceneComponent* calf_[4] = { nullptr, nullptr, nullptr, nullptr };
    UPROPERTY()
    TArray<UStaticMeshComponent*> leg_meshes_;

    UStaticMeshComponent* addMesh(const FString& comp_name, const TCHAR* mesh_asset, USceneComponent* parent, const FVector& rel_loc_cm, const FQuat& rel_rot);
    void loadMeshes();
    void applyStandingPose();

    // ---- socket ----
    FSocket* listener_ = nullptr;
    FSocket* client_ = nullptr;
    TArray<uint8> rx_;
    int32 port_ = 12345;
    void openListener();
    void closeSockets();
    void pumpSocket(float dt);
    void handlePose(const FGo2PosePacket& p);
    void handleHello(const FGo2HelloPacket& h);
    bool sendAll(const uint8* data, int32 len);

    // ---- bridge capabilities (from HELLO) ----
    bool hfield_mode_ = false; // bridge stands on our heightfield -> MuJoCo z is absolute
    bool want_feedback_ = false;
    bool want_grid_ = false;
    int32 grid_n_ = 0;
    float grid_cell_m_ = 0.05f;

    // ---- frame anchor ----
    bool anchored_ = false; // set by the first pose packet
    FVector anchor_loc_ = FVector::ZeroVector; // pawn XY at anchor (UE, cm)
    FQuat anchor_yaw_ = FQuat::Identity; // placed yaw
    float mj_x0_ = 0.f, mj_y0_ = 0.f; // MuJoCo xy at anchor
    float ground_z0_ = 0.f; // UE z of the ground at the placed position = MuJoCo z=0 (cm)
    FVector mjToUe(float x, float y, float z) const; // MuJoCo world (m) -> UE world (cm), z through ground_z0_
    void ueToMj(const FVector& ue, float& x, float& y) const;

    // ---- targets / smoothing ----
    bool have_pose_ = false;
    FVector target_loc_ = FVector::ZeroVector;
    FQuat target_rot_ = FQuat::Identity;
    float target_joints_[12] = { 0 };
    float cur_joints_[12] = { 0 };
    FVector last_loc_ = FVector::ZeroVector;
    FVector velocity_ = FVector::ZeroVector; // cm/s, world
    float last_ground_z_ = 0.f; // trace mode: last accepted ground z under the body (cm)
    bool have_ground_ = false;
    int64 packets_ = 0;

    // ---- terrain sweep ----
    TArray<float> grid_; // grid_n_*grid_n_, m
    int32 sweep_row_ = 0; // next row to trace; grid_n_ = sweep done
    float sweep_ox_ = 0.f, sweep_oy_ = 0.f; // MuJoCo xy of grid[0][0] for the running sweep
    void terrainTick();
    void sendFeedback(bool with_grid);
    FCollisionQueryParams trace_params_;
    void refreshTraceParams();
    bool traceGround(const FVector& xy_ue, float probe_top_z, float& out_z) const;
};
