// MuJoCo-posed vehicle pawn for HERCULES (2026-10/11): the base of the Unitree Go2 (AGo2Pawn) and
// Unitree G1 (AG1Pawn) vehicles.
//
// The robot is an AirSim ComputerVision vehicle: Unreal does no locomotion physics for it. An external
// process (PythonClient/go2/go2_mujoco_bridge.py) simulates the robot in MuJoCo, runs the walking policy
// and streams the base pose (+ 12 joint angles) over a local TCP socket after every control step. This
// pawn follows that pose, so every AirSim camera / sensor attached to the vehicle (cameras, lidar, IMU,
// GPS, ground-truth kinematics) works as for any other HERCULES vehicle, and the ROS 2 bridge / Python
// client see it as a normal vehicle.
//
// Two ways to render the robot's body:
//   * a fixed rig built by a subclass from joint angles (AGo2Pawn: 12 joints, menagerie link meshes);
//   * generic "link" streaming: the bridge announces a list of static-mesh assets (MESHES packet) and then
//     sends the pose of every link relative to the base each step (LINKS packet). This renders any MuJoCo
//     model (the G1 humanoid uses it) without robot-specific code on the Unreal side.
//
// Two-way link (optional, negotiated by a HELLO packet from the bridge):
//   * the pawn samples the Unreal ground around the robot with line traces and sends it back as a
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
#include "MujocoPawn.generated.h"

class FSocket;

#pragma pack(push, 1)
// bridge -> pawn, 80 bytes, little-endian ("GO2\0" protocol of S2R_Go2Sim)
struct FGo2PosePacket
{
    uint32 magic; // 0x474F3200
    float body_x, body_y, body_z; // m, MuJoCo world frame
    float qw, qx, qy, qz; // base orientation, MuJoCo frame
    float joints[12]; // rad; Go2: FL FR RL RR x (hip, thigh, calf); G1: the 12 leg joints (unused by the rig)
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

// bridge -> pawn: the list of link meshes to render (generic rig). Header + n names.
// Asset path of link i: /AirSim/<set>/Meshes/<name_i>  (e.g. /AirSim/G1/Meshes/SM_g1_left_knee_link)
struct FMjMeshesHeader
{
    uint32 magic; // 0x474F3204
    uint32 count; // number of FMjMeshName records that follow
    char set[32]; // content folder under /AirSim, null-terminated
};
static_assert(sizeof(FMjMeshesHeader) == 40, "FMjMeshesHeader must be 40 bytes");
struct FMjMeshName
{
    char name[48]; // asset name, null-terminated
};
static_assert(sizeof(FMjMeshName) == 48, "FMjMeshName must be 48 bytes");

// bridge -> pawn: pose of every announced link relative to the base body (MuJoCo frame, m / wxyz),
// in the order of the MESHES packet. Header + count records.
struct FMjLinksHeader
{
    uint32 magic; // 0x474F3205
    uint32 count;
};
static_assert(sizeof(FMjLinksHeader) == 8, "FMjLinksHeader must be 8 bytes");
struct FMjLinkPose
{
    float x, y, z, qw, qx, qy, qz;
};
static_assert(sizeof(FMjLinkPose) == 28, "FMjLinkPose must be 28 bytes");
#pragma pack(pop)

namespace MjFrames
{
    constexpr float kM2Cm = 100.0f;
    inline FVector mjVecToUe(const FVector& v_m) { return FVector(v_m.X * kM2Cm, -v_m.Y * kM2Cm, v_m.Z * kM2Cm); }
    // MuJoCo (w,x,y,z) right-handed -> Unreal left-handed: mirror y => (w, -x, y, -z)
    inline FQuat mjQuatToUe(float w, float x, float y, float z) { return FQuat(-x, y, -z, w).GetNormalized(); }
}

UCLASS()
class AMujocoPawn : public AComputerVisionPawn
{
    GENERATED_BODY()

public:
    AMujocoPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    // APawn::GetVelocity() reads the movement component (we have none) -> AirSim's kinematics/IMU saw zero
    // linear velocity; report the finite-difference velocity of the posed body instead.
    virtual FVector GetVelocity() const override { return velocity_; }

    // TCP port the pawn listens on (first free of ListenPort..ListenPort+9). Overridden by env
    // GO2_PORT_<VehicleName>, then GO2_PORT.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    int32 ListenPort = 12345;

    // Pose/joint smoothing rate (1/s) toward the latest packet; <= 0 snaps. Env GO2_LERP.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    float LerpRate = 30.0f;

    // Probe height above the body for terrain/ground traces (cm). Anything whose top is higher than this
    // above the body is "overhead" (passable); a trace that starts inside geometry is a wall (cliff).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    float TerrainProbeUpCm = 150.0f;

    // Terrain sweep is spread over this many ticks (rows per tick = grid_n / this).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    int32 TerrainSweepTicks = 5;

    // Standing height of the base body origin above ground used for the initial placement (cm).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    float StandHeightCm = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
    bool bDrawDebug = false;

    bool isBridgeConnected() const { return client_ != nullptr; }

protected:
    // ---- rig hooks for subclasses (fixed rigs) ----
    virtual void setJointAngles(const float* joints_rad) {}
    virtual void applyStandingPose() {}
    virtual void loadMeshes() {}
    UStaticMeshComponent* addMesh(const FString& comp_name, const TCHAR* mesh_asset, USceneComponent* parent, const FVector& rel_loc_cm, const FQuat& rel_rot);
    bool loadMeshInto(UStaticMeshComponent* c, const FString& asset_path);

    // ---- generic link rig (MESHES / LINKS packets) ----
    UPROPERTY()
    TArray<UStaticMeshComponent*> links_;
    TArray<FVector> link_target_loc_;
    TArray<FQuat> link_target_rot_;
    bool have_links_ = false;
    void handleMeshes(const FMjMeshesHeader& h, const FMjMeshName* names);
    void handleLinks(const FMjLinksHeader& h, const FMjLinkPose* poses);
    void clearLinks();

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
    // Outgoing bytes are queued and flushed non-blocking in order, so a feedback packet is never sent
    // partially (a half-sent grid desynchronised the bridge's parser when the editor stalled).
    TArray<uint8> tx_;
    void queueSend(const uint8* data, int32 len);
    void flushSend();

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
