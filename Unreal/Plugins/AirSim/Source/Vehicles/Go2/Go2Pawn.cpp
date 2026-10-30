// Unitree Go2 pawn for HERCULES -- see Go2Pawn.h for the design and the wire protocol.
#include "Go2Pawn.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Common/TcpSocketBuilder.h"
#include "HAL/PlatformMisc.h"
#include "Misc/CString.h"
#include "DrawDebugHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogGo2, Log, All);

namespace
{
    constexpr uint32 kMagicPose = 0x474F3200u;
    constexpr uint32 kMagicFeedback = 0x474F3201u;
    constexpr uint32 kMagicHello = 0x474F3202u;
    constexpr float kM2Cm = 100.0f;
    const TCHAR* kMeshDir = TEXT("/AirSim/Go2/Meshes/");

    // menagerie unitree_go2/go2.xml geometry (m); legs in MuJoCo order FL, FR, RL, RR
    const FVector kHipPos[4] = { FVector(0.1934f, 0.0465f, 0.f), FVector(0.1934f, -0.0465f, 0.f),
                                 FVector(-0.1934f, 0.0465f, 0.f), FVector(-0.1934f, -0.0465f, 0.f) };
    const float kThighY[4] = { 0.0955f, -0.0955f, 0.0955f, -0.0955f };
    constexpr float kCalfZ = -0.213f;
    constexpr float kFootZ = -0.213f;
    const float kStandJoints[12] = { 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f };

    inline FVector mjVecToUe(const FVector& v_m) { return FVector(v_m.X * kM2Cm, -v_m.Y * kM2Cm, v_m.Z * kM2Cm); }
    // MuJoCo (w,x,y,z) right-handed -> Unreal left-handed: mirror y => (w, -x, y, -z)
    inline FQuat mjQuatToUe(float w, float x, float y, float z) { return FQuat(-x, y, -z, w).GetNormalized(); }
}

AGo2Pawn::AGo2Pawn()
{
    PrimaryActorTick.bCanEverTick = true;

    body_ = CreateDefaultSubobject<USceneComponent>(TEXT("go2_body"));
    body_->SetupAttachment(RootComponent);

    static const TCHAR* base_names[5] = { TEXT("SM_go2_base_0"), TEXT("SM_go2_base_1"), TEXT("SM_go2_base_2"),
                                          TEXT("SM_go2_base_3"), TEXT("SM_go2_base_4") };
    for (int i = 0; i < 5; ++i)
        base_meshes_.Add(addMesh(FString::Printf(TEXT("go2_base_%d"), i), base_names[i], body_, FVector::ZeroVector, FQuat::Identity));

    static const TCHAR* leg_names[4] = { TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR") };
    // hip mesh orientation per leg in go2.xml: FL identity, FR 180 deg about x, RL 180 deg about y, RR 180 deg about z
    const FQuat hip_mesh_rot[4] = { FQuat::Identity, mjQuatToUe(0.f, 1.f, 0.f, 0.f), mjQuatToUe(0.f, 0.f, 1.f, 0.f), mjQuatToUe(0.f, 0.f, 0.f, 1.f) };
    for (int i = 0; i < 4; ++i)
    {
        const bool mirror = (i == 1 || i == 3); // right legs use the *_mirror meshes
        const FString L(leg_names[i]);

        hip_[i] = CreateDefaultSubobject<USceneComponent>(*(L + TEXT("_hip")));
        hip_[i]->SetupAttachment(body_);
        hip_[i]->SetRelativeLocation(mjVecToUe(kHipPos[i]));
        leg_meshes_.Add(addMesh(L + TEXT("_hip_0"), TEXT("SM_go2_hip_0"), hip_[i], FVector::ZeroVector, hip_mesh_rot[i]));
        leg_meshes_.Add(addMesh(L + TEXT("_hip_1"), TEXT("SM_go2_hip_1"), hip_[i], FVector::ZeroVector, hip_mesh_rot[i]));

        thigh_[i] = CreateDefaultSubobject<USceneComponent>(*(L + TEXT("_thigh")));
        thigh_[i]->SetupAttachment(hip_[i]);
        thigh_[i]->SetRelativeLocation(mjVecToUe(FVector(0.f, kThighY[i], 0.f)));
        leg_meshes_.Add(addMesh(L + TEXT("_thigh_0"), mirror ? TEXT("SM_go2_thigh_mirror_0") : TEXT("SM_go2_thigh_0"), thigh_[i], FVector::ZeroVector, FQuat::Identity));
        leg_meshes_.Add(addMesh(L + TEXT("_thigh_1"), mirror ? TEXT("SM_go2_thigh_mirror_1") : TEXT("SM_go2_thigh_1"), thigh_[i], FVector::ZeroVector, FQuat::Identity));

        calf_[i] = CreateDefaultSubobject<USceneComponent>(*(L + TEXT("_calf")));
        calf_[i]->SetupAttachment(thigh_[i]);
        calf_[i]->SetRelativeLocation(mjVecToUe(FVector(0.f, 0.f, kCalfZ)));
        leg_meshes_.Add(addMesh(L + TEXT("_calf_0"), mirror ? TEXT("SM_go2_calf_mirror_0") : TEXT("SM_go2_calf_0"), calf_[i], FVector::ZeroVector, FQuat::Identity));
        leg_meshes_.Add(addMesh(L + TEXT("_calf_1"), mirror ? TEXT("SM_go2_calf_mirror_1") : TEXT("SM_go2_calf_1"), calf_[i], FVector::ZeroVector, FQuat::Identity));
        leg_meshes_.Add(addMesh(L + TEXT("_foot"), TEXT("SM_go2_foot"), calf_[i], mjVecToUe(FVector(0.f, 0.f, kFootZ)), FQuat::Identity));
    }
}

UStaticMeshComponent* AGo2Pawn::addMesh(const FString& comp_name, const TCHAR* mesh_asset, USceneComponent* parent, const FVector& rel_loc_cm, const FQuat& rel_rot)
{
    UStaticMeshComponent* c = CreateDefaultSubobject<UStaticMeshComponent>(*comp_name);
    c->SetupAttachment(parent);
    c->SetRelativeLocation(rel_loc_cm);
    c->SetRelativeRotation(rel_rot);
    c->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    c->SetCollisionProfileName(TEXT("NoCollision"));
    c->SetCastShadow(true);
    c->ComponentTags.Add(FName(mesh_asset)); // resolved in loadMeshes()
    return c;
}

void AGo2Pawn::loadMeshes()
{
    int32 missing = 0;
    auto load = [&](UStaticMeshComponent* c) {
        if (!c || c->ComponentTags.Num() == 0) return;
        const FString name = c->ComponentTags[0].ToString();
        const FString path = FString::Printf(TEXT("%s%s.%s"), kMeshDir, *name, *name);
        UStaticMesh* mesh = LoadObject<UStaticMesh>(nullptr, *path);
        if (mesh) c->SetStaticMesh(mesh);
        else { ++missing; UE_LOG(LogGo2, Warning, TEXT("%s: mesh asset missing: %s"), *GetName(), *path); }
    };
    for (auto* c : base_meshes_) load(c);
    for (auto* c : leg_meshes_) load(c);
    UE_LOG(LogGo2, Log, TEXT("%s: rig loaded (%d meshes, %d missing)"), *GetName(), base_meshes_.Num() + leg_meshes_.Num(), missing);
}

void AGo2Pawn::setJointAngles(const float* j)
{
    for (int i = 0; i < 4; ++i)
    {
        // MuJoCo: hip about +x, thigh/calf about +y (right-handed). Mirrored into Unreal: x -> -theta, y -> +theta.
        if (hip_[i]) hip_[i]->SetRelativeRotation(FQuat(FVector::ForwardVector, -j[3 * i + 0]));
        if (thigh_[i]) thigh_[i]->SetRelativeRotation(FQuat(FVector::RightVector, j[3 * i + 1]));
        if (calf_[i]) calf_[i]->SetRelativeRotation(FQuat(FVector::RightVector, j[3 * i + 2]));
    }
}

void AGo2Pawn::applyStandingPose()
{
    FMemory::Memcpy(cur_joints_, kStandJoints, sizeof(cur_joints_));
    FMemory::Memcpy(target_joints_, kStandJoints, sizeof(target_joints_));
    setJointAngles(cur_joints_);
}

void AGo2Pawn::BeginPlay()
{
    Super::BeginPlay();
    loadMeshes();

    // env overrides: GO2_PORT_<VehicleName>, GO2_PORT, GO2_LERP
    port_ = ListenPort;
    FString v = FPlatformMisc::GetEnvironmentVariable(*FString::Printf(TEXT("GO2_PORT_%s"), *GetName()));
    if (v.IsEmpty()) v = FPlatformMisc::GetEnvironmentVariable(TEXT("GO2_PORT"));
    if (!v.IsEmpty()) port_ = FCString::Atoi(*v);
    v = FPlatformMisc::GetEnvironmentVariable(TEXT("GO2_LERP"));
    if (!v.IsEmpty()) LerpRate = FCString::Atof(*v);

    // put the body origin at standing height over the ground under the placed position; that ground is MuJoCo z=0
    refreshTraceParams();
    const FVector loc = GetActorLocation();
    float gz = 0.f;
    if (traceGround(loc, loc.Z + 200.f, gz))
    {
        ground_z0_ = gz;
        SetActorLocation(FVector(loc.X, loc.Y, gz + StandHeightCm), false, nullptr, ETeleportType::TeleportPhysics);
    }
    else
    {
        ground_z0_ = loc.Z - StandHeightCm;
        UE_LOG(LogGo2, Warning, TEXT("%s: no ground under the placed position; MuJoCo z=0 set %.0f cm below the actor"), *GetName(), StandHeightCm);
    }
    anchor_yaw_ = FRotator(0.f, GetActorRotation().Yaw, 0.f).Quaternion();
    last_loc_ = GetActorLocation();
    applyStandingPose();
    openListener();
}

void AGo2Pawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    closeSockets();
    Super::EndPlay(EndPlayReason);
}

// ---------------------------------------------------------------- frames

FVector AGo2Pawn::mjToUe(float x, float y, float z) const
{
    const FVector d((x - mj_x0_) * kM2Cm, -(y - mj_y0_) * kM2Cm, 0.f);
    const FVector r = anchor_yaw_.RotateVector(d);
    return FVector(anchor_loc_.X + r.X, anchor_loc_.Y + r.Y, ground_z0_ + z * kM2Cm);
}

void AGo2Pawn::ueToMj(const FVector& ue, float& x, float& y) const
{
    const FVector d = anchor_yaw_.Inverse().RotateVector(FVector(ue.X - anchor_loc_.X, ue.Y - anchor_loc_.Y, 0.f));
    x = mj_x0_ + d.X / kM2Cm;
    y = mj_y0_ - d.Y / kM2Cm;
}

// ---------------------------------------------------------------- tick

void AGo2Pawn::Tick(float dt)
{
    refreshTraceParams();
    pumpSocket(dt);

    if (have_pose_)
    {
        const float a = (LerpRate > 0.f) ? FMath::Clamp(LerpRate * dt, 0.f, 1.f) : 1.f;
        const FVector loc = FMath::Lerp(GetActorLocation(), target_loc_, a);
        const FQuat rot = FQuat::Slerp(GetActorQuat(), target_rot_, a).GetNormalized();
        SetActorLocationAndRotation(loc, rot, false, nullptr, ETeleportType::TeleportPhysics);
        for (int i = 0; i < 12; ++i) cur_joints_[i] += (target_joints_[i] - cur_joints_[i]) * a;
        setJointAngles(cur_joints_);
    }
    // AirSim's PawnSimApi derives the vehicle twist from the pawn velocity (we have no movement component)
    if (dt > 0.f && RootComponent)
    {
        const FVector now = GetActorLocation();
        velocity_ = (now - last_loc_) / dt;
        RootComponent->ComponentVelocity = velocity_;
        last_loc_ = now;
    }
    if (client_ && want_feedback_) terrainTick();

    Super::Tick(dt);
}

// ---------------------------------------------------------------- socket

void AGo2Pawn::openListener()
{
    for (int32 tries = 0; tries < 10; ++tries)
    {
        const int32 port = port_ + tries;
        FSocket* s = FTcpSocketBuilder(TEXT("go2_listen")).AsReusable().AsNonBlocking()
                         .BoundToAddress(FIPv4Address(127, 0, 0, 1)).BoundToPort(port).Listening(4).Build();
        if (s)
        {
            listener_ = s;
            port_ = port;
            UE_LOG(LogGo2, Log, TEXT("%s: listening for the MuJoCo bridge on 127.0.0.1:%d"), *GetName(), port_);
            return;
        }
    }
    UE_LOG(LogGo2, Error, TEXT("%s: could not bind any port in %d..%d"), *GetName(), port_, port_ + 9);
}

void AGo2Pawn::closeSockets()
{
    ISocketSubsystem* ss = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (client_) { client_->Close(); if (ss) ss->DestroySocket(client_); client_ = nullptr; }
    if (listener_) { listener_->Close(); if (ss) ss->DestroySocket(listener_); listener_ = nullptr; }
}

bool AGo2Pawn::sendAll(const uint8* data, int32 len)
{
    if (!client_) return false;
    int32 off = 0;
    while (off < len)
    {
        int32 sent = 0;
        if (!client_->Send(data + off, len - off, sent) || sent <= 0) return false; // would-block or error: drop
        off += sent;
    }
    return true;
}

void AGo2Pawn::pumpSocket(float dt)
{
    if (!listener_) return;
    ISocketSubsystem* ss = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!client_)
    {
        bool pending = false;
        if (listener_->HasPendingConnection(pending) && pending)
        {
            TSharedRef<FInternetAddr> addr = ss->CreateInternetAddr();
            client_ = listener_->Accept(*addr, TEXT("go2_bridge"));
            if (client_)
            {
                client_->SetNonBlocking(true);
                client_->SetNoDelay(true);
                int32 sz = 1 << 20;
                client_->SetSendBufferSize(sz, sz);
                client_->SetReceiveBufferSize(sz, sz);
                rx_.Reset();
                hfield_mode_ = want_feedback_ = want_grid_ = false;
                grid_n_ = 0;
                UE_LOG(LogGo2, Log, TEXT("%s: bridge connected from %s"), *GetName(), *addr->ToString(true));
            }
        }
        return;
    }

    // FSocketBSD::Recv on a non-blocking stream socket: true + 0 bytes = would block (no data yet),
    // false = graceful close or error.
    uint8 buf[65536];
    int32 n = 0;
    bool closed = false;
    while (true)
    {
        if (!client_->Recv(buf, sizeof(buf), n)) { closed = true; break; }
        if (n <= 0) break;
        rx_.Append(buf, n);
        if (n < (int32)sizeof(buf)) break;
    }
    if (closed || client_->GetConnectionState() == SCS_ConnectionError)
    {
        UE_LOG(LogGo2, Log, TEXT("%s: bridge disconnected after %lld packets"), *GetName(), packets_);
        client_->Close();
        ss->DestroySocket(client_);
        client_ = nullptr;
        return;
    }

    int32 off = 0;
    while (rx_.Num() - off >= 4)
    {
        uint32 magic = 0;
        FMemory::Memcpy(&magic, rx_.GetData() + off, 4);
        if (magic == kMagicPose)
        {
            if (rx_.Num() - off < (int32)sizeof(FGo2PosePacket)) break;
            FGo2PosePacket p;
            FMemory::Memcpy(&p, rx_.GetData() + off, sizeof(p));
            handlePose(p);
            off += sizeof(p);
        }
        else if (magic == kMagicHello)
        {
            if (rx_.Num() - off < (int32)sizeof(FGo2HelloPacket)) break;
            FGo2HelloPacket h;
            FMemory::Memcpy(&h, rx_.GetData() + off, sizeof(h));
            handleHello(h);
            off += sizeof(h);
        }
        else
        {
            off += 1; // resync on the magic number
        }
    }
    if (off > 0) rx_.RemoveAt(0, off, false);
}

void AGo2Pawn::handleHello(const FGo2HelloPacket& h)
{
    hfield_mode_ = (h.flags & 1u) != 0;
    want_feedback_ = (h.flags & 2u) != 0;
    want_grid_ = (h.flags & 4u) != 0;
    grid_cell_m_ = FMath::Max(0.01f, h.grid_cell_m);
    grid_n_ = want_grid_ ? 2 * FMath::RoundToInt(h.grid_half_m / grid_cell_m_) + 1 : 0;
    grid_n_ = FMath::Clamp(grid_n_, 0, 201);
    grid_.SetNumZeroed(grid_n_ * grid_n_);
    sweep_row_ = 0;
    UE_LOG(LogGo2, Log, TEXT("%s: HELLO hfield=%d feedback=%d grid=%d (n=%d cell=%.3f m)"), *GetName(),
           hfield_mode_, want_feedback_, want_grid_, grid_n_, grid_cell_m_);
}

void AGo2Pawn::handlePose(const FGo2PosePacket& p)
{
    if (!anchored_)
    {
        anchored_ = true;
        anchor_loc_ = GetActorLocation();
        mj_x0_ = p.body_x;
        mj_y0_ = p.body_y;
        UE_LOG(LogGo2, Log, TEXT("%s: anchored MuJoCo (%.2f, %.2f) at UE (%.0f, %.0f, ground %.0f)"), *GetName(),
               mj_x0_, mj_y0_, anchor_loc_.X, anchor_loc_.Y, ground_z0_);
    }
    FVector loc = mjToUe(p.body_x, p.body_y, p.body_z);
    if (!hfield_mode_)
    {
        // flat-floor bridge: follow the Unreal ground under the body (reject single-step jumps > 50 cm,
        // e.g. the trace landing on top of a wall the body overlaps)
        float gz = 0.f;
        if (traceGround(loc, loc.Z + TerrainProbeUpCm, gz) && (!have_ground_ || FMath::Abs(gz - last_ground_z_) < 50.f))
        {
            last_ground_z_ = gz;
            have_ground_ = true;
        }
        if (have_ground_) loc.Z = last_ground_z_ + p.body_z * kM2Cm;
    }
    target_loc_ = loc;
    target_rot_ = (anchor_yaw_ * mjQuatToUe(p.qw, p.qx, p.qy, p.qz)).GetNormalized();
    FMemory::Memcpy(target_joints_, p.joints, sizeof(target_joints_));
    if (!have_pose_)
    {
        have_pose_ = true;
        FMemory::Memcpy(cur_joints_, p.joints, sizeof(cur_joints_));
        SetActorLocationAndRotation(target_loc_, target_rot_, false, nullptr, ETeleportType::TeleportPhysics);
        last_loc_ = target_loc_;
    }
    ++packets_;
}

// ---------------------------------------------------------------- terrain

void AGo2Pawn::refreshTraceParams()
{
    trace_params_ = FCollisionQueryParams(SCENE_QUERY_STAT(Go2Ground), false, this);
    trace_params_.bFindInitialOverlaps = true;
    trace_params_.bReturnPhysicalMaterial = false;
    TArray<AActor*> attached;
    GetAttachedActors(attached);
    for (AActor* a : attached) trace_params_.AddIgnoredActor(a);
}

bool AGo2Pawn::traceGround(const FVector& xy, float top_z, float& out_z) const
{
    FHitResult hit;
    const FVector start(xy.X, xy.Y, top_z);
    const FVector end(xy.X, xy.Y, top_z - 5000.f);
    if (GetWorld()->LineTraceSingleByChannel(hit, start, end, ECC_WorldStatic, trace_params_))
    {
        out_z = hit.bStartPenetrating ? top_z : hit.ImpactPoint.Z;
        return true;
    }
    return false;
}

void AGo2Pawn::terrainTick()
{
    if (!anchored_) { sendFeedback(false); return; }
    if (!want_grid_ || grid_n_ <= 0) { sendFeedback(false); return; }

    const int32 n = grid_n_;
    const int32 rows_per_tick = FMath::Max(1, FMath::DivideAndRoundUp(n, FMath::Max(1, TerrainSweepTicks)));
    const FVector body = GetActorLocation();
    if (sweep_row_ == 0)
    {
        // start a sweep centred on the body, snapped to the MuJoCo cell lattice (so successive windows agree)
        float bx = 0.f, by = 0.f;
        ueToMj(body, bx, by);
        const float cx = FMath::RoundToFloat(bx / grid_cell_m_) * grid_cell_m_;
        const float cy = FMath::RoundToFloat(by / grid_cell_m_) * grid_cell_m_;
        sweep_ox_ = cx - 0.5f * (n - 1) * grid_cell_m_;
        sweep_oy_ = cy - 0.5f * (n - 1) * grid_cell_m_;
    }
    const float top_z = body.Z + TerrainProbeUpCm;
    const int32 row_end = FMath::Min(n, sweep_row_ + rows_per_tick);
    for (int32 i = sweep_row_; i < row_end; ++i)
    {
        const float my = sweep_oy_ + i * grid_cell_m_;
        for (int32 j = 0; j < n; ++j)
        {
            const float mx = sweep_ox_ + j * grid_cell_m_;
            const FVector ue = mjToUe(mx, my, 0.f);
            float gz = 0.f;
            float h_m = -0.5f; // no ground: a half-metre pit
            if (traceGround(ue, top_z, gz)) h_m = (gz - ground_z0_) / kM2Cm;
            grid_[i * n + j] = h_m;
            if (bDrawDebug && ((i | j) & 3) == 0)
                DrawDebugPoint(GetWorld(), FVector(ue.X, ue.Y, ground_z0_ + h_m * kM2Cm), 3.f, FColor::Green, false, 0.2f);
        }
    }
    sweep_row_ = row_end;
    if (sweep_row_ >= n)
    {
        sendFeedback(true);
        sweep_row_ = 0;
    }
    else
    {
        sendFeedback(false);
    }
}

void AGo2Pawn::sendFeedback(bool with_grid)
{
    if (!client_ || !want_feedback_) return;
    FGo2FeedbackHeader h;
    h.magic = kMagicFeedback;
    h.grid_n = with_grid ? grid_n_ : 0;
    h.sim_time = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f;
    float gz = 0.f;
    const FVector body = GetActorLocation();
    h.ground_z = traceGround(body, body.Z + TerrainProbeUpCm, gz) ? (gz - ground_z0_) / kM2Cm : -0.5f;
    h.cell = grid_cell_m_;
    h.origin_x = sweep_ox_;
    h.origin_y = sweep_oy_;
    h.reserved = 0.f;
    if (!sendAll(reinterpret_cast<const uint8*>(&h), sizeof(h))) return;
    if (with_grid && grid_n_ > 0)
        sendAll(reinterpret_cast<const uint8*>(grid_.GetData()), grid_.Num() * sizeof(float));
}
