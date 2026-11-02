// MuJoCo-posed vehicle pawn for HERCULES -- see MujocoPawn.h for the design and the wire protocol.
#include "MujocoPawn.h"
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

DEFINE_LOG_CATEGORY_STATIC(LogMujocoPawn, Log, All);

using namespace MjFrames;

namespace
{
    constexpr uint32 kMagicPose = 0x474F3200u;
    constexpr uint32 kMagicFeedback = 0x474F3201u;
    constexpr uint32 kMagicHello = 0x474F3202u;
    constexpr uint32 kMagicMeshes = 0x474F3204u;
    constexpr uint32 kMagicLinks = 0x474F3205u;
    constexpr uint32 kMaxLinks = 256;

    FString cstrField(const char* field, int32 len)
    {
        char tmp[64] = { 0 };
        FMemory::Memcpy(tmp, field, FMath::Min(len, (int32)sizeof(tmp) - 1));
        return FString(UTF8_TO_TCHAR(tmp));
    }

    bool isAssetName(const FString& s)
    {
        if (s.IsEmpty()) return false;
        for (TCHAR c : s)
            if (!FChar::IsAlnum(c) && c != TEXT('_')) return false;
        return true;
    }
}

AMujocoPawn::AMujocoPawn()
{
    PrimaryActorTick.bCanEverTick = true;
}

UStaticMeshComponent* AMujocoPawn::addMesh(const FString& comp_name, const TCHAR* mesh_asset, USceneComponent* parent, const FVector& rel_loc_cm, const FQuat& rel_rot)
{
    UStaticMeshComponent* c = CreateDefaultSubobject<UStaticMeshComponent>(*comp_name);
    c->SetupAttachment(parent);
    c->SetRelativeLocation(rel_loc_cm);
    c->SetRelativeRotation(rel_rot);
    c->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    c->SetCollisionProfileName(TEXT("NoCollision"));
    c->SetCastShadow(true);
    c->ComponentTags.Add(FName(mesh_asset)); // resolved by the subclass' loadMeshes()
    return c;
}

bool AMujocoPawn::loadMeshInto(UStaticMeshComponent* c, const FString& asset_path)
{
    UStaticMesh* mesh = LoadObject<UStaticMesh>(nullptr, *asset_path);
    if (mesh)
    {
        c->SetStaticMesh(mesh);
        return true;
    }
    UE_LOG(LogMujocoPawn, Warning, TEXT("%s: mesh asset missing: %s"), *GetName(), *asset_path);
    return false;
}

void AMujocoPawn::BeginPlay()
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

    // put the base origin at standing height over the ground under the placed position; that ground is MuJoCo z=0
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
        UE_LOG(LogMujocoPawn, Warning, TEXT("%s: no ground under the placed position; MuJoCo z=0 set %.0f cm below the actor"), *GetName(), StandHeightCm);
    }
    anchor_yaw_ = FRotator(0.f, GetActorRotation().Yaw, 0.f).Quaternion();
    last_loc_ = GetActorLocation();
    applyStandingPose();
    openListener();
}

void AMujocoPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    closeSockets();
    Super::EndPlay(EndPlayReason);
}

// ---------------------------------------------------------------- frames

FVector AMujocoPawn::mjToUe(float x, float y, float z) const
{
    const FVector d((x - mj_x0_) * kM2Cm, -(y - mj_y0_) * kM2Cm, 0.f);
    const FVector r = anchor_yaw_.RotateVector(d);
    return FVector(anchor_loc_.X + r.X, anchor_loc_.Y + r.Y, ground_z0_ + z * kM2Cm);
}

void AMujocoPawn::ueToMj(const FVector& ue, float& x, float& y) const
{
    const FVector d = anchor_yaw_.Inverse().RotateVector(FVector(ue.X - anchor_loc_.X, ue.Y - anchor_loc_.Y, 0.f));
    x = mj_x0_ + d.X / kM2Cm;
    y = mj_y0_ - d.Y / kM2Cm;
}

// ---------------------------------------------------------------- tick

void AMujocoPawn::Tick(float dt)
{
    refreshTraceParams();
    pumpSocket(dt);

    const float a = (LerpRate > 0.f) ? FMath::Clamp(LerpRate * dt, 0.f, 1.f) : 1.f;
    if (have_pose_)
    {
        const FVector loc = FMath::Lerp(GetActorLocation(), target_loc_, a);
        const FQuat rot = FQuat::Slerp(GetActorQuat(), target_rot_, a).GetNormalized();
        SetActorLocationAndRotation(loc, rot, false, nullptr, ETeleportType::TeleportPhysics);
        for (int i = 0; i < 12; ++i) cur_joints_[i] += (target_joints_[i] - cur_joints_[i]) * a;
        setJointAngles(cur_joints_);
    }
    if (have_links_)
    {
        for (int32 i = 0; i < links_.Num(); ++i)
        {
            UStaticMeshComponent* c = links_[i];
            if (!c) continue;
            const FVector l = FMath::Lerp(c->GetRelativeLocation(), link_target_loc_[i], a);
            const FQuat r = FQuat::Slerp(c->GetRelativeRotation().Quaternion(), link_target_rot_[i], a).GetNormalized();
            c->SetRelativeLocationAndRotation(l, r);
        }
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
    flushSend();

    Super::Tick(dt);
}

// ---------------------------------------------------------------- generic link rig

void AMujocoPawn::clearLinks()
{
    for (UStaticMeshComponent* c : links_)
        if (c) c->DestroyComponent();
    links_.Reset();
    link_target_loc_.Reset();
    link_target_rot_.Reset();
    have_links_ = false;
}

void AMujocoPawn::handleMeshes(const FMjMeshesHeader& h, const FMjMeshName* names)
{
    const FString set = cstrField(h.set, sizeof(h.set));
    TArray<FString> wanted;
    for (uint32 i = 0; i < h.count; ++i) wanted.Add(cstrField(names[i].name, sizeof(names[i].name)));
    if (!isAssetName(set))
    {
        UE_LOG(LogMujocoPawn, Warning, TEXT("%s: MESHES rejected (bad set name '%s')"), *GetName(), *set);
        return;
    }
    // same list as before (bridge reconnect): keep the components
    bool same = links_.Num() == wanted.Num();
    for (int32 i = 0; same && i < wanted.Num(); ++i)
        same = links_[i] && links_[i]->ComponentTags.Num() > 0 && links_[i]->ComponentTags[0].ToString() == set + TEXT("/") + wanted[i];
    if (same)
    {
        have_links_ = false; // snap on the next LINKS packet
        return;
    }
    clearLinks();
    int32 missing = 0;
    for (int32 i = 0; i < wanted.Num(); ++i)
    {
        const FString& name = wanted[i];
        UStaticMeshComponent* c = NewObject<UStaticMeshComponent>(this, *FString::Printf(TEXT("link_%d_%s"), i, *name));
        c->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        c->SetCollisionProfileName(TEXT("NoCollision"));
        c->SetCastShadow(true);
        c->SetVisibility(false); // until the first LINKS packet places it
        c->ComponentTags.Add(FName(*(set + TEXT("/") + name)));
        c->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
        c->RegisterComponent();
        if (isAssetName(name))
        {
            if (!loadMeshInto(c, FString::Printf(TEXT("/AirSim/%s/Meshes/%s.%s"), *set, *name, *name))) ++missing;
        }
        else
        {
            ++missing;
            UE_LOG(LogMujocoPawn, Warning, TEXT("%s: bad link asset name '%s'"), *GetName(), *name);
        }
        links_.Add(c);
        link_target_loc_.Add(FVector::ZeroVector);
        link_target_rot_.Add(FQuat::Identity);
    }
    UE_LOG(LogMujocoPawn, Log, TEXT("%s: link rig '%s' with %d meshes (%d missing)"), *GetName(), *set, links_.Num(), missing);
}

void AMujocoPawn::handleLinks(const FMjLinksHeader& h, const FMjLinkPose* poses)
{
    if ((int32)h.count != links_.Num()) return;
    const bool first = !have_links_;
    for (int32 i = 0; i < links_.Num(); ++i)
    {
        const FMjLinkPose& p = poses[i];
        link_target_loc_[i] = mjVecToUe(FVector(p.x, p.y, p.z));
        link_target_rot_[i] = mjQuatToUe(p.qw, p.qx, p.qy, p.qz);
        if (first && links_[i])
        {
            links_[i]->SetRelativeLocationAndRotation(link_target_loc_[i], link_target_rot_[i]);
            links_[i]->SetVisibility(true);
        }
    }
    have_links_ = true;
}

// ---------------------------------------------------------------- socket

void AMujocoPawn::openListener()
{
    for (int32 tries = 0; tries < 10; ++tries)
    {
        const int32 port = port_ + tries;
        FSocket* s = FTcpSocketBuilder(TEXT("mujoco_listen")).AsReusable().AsNonBlocking()
                         .BoundToAddress(FIPv4Address(127, 0, 0, 1)).BoundToPort(port).Listening(4).Build();
        if (s)
        {
            listener_ = s;
            port_ = port;
            UE_LOG(LogMujocoPawn, Log, TEXT("%s: listening for the MuJoCo bridge on 127.0.0.1:%d"), *GetName(), port_);
            return;
        }
    }
    UE_LOG(LogMujocoPawn, Error, TEXT("%s: could not bind any port in %d..%d"), *GetName(), port_, port_ + 9);
}

void AMujocoPawn::closeSockets()
{
    ISocketSubsystem* ss = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (client_) { client_->Close(); if (ss) ss->DestroySocket(client_); client_ = nullptr; }
    if (listener_) { listener_->Close(); if (ss) ss->DestroySocket(listener_); listener_ = nullptr; }
}

void AMujocoPawn::queueSend(const uint8* data, int32 len)
{
    if (!client_) return;
    if (tx_.Num() > (8 << 20)) // reader stalled for a long time: drop the backlog rather than grow forever
    {
        tx_.Reset();
        UE_LOG(LogMujocoPawn, Warning, TEXT("%s: feedback backlog dropped (bridge not reading)"), *GetName());
    }
    tx_.Append(data, len);
}

void AMujocoPawn::flushSend()
{
    if (!client_ || tx_.Num() == 0) return;
    int32 off = 0;
    while (off < tx_.Num())
    {
        int32 sent = 0;
        if (!client_->Send(tx_.GetData() + off, tx_.Num() - off, sent) || sent <= 0) break; // would block: keep the rest
        off += sent;
    }
    if (off > 0) tx_.RemoveAt(0, off, false);
}

void AMujocoPawn::pumpSocket(float dt)
{
    if (!listener_) return;
    ISocketSubsystem* ss = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!client_)
    {
        bool pending = false;
        if (listener_->HasPendingConnection(pending) && pending)
        {
            TSharedRef<FInternetAddr> addr = ss->CreateInternetAddr();
            client_ = listener_->Accept(*addr, TEXT("mujoco_bridge"));
            if (client_)
            {
                client_->SetNonBlocking(true);
                client_->SetNoDelay(true);
                int32 sz = 1 << 20;
                client_->SetSendBufferSize(sz, sz);
                client_->SetReceiveBufferSize(sz, sz);
                rx_.Reset();
                tx_.Reset();
                hfield_mode_ = want_feedback_ = want_grid_ = false;
                // A (re)connecting bridge starts its MuJoCo world at the robot's CURRENT pose: anchor again here
                // (a second bridge used to teleport the dog back to the first anchor, 5 m through the air, and
                // the stumble that followed collapsed the gait on every other start).
                anchored_ = false;
                have_ground_ = false;
                grid_n_ = 0;
                UE_LOG(LogMujocoPawn, Log, TEXT("%s: bridge connected from %s"), *GetName(), *addr->ToString(true));
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
        UE_LOG(LogMujocoPawn, Log, TEXT("%s: bridge disconnected after %lld packets"), *GetName(), packets_);
        client_->Close();
        ss->DestroySocket(client_);
        client_ = nullptr;
        return;
    }

    int32 off = 0;
    while (rx_.Num() - off >= 4)
    {
        const int32 avail = rx_.Num() - off;
        uint32 magic = 0;
        FMemory::Memcpy(&magic, rx_.GetData() + off, 4);
        if (magic == kMagicPose)
        {
            if (avail < (int32)sizeof(FGo2PosePacket)) break;
            FGo2PosePacket p;
            FMemory::Memcpy(&p, rx_.GetData() + off, sizeof(p));
            handlePose(p);
            off += sizeof(p);
        }
        else if (magic == kMagicHello)
        {
            if (avail < (int32)sizeof(FGo2HelloPacket)) break;
            FGo2HelloPacket h;
            FMemory::Memcpy(&h, rx_.GetData() + off, sizeof(h));
            handleHello(h);
            off += sizeof(h);
        }
        else if (magic == kMagicMeshes)
        {
            if (avail < (int32)sizeof(FMjMeshesHeader)) break;
            FMjMeshesHeader h;
            FMemory::Memcpy(&h, rx_.GetData() + off, sizeof(h));
            if (h.count > kMaxLinks) { off += 4; continue; } // garbage: resync
            const int32 need = sizeof(FMjMeshesHeader) + (int32)h.count * (int32)sizeof(FMjMeshName);
            if (avail < need) break;
            TArray<FMjMeshName> names;
            names.SetNumUninitialized(h.count);
            if (h.count > 0) FMemory::Memcpy(names.GetData(), rx_.GetData() + off + sizeof(FMjMeshesHeader), h.count * sizeof(FMjMeshName));
            handleMeshes(h, names.GetData());
            off += need;
        }
        else if (magic == kMagicLinks)
        {
            if (avail < (int32)sizeof(FMjLinksHeader)) break;
            FMjLinksHeader h;
            FMemory::Memcpy(&h, rx_.GetData() + off, sizeof(h));
            if (h.count > kMaxLinks) { off += 4; continue; }
            const int32 need = sizeof(FMjLinksHeader) + (int32)h.count * (int32)sizeof(FMjLinkPose);
            if (avail < need) break;
            TArray<FMjLinkPose> poses;
            poses.SetNumUninitialized(h.count);
            if (h.count > 0) FMemory::Memcpy(poses.GetData(), rx_.GetData() + off + sizeof(FMjLinksHeader), h.count * sizeof(FMjLinkPose));
            handleLinks(h, poses.GetData());
            off += need;
        }
        else
        {
            off += 1; // resync on the magic number
        }
    }
    if (off > 0) rx_.RemoveAt(0, off, false);
}

void AMujocoPawn::handleHello(const FGo2HelloPacket& h)
{
    hfield_mode_ = (h.flags & 1u) != 0;
    want_feedback_ = (h.flags & 2u) != 0;
    want_grid_ = (h.flags & 4u) != 0;
    grid_cell_m_ = FMath::Max(0.01f, h.grid_cell_m);
    grid_n_ = want_grid_ ? 2 * FMath::RoundToInt(h.grid_half_m / grid_cell_m_) + 1 : 0;
    grid_n_ = FMath::Clamp(grid_n_, 0, 201);
    grid_.SetNumZeroed(grid_n_ * grid_n_);
    sweep_row_ = 0;
    UE_LOG(LogMujocoPawn, Log, TEXT("%s: HELLO hfield=%d feedback=%d grid=%d (n=%d cell=%.3f m)"), *GetName(),
           hfield_mode_, want_feedback_, want_grid_, grid_n_, grid_cell_m_);
}

void AMujocoPawn::handlePose(const FGo2PosePacket& p)
{
    if (!anchored_)
    {
        anchored_ = true;
        anchor_loc_ = GetActorLocation();
        anchor_yaw_ = FRotator(0.f, GetActorRotation().Yaw, 0.f).Quaternion();
        mj_x0_ = p.body_x;
        mj_y0_ = p.body_y;
        float gz = 0.f; // MuJoCo z = 0 is the ground under the robot at anchoring time
        if (traceGround(anchor_loc_, anchor_loc_.Z + TerrainProbeUpCm, gz)) ground_z0_ = gz;
        sweep_row_ = 0;
        UE_LOG(LogMujocoPawn, Log, TEXT("%s: anchored MuJoCo (%.2f, %.2f) at UE (%.0f, %.0f, ground %.0f)"), *GetName(),
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

void AMujocoPawn::refreshTraceParams()
{
    trace_params_ = FCollisionQueryParams(SCENE_QUERY_STAT(MujocoGround), false, this);
    trace_params_.bFindInitialOverlaps = true;
    trace_params_.bReturnPhysicalMaterial = false;
    TArray<AActor*> attached;
    GetAttachedActors(attached);
    for (AActor* a : attached) trace_params_.AddIgnoredActor(a);
}

bool AMujocoPawn::traceGround(const FVector& xy, float top_z, float& out_z) const
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

void AMujocoPawn::terrainTick()
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

void AMujocoPawn::sendFeedback(bool with_grid)
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
    queueSend(reinterpret_cast<const uint8*>(&h), sizeof(h));
    if (with_grid && grid_n_ > 0)
        queueSend(reinterpret_cast<const uint8*>(grid_.GetData()), grid_.Num() * sizeof(float));
    flushSend();
}
