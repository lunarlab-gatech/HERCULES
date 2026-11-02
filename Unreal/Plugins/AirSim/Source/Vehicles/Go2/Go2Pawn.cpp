// Unitree Go2 pawn for HERCULES -- the rig; everything else is in Vehicles/Mujoco/MujocoPawn.cpp.
#include "Go2Pawn.h"
#include "Engine/StaticMesh.h"

DEFINE_LOG_CATEGORY_STATIC(LogGo2, Log, All);

using namespace MjFrames;

namespace
{
    const TCHAR* kMeshDir = TEXT("/AirSim/Go2/Meshes/");

    // menagerie unitree_go2/go2.xml geometry (m); legs in MuJoCo order FL, FR, RL, RR
    const FVector kHipPos[4] = { FVector(0.1934f, 0.0465f, 0.f), FVector(0.1934f, -0.0465f, 0.f),
                                 FVector(-0.1934f, 0.0465f, 0.f), FVector(-0.1934f, -0.0465f, 0.f) };
    const float kThighY[4] = { 0.0955f, -0.0955f, 0.0955f, -0.0955f };
    constexpr float kCalfZ = -0.213f;
    constexpr float kFootZ = -0.213f;
    const float kStandJoints[12] = { 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f, 0.f, 0.8f, -1.5f };
}

AGo2Pawn::AGo2Pawn()
{
    ListenPort = 12345;
    StandHeightCm = 30.0f;

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

void AGo2Pawn::loadMeshes()
{
    int32 missing = 0;
    auto load = [&](UStaticMeshComponent* c) {
        if (!c || c->ComponentTags.Num() == 0) return;
        const FString name = c->ComponentTags[0].ToString();
        if (!loadMeshInto(c, FString::Printf(TEXT("%s%s.%s"), kMeshDir, *name, *name))) ++missing;
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
