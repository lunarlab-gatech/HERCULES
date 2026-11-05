// Unitree Go2 pawn for HERCULES (2026-10): an AMujocoPawn with a fixed static-mesh rig built from the
// menagerie Go2 link meshes (/AirSim/Go2/Meshes), posed from the 12 joint angles of the pose packet.
// Socket, anchoring, terrain feedback and the wire protocol live in Vehicles/Mujoco/MujocoPawn.h.
#pragma once

#include "CoreMinimal.h"
#include "Vehicles/Mujoco/MujocoPawn.h"
#include "Go2Pawn.generated.h"

UCLASS()
class AGo2Pawn : public AMujocoPawn
{
    GENERATED_BODY()

public:
    AGo2Pawn();

protected:
    virtual void setJointAngles(const float* joints_rad) override;
    virtual void applyStandingPose() override;
    virtual void loadMeshes() override;

private:
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
};
