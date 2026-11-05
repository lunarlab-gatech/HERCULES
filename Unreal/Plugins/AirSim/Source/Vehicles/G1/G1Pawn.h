// Unitree G1 humanoid pawn for HERCULES (2026-11): an AMujocoPawn rendered through the generic link rig.
// The bridge (PythonClient/go2/go2_mujoco_bridge.py --robot g1) simulates Unitree's official g1_12dof MuJoCo
// model with Unitree's pretrained walking policy (unitree_rl_gym) and streams the pelvis pose plus the pose
// of all 27 link meshes (/AirSim/G1/Meshes/SM_g1_<link>) relative to the pelvis every control step.
// Defaults: listens on 12346 so a Go2 (12345) and a G1 can live in the same level.
#pragma once

#include "CoreMinimal.h"
#include "Vehicles/Mujoco/MujocoPawn.h"
#include "G1Pawn.generated.h"

UCLASS()
class AG1Pawn : public AMujocoPawn
{
    GENERATED_BODY()

public:
    AG1Pawn();
};
