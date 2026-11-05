// Unitree G1 humanoid pawn for HERCULES -- see G1Pawn.h; all logic is in Vehicles/Mujoco/MujocoPawn.cpp.
#include "G1Pawn.h"

AG1Pawn::AG1Pawn()
{
    ListenPort = 12346;
    StandHeightCm = 79.3f; // pelvis origin above the ground in the g1_12dof.xml default pose
    TerrainProbeUpCm = 150.0f;
}
