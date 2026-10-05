#pragma once

#include "CoreMinimal.h"
#include "common/AirSimSettings.hpp"

class APawn;
class NedTransform;

// Dresses a vehicle pawn with the visual meshes of a robot imported from a
// URDF by hercules_urdf_import (see docs/urdf_import.md). The meshes are
// built at runtime as procedural mesh components attached to the pawn's
// root, without collision, so the stock pawn keeps driving the physics.
class FUrdfVisuals
{
public:
    // Attach the visuals named by the vehicle's Urdf.Visuals setting.
    // Returns how many mesh components were created. Problems are logged,
    // never fatal: the vehicle then keeps its own look.
    static int32 Attach(APawn* Pawn, const msr::airlib::AirSimSettings::VehicleSetting& Setting,
                        const NedTransform& Transform);

    static const FName ComponentTag;
};
