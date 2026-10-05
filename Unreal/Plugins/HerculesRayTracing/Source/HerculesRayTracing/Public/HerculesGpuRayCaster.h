#pragma once

#include "CoreMinimal.h"

class UWorld;

/** A ray to cast, relative to its batch's origin. Lengths are in Unreal units (cm). */
struct FHerculesGpuRay
{
    FVector3f Origin = FVector3f::ZeroVector; // relative to the batch origin
    float MaxDistance = 0.0f;
    FVector3f Direction = FVector3f::ForwardVector; // unit length
    float MinDistance = 0.0f;
};

/** The first surface a ray hit. */
struct FHerculesGpuHit
{
    float Distance = -1.0f; // along the ray from its origin; negative for a miss
    uint32 PrimitiveComponentId = 0; // UPrimitiveComponent::GetPrimitiveSceneId().PrimIDValue of the surface
    FVector3f Normal = FVector3f::ZeroVector; // world space, facing back along the ray; zero if unavailable

    bool IsHit() const { return Distance >= 0.0f; }
};

/**
 * Casts batches of rays on the GPU, with inline ray tracing against the scene's hardware ray tracing
 * structure, so that sensors such as LiDARs cost almost no CPU time. Rays hit opaque surfaces as
 * rendered with ray tracing (back faces are culled, as by complex collision queries), not collision
 * geometry; surfaces of excluded components are passed through.
 *
 * Ray tracing culls objects relative to the camera by default (r.RayTracing.Culling), which would
 * hide geometry near sensors away from the camera; the first batch turns that culling off.
 */
class HERCULESRAYTRACING_API FHerculesGpuRayCaster
{
public:
    /**
     * Whether rays can be cast: ray tracing is enabled and the GPU supports inline ray tracing. False
     * for a while after a batch found no ray tracing structure (nothing in the scene is ray traced).
     */
    static bool IsSupported();

    /** Called on the game thread: whether the rays were cast, and if so the hits in ray order. */
    using FOnComplete = TUniqueFunction<void(bool bCast, TArray<FHerculesGpuHit>&& Hits)>;

    /**
     * Casts the rays during the next frame rendered for World (from any thread). The
     * results usually arrive one to three frames later. If the scene has no ray tracing structure
     * for a while (nothing rendered with ray tracing), OnComplete reports that the rays were not
     * cast, so the caller can fall back to the CPU.
     */
    static void Submit(UWorld& World, const FVector& Origin, TArray<FHerculesGpuRay>&& Rays,
                       TArray<uint32> ExcludedComponentIds, FOnComplete&& OnComplete);
};
