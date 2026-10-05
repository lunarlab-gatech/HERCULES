// hercules.GpuRayTest [rays] [range_m]: casts the same rays from the camera on the GPU and with CPU
// line traces (the visibility channel against complex collision, as the CPU LiDAR does) and logs how
// they compare.

#include "HerculesGpuRayCaster.h"

#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Math/RandomStream.h"

DEFINE_LOG_CATEGORY_STATIC(LogHerculesGpuRayTest, Log, All);

static void RunGpuRayTest(const TArray<FString>& Args, UWorld* World)
{
    APlayerController* Player = World != nullptr ? World->GetFirstPlayerController() : nullptr;
    if (Player == nullptr || Player->PlayerCameraManager == nullptr) {
        UE_LOG(LogHerculesGpuRayTest, Warning, TEXT("No player camera to cast from"));
        return;
    }
    if (!FHerculesGpuRayCaster::IsSupported()) {
        UE_LOG(LogHerculesGpuRayTest, Warning, TEXT("GPU ray casting is not supported here"));
        return;
    }
    const int32 RayCount = Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 10000;
    const float Range = (Args.Num() > 1 ? FCString::Atof(*Args[1]) : 100.0f) * 100.0f;
    const FVector Origin = Player->PlayerCameraManager->GetCameraLocation();
    const FRotator Rotation = Player->PlayerCameraManager->GetCameraRotation();

    // Rays spread over the camera's forward hemisphere and below it, where most surfaces are
    FRandomStream Random(1234);
    TArray<FHerculesGpuRay> Rays;
    Rays.SetNum(RayCount);
    for (FHerculesGpuRay& Ray : Rays) {
        const FRotator Offset(Random.FRandRange(-60.0f, 15.0f), Random.FRandRange(-90.0f, 90.0f), 0.0f);
        Ray.Direction = FVector3f((Rotation + Offset).Vector());
        Ray.MaxDistance = Range;
    }

    // CPU reference, now
    TArray<FHitResult> CpuHits;
    CpuHits.SetNum(RayCount);
    FCollisionQueryParams Query(SCENE_QUERY_STAT(HerculesGpuRayTest), true);
    Query.bReturnPhysicalMaterial = true;
    const double CpuStart = FPlatformTime::Seconds();
    for (int32 Index = 0; Index < RayCount; ++Index) {
        const FVector Direction(Rays[Index].Direction);
        World->LineTraceSingleByChannel(CpuHits[Index], Origin, Origin + Direction * Range, ECC_Visibility, Query);
    }
    const double CpuSeconds = FPlatformTime::Seconds() - CpuStart;

    const double GpuStart = FPlatformTime::Seconds();
    FHerculesGpuRayCaster::Submit(*World, Origin, MoveTemp(Rays), {}, [CpuHits = MoveTemp(CpuHits), CpuSeconds, GpuStart, RayCount](bool bCast, TArray<FHerculesGpuHit>&& GpuHits) {
        if (!bCast) {
            UE_LOG(LogHerculesGpuRayTest, Warning, TEXT("The rays were not cast on the GPU (no ray tracing scene)"));
            return;
        }
        int32 BothHit = 0, CpuOnly = 0, GpuOnly = 0, SameComponent = 0, Within1cm = 0, Within10cm = 0;
        double SumAbs = 0.0;
        TMap<FString, int32> CpuOnlyActors, GpuOnlyComponents, OtherComponent;
        for (int32 Index = 0; Index < RayCount; ++Index) {
            const FHitResult& Cpu = CpuHits[Index];
            const FHerculesGpuHit& Gpu = GpuHits[Index];
            if (Cpu.bBlockingHit && Gpu.IsHit()) {
                ++BothHit;
                const double Difference = FMath::Abs(Cpu.Distance - Gpu.Distance);
                SumAbs += Difference;
                Within1cm += Difference <= 1.0;
                Within10cm += Difference <= 10.0;
                const UPrimitiveComponent* Component = Cpu.GetComponent();
                if (Component != nullptr && Component->GetPrimitiveSceneId().PrimIDValue == Gpu.PrimitiveComponentId)
                    ++SameComponent;
                else if (Component != nullptr)
                    ++OtherComponent.FindOrAdd(Component->GetOwner() ? Component->GetOwner()->GetName() : Component->GetName());
            }
            else if (Cpu.bBlockingHit) {
                ++CpuOnly;
                ++CpuOnlyActors.FindOrAdd(Cpu.GetActor() ? Cpu.GetActor()->GetName() : TEXT("?"));
            }
            else if (Gpu.IsHit()) {
                ++GpuOnly;
                ++GpuOnlyComponents.FindOrAdd(FString::Printf(TEXT("component %u"), Gpu.PrimitiveComponentId));
            }
        }
        UE_LOG(LogHerculesGpuRayTest, Log, TEXT("%d rays: both hit %d (same component %d, distance within 1 cm %d, within 10 cm %d, mean difference %.2f cm), CPU only %d, GPU only %d; CPU %.1f ms, GPU result after %.1f ms"),
               RayCount, BothHit, SameComponent, Within1cm, Within10cm, BothHit ? SumAbs / BothHit : 0.0, CpuOnly, GpuOnly,
               CpuSeconds * 1000.0, (FPlatformTime::Seconds() - GpuStart) * 1000.0);
        for (const TPair<FString, int32>& Pair : CpuOnlyActors)
            UE_LOG(LogHerculesGpuRayTest, Log, TEXT("  CPU only on %s: %d"), *Pair.Key, Pair.Value);
        for (const TPair<FString, int32>& Pair : OtherComponent)
            UE_LOG(LogHerculesGpuRayTest, Log, TEXT("  GPU hit another component where the CPU hit %s: %d"), *Pair.Key, Pair.Value);
        int32 Listed = 0;
        for (const TPair<FString, int32>& Pair : GpuOnlyComponents) {
            if (Listed++ < 10)
                UE_LOG(LogHerculesGpuRayTest, Log, TEXT("  GPU only on %s: %d"), *Pair.Key, Pair.Value);
        }
    });
}

static FAutoConsoleCommandWithWorldAndArgs GpuRayTestCommand(
    TEXT("hercules.GpuRayTest"),
    TEXT("hercules.GpuRayTest [rays] [range_m]: casts rays from the camera on the GPU and the CPU and compares them"),
    FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&RunGpuRayTest));
