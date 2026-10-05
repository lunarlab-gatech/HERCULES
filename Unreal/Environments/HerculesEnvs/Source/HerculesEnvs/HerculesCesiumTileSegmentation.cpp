#include "HerculesCesiumTileSegmentation.h"

#include "CesiumLoadedTile.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "SimMode/SimModeBase.h"

DEFINE_LOG_CATEGORY_STATIC(LogHerculesTileSegmentation, Log, All);

void UHerculesCesiumTileSegmentation::OnTileMeshPrimitiveLoaded(ICesiumLoadedTilePrimitive& TilePrimitive)
{
    ICesium3DTilesetLifecycleEventReceiver::OnTileMeshPrimitiveLoaded(TilePrimitive);
    UMeshComponent* Component = &TilePrimitive.GetMeshComponent();
    TileComponents.FindOrAdd(&TilePrimitive.GetLoadedTile()).Add(Component);
    ToPaint.Add(Component);
}

void UHerculesCesiumTileSegmentation::OnTileUnloading(ICesiumLoadedTile& Tile)
{
    ICesium3DTilesetLifecycleEventReceiver::OnTileUnloading(Tile);
    TArray<TWeakObjectPtr<UMeshComponent>> Components;
    if (!TileComponents.RemoveAndCopyValue(&Tile, Components))
        return;
    ASimModeBase* SimMode = ASimModeBase::getSimMode();
    for (const TWeakObjectPtr<UMeshComponent>& Component : Components) {
        // Remove the paint now: the tile's components are destroyed after this returns. Cameras
        // keep weak references to the paint, so they need no refresh.
        if (Painted.Remove(Component) > 0 && SimMode != nullptr && Component.IsValid())
            SimMode->UnpaintInstanceSegmentationComponent(Component.Get(), false);
        ToPaint.Remove(Component);
    }
}

bool UHerculesCesiumTileSegmentation::EnsureObject(UWorld& World, ASimModeBase& SimMode)
{
    if (ObjectComponent.IsValid())
        return true;
    // An object for the tiles to share: a hidden mesh far below the world, registered like any other
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    FActorSpawnParameters Params;
    Params.Name = MakeUniqueObjectName(World.GetCurrentLevel(), AStaticMeshActor::StaticClass(), TEXT("CesiumPhotorealisticTiles"));
    AStaticMeshActor* Actor = World.SpawnActor<AStaticMeshActor>(FVector(0.0, 0.0, -1.0e7), FRotator::ZeroRotator, Params);
    if (Actor == nullptr || Mesh == nullptr)
        return false;
    Actor->SetMobility(EComponentMobility::Movable);
    Actor->GetStaticMeshComponent()->SetStaticMesh(Mesh);
    Actor->GetStaticMeshComponent()->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Actor->GetStaticMeshComponent()->SetCastShadow(false);
    if (!SimMode.AddNewActorToInstanceSegmentation(Actor, false)) {
        Actor->Destroy();
        return false;
    }
    ObjectActor = Actor;
    ObjectComponent = Actor->GetStaticMeshComponent();
    UE_LOG(LogHerculesTileSegmentation, Log, TEXT("Cesium tiles share the instance segmentation ID of %s"), *Actor->GetName());
    return true;
}

void UHerculesCesiumTileSegmentation::Flush(UWorld& World)
{
    ASimModeBase* SimMode = ASimModeBase::getSimMode();
    if (ToPaint.Num() == 0 || SimMode == nullptr || !EnsureObject(World, *SimMode))
        return;
    TArray<UMeshComponent*> Components;
    for (const TWeakObjectPtr<UMeshComponent>& Component : ToPaint) {
        if (Component.IsValid())
            Components.Add(Component.Get());
    }
    ToPaint.Reset();
    for (int32 i = 0; i < Components.Num(); ++i) {
        // Refresh the cameras' segmentation views once, with the last component of the batch
        if (SimMode->PaintInstanceSegmentationComponentAs(Components[i], ObjectComponent.Get(), i == Components.Num() - 1))
            Painted.Add(Components[i]);
    }
}
