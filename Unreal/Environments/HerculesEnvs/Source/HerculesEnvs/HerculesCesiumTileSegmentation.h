#pragma once

#include "CoreMinimal.h"
#include "Cesium3DTilesetLifecycleEventReceiver.h"
#include "HerculesCesiumTileSegmentation.generated.h"

class ASimModeBase;
class UMeshComponent;

/**
 * Puts the meshes of a streamed Cesium tileset into AirSim's instance segmentation as one object, so
 * segmentation images separate the 3D tiles (terrain, buildings, vegetation) from the sky and the
 * robots. Tiles are painted as they load and unpainted before they unload; their visibility, which
 * Cesium switches as it refines, carries over to the paint.
 *
 * Set it as the tileset's lifecycle event receiver, then call Flush regularly from the game thread.
 */
UCLASS()
class HERCULESENVS_API UHerculesCesiumTileSegmentation : public UObject, public ICesium3DTilesetLifecycleEventReceiver
{
    GENERATED_BODY()

public:
    /** Applies the tiles loaded and unloaded since the last call, once AirSim's sim mode is running. */
    void Flush(UWorld& World);

    virtual void OnTileMeshPrimitiveLoaded(ICesiumLoadedTilePrimitive& TilePrimitive) override;
    virtual void OnTileUnloading(ICesiumLoadedTile& Tile) override;

private:
    bool EnsureObject(UWorld& World, ASimModeBase& SimMode);

    /** Hidden mesh registered as the object whose ID the tiles share */
    UPROPERTY()
    TObjectPtr<AActor> ObjectActor;

    TWeakObjectPtr<UMeshComponent> ObjectComponent;
    TMap<const ICesiumLoadedTile*, TArray<TWeakObjectPtr<UMeshComponent>>> TileComponents;
    TArray<TWeakObjectPtr<UMeshComponent>> ToPaint;
    TSet<TWeakObjectPtr<UMeshComponent>> Painted;
};
