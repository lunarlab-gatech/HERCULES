#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "HerculesCesiumSubsystem.generated.h"

/**
 * Sets up a HERCULES Cesium world when play begins, so packaged builds carry no Cesium ion access
 * token and can be placed anywhere on Earth:
 *
 *   -CesiumIonToken=<token>       or the CESIUM_ION_TOKEN environment variable
 *   -CesiumIonAssets=<id>[,<id>]  ion asset IDs to stream (default 2275207, Google Photorealistic 3D Tiles)
 *   -CesiumOrigin=<lat>,<lon>[,<height_m>]  where the robots start; without a height the ground is found
 *   -CesiumSpawnSearchRadius=<m>  how far to look for open ground for the robots (default 150, 0 to stay put)
 *   -CesiumMaxScreenSpaceError=<px>  tile detail, lower is finer (default 2; Cesium's own default is 16)
 *   -CesiumMaxSimultaneousTileLoads=<n>  tile requests in flight (default 64; Cesium's own default is 20)
 *   -CesiumNoPrompt               don't ask for the location at launch
 *   -CesiumLogSelectionStats      log Cesium's tile selection statistics (tiles visited, rendered, loading)
 *
 * Without -CesiumOrigin, a window asks for the location (and for a token if none was given), unless
 * the run is unattended or offscreen. Levels whose georeference has the actor tag
 * "HerculesIonTilesets" get their ion tilesets spawned here, once a token is known, so that cooking
 * and opening the level never contact Cesium ion. Ion tilesets placed in a level receive the token
 * and asset IDs instead.
 *
 * The robots spawn before any tiles exist, so the origin may turn out to be inside a tree or a
 * building; an invisible pad (tagged "HerculesSpawnPad") holds them meanwhile. Once the tiles around
 * the origin have loaded, the nearest flat, open, ground-level area that fits the whole team is
 * found and the georeference is shifted to put it under the robots, its highest point at Unreal
 * Z = 0. The robots' start poses are unchanged, and their GPS home moves to the new place. When
 * tiles are under every robot, the pad stops colliding and the robots settle onto the ground.
 *
 * The tiles share one instance segmentation ID (UHerculesCesiumTileSegmentation), which separates
 * them from the sky and the robots in segmentation images.
 *
 * Until the robots are placed, tile loading is steered to the spawn area by a camera looking down
 * on it from above. After that, Cesium refines tiles for the player's view and for the robots'
 * cameras that capture images, added to its camera manager every frame (co-located cameras, such
 * as a stereo pair, share one view).
 */
UCLASS()
class HERCULESENVS_API UHerculesCesiumSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void OnWorldBeginPlay(UWorld& InWorld) override;
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;

protected:
    virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
    bool TilesLoaded() const;
    void OnGroundSampled(class ACesium3DTileset* Tileset, const TArray<struct FCesiumSampleHeightResult>& Results, const TArray<FString>& Warnings);
    void PlaceSpawnArea();
    void SetAirSimOrigin(const FVector& LongitudeLatitudeHeight) const;
    void UpdateRobotCameras() const;
    void ReleaseRobotsFromSpawnPad();

    UPROPERTY()
    TObjectPtr<class UHerculesCesiumTileSegmentation> TileSegmentation;
    float SinceSegmentationFlush = 0.0f;

    TWeakObjectPtr<class ACesiumGeoreference> Georeference;
    TArray<TWeakObjectPtr<class ACesium3DTileset>> Tilesets;
    FVector GroundSampleLongitudeLatitude = FVector::ZeroVector;
    bool bGroundSamplePending = false;
    bool bGroundSampleInFlight = false;
    bool bGroundKnown = false; // the georeference origin height is the ground height
    bool bPlacingRobots = false; // tile loading is focused on the spawn area
    mutable int32 LoggedCameraCount = -1;
    TWeakObjectPtr<AActor> SpawnPad; // holds the robots until tiles under them have loaded
    float SincePadCheck = 0.0f;
    float SpawnSearchRadiusM = 150.0f;
    bool bSpawnSearchPending = false;
    int32 SpawnSearchAttempts = 0;
    float WaitedSeconds = 0.0f;
};
