#include "HerculesCesiumSubsystem.h"
#include "HerculesCesiumTileSegmentation.h"

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "CesiumCamera.h"
#include "CesiumCameraManager.h"
#include "CesiumSampleHeightResult.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/DefaultPawn.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerStart.h"
#include "GameFramework/SpectatorPawn.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMemory.h"
#include "HAL/PlatformMisc.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Parse.h"
#include "PIPCamera.h"
#include "RHI.h"
#include "RHIStats.h"
#include "SimMode/SimModeBase.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SWindow.h"
#include "Widgets/Text/STextBlock.h"

#include <limits>

DEFINE_LOG_CATEGORY_STATIC(LogHerculesCesium, Log, All);

namespace
{
const TCHAR* LocationSection = TEXT("HerculesCesium");

struct FLocationPreset
{
    const TCHAR* Name;
    double Latitude;
    double Longitude;
};

const FLocationPreset LocationPresets[] = {
    {TEXT("Caltech, Pasadena"), 34.137658, -118.125269},
    {TEXT("Georgia Tech, Atlanta"), 33.775620, -84.396285},
};

/** Sizes the budgets that detailed 3D tiles fill to this machine: Cesium tiles are runtime meshes
 *  whose ray tracing geometry always stays resident, and UE's 400 MB default pool is far too small
 *  for them. Uses 1/8 of the GPU's memory for that pool (400 MB to 8 GB), unless set on the command
 *  line or console, and returns a tile cache size of 1/16 of system memory (256 MB to 4 GB). */
int64 ApplyDeviceBudgets()
{
    FTextureMemoryStats Stats;
    RHIGetTextureMemoryStats(Stats);
    const int64 VideoMemory = Stats.GetTotalDeviceWorkingMemory();
    IConsoleVariable* Pool = IConsoleManager::Get().FindConsoleVariable(TEXT("r.RayTracing.ResidentGeometryMemoryPoolSizeInMB"));
    if (Pool != nullptr && VideoMemory > 0
        && (uint32(Pool->GetFlags()) & uint32(ECVF_SetByMask)) < uint32(ECVF_SetByCommandline)) {
        const int32 PoolMB = FMath::Clamp(int32(VideoMemory / 8 / (1024 * 1024)), 400, 8192);
        Pool->Set(PoolMB, ECVF_SetByCode);
        UE_LOG(LogHerculesCesium, Log, TEXT("Ray tracing geometry pool %d MB for %lld MB of video memory"), PoolMB, VideoMemory / (1024 * 1024));
    }
    const int64 SystemMemory = int64(FPlatformMemory::GetConstants().TotalPhysical);
    return FMath::Clamp<int64>(SystemMemory / 16, 256ll * 1024 * 1024, 4096ll * 1024 * 1024);
}

bool ShouldPrompt()
{
    const TCHAR* Cmd = FCommandLine::Get();
    return !FApp::IsUnattended() && !IsRunningCommandlet() && FApp::CanEverRender()
           && !FParse::Param(Cmd, TEXT("RenderOffscreen")) && !FParse::Param(Cmd, TEXT("CesiumNoPrompt"))
           && FSlateApplication::IsInitialized();
}

FText Degrees(double Value)
{
    return FText::FromString(FString::Printf(TEXT("%.6f"), Value));
}

TSharedRef<SWidget> LabelledRow(const TCHAR* Label, const TSharedRef<SWidget>& Field)
{
    return SNew(SHorizontalBox)
           + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
           [
               SNew(SBox).WidthOverride(130)[SNew(STextBlock).Text(FText::FromString(Label))]
           ]
           + SHorizontalBox::Slot().FillWidth(1.0f)[Field];
}

/** Asks where the robots should start, and for a token if none was given. False if cancelled. */
bool PromptForLocation(double& Latitude, double& Longitude, FString& Token)
{
    const bool bAskToken = Token.IsEmpty();
    TSharedPtr<SEditableTextBox> LatitudeBox, LongitudeBox, TokenBox;
    FText Error;
    bool bAccepted = false;

    TSharedRef<SWindow> Window = SNew(SWindow)
                                     .Title(FText::FromString(TEXT("HERCULES Cesium world")))
                                     .SizingRule(ESizingRule::Autosized)
                                     .SupportsMinimize(false)
                                     .SupportsMaximize(false);
    TWeakPtr<SWindow> WeakWindow = Window;

    TSharedRef<SHorizontalBox> Presets = SNew(SHorizontalBox);
    for (const FLocationPreset& Preset : LocationPresets) {
        Presets->AddSlot().AutoWidth().Padding(0, 0, 8, 0)
        [
            SNew(SButton)
            .Text(FText::FromString(Preset.Name))
            .OnClicked_Lambda([&LatitudeBox, &LongitudeBox, Preset]() {
                LatitudeBox->SetText(Degrees(Preset.Latitude));
                LongitudeBox->SetText(Degrees(Preset.Longitude));
                return FReply::Handled();
            })
        ];
    }

    TSharedRef<SVerticalBox> Fields = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
        [
            SNew(STextBlock).Text(FText::FromString(TEXT("Where should the robots start? Enter a WGS84 latitude and longitude in degrees.\n"
                                                         "The robots are placed on the nearest flat, open ground.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
        [
            LabelledRow(TEXT("Latitude"), SAssignNew(LatitudeBox, SEditableTextBox).Text(Degrees(Latitude)))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
        [
            LabelledRow(TEXT("Longitude"), SAssignNew(LongitudeBox, SEditableTextBox).Text(Degrees(Longitude)))
        ];
    if (bAskToken) {
        Fields->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
        [
            LabelledRow(TEXT("Cesium ion token"), SAssignNew(TokenBox, SEditableTextBox)
                                                      .IsPassword(true)
                                                      .HintText(FText::FromString(TEXT("from https://ion.cesium.com/tokens"))))
        ];
    }
    Fields->AddSlot().AutoHeight().Padding(0, 6, 0, 0)[Presets];
    Fields->AddSlot().AutoHeight().Padding(0, 8, 0, 0)
    [
        SNew(STextBlock).ColorAndOpacity(FLinearColor(1.0f, 0.3f, 0.3f)).Text_Lambda([&Error]() { return Error; })
    ];
    Fields->AddSlot().AutoHeight().HAlign(HAlign_Right).Padding(0, 8, 0, 0)
    [
        SNew(SHorizontalBox)
        + SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
        [
            SNew(SButton)
            .Text(FText::FromString(TEXT("Use the level's location")))
            .OnClicked_Lambda([WeakWindow]() {
                if (WeakWindow.IsValid())
                    WeakWindow.Pin()->RequestDestroyWindow();
                return FReply::Handled();
            })
        ]
        + SHorizontalBox::Slot().AutoWidth()
        [
            SNew(SButton)
            .Text(FText::FromString(TEXT("Start")))
            .OnClicked_Lambda([&, WeakWindow, bAskToken]() {
                double Lat = 0.0, Lon = 0.0;
                if (!LexTryParseString(Lat, *LatitudeBox->GetText().ToString().TrimStartAndEnd()) || Lat < -90.0 || Lat > 90.0
                    || !LexTryParseString(Lon, *LongitudeBox->GetText().ToString().TrimStartAndEnd()) || Lon < -180.0 || Lon > 180.0) {
                    Error = FText::FromString(TEXT("Latitude must be -90 to 90 and longitude -180 to 180 degrees."));
                    return FReply::Handled();
                }
                if (bAskToken && TokenBox->GetText().IsEmpty()) {
                    Error = FText::FromString(TEXT("Enter a Cesium ion access token."));
                    return FReply::Handled();
                }
                Latitude = Lat;
                Longitude = Lon;
                if (bAskToken)
                    Token = TokenBox->GetText().ToString().TrimStartAndEnd();
                bAccepted = true;
                if (WeakWindow.IsValid())
                    WeakWindow.Pin()->RequestDestroyWindow();
                return FReply::Handled();
            })
        ]
    ];

    Window->SetContent(SNew(SBorder).Padding(16)[SNew(SBox).MinDesiredWidth(520)[Fields]]);
    TSharedPtr<SWindow> Parent = (GEngine != nullptr && GEngine->GameViewport != nullptr) ? GEngine->GameViewport->GetWindow() : nullptr;
    FSlateApplication::Get().AddModalWindow(Window, Parent);
    return bAccepted;
}
}

bool UHerculesCesiumSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UHerculesCesiumSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
    Super::OnWorldBeginPlay(InWorld);

    static const FName SpawnTilesetsTag(TEXT("HerculesIonTilesets"));
    ACesiumGeoreference* Georef = nullptr;
    ACesiumGeoreference* SpawnFor = nullptr;
    for (TActorIterator<ACesiumGeoreference> It(&InWorld); It; ++It) {
        if (Georef == nullptr)
            Georef = *It;
        if (It->ActorHasTag(SpawnTilesetsTag))
            SpawnFor = *It;
    }
    if (SpawnFor != nullptr)
        Georef = SpawnFor;

    TArray<ACesium3DTileset*> IonTilesets;
    for (TActorIterator<ACesium3DTileset> It(&InWorld); It; ++It) {
        if (It->GetTilesetSource() == ETilesetSource::FromCesiumIon)
            IonTilesets.Add(*It);
    }
    if (Georef == nullptr && IonTilesets.Num() == 0)
        return;
    static const FName SpawnPadTag(TEXT("HerculesSpawnPad"));
    for (TActorIterator<AActor> It(&InWorld); It; ++It) {
        if (It->ActorHasTag(SpawnPadTag)) {
            SpawnPad = *It;
            break;
        }
    }

    FString Token;
    if (!FParse::Value(FCommandLine::Get(), TEXT("CesiumIonToken="), Token))
        Token = FPlatformMisc::GetEnvironmentVariable(TEXT("CESIUM_ION_TOKEN"));

    // Where the robots start: -CesiumOrigin, else ask, else the level's own location
    bool bOriginGiven = false, bHeightGiven = false;
    FVector LongitudeLatitudeHeight = Georef != nullptr ? Georef->GetOriginLongitudeLatitudeHeight() : FVector::ZeroVector;
    FString Origin;
    if (FParse::Value(FCommandLine::Get(), TEXT("CesiumOrigin="), Origin, false)) {
        TArray<FString> Parts;
        Origin.ParseIntoArray(Parts, TEXT(","));
        if (Parts.Num() == 2 || Parts.Num() == 3) {
            LongitudeLatitudeHeight.X = FCString::Atod(*Parts[1]);
            LongitudeLatitudeHeight.Y = FCString::Atod(*Parts[0]);
            bHeightGiven = Parts.Num() == 3;
            if (bHeightGiven)
                LongitudeLatitudeHeight.Z = FCString::Atod(*Parts[2]);
            bOriginGiven = true;
        }
        else {
            UE_LOG(LogHerculesCesium, Error, TEXT("-CesiumOrigin needs <lat>,<lon>[,<height_m>], got '%s'"), *Origin);
        }
    }
    else if (SpawnFor != nullptr && ShouldPrompt()) {
        double Latitude = LongitudeLatitudeHeight.Y, Longitude = LongitudeLatitudeHeight.X;
        GConfig->GetDouble(LocationSection, TEXT("Latitude"), Latitude, GGameUserSettingsIni);
        GConfig->GetDouble(LocationSection, TEXT("Longitude"), Longitude, GGameUserSettingsIni);
        if (PromptForLocation(Latitude, Longitude, Token)) {
            LongitudeLatitudeHeight.X = Longitude;
            LongitudeLatitudeHeight.Y = Latitude;
            bOriginGiven = true;
            // Remember the place (never the token) for the next launch
            GConfig->SetDouble(LocationSection, TEXT("Latitude"), Latitude, GGameUserSettingsIni);
            GConfig->SetDouble(LocationSection, TEXT("Longitude"), Longitude, GGameUserSettingsIni);
            GConfig->Flush(false, GGameUserSettingsIni);
        }
    }
    if (bOriginGiven && Georef != nullptr) {
        for (TActorIterator<ACesiumGeoreference> It(&InWorld); It; ++It)
            It->SetOriginLongitudeLatitudeHeight(LongitudeLatitudeHeight);
        UE_LOG(LogHerculesCesium, Log, TEXT("Location set to lat %.6f, lon %.6f"), LongitudeLatitudeHeight.Y, LongitudeLatitudeHeight.X);
    }

    if (Token.IsEmpty()) {
        const FString Message = TEXT("No Cesium ion access token: start with -CesiumIonToken=<token> or set CESIUM_ION_TOKEN. "
                                     "Create a token at https://ion.cesium.com/tokens");
        UE_LOG(LogHerculesCesium, Warning, TEXT("%s"), *Message);
        if (GEngine)
            GEngine->AddOnScreenDebugMessage(-1, 60.0f, FColor::Red, Message);
        return;
    }

    TArray<int64> AssetIds;
    FString Assets;
    if (FParse::Value(FCommandLine::Get(), TEXT("CesiumIonAssets="), Assets, false)) {
        TArray<FString> Ids;
        Assets.ParseIntoArray(Ids, TEXT(","));
        for (const FString& Id : Ids)
            AssetIds.Add(FCString::Atoi64(*Id));
    }

    if (SpawnFor != nullptr && IonTilesets.Num() == 0) {
        if (AssetIds.Num() == 0)
            AssetIds.Add(2275207); // Google Photorealistic 3D Tiles
        for (int64 AssetId : AssetIds) {
            FActorSpawnParameters Params;
            Params.Name = MakeUniqueObjectName(InWorld.GetCurrentLevel(), ACesium3DTileset::StaticClass(), *FString::Printf(TEXT("CesiumIon_%lld"), AssetId));
            ACesium3DTileset* Tileset = InWorld.SpawnActor<ACesium3DTileset>(Params);
            Tileset->SetGeoreference(SpawnFor);
            Tileset->SetTilesetSource(ETilesetSource::FromCesiumIon);
            Tileset->SetIonAssetID(AssetId);
            IonTilesets.Add(Tileset);
        }
    }
    else {
        for (int32 i = 0; i < AssetIds.Num() && i < IonTilesets.Num(); ++i)
            IonTilesets[i]->SetIonAssetID(AssetIds[i]);
    }

    const int64 CacheBytes = ApplyDeviceBudgets();
    double MaxScreenSpaceError = 2.0;
    FParse::Value(FCommandLine::Get(), TEXT("CesiumMaxScreenSpaceError="), MaxScreenSpaceError);
    int32 SimultaneousTileLoads = 64;
    FParse::Value(FCommandLine::Get(), TEXT("CesiumMaxSimultaneousTileLoads="), SimultaneousTileLoads);
    TileSegmentation = NewObject<UHerculesCesiumTileSegmentation>(this);
    for (ACesium3DTileset* Tileset : IonTilesets) {
        if (Tileset->GetLifecycleEventReceiver() == nullptr)
            Tileset->SetLifecycleEventReceiver(TileSegmentation);
        Tileset->SetMaximumScreenSpaceError(MaxScreenSpaceError);
        // Fine tiles for several robots: keep more of them cached, and load more at once
        Tileset->MaximumCachedBytes = CacheBytes;
        Tileset->MaximumSimultaneousTileLoads = SimultaneousTileLoads;
        Tileset->LogSelectionStats = FParse::Param(FCommandLine::Get(), TEXT("CesiumLogSelectionStats"));
        Tileset->SetIonAccessToken(Token);
        Tileset->RefreshTileset();
        Tilesets.Add(Tileset);
    }
    UE_LOG(LogHerculesCesium, Log, TEXT("Streaming %d Cesium ion tileset(s)"), IonTilesets.Num());

    Georeference = Georef;
    WaitedSeconds = 0.0f;
    SpawnSearchAttempts = 0;
    bGroundKnown = bHeightGiven;
    bPlacingRobots = Georef != nullptr;
    SpawnSearchRadiusM = 150.0f;
    FParse::Value(FCommandLine::Get(), TEXT("CesiumSpawnSearchRadius="), SpawnSearchRadiusM);
    if (Georef != nullptr && bOriginGiven && !bHeightGiven && Tilesets.Num() > 0) {
        // Bring the ground at the chosen place near the robots first, so its tiles load around them.
        // Asked for from Tick, once the tileset has been rebuilt with the token.
        GroundSampleLongitudeLatitude = FVector(LongitudeLatitudeHeight.X, LongitudeLatitudeHeight.Y, 0.0);
        bGroundSamplePending = true;
    }
    else {
        bSpawnSearchPending = Georef != nullptr;
    }
}

void UHerculesCesiumSubsystem::OnGroundSampled(ACesium3DTileset* Tileset, const TArray<FCesiumSampleHeightResult>& Results, const TArray<FString>& Warnings)
{
    if (!bGroundSampleInFlight)
        return; // timed out, the search has already started
    bGroundSampleInFlight = false;
    ACesiumGeoreference* Georef = Georeference.Get();
    if (Georef != nullptr && Results.Num() > 0 && Results[0].SampleSuccess) {
        Georef->SetOriginLongitudeLatitudeHeight(Results[0].LongitudeLatitudeHeight);
        bGroundKnown = true;
        UE_LOG(LogHerculesCesium, Log, TEXT("Ground at %.1f m"), Results[0].LongitudeLatitudeHeight.Z);
    }
    else {
        UE_LOG(LogHerculesCesium, Warning, TEXT("Could not sample the ground height (%s); searching from the level's height"),
               *FString::Join(Warnings, TEXT("; ")));
    }
    bSpawnSearchPending = Georef != nullptr;
    WaitedSeconds = 0.0f;
}

TStatId UHerculesCesiumSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UHerculesCesiumSubsystem, STATGROUP_Tickables);
}

bool UHerculesCesiumSubsystem::TilesLoaded() const
{
    for (const TWeakObjectPtr<ACesium3DTileset>& Tileset : Tilesets) {
        if (Tileset.IsValid() && Tileset->GetLoadProgress() < 100.0f)
            return false;
    }
    return true;
}

void UHerculesCesiumSubsystem::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (Tilesets.Num() > 0)
        UpdateRobotCameras();
    SinceSegmentationFlush += DeltaTime;
    if (TileSegmentation != nullptr && SinceSegmentationFlush >= 0.25f && GetWorld() != nullptr) {
        SinceSegmentationFlush = 0.0f;
        TileSegmentation->Flush(*GetWorld());
    }
    if (SpawnPad.IsValid() && !bPlacingRobots) {
        SincePadCheck += DeltaTime;
        if (SincePadCheck >= 0.5f) {
            SincePadCheck = 0.0f;
            ReleaseRobotsFromSpawnPad();
        }
    }
    if (bGroundSamplePending) {
        WaitedSeconds += DeltaTime;
        if (WaitedSeconds >= 1.0f && Tilesets.Num() > 0 && Tilesets[0].IsValid()) {
            bGroundSamplePending = false;
            bGroundSampleInFlight = true;
            WaitedSeconds = 0.0f;
            Tilesets[0]->SampleHeightMostDetailed({GroundSampleLongitudeLatitude},
                                                  FCesiumSampleHeightMostDetailedCallback::CreateUObject(this, &UHerculesCesiumSubsystem::OnGroundSampled));
        }
        return;
    }
    if (bGroundSampleInFlight) {
        WaitedSeconds += DeltaTime;
        if (WaitedSeconds < 20.0f)
            return;
        UE_LOG(LogHerculesCesium, Warning, TEXT("No ground height after 20 s; searching from the level's height"));
        bGroundSampleInFlight = false;
        bSpawnSearchPending = true;
        WaitedSeconds = 0.0f;
    }
    if (!bSpawnSearchPending)
        return;
    WaitedSeconds += DeltaTime;
    // Wait for the tiles around the origin; after 30 s search with what is there
    if ((WaitedSeconds < 3.0f || !TilesLoaded()) && WaitedSeconds < 30.0f)
        return;
    bSpawnSearchPending = false;
    PlaceSpawnArea();
}

void UHerculesCesiumSubsystem::PlaceSpawnArea()
{
    UWorld* World = GetWorld();
    ACesiumGeoreference* Georef = Georeference.Get();
    if (World == nullptr || Georef == nullptr)
        return;

    const float RadiusM = SpawnSearchRadiusM;

    // The team's footprint around the PlayerStart, in Unreal units (cm), with a margin
    FBox2D Footprint(FVector2D(-300.0, -300.0), FVector2D(300.0, 300.0));
    FCollisionQueryParams Query(SCENE_QUERY_STAT(HerculesSpawnSearch), true);
    for (TActorIterator<APawn> It(World); It; ++It) {
        if (It->IsA<ASpectatorPawn>() || It->IsA<ADefaultPawn>())
            continue;
        const FVector L = It->GetActorLocation();
        Footprint += FVector2D(L.X - 300.0, L.Y - 300.0);
        Footprint += FVector2D(L.X + 300.0, L.Y + 300.0);
        Query.AddIgnoredActor(*It);
    }

    // Height of the top surface on a 2 m grid (cells hold NaN where nothing was hit). Traces span
    // 3 km up and down in case the ground height could not be sampled beforehand.
    const double Cell = 200.0;
    const double TraceHalfLength = 300000.0;
    const int32 Half = FMath::Max(FMath::CeilToInt32(RadiusM * 100.0 / Cell), 0)
                     + FMath::CeilToInt32(FMath::Max(Footprint.Max.Size(), Footprint.Min.Size()) / Cell);
    const int32 N = 2 * Half + 1;
    TArray<double> Top;
    Top.SetNumUninitialized(N * N);
    TArray<double> Valid;
    for (int32 j = 0; j < N; ++j) {
        for (int32 i = 0; i < N; ++i) {
            const FVector XY((i - Half) * Cell, (j - Half) * Cell, 0.0);
            FHitResult Hit;
            const bool bHit = World->LineTraceSingleByChannel(Hit, XY + FVector(0, 0, TraceHalfLength), XY - FVector(0, 0, TraceHalfLength), ECC_Visibility, Query);
            // Tiles that failed to load leave holes that expose stray surfaces far from the ground;
            // with the ground height known, only surfaces near it count
            const bool bUsable = bHit && (!bGroundKnown || FMath::Abs(Hit.ImpactPoint.Z) <= 3000.0);
            Top[j * N + i] = bUsable ? Hit.ImpactPoint.Z : std::numeric_limits<double>::quiet_NaN();
            if (bUsable)
                Valid.Add(Hit.ImpactPoint.Z);
        }
    }
    if (Valid.Num() == 0) {
        UE_LOG(LogHerculesCesium, Warning, TEXT("No tiles under the spawn area; leaving the origin unchanged"));
        bPlacingRobots = false;
        return;
    }
    Valid.Sort();
    // Streets and lawns are the low surfaces; roofs and canopies are above them
    const double Ground = Valid[Valid.Num() / 10];

    const int32 FI0 = FMath::FloorToInt32(Footprint.Min.X / Cell), FI1 = FMath::CeilToInt32(Footprint.Max.X / Cell);
    const int32 FJ0 = FMath::FloorToInt32(Footprint.Min.Y / Cell), FJ1 = FMath::CeilToInt32(Footprint.Max.Y / Cell);
    auto AreaHeight = [&](int32 ci, int32 cj, double& OutHeight) {
        double Lo = TNumericLimits<double>::Max(), Hi = -TNumericLimits<double>::Max(), Sum = 0.0;
        int32 Count = 0;
        for (int32 j = cj + FJ0; j <= cj + FJ1; ++j) {
            for (int32 i = ci + FI0; i <= ci + FI1; ++i) {
                if (i < 0 || j < 0 || i >= N || j >= N)
                    return false;
                const double Z = Top[j * N + i];
                if (FMath::IsNaN(Z))
                    return false;
                Lo = FMath::Min(Lo, Z);
                Hi = FMath::Max(Hi, Z);
                Sum += Z;
                ++Count;
            }
        }
        // The highest point goes at the spawn pad's top, so no ground in the footprint rises into the
        // robots; once the pad is removed they settle at most 50 cm onto the tiles
        OutHeight = Hi;
        // flat to 50 cm across the footprint, and within 2 m of ground level
        return Hi - Lo <= 50.0 && FMath::Abs(Sum / Count - Ground) <= 200.0;
    };

    // Nearest suitable area to the requested place
    int32 BestI = -1, BestJ = -1;
    double BestD2 = TNumericLimits<double>::Max(), BestZ = 0.0;
    for (int32 cj = 0; cj < N; ++cj) {
        for (int32 ci = 0; ci < N; ++ci) {
            const double D2 = FMath::Square(double(ci - Half)) + FMath::Square(double(cj - Half));
            if (D2 * Cell * Cell > FMath::Square(RadiusM * 100.0) || D2 >= BestD2)
                continue;
            double Z;
            if (AreaHeight(ci, cj, Z)) {
                BestI = ci;
                BestJ = cj;
                BestD2 = D2;
                BestZ = Z;
            }
        }
    }

    FVector Target;
    if (BestI >= 0) {
        Target = FVector((BestI - Half) * Cell, (BestJ - Half) * Cell, BestZ);
    }
    else if (SpawnSearchAttempts++ == 0) {
        // The tiles may still be coarse, e.g. when the robots started far above or below the ground:
        // bring the ground level to the robots and search again once the nearby tiles have refined
        if (!bGroundKnown) {
            Georef->SetOriginLongitudeLatitudeHeight(Georef->TransformUnrealPositionToLongitudeLatitudeHeight(FVector(0.0, 0.0, Ground)));
            bGroundKnown = true;
        }
        UE_LOG(LogHerculesCesium, Log, TEXT("No open area yet; searching again once the tiles have refined"));
        bSpawnSearchPending = true;
        WaitedSeconds = 0.0f;
        return;
    }
    else {
        // No open area: keep the place, with the ground (not a roof or canopy) at the robots' feet
        Target = FVector(0.0, 0.0, bGroundKnown ? 0.0 : Ground);
        UE_LOG(LogHerculesCesium, Warning, TEXT("No flat, open area for the robots within %.0f m; check the spawn area or pick another location"), RadiusM);
    }

    const FVector Placed = Georef->TransformUnrealPositionToLongitudeLatitudeHeight(Target);
    Georef->SetOriginLongitudeLatitudeHeight(Placed);
    const FString Message = FString::Printf(TEXT("Robots placed at lat %.6f, lon %.6f, height %.1f m (%.0f m from the requested place)"),
                                            Placed.Y, Placed.X, Placed.Z, FVector2D(Target).Size() / 100.0);
    UE_LOG(LogHerculesCesium, Log, TEXT("%s"), *Message);
    if (GEngine)
        GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Green, Message);
    bPlacingRobots = false;

    // AirSim's geographic origin is the PlayerStart
    TActorIterator<APlayerStart> FirstPlayerStart(World);
    const FVector PlayerStart = FirstPlayerStart ? FirstPlayerStart->GetActorLocation() : FVector::ZeroVector;
    SetAirSimOrigin(Georef->TransformUnrealPositionToLongitudeLatitudeHeight(PlayerStart));
}

void UHerculesCesiumSubsystem::ReleaseRobotsFromSpawnPad()
{
    UWorld* World = GetWorld();
    AActor* Pad = SpawnPad.Get();
    if (World == nullptr || Pad == nullptr)
        return;
    // Only once tiles are under every robot, or one would fall through a hole. The pad ignores the
    // visibility channel, which the tiles block.
    FCollisionQueryParams Query(SCENE_QUERY_STAT(HerculesSpawnPad), true);
    TArray<APawn*> Robots;
    for (TActorIterator<APawn> It(World); It; ++It) {
        if (It->IsA<ASpectatorPawn>() || It->IsA<ADefaultPawn>())
            continue;
        Robots.Add(*It);
        Query.AddIgnoredActor(*It);
    }
    for (const APawn* Robot : Robots) {
        FVector Center, Extent;
        Robot->GetActorBounds(true, Center, Extent);
        const FVector Bottom = Center - FVector(0.0, 0.0, Extent.Z);
        FHitResult Hit;
        if (!World->LineTraceSingleByChannel(Hit, Bottom + FVector(0.0, 0.0, 100.0), Bottom - FVector(0.0, 0.0, 300.0), ECC_Visibility, Query))
            return;
    }
    // The robots settle onto the tiles (at most 50 cm below the pad's top, see PlaceSpawnArea)
    Pad->SetActorEnableCollision(false);
    SpawnPad.Reset();
    UE_LOG(LogHerculesCesium, Log, TEXT("Tiles are under all %d robots; the spawn pad no longer holds them"), Robots.Num());
}

void UHerculesCesiumSubsystem::SetAirSimOrigin(const FVector& LongitudeLatitudeHeight) const
{
    ASimModeBase* SimMode = ASimModeBase::getSimMode();
    if (SimMode == nullptr)
        return;
    SimMode->SetOriginGeoPoint(LongitudeLatitudeHeight.Y, LongitudeLatitudeHeight.X, LongitudeLatitudeHeight.Z);
    UE_LOG(LogHerculesCesium, Log, TEXT("Robots' GPS origin set to lat %.6f, lon %.6f, altitude %.1f m"), LongitudeLatitudeHeight.Y, LongitudeLatitudeHeight.X, LongitudeLatitudeHeight.Z);
}

void UHerculesCesiumSubsystem::UpdateRobotCameras() const
{
    UWorld* World = GetWorld();
    ACesiumCameraManager* Manager = World != nullptr ? ACesiumCameraManager::GetDefaultCameraManager(World) : nullptr;
    if (Manager == nullptr)
        return;
    Manager->AdditionalCameras.Reset();
    if (bPlacingRobots) {
        // Look straight down on the spawn search area, high enough to see all of it with a 90 degree
        // field of view; the large virtual viewport refines its ground finely (to ~0.3 m at the
        // default detail), which the flatness test needs
        const double HeightCm = (SpawnSearchRadiusM + 20.0) * 100.0;
        Manager->AdditionalCameras.Add(FCesiumCamera(FVector2D(4096.0, 4096.0), FVector(0.0, 0.0, HeightCm), FRotator(-90.0, 0.0, 0.0), 90.0));
        return;
    }
    // Each camera view costs Cesium a traversal of the tiles every frame, so only cameras that
    // capture images count: AirSim activates a camera's captures when their images are first
    // requested. Of the cameras a robot carries, most usually capture nothing.
    TArray<USceneCaptureComponent2D*> Captures;
    for (TActorIterator<APIPCamera> It(World); It; ++It) {
        It->GetComponents(Captures);
        const USceneCaptureComponent2D* Active = nullptr;
        FVector2D Size(0.0, 0.0);
        double FieldOfView = 0.0;
        for (const USceneCaptureComponent2D* Capture : Captures) {
            if (!Capture->IsActive() || Capture->TextureTarget == nullptr)
                continue;
            Active = Capture;
            Size = FVector2D(FMath::Max<double>(Size.X, Capture->TextureTarget->SizeX), FMath::Max<double>(Size.Y, Capture->TextureTarget->SizeY));
            FieldOfView = FMath::Max<double>(FieldOfView, Capture->FOVAngle);
        }
        if (Active == nullptr)
            continue;
        const FTransform Pose = Active->GetComponentTransform();
        // Cameras a few centimetres apart looking the same way (e.g. a stereo pair next to
        // front_center) need the same tiles: share one view, wide and fine enough for each
        FCesiumCamera* Shared = Manager->AdditionalCameras.FindByPredicate([&Pose](const FCesiumCamera& Other) {
            return FVector::DistSquared(Other.Location, Pose.GetLocation()) < 100.0 * 100.0
                && Other.Rotation.Vector().Dot(Pose.Rotator().Vector()) > FMath::Cos(FMath::DegreesToRadians(10.0));
        });
        if (Shared == nullptr) {
            Manager->AdditionalCameras.Add(FCesiumCamera(Size, Pose.GetLocation(), Pose.Rotator(), FieldOfView));
            continue;
        }
        Shared->ViewportSize = FVector2D(FMath::Max(Shared->ViewportSize.X, Size.X), FMath::Max(Shared->ViewportSize.Y, Size.Y));
        Shared->FieldOfViewDegrees = FMath::Max(Shared->FieldOfViewDegrees, FieldOfView);
    }
    if (Manager->AdditionalCameras.Num() != LoggedCameraCount) {
        LoggedCameraCount = Manager->AdditionalCameras.Num();
        UE_LOG(LogHerculesCesium, Log, TEXT("Tiles refine for %d robot camera view(s)"), LoggedCameraCount);
    }
}
