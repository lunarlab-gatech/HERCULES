#include "UrdfVisuals.h"

#include "AirBlueprintLib.h"
#include "NedTransform.h"
#include "common/Settings.hpp"
#include "common/UrdfVisualManifest.hpp"

#include "Components/MeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Pawn.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"

#include <exception>
#include <map>
#include <vector>

const FName FUrdfVisuals::ComponentTag(TEXT("HerculesUrdfVisual"));

namespace
{
using msr::airlib::UrdfVisualManifest;
using msr::airlib::Vector3r;

// Relative manifest paths are relative to the settings file.
std::string ResolveManifestPath(const std::string& path)
{
    const std::string settings_file = msr::airlib::Settings::singleton().getFullFilePath();
    return UrdfVisualManifest::resolvePath(path, UrdfVisualManifest::directoryOf(settings_file));
}

// FLU direction -> Unreal axes (x forward, y right, z up), unscaled. This is
// NedTransform's FRD -> Unreal mapping applied after UrdfVisualManifest::fluToFrd.
FVector FluDirectionToUnreal(const Vector3r& v)
{
    return FVector(v.x(), -v.y(), v.z());
}

// One flat-shaded mesh section in Unreal units. Unreal's axes are a mirror
// image of the right-handed FLU frame and its front faces are clockwise, so
// the counter-clockwise URDF triangles keep their vertex order.
struct FSection
{
    TArray<FVector> Positions;
    TArray<int32> Triangles;
    TArray<FVector> Normals;
};

// URDF names may hold characters UObject names cannot (".", " ", ...).
FString SafeObjectName(const std::string& Name)
{
    FString Out = UTF8_TO_TCHAR(Name.c_str());
    for (int32 i = 0; i < Out.Len(); ++i) {
        if (!FChar::IsAlnum(Out[i]) && Out[i] != TEXT('_'))
            Out[i] = TEXT('_');
    }
    return TEXT("Urdf_") + Out;
}

FSection BuildSection(const std::vector<Vector3r>& Vertices, const NedTransform& Ned, bool bTwoSided)
{
    FSection Section;
    const int32 Count = static_cast<int32>(Vertices.size());
    const int32 Reserve = bTwoSided ? 2 * Count : Count;
    Section.Positions.Reserve(Reserve);
    Section.Normals.Reserve(Reserve);
    Section.Triangles.Reserve(Reserve);
    for (int32 i = 0; i + 2 < Count; i += 3) {
        const FVector Normal = FluDirectionToUnreal(
            UrdfVisualManifest::faceNormal(Vertices[i], Vertices[i + 1], Vertices[i + 2]));
        const int32 Base = Section.Positions.Num();
        for (int32 Corner = 0; Corner < 3; ++Corner) {
            Section.Positions.Add(Ned.fromRelativeNed(UrdfVisualManifest::fluToFrd(Vertices[i + Corner])));
            Section.Normals.Add(Normal);
            Section.Triangles.Add(Base + Corner);
        }
        if (bTwoSided) {
            for (int32 Corner = 0; Corner < 3; ++Corner) {
                // copy first: adding an element of the array being grown trips TArray's alias check
                const FVector Position = Section.Positions[Base + Corner];
                Section.Positions.Add(Position);
                Section.Normals.Add(-Normal);
            }
            Section.Triangles.Add(Base + 3);
            Section.Triangles.Add(Base + 5);
            Section.Triangles.Add(Base + 4);
        }
    }
    return Section;
}
}

int32 FUrdfVisuals::Attach(APawn* Pawn, const msr::airlib::AirSimSettings::VehicleSetting& Setting,
                           const NedTransform& Ned)
{
    if (Pawn == nullptr || Setting.urdf.visuals.empty())
        return 0;
    USceneComponent* Root = Pawn->GetRootComponent();
    if (Root == nullptr) {
        UAirBlueprintLib::LogMessageString("URDF visuals: pawn has no root component: ", Setting.vehicle_name,
                                           LogDebugLevel::Failure);
        return 0;
    }

    TArray<UPrimitiveComponent*> Original;
    Pawn->GetComponents<UPrimitiveComponent>(Original);
    for (const UPrimitiveComponent* Component : Original) {
        if (Component->ComponentHasTag(ComponentTag))
            return 0; // already dressed
    }

    UrdfVisualManifest Manifest;
    const std::string ManifestPath = ResolveManifestPath(Setting.urdf.visuals);
    try {
        Manifest = UrdfVisualManifest::load(ManifestPath);
    }
    catch (const std::exception& e) {
        UAirBlueprintLib::LogMessageString("URDF visuals: ", e.what(), LogDebugLevel::Failure);
        return 0;
    }

    UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (BaseMaterial == nullptr)
        BaseMaterial = UMaterial::GetDefaultMaterial(MD_Surface);

    // Segmentation stencils are painted before vehicles spawn, so reuse the
    // value of the pawn's own mesh for the new components.
    bool bRenderCustomDepth = false;
    int32 Stencil = 0;
    for (const UPrimitiveComponent* Component : Original) {
        if (Component->bRenderCustomDepth) {
            bRenderCustomDepth = true;
            Stencil = Component->CustomDepthStencilValue;
            break;
        }
    }

    std::map<std::string, std::vector<Vector3r>> MeshCache;
    int32 Created = 0;
    for (const auto& Visual : Manifest.visuals) {
        auto Cached = MeshCache.find(Visual.mesh_path);
        if (Cached == MeshCache.end()) {
            try {
                Cached = MeshCache.emplace(Visual.mesh_path, UrdfVisualManifest::loadStl(Visual.mesh_path)).first;
            }
            catch (const std::exception& e) {
                UAirBlueprintLib::LogMessageString("URDF visuals: ", e.what(), LogDebugLevel::Failure);
                continue;
            }
        }
        if (Cached->second.empty())
            continue;

        const FSection Section = BuildSection(Cached->second, Ned, Setting.urdf.two_sided);
        const FName Name = MakeUniqueObjectName(Pawn, UProceduralMeshComponent::StaticClass(),
                                                FName(*SafeObjectName(Visual.name)));
        UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(Pawn, Name);
        Mesh->ComponentTags.Add(ComponentTag);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->SetCanEverAffectNavigation(false);
        Mesh->SetGenerateOverlapEvents(false);
        Mesh->CreateMeshSection_LinearColor(0, Section.Positions, Section.Triangles, Section.Normals,
                                            TArray<FVector2D>(), TArray<FLinearColor>(), TArray<FProcMeshTangent>(),
                                            false);

        UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(BaseMaterial, Mesh);
        if (Material != nullptr) {
            Material->SetVectorParameterValue(TEXT("Color"), FLinearColor(Visual.color[0], Visual.color[1],
                                                                          Visual.color[2], Visual.color[3]));
            Mesh->SetMaterial(0, Material);
        }
        if (bRenderCustomDepth) {
            Mesh->SetRenderCustomDepth(true);
            Mesh->SetCustomDepthStencilValue(Stencil);
        }

        Mesh->RegisterComponent();
        Mesh->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
        FTransform Placement = Ned.fromRelativeNed(Visual.pose);
        Placement.SetScale3D(FVector(Visual.scale.x(), Visual.scale.y(), Visual.scale.z()));
        Mesh->SetRelativeTransform(Placement);
        Pawn->AddInstanceComponent(Mesh);
        ++Created;
    }

    if (Created > 0 && Setting.urdf.hide_base_mesh) {
        // Only rendering is turned off: the stock meshes keep their collision
        // and physics, which is what drives the vehicle.
        for (UPrimitiveComponent* Component : Original) {
            if (Component->IsA<UMeshComponent>())
                Component->SetHiddenInGame(true);
        }
    }
    UAirBlueprintLib::LogMessageString("URDF visuals: ",
                                       Setting.vehicle_name + " dressed as " + Manifest.robot + " (" +
                                           std::to_string(Created) + " of " + std::to_string(Manifest.visuals.size()) +
                                           " meshes)",
                                       Created == static_cast<int32>(Manifest.visuals.size()) ? LogDebugLevel::Informational
                                                                                               : LogDebugLevel::Failure);
    return Created;
}
