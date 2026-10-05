using UnrealBuildTool;

public class HerculesRayTracing : ModuleRules
{
    public HerculesRayTracing(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
        PrivateDependencyModuleNames.AddRange(new string[] { "Projects", "RenderCore", "Renderer", "RHI" });
    }
}
