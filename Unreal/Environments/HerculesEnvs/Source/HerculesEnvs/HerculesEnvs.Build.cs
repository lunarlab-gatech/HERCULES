using UnrealBuildTool;

public class HerculesEnvs : ModuleRules
{
	public HerculesEnvs(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		// Cesium's headers use exceptions
		bEnableExceptions = true;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "CesiumRuntime", "Slate", "SlateCore", "RHI", "AirSim" });
	}
}
