using UnrealBuildTool;

public class HerculesEnvsTarget : TargetRules
{
	public HerculesEnvsTarget(TargetInfo Target) : base(Target)
	{
		DefaultBuildSettings = BuildSettingsVersion.V7;
		Type = TargetType.Game;
		ExtraModuleNames.AddRange(new string[] { "HerculesEnvs" });
		if (Target.Platform == UnrealTargetPlatform.Linux)
			bUsePCHFiles = false;
	}
}
