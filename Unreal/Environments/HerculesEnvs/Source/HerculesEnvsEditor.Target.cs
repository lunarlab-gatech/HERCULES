using UnrealBuildTool;

public class HerculesEnvsEditorTarget : TargetRules
{
	public HerculesEnvsEditorTarget(TargetInfo Target) : base(Target)
	{
		DefaultBuildSettings = BuildSettingsVersion.V7;
		Type = TargetType.Editor;
		ExtraModuleNames.AddRange(new string[] { "HerculesEnvs" });
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
	}
}
