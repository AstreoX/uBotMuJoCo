using UnrealBuildTool;

public class UBotMuJoCo : ModuleRules
{
    public UBotMuJoCo(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "UBotCore", "UBotPhysics" });
        PrivateDependencyModuleNames.AddRange(new[] { "UBotMuJoCoNative", "Projects" });
    }
}
