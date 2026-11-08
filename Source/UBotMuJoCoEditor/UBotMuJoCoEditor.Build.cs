using UnrealBuildTool;

public class UBotMuJoCoEditor : ModuleRules
{
    public UBotMuJoCoEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "UnrealEd" });
        PrivateDependencyModuleNames.AddRange(new[] { "UBotCore", "UBotPhysics", "UBotMuJoCo", "UBotMuJoCoNative", "AssetRegistry", "AssetTools", "Projects", "XmlParser" });
    }
}
