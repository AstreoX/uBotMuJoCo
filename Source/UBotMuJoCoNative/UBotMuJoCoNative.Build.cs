using UnrealBuildTool;
using System.IO;
public class UBotMuJoCoNative : ModuleRules
{
    public UBotMuJoCoNative(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.NoPCHs;
        bEnableExceptions = true;
        bUseUnity = false;
        CppStandard = CppStandardVersion.Cpp17;
        PrivateDependencyModuleNames.AddRange(new [] { "Core", "Projects" });
        string SDK = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../ThirdParty/MuJoCo/4.0.0"));
        PrivateIncludePaths.Add(Path.Combine(SDK, "include"));
        PublicAdditionalLibraries.Add(Path.Combine(SDK, "lib/Win64/mujoco.lib"));
        PublicDelayLoadDLLs.Add("mujoco.dll");
        RuntimeDependencies.Add(Path.Combine(SDK, "lib/Win64/mujoco.dll"));
        RuntimeDependencies.Add(Path.Combine(SDK, "LICENSE"), StagedFileType.NonUFS);
        RuntimeDependencies.Add(Path.Combine(SDK, "THIRD_PARTY_NOTICES.txt"), StagedFileType.NonUFS);
    }
}
