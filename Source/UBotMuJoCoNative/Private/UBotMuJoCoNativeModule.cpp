#include "HAL/PlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "UBotMuJoCoNative.h"
namespace UBot::MuJoCo::Native
{
void SetRuntimeAvailable(bool Available);
}
class FUBotMuJoCoNativeModule : public IModuleInterface
{
    void *Library = nullptr;

  public:
    void StartupModule() override
    {
        const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UBotMuJoCo"));
        if (Plugin.IsValid())
        {
            Library = FPlatformProcess::GetDllHandle(
                *FPaths::Combine(Plugin->GetBaseDir(), TEXT("ThirdParty/MuJoCo/4.0.0/lib/Win64/mujoco.dll")));
            UBot::MuJoCo::Native::SetRuntimeAvailable(Library != nullptr);
            if (Library && UBot::MuJoCo::Native::Version() != 4000000)
            {
                UBot::MuJoCo::Native::SetRuntimeAvailable(false);
                FPlatformProcess::FreeDllHandle(Library);
                Library = nullptr;
            }
        }
    }
    void ShutdownModule() override
    {
        UBot::MuJoCo::Native::SetRuntimeAvailable(false);
        if (Library)
        {
            FPlatformProcess::FreeDllHandle(Library);
            Library = nullptr;
        }
    }
};
IMPLEMENT_MODULE(FUBotMuJoCoNativeModule, UBotMuJoCoNative)
