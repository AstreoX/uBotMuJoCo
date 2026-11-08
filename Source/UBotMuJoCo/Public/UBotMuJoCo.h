#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

UBOTMUJOCO_API DECLARE_LOG_CATEGORY_EXTERN(LogUBotMuJoCo, Log, All);

namespace UBot::MuJoCo
{
inline const FName PackageName(TEXT("UBotMuJoCo"));
inline const FName BackendName(TEXT("MuJoCo"));
} // namespace UBot::MuJoCo

/** Registers the MuJoCo factory; solvers and their data remain world-local. */
class UBOTMUJOCO_API FUBotMuJoCoModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
