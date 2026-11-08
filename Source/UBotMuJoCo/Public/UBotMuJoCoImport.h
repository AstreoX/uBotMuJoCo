#pragma once

#include "CoreMinimal.h"
#include "UBotMuJoCoModelData.h"

namespace UBot::MuJoCo
{
/** Imports a virtual filename and its explicit resources. No disk or World is accessed. Game thread only.
 * Outer owns the extension; Preset receives the suggested global settings. Output changes only on success.
 */
UBOTMUJOCO_API bool ImportModel(const FString& VirtualFilename, const TArray<FUBotMuJoCoResource>& Resources,
                                UObject* Outer, UUBotMuJoCoSolverPreset* Preset, FUBotPhysicsModel& OutModel,
                                FText& OutError);
} // namespace UBot::MuJoCo
