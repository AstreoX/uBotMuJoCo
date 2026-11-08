#pragma once

#include "CoreMinimal.h"

class FUBotMuJoCoSolver;
class UWorld;

namespace UBot::MuJoCo::Private
{
/** Pins only the solver still owned by this world's Physics slot. No stale native pointer is exposed. */
TSharedPtr<FUBotMuJoCoSolver> FindSolver(UWorld* World);
} // namespace UBot::MuJoCo::Private
