#include "UBotMuJoCoLibrary.h"

#include "UBotMuJoCoFactoryAccess.h"
#include "UBotMuJoCoSolver.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotMuJoCo.h"

namespace
{
TSharedPtr<FUBotMuJoCoSolver> FindBoundSolver(const UUBotPhysicsBindingComponent* Binding)
{
    if (!IsInGameThread() || !IsValid(Binding) || Binding->Backend != UBot::MuJoCo::BackendName ||
        Binding->GetBindingStatus() != EUBotPhysicsBindingStatus::Bound)
        return nullptr;
    return UBot::MuJoCo::Private::FindSolver(Binding->GetWorld());
}
} // namespace

bool UUBotMuJoCoLibrary::GetActuators(const UUBotPhysicsBindingComponent* Binding,
                                      TArray<FUBotMuJoCoActuatorDescription>& OutActuators)
{
    const auto Solver = FindBoundSolver(Binding);
    return Solver && Solver->GetActuators(Binding->GetModelInstanceId(), OutActuators);
}

bool UUBotMuJoCoLibrary::GetActuatorActivation(const UUBotPhysicsBindingComponent* Binding, const FString& Name,
                                               FUBotMuJoCoNativeValues& Out)
{
    const auto Solver = FindBoundSolver(Binding);
    return Solver && Solver->GetActivation(Binding->GetModelInstanceId(), Name, Out);
}

bool UUBotMuJoCoLibrary::GetSensorValues(const UUBotPhysicsBindingComponent* Binding, const FString& Name,
                                         FUBotMuJoCoNativeValues& Out)
{
    const auto Solver = FindBoundSolver(Binding);
    return Solver && Solver->GetSensor(Binding->GetModelInstanceId(), Name, Out);
}
