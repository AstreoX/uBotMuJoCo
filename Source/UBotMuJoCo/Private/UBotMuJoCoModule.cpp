#include "UBotMuJoCo.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "UBotExtensionRegistry.h"
#include "UBotMuJoCoConversion.h"
#include "UBotMuJoCoModelData.h"
#include "UBotMuJoCoSolver.h"
#include "UBotMuJoCoWorldSettings.h"
#include "UBotPhysics.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotMuJoCoFactoryAccess.h"

DEFINE_LOG_CATEGORY(LogUBotMuJoCo);

namespace
{
class FUBotMuJoCoFactory final : public IUBotPhysicsBackendFactory
{
public:
    TMap<TWeakObjectPtr<UWorld>, TWeakPtr<FUBotMuJoCoSolver>> Solvers;
    virtual FName GetExtensionName() const override
    {
        return UBot::MuJoCo::BackendName;
    }
    virtual FUBotPhysicsCapabilities GetCapabilities() const override
    {
        FUBotPhysicsCapabilities C;
        C.bRigidBodies = true;
        C.bArticulations = true;
        C.bContacts = true;
        C.bStaticSceneGeometry = true;
        C.bKinematicForeignBodies = true;
        // Acceleration, differentiability and cross-platform determinism are not promised.
        return C;
    }
    virtual TSharedPtr<IUBotPhysicsSolver> CreateSolver(UWorld& World) override
    {
        for (auto It = Solvers.CreateIterator(); It; ++It)
        {
            if (!It.Key().IsValid() || !It.Value().IsValid())
                It.RemoveCurrent();
        }
        UBot::MuJoCo::Native::Configuration C;
        C.Gravity = UBot::MuJoCo::Conversion::Polar(FVector(0.0, 0.0, World.GetGravityZ() / 100.0));
        const AUBotMuJoCoWorldSettings* Settings = nullptr;
        for (TActorIterator<AUBotMuJoCoWorldSettings> It(&World); It; ++It)
        {
            if (Settings)
            {
                UE_LOG(LogUBotMuJoCo, Error,
                       TEXT("The world has more than one MuJoCo World Settings actor. Solver creation was rejected."));
                return nullptr;
            }
            Settings = *It;
        }
        if (Settings && Settings->SolverPreset)
        {
            C.StepSeconds = Settings->SolverPreset->StepSeconds;
            C.Gravity = UBot::MuJoCo::Conversion::Polar(Settings->SolverPreset->GravityMetersPerSecondSquared);
            C.Options = TCHAR_TO_UTF8(*Settings->SolverPreset->NativeOptions);
        }
        if (!FMath::IsFinite(C.StepSeconds) || C.StepSeconds < 0.0001 || C.StepSeconds > 100.0 ||
            !FMath::IsFinite(C.Gravity.X) || !FMath::IsFinite(C.Gravity.Y) || !FMath::IsFinite(C.Gravity.Z))
        {
            UE_LOG(
                LogUBotMuJoCo, Error,
                TEXT("MuJoCo World configuration must have a finite step in [0.0001,100] seconds and finite gravity."));
            return nullptr;
        }
        C.AllowWorldOptionsOverride = Settings && Settings->bAllowWorldOptionsOverride;
        auto Solver = MakeShared<FUBotMuJoCoSolver>(C, Settings && Settings->bAllowSceneMeshConvexApproximation);
        Solvers.Add(&World, Solver);
        return Solver;
    }
};
TSharedPtr<FUBotMuJoCoFactory> Factory;
} // namespace

TSharedPtr<FUBotMuJoCoSolver> UBot::MuJoCo::Private::FindSolver(UWorld* World)
{
    if (!IsInGameThread() || !IsValid(World) || !Factory)
        return nullptr;
    const auto* WeakSolver = Factory->Solvers.Find(World);
    auto Solver = WeakSolver ? WeakSolver->Pin() : nullptr;
    const auto* Physics = World->GetSubsystem<UUBotPhysicsSubsystem>();
    return Solver && Physics && Physics->FindSolver(UBot::MuJoCo::BackendName) == Solver.Get() ? Solver : nullptr;
}

void FUBotMuJoCoModule::StartupModule()
{
    Factory = MakeShared<FUBotMuJoCoFactory>();
    FUBotExtensionRegistry::Get().RegisterExtension(UBot::Physics::BackendExtensionPoint, UBot::MuJoCo::PackageName,
                                                    Factory.ToSharedRef());
}

void FUBotMuJoCoModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(UBot::MuJoCo::PackageName);
    Factory.Reset();
}

IMPLEMENT_MODULE(FUBotMuJoCoModule, UBotMuJoCo)
