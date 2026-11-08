#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Info.h"
#include "UBotMuJoCoWorldSettings.generated.h"

class UUBotMuJoCoSolverPreset;

/** Optional world-local configuration. Place at most one; its configuration is frozen when the solver is created. */
UCLASS(BlueprintType, hidecategories = (Actor, Collision, Input, Replication, Rendering))
class UBOTMUJOCO_API AUBotMuJoCoWorldSettings : public AInfo
{
    GENERATED_BODY()
  public:
    /** Global native configuration. Without a preset, use 0.002 seconds and this World's gravity. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    TObjectPtr<UUBotMuJoCoSolverPreset> SolverPreset;
    /** Explicitly permit World configuration to replace conflicting imported global options. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    bool bAllowWorldOptionsOverride = false;
    /** Explicitly approximate exported triangle-mesh collision with its convex hull. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    bool bAllowSceneMeshConvexApproximation = false;
};
