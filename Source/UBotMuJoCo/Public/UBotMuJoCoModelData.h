#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "UBotPhysicsModel.h"
#include "UBotMuJoCoModelData.generated.h"

/** One self-contained native mesh, texture or heightfield resource, addressed by virtual filename. */
USTRUCT(BlueprintType)
struct UBOTMUJOCO_API FUBotMuJoCoResource
{
    GENERATED_BODY()
    /** Relative virtual filename. Source machine paths are not stored. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    FString Name;
    /** Original resource bytes needed by the native compiler after cooking. */
    UPROPERTY()
    TArray<uint8> Bytes;
};

/** Associates a common stable identity with its original, case-sensitive native name. */
USTRUCT()
struct UBOTMUJOCO_API FUBotMuJoCoElementName
{
    GENERATED_BODY()
    UPROPERTY()
    FString Id;
    UPROPERTY()
    FString Name;
    UPROPERTY()
    bool bModelFrame = false;
};

/** MuJoCo's three friction coefficients have exclusive native semantics. */
USTRUCT()
struct UBOTMUJOCO_API FUBotMuJoCoGeometryMaterial
{
    GENERATED_BODY()
    UPROPERTY()
    FString GeometryId;
    UPROPERTY()
    FVector Friction = FVector(1.0, 0.005, 0.0001);
};

/** Immutable global native solver settings. Imported models produce a suggested preset. */
UCLASS(BlueprintType)
class UBOTMUJOCO_API UUBotMuJoCoSolverPreset : public UDataAsset
{
    GENERATED_BODY()
public:
    /** Fixed integration interval, seconds; changing it requires a new solver. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo", meta = (ClampMin = "0.0001", ClampMax = "100"))
    double StepSeconds = 0.002;
    /** Gravity in world UE axes, m/s squared. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    FVector GravityMetersPerSecondSquared = FVector(0.0, 0.0, -9.81);
    /** Exclusive native solver options preserved by the importer. */
    UPROPERTY(VisibleAnywhere, Category = "MuJoCo")
    FString NativeOptions;
};

/** Exclusive native semantics; common physical properties live only in the Physics model. */
UCLASS(BlueprintType, EditInlineNew, DefaultToInstanced)
class UBOTMUJOCO_API UUBotMuJoCoModelData : public UUBotPhysicsModelExtension
{
    GENERATED_BODY()
public:
    UUBotMuJoCoModelData();
    /** Native-only elements and attributes. This is not a complete MJCF model or an override template. */
    UPROPERTY()
    FString NativeData;
    UPROPERTY()
    double ImportedStepSeconds = 0.002;
    UPROPERTY()
    FVector ImportedGravity = FVector(0.0, 0.0, -9.81);
    UPROPERTY()
    FString ImportedOptions;
    /** Self-contained native resources. */
    UPROPERTY()
    TArray<FUBotMuJoCoResource> Resources;
    UPROPERTY()
    TArray<FUBotMuJoCoElementName> BodyNames;
    UPROPERTY()
    TArray<FUBotMuJoCoElementName> JointNames;
    UPROPERTY()
    TArray<FUBotMuJoCoElementName> GeometryNames;
    UPROPERTY()
    TArray<FUBotMuJoCoGeometryMaterial> GeometryMaterials;
    /** Imported global options. Select this preset on the world's settings actor for native-equivalent settings. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    TObjectPtr<UUBotMuJoCoSolverPreset> SuggestedPreset;
    /** Import diagnostics describing supported, approximated or rejected features. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo")
    TArray<FString> ImportMessages;
#if WITH_EDITORONLY_DATA
    /** Source basename for reimport. Absolute source paths are session-local editor data. */
    UPROPERTY(VisibleAnywhere, Category = "Import")
    FString SourceFilename;
#endif
};
