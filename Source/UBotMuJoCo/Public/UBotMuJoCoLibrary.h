#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UBotMuJoCoLibrary.generated.h"

class UUBotPhysicsBindingComponent;

/** Authored native actuator input layout; names remain case-sensitive. */
USTRUCT(BlueprintType)
struct UBOTMUJOCO_API FUBotMuJoCoActuatorDescription
{
    GENERATED_BODY()
    /** Native actuator name inside its model instance. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    FString Name;
    /** Number of simultaneous inputs accepted by this actuator. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    int32 InputCount = 0;
    /** Whether each authored control channel has an enforced range. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    TArray<bool> InputLimited;
    /** One native range per input channel, in that channel's authored units. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    TArray<FVector2D> InputRanges;
};

/** Read-only native endpoint values. These retain native sensor/activation units and conventions. */
USTRUCT(BlueprintType)
struct UBOTMUJOCO_API FUBotMuJoCoNativeValues
{
    GENERATED_BODY()
    /** Values in the native model's declared layout. Sensor spatial quantities retain right-handed axes. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    TArray<double> Values;
    /** uBot simulation time of the last successfully integrated endpoint. */
    UPROPERTY(BlueprintReadOnly, Category = "MuJoCo")
    double TimestampSeconds = 0.0;
};

/** Native read-only queries; ordinary body and joint state is supplied by Physics. Game thread only. */
UCLASS()
class UBOTMUJOCO_API UUBotMuJoCoLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    /** Returns actuator layouts for this binding's active MuJoCo instance. */
    UFUNCTION(BlueprintPure, Category = "uBot|MuJoCo", meta = (DisplayName = "Get MuJoCo Actuators"))
    static bool GetActuators(const UUBotPhysicsBindingComponent* Binding,
                             TArray<FUBotMuJoCoActuatorDescription>& OutActuators);
    /** Reads native actuator activation state. False before a step, after reset, or for an unknown name. */
    UFUNCTION(BlueprintPure, Category = "uBot|MuJoCo", meta = (DisplayName = "Get MuJoCo Actuator Activation"))
    static bool GetActuatorActivation(const UUBotPhysicsBindingComponent* Binding, const FString& ActuatorName,
                                      FUBotMuJoCoNativeValues& OutState);
    /** Reads one native numeric sensor, preserving its native layout, units and right-handed convention. */
    UFUNCTION(BlueprintPure, Category = "uBot|MuJoCo", meta = (DisplayName = "Get MuJoCo Sensor Values"))
    static bool GetSensorValues(const UUBotPhysicsBindingComponent* Binding, const FString& SensorName,
                                FUBotMuJoCoNativeValues& OutState);
};
