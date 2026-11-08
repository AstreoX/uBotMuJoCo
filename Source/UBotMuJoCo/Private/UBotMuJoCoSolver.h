#pragma once

#include "UBotMuJoCoNative.h"
#include "UBotPhysicsBackend.h"
#include "UBotMuJoCoLibrary.h"

/** UE adaptation only. Native integration state and resources belong to Scene. */
class FUBotMuJoCoSolver final : public IUBotPhysicsSolver
{
public:
    FUBotMuJoCoSolver(const UBot::MuJoCo::Native::Configuration& Configuration, bool bAllowSceneMeshApproximation);
    virtual bool AddModel(const FUBotPhysicsModelInstanceDesc& Desc, FUBotPhysicsModelInstanceResult& OutResult,
                          FText& OutError) override;
    virtual bool RemoveModel(int64 InstanceId, FText& OutError) override;
    virtual double GetStepSeconds() const override;
    virtual bool Step(double StepEndTimeSeconds, FText& OutError) override;
    virtual EUBotPhysicsOverrunPolicy GetOverrunPolicy() const override
    {
        return EUBotPhysicsOverrunPolicy::Suspend;
    }
    virtual bool Reset(double SimTimeSeconds, FText& OutError) override;
    virtual bool ApplyInputs(int64 InstanceId, const FUBotPhysicsInputs& Inputs, FText& OutError) override;
    virtual bool GetJointState(int64 InstanceId, const FString& JointId,
                               FUBotPhysicsJointState& OutState) const override;
    virtual bool GetBodyState(int32 BodyHandle, FUBotBodyState& OutState) const override;
    virtual bool UpdateScene(const FUBotPhysicsSceneUpdate& Update, FText& OutError) override;
    bool GetActuators(int64 InstanceId, TArray<FUBotMuJoCoActuatorDescription>& OutActuators) const;
    bool GetActivation(int64 InstanceId, const FString& Name, FUBotMuJoCoNativeValues& OutState) const;
    bool GetSensor(int64 InstanceId, const FString& Name, FUBotMuJoCoNativeValues& OutState) const;

private:
    struct FInstance
    {
        UBot::MuJoCo::Native::Instance Native;
        TArray<FUBotPhysicsModelJoint> Joints;
        FUBotPhysicsInputs Inputs;
    };
    struct FHandle
    {
        int64 InstanceId;
        FString BodyId;
        bool bPublished = false;
    };
    bool Compile(const TArray<FInstance>& Models, const TMap<int64, FUBotPhysicsSceneBody>& Foreign, bool bPreserve,
                 FText& OutError);
    const FInstance* FindInstance(int64 InstanceId) const;
    UBot::MuJoCo::Native::Inputs MakeInputs() const;
    double SnapshotTime(double NativeTimeSeconds) const;
    TArray<FInstance> Instances;
    TMap<int32, FHandle> Handles;
    TMap<int64, FUBotPhysicsSceneBody> ForeignBodies;
    UBot::MuJoCo::Native::Scene Scene;
    UBot::MuJoCo::Native::Configuration Config;
    int32 NextHandle = 0;
    bool bAllowSceneMeshApproximation = false;
    bool bHasEpoch = false;
    bool bNeedsRebuild = false;
    double EpochSimSeconds = 0.0;
    double EpochNativeSeconds = 0.0;
    int64 SuccessfulSteps = 0;
};
