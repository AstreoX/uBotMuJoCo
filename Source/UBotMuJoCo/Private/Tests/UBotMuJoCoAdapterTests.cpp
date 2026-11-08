#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "UBotMuJoCoConversion.h"
#include "UBotMuJoCoModelData.h"
#include "UBotMuJoCoSolver.h"
#include "UBotMuJoCoImport.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
FUBotPhysicsModel MakeQuaternionModel()
{
    FUBotPhysicsModel M;
    M.Revision = FGuid::NewGuid();
    FUBotPhysicsModelBody Root;
    Root.Id = TEXT("root");
    M.Bodies.Add(Root);
    FUBotPhysicsModelBody Child;
    Child.Id = TEXT("child");
    Child.ParentId = Root.Id;
    Child.LocalTransform.SetTranslation(FVector(0.5, 0.0, 0.0));
    M.Bodies.Add(Child);
    FUBotPhysicsModelJoint Free;
    Free.Id = TEXT("free");
    Free.BodyId = Root.Id;
    Free.Type = EUBotPhysicsJointType::Free;
    Free.InitialPosition = {1.0, 2.0, 3.0, 0.2, 0.3, -0.4, FMath::Sqrt(0.71)};
    Free.InitialVelocity.Init(0.0, 6);
    M.Joints.Add(Free);
    FUBotPhysicsModelJoint Ball;
    Ball.Id = TEXT("ball");
    Ball.BodyId = Child.Id;
    Ball.Type = EUBotPhysicsJointType::Ball;
    Ball.InitialPosition = {-0.1, 0.5, 0.2, FMath::Sqrt(0.70)};
    Ball.InitialVelocity.Init(0.0, 3);
    M.Joints.Add(Ball);
    return M;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotMuJoCoCoordinateLayoutTest, "UBotMuJoCo.Adapter.QuaternionLayouts",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotMuJoCoCoordinateLayoutTest::RunTest(const FString& Parameters)
{
    const auto M = MakeQuaternionModel();
    UBot::MuJoCo::Native::Model Native;
    FText Error;
    if (!TestTrue(TEXT("Common model converts"), UBot::MuJoCo::Conversion::ToNative(M, Native, Error)))
        return false;
    const auto& Q = Native.Joints[0].InitialPosition;
    TestEqual(TEXT("Free translation mirrors Y"), Q[1], -2.0);
    TestEqual(TEXT("Free quaternion starts with W"), Q[3], FMath::Sqrt(0.71));
    TestEqual(TEXT("Axial quaternion X mirrors"), Q[4], -0.2);
    TestEqual(TEXT("Axial quaternion Y stays"), Q[5], 0.3);
    TestEqual(TEXT("Axial quaternion Z mirrors"), Q[6], 0.4);
    TestEqual(TEXT("Ball quaternion starts with W"), Native.Joints[1].InitialPosition[0], FMath::Sqrt(0.70));
    TestEqual(TEXT("Ball quaternion X mirrors"), Native.Joints[1].InitialPosition[1], 0.1);
    auto* Preset = NewObject<UUBotMuJoCoSolverPreset>();
    UBot::MuJoCo::Native::ImportResult Imported;
    Imported.Value = Native;
    FUBotPhysicsModel RoundTrip;
    if (!TestTrue(TEXT("Native model converts back"),
                  UBot::MuJoCo::Conversion::FromNative(Imported, GetTransientPackage(), Preset, RoundTrip, Error)))
        return false;
    TestEqual(TEXT("Free nq"), RoundTrip.Joints[0].InitialPosition.Num(), 7);
    TestEqual(TEXT("Ball nq"), RoundTrip.Joints[1].InitialPosition.Num(), 4);
    for (int32 J = 0; J < M.Joints.Num(); ++J)
        for (int32 K = 0; K < M.Joints[J].InitialPosition.Num(); ++K)
            TestEqual(TEXT("Coordinate survives roundtrip"), RoundTrip.Joints[J].InitialPosition[K],
                      M.Joints[J].InitialPosition[K]);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotMuJoCoNativeEndpointTest, "UBotMuJoCo.Adapter.NativeEndpointAndReset",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotMuJoCoNativeEndpointTest::RunTest(const FString& Parameters)
{
    UBot::MuJoCo::Native::Configuration Config;
    Config.Gravity = {0, 0, 0};
    FUBotMuJoCoSolver Solver(Config, false);
    FUBotPhysicsModelInstanceDesc Desc;
    Desc.InstanceId = 1;
    Desc.Model = MakeQuaternionModel();
    FUBotPhysicsModelBodyBindingDesc Display;
    Display.BodyId = TEXT("root");
    Desc.BodyBindings.Add(Display);
    FUBotPhysicsModelInstanceResult Result;
    FText Error;
    if (!TestTrue(TEXT("Register complete model"), Solver.AddModel(Desc, Result, Error)))
    {
        AddError(Error.ToString());
        return false;
    }
    TestEqual(TEXT("One displayed handle"), Result.Bodies.Num(), 1);
    FUBotBodyState Body;
    TestFalse(TEXT("No state before integration"), Solver.GetBodyState(Result.Bodies[0].Handle, Body));
    if (!TestTrue(TEXT("First step at a nonzero sim origin"), Solver.Step(10.002, Error)))
    {
        AddError(Error.ToString());
        return false;
    }
    TestTrue(TEXT("State available"), Solver.GetBodyState(Result.Bodies[0].Handle, Body));
    TestTrue(TEXT("Body origin in SI UE axes"), Body.PositionMeters.Equals(FVector(1, 2, 3), 1.e-8));
    const FQuat Expected(0.2, 0.3, -0.4, FMath::Sqrt(0.71));
    TestTrue(TEXT("Nontrivial body quaternion"), Body.Orientation.Equals(Expected, 1.e-8));
    TestTrue(TEXT("Epoch maps endpoint"), FMath::IsNearlyEqual(Body.TimestampSeconds, 10.002, 1.e-9));
    FUBotPhysicsJointState Joint;
    TestTrue(TEXT("Ball state available"), Solver.GetJointState(1, TEXT("ball"), Joint));
    TestEqual(TEXT("Ball position coordinate count"), Joint.Position.Num(), 4);
    TestEqual(TEXT("Ball velocity coordinate count"), Joint.Velocity.Num(), 3);
    for (int32 K = 0; K < 4; ++K)
        TestTrue(TEXT("Ball coordinate matches authored XYZW"),
                 FMath::IsNearlyEqual(Joint.Position[K], Desc.Model.Joints[1].InitialPosition[K], 1.e-8));
    TestFalse(TEXT("A discontinuous requested step is refused"), Solver.Step(11.0, Error));
    TestTrue(TEXT("Last endpoint remains available"), Solver.GetBodyState(Result.Bodies[0].Handle, Body));
    TestTrue(TEXT("Refusal does not restamp state"), FMath::IsNearlyEqual(Body.TimestampSeconds, 10.002, 1.e-9));
    TestTrue(TEXT("Explicit scene reset"), Solver.Reset(20.0, Error));
    TestFalse(TEXT("Reset invalidates published state"), Solver.GetBodyState(Result.Bodies[0].Handle, Body));
    TestTrue(TEXT("Step follows new origin"), Solver.Step(20.002, Error));
    TestTrue(TEXT("Remove whole model"), Solver.RemoveModel(1, Error));
    TestFalse(TEXT("Old handle stays invalid"), Solver.GetBodyState(Result.Bodies[0].Handle, Body));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotMuJoCoNativeObservationsTest, "UBotMuJoCo.Adapter.NativeObservations",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotMuJoCoNativeObservationsTest::RunTest(const FString& Parameters)
{
    const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("UBotMuJoCo"));
    if (!TestTrue(TEXT("Plugin available"), Plugin.IsValid()))
        return false;
    FUBotMuJoCoResource Resource;
    Resource.Name = TEXT("coupled.xml");
    if (!TestTrue(
            TEXT("Fixture available"),
            FFileHelper::LoadFileToArray(
                Resource.Bytes, *FPaths::Combine(Plugin->GetBaseDir(), TEXT("Tests/Native/fixtures/coupled.xml")))))
        return false;
    auto* Preset = NewObject<UUBotMuJoCoSolverPreset>();
    FUBotPhysicsModel Model;
    FText Error;
    if (!TestTrue(TEXT("Import full native features"),
                  UBot::MuJoCo::ImportModel(Resource.Name, {Resource}, GetTransientPackage(), Preset, Model, Error)))
    {
        AddError(Error.ToString());
        return false;
    }
    UBot::MuJoCo::Native::Configuration Config;
    Config.StepSeconds = Preset->StepSeconds;
    Config.Gravity = UBot::MuJoCo::Conversion::Polar(Preset->GravityMetersPerSecondSquared);
    Config.Options = TCHAR_TO_UTF8(*Preset->NativeOptions);
    FUBotMuJoCoSolver Solver(Config, false);
    FUBotPhysicsModelInstanceDesc Desc;
    Desc.InstanceId = 1;
    Desc.Model = Model;
    FUBotPhysicsModelInstanceResult Result;
    if (!TestTrue(TEXT("Compile native extension"), Solver.AddModel(Desc, Result, Error)))
    {
        AddError(Error.ToString());
        return false;
    }
    TArray<FUBotMuJoCoActuatorDescription> Descriptions;
    TestTrue(TEXT("Describe authored actuator"), Solver.GetActuators(1, Descriptions));
    TestEqual(TEXT("One native actuator"), Descriptions.Num(), 1);
    if (Descriptions.Num() != 1)
        return false;
    TestEqual(TEXT("Authored identity"), Descriptions[0].Name, FString(TEXT("filtered")));
    TestEqual(TEXT("Authored input width"), Descriptions[0].InputCount, 1);
    FUBotMuJoCoNativeValues Activation, Sensor;
    TestFalse(TEXT("Activation is not published before integration"),
              Solver.GetActivation(1, TEXT("filtered"), Activation));
    TestFalse(TEXT("Sensor is not published before integration"), Solver.GetSensor(1, TEXT("angle"), Sensor));
    FUBotPhysicsInputs Input;
    Input.Actuators.Add({TEXT("filtered"), {0.2}});
    TestTrue(TEXT("Native actuator input accepted"), Solver.ApplyInputs(1, Input, Error));
    for (int32 Step = 1; Step <= 10; ++Step)
        if (!TestTrue(TEXT("Integrate native actuator"), Solver.Step(17.0 + Step * Config.StepSeconds, Error)))
            return false;
    TestTrue(TEXT("Activation endpoint available"), Solver.GetActivation(1, TEXT("filtered"), Activation));
    TestTrue(TEXT("Sensor endpoint available"), Solver.GetSensor(1, TEXT("angle"), Sensor));
    TestEqual(TEXT("Activation state width"), Activation.Values.Num(), 1);
    TestEqual(TEXT("Sensor state width"), Sensor.Values.Num(), 1);
    if (Activation.Values.Num() == 1)
        TestTrue(TEXT("Filtered activation evolves within authored input"),
                 Activation.Values[0] > 0.0 && Activation.Values[0] < 0.2);
    TestTrue(TEXT("Activation timestamp uses uBot endpoint"),
             FMath::IsNearlyEqual(Activation.TimestampSeconds, 17.02, 1.e-9));
    TestTrue(TEXT("Sensor shares the same endpoint"),
             FMath::IsNearlyEqual(Sensor.TimestampSeconds, Activation.TimestampSeconds, 1.e-9));
    TestFalse(TEXT("Native names are case-sensitive"), Solver.GetSensor(1, TEXT("ANGLE"), Sensor));
    TestTrue(TEXT("Reset native state"), Solver.Reset(20.0, Error));
    TestFalse(TEXT("Reset clears activation snapshot"), Solver.GetActivation(1, TEXT("filtered"), Activation));
    TestFalse(TEXT("Reset clears sensor snapshot"), Solver.GetSensor(1, TEXT("angle"), Sensor));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotMuJoCoLongRunClockTest, "UBotMuJoCo.Adapter.LongRunTimeGrid",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotMuJoCoLongRunClockTest::RunTest(const FString& Parameters)
{
    UBot::MuJoCo::Native::Configuration Config;
    Config.StepSeconds = 0.1;
    Config.Gravity = {0, 0, 0};
    FUBotMuJoCoSolver Solver(Config, false);
    FUBotPhysicsModelInstanceDesc Desc;
    Desc.InstanceId = 1;
    Desc.Model = MakeQuaternionModel();
    FUBotPhysicsModelBodyBindingDesc Display;
    Display.BodyId = TEXT("root");
    Desc.BodyBindings.Add(Display);
    FUBotPhysicsModelInstanceResult Result;
    FText Error;
    if (!TestTrue(TEXT("Register clock regression model"), Solver.AddModel(Desc, Result, Error)))
        return false;
    // 100,000 additions of 0.1 produce native clock roundoff above the old fixed 1e-8
    // continuity threshold. No forces or velocities are present, isolating time bookkeeping.
    constexpr int32 Steps = 100000;
    for (int32 Step = 1; Step <= Steps; ++Step)
    {
        if (!Solver.Step(42.0 + static_cast<double>(Step) * Config.StepSeconds, Error))
        {
            AddError(FString::Printf(TEXT("Clock continuity failed at step %d: %s"), Step, *Error.ToString()));
            return false;
        }
    }
    FUBotBodyState State;
    TestTrue(TEXT("Final body endpoint available"), Solver.GetBodyState(Result.Bodies[0].Handle, State));
    TestTrue(TEXT("Snapshot remains on the integration grid"),
             FMath::IsNearlyEqual(State.TimestampSeconds, 10042.0, 1.e-9));
    TestTrue(TEXT("No-motion state stays unchanged"), State.PositionMeters.Equals(FVector(1, 2, 3), 1.e-8));
    return true;
}

#endif
