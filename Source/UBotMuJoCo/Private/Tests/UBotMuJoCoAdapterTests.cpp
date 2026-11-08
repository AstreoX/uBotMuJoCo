#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "UBotMuJoCoConversion.h"
#include "UBotMuJoCoModelData.h"
#include "UBotMuJoCoSolver.h"

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

bool FUBotMuJoCoCoordinateLayoutTest::RunTest(const FString &Parameters)
{
    const auto M = MakeQuaternionModel();
    UBot::MuJoCo::Native::Model Native;
    FText Error;
    if (!TestTrue(TEXT("Common model converts"), UBot::MuJoCo::Conversion::ToNative(M, Native, Error)))
        return false;
    const auto &Q = Native.Joints[0].InitialPosition;
    TestEqual(TEXT("Free translation mirrors Y"), Q[1], -2.0);
    TestEqual(TEXT("Free quaternion starts with W"), Q[3], FMath::Sqrt(0.71));
    TestEqual(TEXT("Axial quaternion X mirrors"), Q[4], -0.2);
    TestEqual(TEXT("Axial quaternion Y stays"), Q[5], 0.3);
    TestEqual(TEXT("Axial quaternion Z mirrors"), Q[6], 0.4);
    TestEqual(TEXT("Ball quaternion starts with W"), Native.Joints[1].InitialPosition[0], FMath::Sqrt(0.70));
    TestEqual(TEXT("Ball quaternion X mirrors"), Native.Joints[1].InitialPosition[1], 0.1);
    auto *Preset = NewObject<UUBotMuJoCoSolverPreset>();
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

bool FUBotMuJoCoNativeEndpointTest::RunTest(const FString &Parameters)
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

#endif
