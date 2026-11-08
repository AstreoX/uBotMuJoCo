#include "UBotMuJoCoSolver.h"

#include "UBotMuJoCo.h"
#include "UBotMuJoCoConversion.h"
#include "UBotMuJoCoModelData.h"

#include <algorithm>
#include <limits>

namespace N = UBot::MuJoCo::Native;
namespace C = UBot::MuJoCo::Conversion;

namespace
{
constexpr std::uint64_t ForeignInstance = std::numeric_limits<std::uint64_t>::max();
std::string UTF8(const FString& S)
{
    return std::string(TCHAR_TO_UTF8(*S));
}
bool Fail(FText& Error, const std::string& Message)
{
    Error = FText::FromString(UTF8_TO_TCHAR(Message.c_str()));
    return false;
}
bool Finite(const TArray<double>& Values)
{
    for (double V : Values)
        if (!FMath::IsFinite(V))
            return false;
    return true;
}
void MirrorJoint(std::vector<double>& Q, std::vector<double>& V, EUBotPhysicsJointType Type)
{
    const size_t Offset = Type == EUBotPhysicsJointType::Free ? 3 : 0;
    if (Type == EUBotPhysicsJointType::Ball || Type == EUBotPhysicsJointType::Free)
    {
        if (Q.size() == Offset + 4)
        {
            const auto Copy = Q;
            Q[Offset] = -Copy[Offset + 1];
            Q[Offset + 1] = Copy[Offset + 2];
            Q[Offset + 2] = -Copy[Offset + 3];
            Q[Offset + 3] = Copy[Offset];
        }
        if (V.size() == Offset + 3)
        {
            V[Offset] *= -1;
            V[Offset + 2] *= -1;
        }
    }
    if (Type == EUBotPhysicsJointType::Free)
    {
        if (Q.size() == 7)
            Q[1] *= -1;
        if (V.size() == 6)
            V[1] *= -1;
    }
}
} // namespace

FUBotMuJoCoSolver::FUBotMuJoCoSolver(const N::Configuration& Configuration, bool bApproximate)
    : Config(Configuration), bAllowSceneMeshApproximation(bApproximate)
{
}

double FUBotMuJoCoSolver::GetStepSeconds() const
{
    return Config.StepSeconds;
}

const FUBotMuJoCoSolver::FInstance* FUBotMuJoCoSolver::FindInstance(int64 Id) const
{
    return Instances.FindByPredicate(
        [Id](const FInstance& I) { return I.Native.Id == static_cast<std::uint64_t>(Id); });
}

bool FUBotMuJoCoSolver::Compile(const TArray<FInstance>& Models, const TMap<int64, FUBotPhysicsSceneBody>& Foreign,
                                bool bPreserve, FText& Error)
{
    std::vector<N::Instance> Values;
    for (const FInstance& I : Models)
        Values.push_back(I.Native);
    if (!Foreign.IsEmpty())
    {
        N::Instance I;
        I.Id = ForeignInstance;
        I.Value.SuggestedStepSeconds = Config.StepSeconds;
        I.Value.SuggestedGravity = Config.Gravity;
        I.Value.SuggestedOptions = Config.Options;
        TArray<int64> Ids;
        Foreign.GetKeys(Ids);
        Ids.Sort();
        for (int64 Id : Ids)
        {
            const auto& Body = Foreign[Id];
            N::Body B;
            B.Id = UTF8(FString::Printf(TEXT("scene_%lld"), Id));
            B.Name = B.Id;
            B.Pose = C::Pose(Body.WorldTransform);
            B.Kinematic = Body.Kind == EUBotPhysicsSceneBodyKind::Kinematic;
            I.Value.Bodies.push_back(B);
            for (int32 ShapeIndex = 0; ShapeIndex < Body.Shapes.Num(); ++ShapeIndex)
            {
                const auto& Shape = Body.Shapes[ShapeIndex];
                if (Shape.Type == EUBotPhysicsShapeType::TriangleMesh && !bAllowSceneMeshApproximation)
                {
                    Error = INVTEXT("MuJoCo cannot preserve triangle-mesh collision. Author simple/convex collision or "
                                    "explicitly permit convex approximation in MuJoCo World Settings.");
                    return false;
                }
                N::Geometry G;
                G.Id = B.Id + "_" + std::to_string(ShapeIndex);
                G.Name = G.Id;
                G.BodyId = B.Id;
                G.Pose = C::Pose(Shape.LocalTransform);
                G.Size = {Shape.Size.X, Shape.Size.Y, Shape.Size.Z};
                G.Friction[0] = Body.Friction;
                switch (Shape.Type)
                {
                case EUBotPhysicsShapeType::Sphere:
                    G.Type = N::GeometryType::Sphere;
                    break;
                case EUBotPhysicsShapeType::Box:
                    G.Type = N::GeometryType::Box;
                    break;
                case EUBotPhysicsShapeType::Capsule:
                    G.Type = N::GeometryType::Capsule;
                    break;
                default:
                    G.Type = N::GeometryType::Mesh;
                    break;
                }
                for (const FVector& V : Shape.Vertices)
                    G.Vertices.push_back(C::Polar(V));
                for (int32 K = 0; K + 2 < Shape.Indices.Num(); K += 3)
                {
                    G.Indices.push_back(Shape.Indices[K]);
                    G.Indices.push_back(Shape.Indices[K + 2]);
                    G.Indices.push_back(Shape.Indices[K + 1]);
                }
                I.Value.Geometries.push_back(std::move(G));
            }
        }
        Values.push_back(std::move(I));
    }
    std::string NativeError;
    if (!Scene.SetModels(Values, Config, bPreserve, NativeError))
        return Fail(Error, NativeError);
    bNeedsRebuild = false;
    return true;
}

bool FUBotMuJoCoSolver::AddModel(const FUBotPhysicsModelInstanceDesc& Desc, FUBotPhysicsModelInstanceResult& Out,
                                 FText& Error)
{
    if (Desc.InstanceId <= 0 || FindInstance(Desc.InstanceId))
    {
        Error = INVTEXT("The model instance identity is invalid or already registered.");
        return false;
    }
    FInstance Candidate;
    Candidate.Native.Id = static_cast<std::uint64_t>(Desc.InstanceId);
    Candidate.Native.Placement = C::Pose(Desc.WorldTransform);
    if (!C::ToNative(Desc.Model, Candidate.Native.Value, Error))
        return false;
    if (!Desc.Model.Extension)
    {
        Candidate.Native.Value.SuggestedStepSeconds = Config.StepSeconds;
        Candidate.Native.Value.SuggestedGravity = Config.Gravity;
        Candidate.Native.Value.SuggestedOptions = Config.Options;
    }
    Candidate.Joints = Desc.Model.Joints;
    for (const auto& Binding : Desc.BodyBindings)
    {
        if (!Desc.Model.Bodies.ContainsByPredicate(
                [&Binding](const auto& B) { return B.Id.Equals(Binding.BodyId, ESearchCase::CaseSensitive); }))
        {
            Error = INVTEXT("The binding references an unknown model body.");
            return false;
        }
    }
    TArray<FInstance> Next = Instances;
    Next.Add(MoveTemp(Candidate));
    if (!Compile(Next, ForeignBodies, !Instances.IsEmpty(), Error))
        return false;
    Instances = MoveTemp(Next);
    FUBotPhysicsModelInstanceResult Result;
    for (const auto& Binding : Desc.BodyBindings)
    {
        const int32 Handle = NextHandle++;
        Handles.Add(Handle, FHandle{Desc.InstanceId, Binding.BodyId, false});
        Result.Bodies.Add({Binding.BodyId, Handle});
    }
    Out = MoveTemp(Result);
    return true;
}

bool FUBotMuJoCoSolver::RemoveModel(int64 Id, FText& Error)
{
    if (!FindInstance(Id))
        return true;
    TArray<FInstance> Next = Instances;
    Next.RemoveAll([Id](const auto& I) { return I.Native.Id == static_cast<std::uint64_t>(Id); });
    const bool bSuccess = Compile(Next, ForeignBodies, true, Error);
    // Ownership must end even if a native rebuild failed. Physics suspends this solver on failure.
    Instances = MoveTemp(Next);
    for (auto It = Handles.CreateIterator(); It; ++It)
        if (It.Value().InstanceId == Id)
            It.RemoveCurrent();
    bNeedsRebuild = !bSuccess;
    if (Instances.IsEmpty())
        bHasEpoch = false;
    return bSuccess;
}

N::Inputs FUBotMuJoCoSolver::MakeInputs() const
{
    N::Inputs Result;
    for (const auto& I : Instances)
    {
        for (const auto& A : I.Inputs.Actuators)
        {
            N::ActuatorInput R;
            R.InstanceId = I.Native.Id;
            R.Name = UTF8(A.Id);
            if (!A.Values.IsEmpty())
                R.Values.assign(A.Values.GetData(), A.Values.GetData() + A.Values.Num());
            Result.Actuators.push_back(std::move(R));
        }
        for (const auto& F : I.Inputs.GeneralizedForces)
        {
            N::JointForce R;
            R.InstanceId = I.Native.Id;
            R.JointId = UTF8(F.JointId);
            if (!F.Values.IsEmpty())
                R.Values.assign(F.Values.GetData(), F.Values.GetData() + F.Values.Num());
            const auto* J = I.Joints.FindByPredicate(
                [&F](const auto& V) { return V.Id.Equals(F.JointId, ESearchCase::CaseSensitive); });
            std::vector<double> Empty;
            if (J)
                MirrorJoint(Empty, R.Values, J->Type);
            Result.JointForces.push_back(std::move(R));
        }
        for (const auto& W : I.Inputs.BodyWrenches)
            Result.Wrenches.push_back(
                {I.Native.Id, UTF8(W.BodyId), C::Polar(W.ForceNewtons), C::Axial(W.TorqueNewtonMeters)});
        for (const auto& K : I.Inputs.KinematicTargets)
            Result.Kinematics.push_back({I.Native.Id, UTF8(K.BodyId), C::Pose(K.WorldTransform)});
    }
    for (const auto& Entry : ForeignBodies)
    {
        if (Entry.Value.Kind == EUBotPhysicsSceneBodyKind::Kinematic)
            Result.Kinematics.push_back({ForeignInstance, UTF8(FString::Printf(TEXT("scene_%lld"), Entry.Key)),
                                         C::Pose(Entry.Value.WorldTransform)});
    }
    return Result;
}

bool FUBotMuJoCoSolver::Step(double End, FText& Error)
{
    if (bNeedsRebuild)
    {
        Error = INVTEXT("The scene must be rebuilt and reset before stepping.");
        return false;
    }
    if (!bHasEpoch)
    {
        EpochSimSeconds = End - Config.StepSeconds;
        EpochNativeSeconds = Scene.GetTimeSeconds();
        SuccessfulSteps = 0;
        bHasEpoch = true;
    }
    // The scheduler uses a grid; MuJoCo accumulates its floating-point clock. Validate against
    // successful integration count so ordinary roundoff cannot become a false discontinuity.
    const double Expected = EpochSimSeconds + static_cast<double>(SuccessfulSteps + 1) * Config.StepSeconds;
    const double TimeTolerance =
        1.e-9 + 8.0 * std::numeric_limits<double>::epsilon() * FMath::Max(FMath::Abs(End), FMath::Abs(Expected));
    if (!FMath::IsFinite(End) || !FMath::IsNearlyEqual(End, Expected, TimeTolerance))
    {
        Error = INVTEXT(
            "The requested step is not continuous with native time. Reset the backend to establish a new time origin.");
        return false;
    }
    std::string NativeError;
    if (!Scene.Step(MakeInputs(), NativeError))
        return Fail(Error, NativeError);
    ++SuccessfulSteps;
    for (auto& Pair : Handles)
        Pair.Value.bPublished = true;
    return true;
}

bool FUBotMuJoCoSolver::Reset(double Now, FText& Error)
{
    if (!FMath::IsFinite(Now))
    {
        Error = INVTEXT("Reset time must be finite.");
        return false;
    }
    if (!Compile(Instances, ForeignBodies, false, Error))
        return false;
    std::string NativeError;
    if (!Scene.Reset("", NativeError))
        return Fail(Error, NativeError);
    EpochSimSeconds = Now;
    EpochNativeSeconds = Scene.GetTimeSeconds();
    SuccessfulSteps = 0;
    bHasEpoch = true;
    for (auto& I : Instances)
        I.Inputs = FUBotPhysicsInputs();
    for (auto& Pair : Handles)
        Pair.Value.bPublished = false;
    return true;
}

bool FUBotMuJoCoSolver::GetBodyState(int32 Handle, FUBotBodyState& Out) const
{
    const auto* H = Handles.Find(Handle);
    if (!H || !H->bPublished || !bHasEpoch)
        return false;
    N::BodyState State;
    if (!Scene.GetBodyState(H->InstanceId, UTF8(H->BodyId), State))
        return false;
    FUBotBodyState R;
    const FTransform T = C::Pose(State.Pose);
    R.PositionMeters = T.GetTranslation();
    R.Orientation = T.GetRotation();
    R.LinearVelocityMetersPerSecond = C::Polar(State.LinearVelocity);
    R.AngularVelocityRadiansPerSecond = C::Axial(State.AngularVelocity);
    R.TimestampSeconds = SnapshotTime(State.NativeTimeSeconds);
    R.Source = UBot::MuJoCo::BackendName;
    Out = R;
    return true;
}

bool FUBotMuJoCoSolver::GetJointState(int64 Id, const FString& JointId, FUBotPhysicsJointState& Out) const
{
    if (!bHasEpoch)
        return false;
    const auto* I = FindInstance(Id);
    if (!I)
        return false;
    const auto* J = I->Joints.FindByPredicate(
        [&JointId](const auto& V) { return V.Id.Equals(JointId, ESearchCase::CaseSensitive); });
    if (!J)
        return false;
    N::JointState State;
    if (!Scene.GetJointState(Id, UTF8(JointId), State))
        return false;
    MirrorJoint(State.Position, State.Velocity, J->Type);
    FUBotPhysicsJointState R;
    if (!State.Position.empty())
        R.Position.Append(State.Position.data(), static_cast<int32>(State.Position.size()));
    if (!State.Velocity.empty())
        R.Velocity.Append(State.Velocity.data(), static_cast<int32>(State.Velocity.size()));
    R.TimestampSeconds = SnapshotTime(State.NativeTimeSeconds);
    Out = MoveTemp(R);
    return true;
}

bool FUBotMuJoCoSolver::ApplyInputs(int64 Id, const FUBotPhysicsInputs& Input, FText& Error)
{
    auto* I = Instances.FindByPredicate([Id](const auto& V) { return V.Native.Id == static_cast<std::uint64_t>(Id); });
    if (!I)
    {
        Error = INVTEXT("Unknown model instance.");
        return false;
    }
    const auto Actuators = Scene.GetActuators(Id);
    TArray<FString> Seen;
    const auto HasSeen = [&Seen](const FString& Id) {
        return Seen.ContainsByPredicate(
            [&Id](const FString& Existing) { return Existing.Equals(Id, ESearchCase::CaseSensitive); });
    };
    for (const auto& A : Input.Actuators)
    {
        const auto Name = UTF8(A.Id);
        const auto Found =
            std::find_if(Actuators.begin(), Actuators.end(), [&Name](const auto& V) { return V.Name == Name; });
        if (Found == Actuators.end() || A.Values.Num() != Found->InputCount || !Finite(A.Values) || HasSeen(A.Id))
        {
            Error = INVTEXT("Actuator input identity, dimension or values are invalid.");
            return false;
        }
        Seen.Add(A.Id);
        for (int32 K = 0; K < A.Values.Num(); ++K)
            if (Found->InputLimited[K])
                if (A.Values[K] < Found->InputMinimum[K] || A.Values[K] > Found->InputMaximum[K])
                {
                    Error = INVTEXT("Actuator input is outside its authored range.");
                    return false;
                }
    }
    Seen.Reset();
    for (const auto& F : Input.GeneralizedForces)
    {
        const auto* J = I->Joints.FindByPredicate(
            [&F](const auto& V) { return V.Id.Equals(F.JointId, ESearchCase::CaseSensitive); });
        const int32 Count = J ? (J->Type == EUBotPhysicsJointType::Free    ? 6
                                 : J->Type == EUBotPhysicsJointType::Ball  ? 3
                                 : J->Type == EUBotPhysicsJointType::Fixed ? 0
                                                                           : 1)
                              : -1;
        if (!J || F.Values.Num() != Count || !Finite(F.Values) || HasSeen(F.JointId))
        {
            Error = INVTEXT("Generalized force identity, dimension or values are invalid.");
            return false;
        }
        Seen.Add(F.JointId);
    }
    Seen.Reset();
    for (const auto& W : Input.BodyWrenches)
    {
        const auto BodyId = UTF8(W.BodyId);
        const bool bFound = std::any_of(I->Native.Value.Bodies.begin(), I->Native.Value.Bodies.end(),
                                        [&BodyId](const auto& B) { return B.Id == BodyId; });
        if (!bFound || W.ForceNewtons.ContainsNaN() || W.TorqueNewtonMeters.ContainsNaN() || HasSeen(W.BodyId))
        {
            Error = INVTEXT("Body wrench identity or values are invalid.");
            return false;
        }
        Seen.Add(W.BodyId);
    }
    Seen.Reset();
    for (const auto& K : Input.KinematicTargets)
    {
        const auto BodyId = UTF8(K.BodyId);
        const bool bFound = std::any_of(I->Native.Value.Bodies.begin(), I->Native.Value.Bodies.end(),
                                        [&BodyId](const auto& B) { return B.Id == BodyId && B.Kinematic; });
        if (!bFound || K.WorldTransform.ContainsNaN() || !K.WorldTransform.GetScale3D().Equals(FVector::OneVector) ||
            !K.WorldTransform.GetRotation().IsNormalized() || HasSeen(K.BodyId))
        {
            Error = INVTEXT("Kinematic target must name a kinematic body and contain a finite rigid transform.");
            return false;
        }
        Seen.Add(K.BodyId);
    }
    I->Inputs = Input;
    return true;
}

bool FUBotMuJoCoSolver::UpdateScene(const FUBotPhysicsSceneUpdate& Update, FText& Error)
{
    auto Next = ForeignBodies;
    for (int64 Id : Update.Removed)
        Next.Remove(Id);
    for (const auto& Body : Update.Added)
        Next.Add(Body.Id, Body);
    for (const auto& P : Update.KinematicPoses)
        if (auto* Body = Next.Find(P.Id))
            Body->WorldTransform = P.WorldTransform;
    if (!Update.Removed.IsEmpty() || !Update.Added.IsEmpty())
    {
        if (!Compile(Instances, Next, true, Error))
        {
            ForeignBodies = MoveTemp(Next);
            bNeedsRebuild = true;
            return false;
        }
    }
    ForeignBodies = MoveTemp(Next);
    return true;
}

bool FUBotMuJoCoSolver::GetActuators(int64 Id, TArray<FUBotMuJoCoActuatorDescription>& Out) const
{
    if (!FindInstance(Id))
        return false;
    TArray<FUBotMuJoCoActuatorDescription> Result;
    for (const auto& A : Scene.GetActuators(Id))
    {
        auto& Value = Result.AddDefaulted_GetRef();
        Value.Name = UTF8_TO_TCHAR(A.Name.c_str());
        Value.InputCount = A.InputCount;
        for (int32 K = 0; K < A.InputCount; ++K)
        {
            Value.InputLimited.Add(A.InputLimited[K]);
            Value.InputRanges.Add(FVector2D(A.InputMinimum[K], A.InputMaximum[K]));
        }
    }
    Out = MoveTemp(Result);
    return true;
}

bool FUBotMuJoCoSolver::GetActivation(int64 Id, const FString& Name, FUBotMuJoCoNativeValues& Out) const
{
    if (!bHasEpoch || !FindInstance(Id))
        return false;
    N::ActuatorState State;
    if (!Scene.GetActuatorState(Id, UTF8(Name), State))
        return false;
    FUBotMuJoCoNativeValues Result;
    if (!State.Activation.empty())
        Result.Values.Append(State.Activation.data(), static_cast<int32>(State.Activation.size()));
    Result.TimestampSeconds = SnapshotTime(State.NativeTimeSeconds);
    Out = MoveTemp(Result);
    return true;
}

bool FUBotMuJoCoSolver::GetSensor(int64 Id, const FString& Name, FUBotMuJoCoNativeValues& Out) const
{
    if (!bHasEpoch || !FindInstance(Id))
        return false;
    N::SensorState State;
    if (!Scene.GetSensorState(Id, UTF8(Name), State))
        return false;
    FUBotMuJoCoNativeValues Result;
    if (!State.Values.empty())
        Result.Values.Append(State.Values.data(), static_cast<int32>(State.Values.size()));
    Result.TimestampSeconds = SnapshotTime(State.NativeTimeSeconds);
    Out = MoveTemp(Result);
    return true;
}

double FUBotMuJoCoSolver::SnapshotTime(double NativeTimeSeconds) const
{
    // Every published endpoint follows one successful fixed step. Rounding recovers its grid
    // index without changing the native clock or manufacturing any skipped integration.
    const double Index = FMath::RoundToDouble((NativeTimeSeconds - EpochNativeSeconds) / Config.StepSeconds);
    return EpochSimSeconds + Index * Config.StepSeconds;
}
