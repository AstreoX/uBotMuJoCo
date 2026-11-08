#include "UBotMuJoCoConversion.h"

#include "UBotMuJoCoImport.h"
#include "UBotMuJoCoModelData.h"
#include "UBotRightHandedConventions.h"

namespace UBot::MuJoCo::Conversion
{
static std::string UTF8(const FString& Value)
{
    return std::string(TCHAR_TO_UTF8(*Value));
}
static FString String(const std::string& Value)
{
    return FString(UTF8_TO_TCHAR(Value.c_str()));
}
static N::Vector3 Raw(const FVector& V)
{
    return {V.X, V.Y, V.Z};
}
static FVector Raw(const N::Vector3& V)
{
    return FVector(V.X, V.Y, V.Z);
}
static N::Quaternion Rotation(const FQuat& Q)
{
    const FQuat R = UBot::RightHanded::RotationFromUE(Q);
    return {R.X, R.Y, R.Z, R.W};
}
static FQuat Rotation(const N::Quaternion& Q)
{
    return UBot::RightHanded::RotationToUE(FQuat(Q.X, Q.Y, Q.Z, Q.W));
}
N::Vector3 Polar(const FVector& V)
{
    return Raw(UBot::RightHanded::VectorFromUE(V));
}
N::Vector3 Axial(const FVector& V)
{
    return Raw(UBot::RightHanded::AngularVelocityFromUE(V));
}
FVector Polar(const N::Vector3& V)
{
    return UBot::RightHanded::VectorToUE(Raw(V));
}
FVector Axial(const N::Vector3& V)
{
    return UBot::RightHanded::AngularVelocityToUE(Raw(V));
}
N::Transform Pose(const FTransform& T)
{
    return {Polar(T.GetTranslation()), Rotation(T.GetRotation())};
}
FTransform Pose(const N::Transform& T)
{
    return FTransform(Rotation(T.Rotation), Polar(T.Position));
}

static void ConvertCoordinates(std::vector<double>& Q, std::vector<double>& V, EUBotPhysicsJointType Type,
                               bool bToNative)
{
    const int32 Offset = Type == EUBotPhysicsJointType::Free ? 3 : 0;
    if (Type == EUBotPhysicsJointType::Ball || Type == EUBotPhysicsJointType::Free)
    {
        if (Q.size() == static_cast<size_t>(Offset + 4))
        {
            const auto Copy = Q;
            if (bToNative)
            {
                Q[Offset] = Copy[Offset + 3];
                Q[Offset + 1] = -Copy[Offset];
                Q[Offset + 2] = Copy[Offset + 1];
                Q[Offset + 3] = -Copy[Offset + 2];
            }
            else
            {
                Q[Offset] = -Copy[Offset + 1];
                Q[Offset + 1] = Copy[Offset + 2];
                Q[Offset + 2] = -Copy[Offset + 3];
                Q[Offset + 3] = Copy[Offset];
            }
        }
        if (V.size() == static_cast<size_t>(Offset + 3))
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

static std::string FindName(const TArray<FUBotMuJoCoElementName>* Names, const FString& Id)
{
    if (Names)
    {
        for (const auto& Entry : *Names)
        {
            if (Entry.Id.Equals(Id, ESearchCase::CaseSensitive))
                return UTF8(Entry.Name);
        }
    }
    return UTF8(Id);
}

bool ToNative(const FUBotPhysicsModel& M, N::Model& Out, FText& Error)
{
    if (!M.Validate(Error))
        return false;
    const auto* Extra = Cast<UUBotMuJoCoModelData>(M.Extension);
    if (M.Extension && !Extra)
    {
        Error = INVTEXT("The model contains an extension which MuJoCo cannot compile.");
        return false;
    }
    N::Model Result;
    if (Extra)
    {
        Result.NativeData = UTF8(Extra->NativeData);
        Result.SuggestedStepSeconds = Extra->ImportedStepSeconds;
        Result.SuggestedGravity = Polar(Extra->ImportedGravity);
        Result.SuggestedOptions = UTF8(Extra->ImportedOptions);
        for (const auto& Resource : Extra->Resources)
        {
            N::Resource R;
            R.Name = UTF8(Resource.Name);
            if (!Resource.Bytes.IsEmpty())
                R.Bytes.assign(Resource.Bytes.GetData(), Resource.Bytes.GetData() + Resource.Bytes.Num());
            Result.Resources.push_back(std::move(R));
        }
    }
    for (const auto& B : M.Bodies)
    {
        N::Body R;
        R.Id = UTF8(B.Id);
        R.Name = FindName(Extra ? &Extra->BodyNames : nullptr, B.Id);
        R.ParentId = UTF8(B.ParentId);
        R.Pose = Pose(B.LocalTransform);
        R.Mass = B.MassKilograms;
        R.CenterOfMass = Polar(B.InertialTransform.GetTranslation());
        R.InertiaOrientation = Rotation(B.InertialTransform.GetRotation());
        R.DiagonalInertia = Raw(B.InertiaKilogramMetersSquared);
        R.Kinematic = B.bKinematic;
        Result.Bodies.push_back(std::move(R));
    }
    for (const auto& J : M.Joints)
    {
        if (J.Type == EUBotPhysicsJointType::Fixed)
            continue;
        N::Joint R;
        R.Id = UTF8(J.Id);
        R.Name = FindName(Extra ? &Extra->JointNames : nullptr, J.Id);
        R.BodyId = UTF8(J.BodyId);
        switch (J.Type)
        {
        case EUBotPhysicsJointType::Free:
            R.Type = N::JointType::Free;
            break;
        case EUBotPhysicsJointType::Ball:
            R.Type = N::JointType::Ball;
            break;
        case EUBotPhysicsJointType::Slide:
            R.Type = N::JointType::Slide;
            break;
        default:
            R.Type = N::JointType::Hinge;
            break;
        }
        R.Position = Polar(J.PositionMeters);
        R.Axis = J.Type == EUBotPhysicsJointType::Slide ? Polar(J.Axis) : Axial(J.Axis);
        R.Limited = J.bLimited;
        R.LowerLimit = J.Limits.X;
        R.UpperLimit = J.Limits.Y;
        if (!J.InitialPosition.IsEmpty())
            R.InitialPosition.assign(J.InitialPosition.GetData(),
                                     J.InitialPosition.GetData() + J.InitialPosition.Num());
        if (!J.InitialVelocity.IsEmpty())
            R.InitialVelocity.assign(J.InitialVelocity.GetData(),
                                     J.InitialVelocity.GetData() + J.InitialVelocity.Num());
        ConvertCoordinates(R.InitialPosition, R.InitialVelocity, J.Type, true);
        Result.Joints.push_back(std::move(R));
    }
    for (const auto& G : M.Geometries)
    {
        N::Geometry R;
        R.Id = UTF8(G.Id);
        R.Name = FindName(Extra ? &Extra->GeometryNames : nullptr, G.Id);
        R.BodyId = UTF8(G.BodyId);
        R.Pose = Pose(G.LocalTransform);
        R.Size = Raw(G.Size);
        R.AssetName = UTF8(G.Resource);
        switch (G.Type)
        {
        case EUBotPhysicsGeometryType::Sphere:
            R.Type = N::GeometryType::Sphere;
            break;
        case EUBotPhysicsGeometryType::Box:
            R.Type = N::GeometryType::Box;
            break;
        case EUBotPhysicsGeometryType::Capsule:
            R.Type = N::GeometryType::Capsule;
            break;
        case EUBotPhysicsGeometryType::Cylinder:
            R.Type = N::GeometryType::Cylinder;
            break;
        case EUBotPhysicsGeometryType::Ellipsoid:
            R.Type = N::GeometryType::Ellipsoid;
            break;
        case EUBotPhysicsGeometryType::Plane:
            R.Type = N::GeometryType::Plane;
            break;
        case EUBotPhysicsGeometryType::HeightField:
            R.Type = N::GeometryType::HeightField;
            break;
        default:
            R.Type = N::GeometryType::Mesh;
            break;
        }
        R.Collision = G.bCollision;
        if (Extra)
        {
            for (const auto& Material : Extra->GeometryMaterials)
            {
                if (Material.GeometryId.Equals(G.Id, ESearchCase::CaseSensitive))
                {
                    R.Friction[0] = Material.Friction.X;
                    R.Friction[1] = Material.Friction.Y;
                    R.Friction[2] = Material.Friction.Z;
                    break;
                }
            }
        }
        R.Color[0] = G.Color.R;
        R.Color[1] = G.Color.G;
        R.Color[2] = G.Color.B;
        R.Color[3] = G.Color.A;
        for (const FVector& V : G.Vertices)
            R.Vertices.push_back(Polar(V));
        for (int32 I = 0; I + 2 < G.Indices.Num(); I += 3)
        {
            R.Indices.push_back(G.Indices[I]);
            R.Indices.push_back(G.Indices[I + 2]);
            R.Indices.push_back(G.Indices[I + 1]);
        }
        Result.Geometries.push_back(std::move(R));
    }
    for (const auto& D : M.Drives)
    {
        N::Drive R;
        R.Id = UTF8(D.Id);
        R.JointId = UTF8(D.JointId);
        R.Stiffness = D.Stiffness;
        R.Damping = D.Damping;
        R.Type = D.Type == EUBotPhysicsDriveType::Position   ? N::DriveType::Position
                 : D.Type == EUBotPhysicsDriveType::Velocity ? N::DriveType::Velocity
                                                             : N::DriveType::Effort;
        Result.Drives.push_back(std::move(R));
    }
    Out = std::move(Result);
    return true;
}

bool FromNative(const N::ImportResult& Import, UObject* Outer, UUBotMuJoCoSolverPreset* Preset, FUBotPhysicsModel& Out,
                FText& Error)
{
    if (!Outer || !Preset)
    {
        Error = INVTEXT("Import requires an owner and a solver preset.");
        return false;
    }
    FUBotPhysicsModel Result;
    Result.Revision = FGuid::NewGuid();
    auto* Extra = NewObject<UUBotMuJoCoModelData>(Outer);
    const auto& M = Import.Value;
    Extra->NativeData = String(M.NativeData);
    Extra->ImportedStepSeconds = M.SuggestedStepSeconds;
    Extra->ImportedGravity = Polar(M.SuggestedGravity);
    Extra->ImportedOptions = String(M.SuggestedOptions);
    Extra->SuggestedPreset = Preset;
    for (const auto& Message : Import.Messages)
        Extra->ImportMessages.Add(String(Message));
    for (const auto& R : M.Resources)
    {
        auto& Value = Extra->Resources.AddDefaulted_GetRef();
        Value.Name = String(R.Name);
        if (!R.Bytes.empty())
            Value.Bytes.Append(R.Bytes.data(), static_cast<int32>(R.Bytes.size()));
    }
    for (const auto& B : M.Bodies)
    {
        auto& R = Result.Bodies.AddDefaulted_GetRef();
        R.Id = String(B.Id);
        R.ParentId = String(B.ParentId);
        R.LocalTransform = Pose(B.Pose);
        R.MassKilograms = B.Mass;
        R.InertialTransform = FTransform(Rotation(B.InertiaOrientation), Polar(B.CenterOfMass));
        R.InertiaKilogramMetersSquared = Raw(B.DiagonalInertia);
        R.bKinematic = B.Kinematic;
        auto& Name = Extra->BodyNames.AddDefaulted_GetRef();
        Name.Id = R.Id;
        Name.Name = String(B.Name);
    }
    for (const auto& J : M.Joints)
    {
        auto& R = Result.Joints.AddDefaulted_GetRef();
        R.Id = String(J.Id);
        R.BodyId = String(J.BodyId);
        R.Type = J.Type == N::JointType::Free    ? EUBotPhysicsJointType::Free
                 : J.Type == N::JointType::Ball  ? EUBotPhysicsJointType::Ball
                 : J.Type == N::JointType::Slide ? EUBotPhysicsJointType::Slide
                                                 : EUBotPhysicsJointType::Hinge;
        R.PositionMeters = Polar(J.Position);
        R.Axis = J.Type == N::JointType::Slide ? Polar(J.Axis) : Axial(J.Axis);
        R.bLimited = J.Limited;
        R.Limits = FVector2D(J.LowerLimit, J.UpperLimit);
        auto Q = J.InitialPosition, V = J.InitialVelocity;
        ConvertCoordinates(Q, V, R.Type, false);
        if (!Q.empty())
            R.InitialPosition.Append(Q.data(), static_cast<int32>(Q.size()));
        if (!V.empty())
            R.InitialVelocity.Append(V.data(), static_cast<int32>(V.size()));
        auto& Name = Extra->JointNames.AddDefaulted_GetRef();
        Name.Id = R.Id;
        Name.Name = String(J.Name);
    }
    for (const auto& G : M.Geometries)
    {
        auto& R = Result.Geometries.AddDefaulted_GetRef();
        R.Id = String(G.Id);
        R.BodyId = String(G.BodyId);
        R.LocalTransform = Pose(G.Pose);
        R.Size = Raw(G.Size);
        R.Resource = String(G.AssetName);
        R.bCollision = G.Collision;
        R.Type = G.Type == N::GeometryType::Sphere        ? EUBotPhysicsGeometryType::Sphere
                 : G.Type == N::GeometryType::Box         ? EUBotPhysicsGeometryType::Box
                 : G.Type == N::GeometryType::Capsule     ? EUBotPhysicsGeometryType::Capsule
                 : G.Type == N::GeometryType::Cylinder    ? EUBotPhysicsGeometryType::Cylinder
                 : G.Type == N::GeometryType::Ellipsoid   ? EUBotPhysicsGeometryType::Ellipsoid
                 : G.Type == N::GeometryType::Plane       ? EUBotPhysicsGeometryType::Plane
                 : G.Type == N::GeometryType::HeightField ? EUBotPhysicsGeometryType::HeightField
                                                          : EUBotPhysicsGeometryType::Convex;
        R.Color = FLinearColor(G.Color[0], G.Color[1], G.Color[2], G.Color[3]);
        for (const auto& V : G.Vertices)
            R.Vertices.Add(Polar(V));
        for (size_t I = 0; I + 2 < G.Indices.size(); I += 3)
        {
            R.Indices.Add(G.Indices[I]);
            R.Indices.Add(G.Indices[I + 2]);
            R.Indices.Add(G.Indices[I + 1]);
        }
        auto& Name = Extra->GeometryNames.AddDefaulted_GetRef();
        Name.Id = R.Id;
        Name.Name = String(G.Name);
        auto& Material = Extra->GeometryMaterials.AddDefaulted_GetRef();
        Material.GeometryId = R.Id;
        Material.Friction = FVector(G.Friction[0], G.Friction[1], G.Friction[2]);
    }
    Result.Extension = Extra;
    if (!Result.Validate(Error))
        return false;
    Preset->StepSeconds = M.SuggestedStepSeconds;
    Preset->GravityMetersPerSecondSquared = Extra->ImportedGravity;
    Preset->NativeOptions = Extra->ImportedOptions;
    Out = MoveTemp(Result);
    return true;
}
} // namespace UBot::MuJoCo::Conversion

bool UBot::MuJoCo::ImportModel(const FString& Filename, const TArray<FUBotMuJoCoResource>& Resources, UObject* Outer,
                               UUBotMuJoCoSolverPreset* Preset, FUBotPhysicsModel& OutModel, FText& OutError)
{
    std::vector<Native::Resource> Files;
    for (const auto& R : Resources)
    {
        Native::Resource F;
        F.Name = TCHAR_TO_UTF8(*R.Name);
        if (!R.Bytes.IsEmpty())
            F.Bytes.assign(R.Bytes.GetData(), R.Bytes.GetData() + R.Bytes.Num());
        Files.push_back(std::move(F));
    }
    Native::ImportResult Result;
    std::string Error;
    if (!Native::Import(TCHAR_TO_UTF8(*Filename), Files, Result, Error))
    {
        OutError = FText::FromString(UTF8_TO_TCHAR(Error.c_str()));
        return false;
    }
    return Conversion::FromNative(Result, Outer, Preset, OutModel, OutError);
}
