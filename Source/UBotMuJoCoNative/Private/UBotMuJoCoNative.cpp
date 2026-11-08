#include "UBotMuJoCoNative.h"
#include "ThirdParty/tinyxml2.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <mujoco/mujoco.h>
#include <set>
#include <sstream>
#include <stdexcept>

namespace UBot::MuJoCo::Native
{
namespace
{
using namespace tinyxml2;
struct NativeError : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
thread_local int CallDepth = 0;
thread_local std::vector<std::string> *WarningOutput = nullptr;
struct WarningCapture
{
    std::vector<std::string> *Previous;
    explicit WarningCapture(std::vector<std::string> &Output) : Previous(WarningOutput)
    {
        WarningOutput = &Output;
    }
    ~WarningCapture()
    {
        WarningOutput = Previous;
    }
};
mjfLogHandler PreviousHandler = nullptr;
bool HandlerInstalled = false;
#if defined(UBOT_MUJOCO_STANDALONE)
bool RuntimeAvailable = true;
#else
bool RuntimeAvailable = false;
#endif
void LogHandler(const mjLogMessage *Message)
{
    if (CallDepth && Message->level == mjLOG_ERROR)
    {
        throw NativeError(Message->subject);
    }
    if (CallDepth && Message->level == mjLOG_WARNING && WarningOutput)
    {
        WarningOutput->push_back(Message->subject);
    }
    if (!CallDepth && PreviousHandler)
    {
        PreviousHandler(Message);
    }
}
struct CallScope
{
    CallScope()
    {
        if (!RuntimeAvailable)
        {
            throw NativeError("MuJoCo 4.0.0 runtime is unavailable");
        }
        if (!HandlerInstalled)
        {
            PreviousHandler = mju_setLogHandler(LogHandler);
            HandlerInstalled = true;
        }
        ++CallDepth;
    }
    ~CallScope()
    {
        --CallDepth;
    }
};
template <class F> bool Guard(std::string &Error, F &&Function)
{
    Error.clear();
    try
    {
        CallScope Scope;
        Function();
        return true;
    }
    catch (const std::exception &E)
    {
        Error = E.what();
        return false;
    }
    catch (...)
    {
        Error = "Unknown native MuJoCo failure";
        return false;
    }
}
void Require(bool Condition, const std::string &Message)
{
    if (!Condition)
    {
        throw NativeError(Message);
    }
}
std::string Name(const mjModel *M, mjtObj Type, int Id)
{
    const char *Value = mj_id2name(M, Type, Id);
    Require(Value != nullptr, "Unnamed normalized element");
    return Value;
}
Vector3 Vec(const mjtNum *P)
{
    return {P[0], P[1], P[2]};
}
Quaternion Quat(const mjtNum *Q)
{
    return {Q[1], Q[2], Q[3], Q[0]};
}
std::string Numbers(std::initializer_list<double> Values)
{
    std::ostringstream S;
    S << std::setprecision(17);
    for (double V : Values)
    {
        Require(std::isfinite(V), "Non-finite model value");
        S << V << ' ';
    }
    return S.str();
}
std::string Text(const XMLNode *N)
{
    XMLPrinter P;
    N->Accept(&P);
    return P.CStr();
}
void SetPose(XMLElement *E, const Transform &P)
{
    E->SetAttribute("pos", Numbers({P.Position.X, P.Position.Y, P.Position.Z}).c_str());
    E->SetAttribute("quat", Numbers({P.Rotation.W, P.Rotation.X, P.Rotation.Y, P.Rotation.Z}).c_str());
}
void Remove(XMLElement *E, std::initializer_list<const char *> Names)
{
    for (const char *N : Names)
    {
        E->DeleteAttribute(N);
    }
}
void StripPose(XMLElement *E)
{
    Remove(E, {"pos", "quat", "axisangle", "xyaxes", "zaxis", "euler", "fromto"});
}
void CopyAttributes(const XMLElement *Src, XMLElement *Dst)
{
    for (const XMLAttribute *A = Src->FirstAttribute(); A; A = A->Next())
    {
        Dst->SetAttribute(A->Name(), A->Value());
    }
}
struct VFS
{
    mjVFS Value;
    explicit VFS(const std::vector<Resource> &R)
    {
        mj_defaultVFS(&Value);
        try
        {
            for (const Resource &F : R)
            {
                Require(!F.Name.empty() && F.Name.find(':') == std::string::npos && F.Name.front() != '/' &&
                            F.Name.find("..") == std::string::npos,
                        "Resource names must be relative paths without parent traversal");
                Require(F.Bytes.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()),
                        "Resource too large");
                Require(mj_addBufferVFS(&Value, F.Name.c_str(), F.Bytes.data(), static_cast<int>(F.Bytes.size())) == 0,
                        "Invalid or duplicate resource: " + F.Name);
            }
        }
        catch (...)
        {
            mj_deleteVFS(&Value);
            throw;
        }
    }
    ~VFS()
    {
        mj_deleteVFS(&Value);
    }
};
SpecPtr Parse(const std::string &XML, const VFS &Files)
{
    char Error[2048]{};
    SpecPtr S(mj_parseXMLString(XML.c_str(), &Files.Value, Error, sizeof(Error)), mj_deleteSpec);
    Require(S != nullptr, Error);
    return S;
}
ModelPtr Compile(mjSpec *S, const VFS &Files)
{
    ModelPtr M(mj_compile(S, &Files.Value), mj_deleteModel);
    Require(M != nullptr, mjs_getError(S));
    return M;
}
std::string Save(mjSpec *S)
{
    char Error[2048]{};
    const int N = mj_saveXMLString(S, nullptr, 0, Error, sizeof(Error));
    Require(N > 0, Error);
    std::vector<char> Bytes(static_cast<std::size_t>(N) + 1);
    Require(mj_saveXMLString(S, Bytes.data(), static_cast<int>(Bytes.size()), Error, sizeof(Error)) == 0, Error);
    return Bytes.data();
}
void CheckUnsupported(XMLNode *Node)
{
    for (XMLElement *E = Node->FirstChildElement(); E; E = E->NextSiblingElement())
    {
        const std::string T = E->Name();
        Require(T != "flex" && T != "flexcomp" && T != "skin" && T != "plugin" && T != "extension" && T != "composite",
                "Unsupported native feature: " + T);
        if (const char *D = E->Attribute("dyntype"))
        {
            Require(std::strcmp(D, "user") != 0, "User dynamics callbacks are unsupported");
        }
        if (const char *D = E->Attribute("gaintype"))
        {
            Require(std::strcmp(D, "user") != 0, "User gain callbacks are unsupported");
        }
        if (const char *D = E->Attribute("biastype"))
        {
            Require(std::strcmp(D, "user") != 0, "User bias callbacks are unsupported");
        }
        Require(T != "user" || !E->Parent() || !E->Parent()->ToElement() ||
                    std::strcmp(E->Parent()->Value(), "sensor") != 0,
                "User sensor callbacks are unsupported");
        CheckUnsupported(E);
    }
}
void CheckVirtualReferences(const std::vector<Resource> &Resources)
{
    std::set<std::string> Names;
    for (const Resource &R : Resources)
    {
        Names.insert(R.Name);
    }
    auto Relative = [](std::string Value)
    {
        std::replace(Value.begin(), Value.end(), '\\', '/');
        Require(!Value.empty() && Value.front() != '/' && Value.find(':') == std::string::npos &&
                    Value.find("..") == std::string::npos,
                "Only relative bundled resource paths are supported");
        while (Value.rfind("./", 0) == 0)
        {
            Value.erase(0, 2);
        }
        return Value;
    };
    std::vector<std::string> AssetDirectories;
    for (const Resource &R : Resources)
    {
        XMLDocument D;
        if (D.Parse(reinterpret_cast<const char *>(R.Bytes.data()), R.Bytes.size()) != XML_SUCCESS)
        {
            continue;
        }
        auto Directories = [&](auto &&Self, XMLElement *E) -> void
        {
            for (const char *A : {"meshdir", "texturedir", "assetdir"})
            {
                if (const char *Value = E->Attribute(A))
                {
                    AssetDirectories.push_back(Relative(Value));
                }
            }
            for (XMLElement *C = E->FirstChildElement(); C; C = C->NextSiblingElement())
            {
                Self(Self, C);
            }
        };
        if (D.RootElement())
        {
            Directories(Directories, D.RootElement());
        }
    }
    for (const Resource &R : Resources)
    {
        XMLDocument D;
        if (D.Parse(reinterpret_cast<const char *>(R.Bytes.data()), R.Bytes.size()) != XML_SUCCESS)
        {
            continue;
        }
        const auto Slash = R.Name.find_last_of('/');
        const std::string Directory = Slash == std::string::npos ? "" : R.Name.substr(0, Slash + 1);
        auto References = [&](auto &&Self, XMLElement *E) -> void
        {
            for (const char *A : {"file", "filename"})
            {
                if (const char *Value = E->Attribute(A))
                {
                    const std::string File = Relative(Value);
                    bool Bundled = Names.count(File) != 0 || Names.count(Directory + File) != 0;
                    for (const std::string &Asset : AssetDirectories)
                    {
                        Bundled |=
                            Names.count(Asset + "/" + File) != 0 || Names.count(Directory + Asset + "/" + File) != 0;
                    }
                    Require(Bundled, "Missing bundled dependency: " + File);
                }
            }
            for (XMLElement *C = E->FirstChildElement(); C; C = C->NextSiblingElement())
            {
                Self(Self, C);
            }
        };
        if (D.RootElement())
        {
            References(References, D.RootElement());
        }
    }
}
void CleanDefault(XMLElement *E)
{
    const std::string T = E->Name();
    if (T == "geom")
    {
        StripPose(E);
        Remove(E, {"type", "size", "mesh", "hfield", "rgba", "mass", "density", "friction"});
    }
    if (T == "joint")
    {
        Remove(E, {"type", "pos", "axis", "limited", "range"});
    }
    for (XMLElement *C = E->FirstChildElement(); C; C = C->NextSiblingElement())
    {
        CleanDefault(C);
    }
}
const char *JointName(JointType T)
{
    switch (T)
    {
    case JointType::Free:
        return "free";
    case JointType::Ball:
        return "ball";
    case JointType::Slide:
        return "slide";
    default:
        return "hinge";
    }
}
const char *GeometryName(GeometryType T)
{
    switch (T)
    {
    case GeometryType::Plane:
        return "plane";
    case GeometryType::Sphere:
        return "sphere";
    case GeometryType::Capsule:
        return "capsule";
    case GeometryType::Ellipsoid:
        return "ellipsoid";
    case GeometryType::Cylinder:
        return "cylinder";
    case GeometryType::Box:
        return "box";
    case GeometryType::HeightField:
        return "hfield";
    default:
        return "mesh";
    }
}
int NQ(int T)
{
    return T == mjJNT_FREE ? 7 : T == mjJNT_BALL ? 4 : 1;
}
int NV(int T)
{
    return T == mjJNT_FREE ? 6 : T == mjJNT_BALL ? 3 : 1;
}
std::string Prefix(std::uint64_t Id)
{
    return "ubot_" + std::to_string(Id) + "_";
}
} // namespace

// Called solely by the UE module startup/shutdown; standalone harness links the same implementation.
void SetRuntimeAvailable(bool Available)
{
    if (!Available && HandlerInstalled)
    {
        mju_setLogHandler(PreviousHandler);
        HandlerInstalled = false;
    }
    RuntimeAvailable = Available;
}
int Version()
{
    return RuntimeAvailable ? mj_version() : 0;
}

bool Import(const std::string &Filename, const std::vector<Resource> &Resources, ImportResult &Out, std::string &Error)
{
    return Guard(
        Error,
        [&]
        {
            std::vector<std::string> ImportMessages;
            WarningCapture Warnings(ImportMessages);
            CheckVirtualReferences(Resources);
            Require(
                std::any_of(Resources.begin(), Resources.end(), [&](const Resource &R) { return R.Name == Filename; }),
                "Input document is not present in the virtual resources");
            XMLDocument Source;
            for (const Resource &R : Resources)
            {
                if (R.Name == Filename)
                {
                    Require(Source.Parse(reinterpret_cast<const char *>(R.Bytes.data()), R.Bytes.size()) == XML_SUCCESS,
                            "Invalid source XML");
                    break;
                }
            }
            CheckUnsupported(&Source);
            VFS Files(Resources);
            char Message[2048]{};
            SpecPtr S(mj_parseXML(Filename.c_str(), &Files.Value, Message, sizeof(Message)), mj_deleteSpec);
            Require(S != nullptr, Message);
            // First compile honors source compiler semantics. Common values are extracted from
            // the resulting normalized frames; reconstruction disables a second normalization.

            S->compiler.usethread = false;
            if (Source.RootElement() && std::strcmp(Source.RootElement()->Name(), "robot") == 0)
            {
                Require(!Source.RootElement()->FirstChildElement("transmission"),
                        "URDF transmission elements require explicit actuator authoring");
                for (XMLElement *E = Source.RootElement()->FirstChildElement(); E; E = E->NextSiblingElement())
                {
                    const std::string T = E->Name();
                    if (T != "link" && T != "joint" && T != "material" && T != "mujoco")
                    {
                        ImportMessages.push_back("Ignored URDF extension: " + T);
                    }
                }
                for (XMLElement *J = Source.RootElement()->FirstChildElement("joint"); J;
                     J = J->NextSiblingElement("joint"))
                {
                    if (XMLElement *Mimic = J->FirstChildElement("mimic"))
                    {
                        Require(J->Attribute("name") && Mimic->Attribute("joint"), "Invalid URDF mimic reference");
                        mjsEquality *Existing = nullptr;
                        for (mjsElement *Element = mjs_firstElement(S.get(), mjOBJ_EQUALITY); Element;
                             Element = mjs_nextElement(S.get(), Element))
                        {
                            mjsEquality *Candidate = mjs_asEquality(Element);
                            if (Candidate && Candidate->type == mjEQ_JOINT &&
                                *Candidate->name1 == J->Attribute("name") &&
                                *Candidate->name2 == Mimic->Attribute("joint"))
                            {
                                Existing = Candidate;
                                break;
                            }
                        }
                        if (Existing)
                        {
                            Require(std::abs(Existing->data[0] - Mimic->DoubleAttribute("offset", 0)) < 1e-12 &&
                                        std::abs(Existing->data[1] - Mimic->DoubleAttribute("multiplier", 1)) < 1e-12,
                                    "Native URDF mimic mapping does not match its source equation");
                            ImportMessages.push_back("Retained URDF mimic joint as native equality: " +
                                                     *Existing->name1);
                            continue;
                        }
                        mjsEquality *E = mjs_addEquality(S.get(), nullptr);
                        E->type = mjEQ_JOINT;
                        E->objtype = mjOBJ_JOINT;
                        E->active = true;
                        *E->name1 = J->Attribute("name");
                        *E->name2 = Mimic->Attribute("joint");
                        E->data[0] = Mimic->DoubleAttribute("offset", 0);
                        E->data[1] = Mimic->DoubleAttribute("multiplier", 1);
                        mjs_setName(E->element, ("ubot_urdf_mimic_" + *E->name1).c_str());
                        ImportMessages.push_back("Converted URDF mimic joint to native equality: " + *E->name1);
                    }
                }
            }
            for (mjtObj T : {mjOBJ_BODY, mjOBJ_JOINT, mjOBJ_GEOM, mjOBJ_ACTUATOR, mjOBJ_SITE, mjOBJ_KEY, mjOBJ_SENSOR,
                             mjOBJ_CAMERA, mjOBJ_LIGHT})
            {
                int Index = 0;
                for (mjsElement *E = mjs_firstElement(S.get(), T); E; E = mjs_nextElement(S.get(), E), ++Index)
                {
                    if (mjs_getName(E)->empty() && !(T == mjOBJ_BODY && Index == 0))
                    {
                        const std::string Generated =
                            "ubot_unnamed_" + std::to_string(static_cast<int>(T)) + "_" + std::to_string(Index);
                        Require(mjs_setName(E, Generated.c_str()) == 0, "Could not assign an element identity");
                    }
                }
            }
            ModelPtr M = Compile(S.get(), Files);
            Require(mj_copyBack(S.get(), M.get()) == 1, "Cannot normalize compiled model");
            const std::string XML = Save(S.get());
            XMLDocument D;
            Require(D.Parse(XML.c_str()) == XML_SUCCESS, "Invalid normalized XML");
            CheckUnsupported(&D);
            ImportResult Result;
            Result.Messages = std::move(ImportMessages);
            Model &V = Result.Value;
            for (const Resource &R : Resources)
            {
                std::string Extension = R.Name.substr(
                    R.Name.find_last_of('.') == std::string::npos ? R.Name.size() : R.Name.find_last_of('.'));
                std::transform(Extension.begin(), Extension.end(), Extension.begin(),
                               [](unsigned char C) { return static_cast<char>(std::tolower(C)); });
                if (R.Name != Filename && Extension != ".xml" && Extension != ".urdf")
                {
                    V.Resources.push_back(R);
                }
            }
            V.SuggestedStepSeconds = M->opt.timestep;
            V.SuggestedGravity = Vec(M->opt.gravity);
            XMLElement *Root = D.RootElement();
            if (XMLElement *Option = Root->FirstChildElement("option"))
            {
                V.SuggestedOptions = Text(Option);
                Root->DeleteChild(Option);
            }
            XMLDocument Extras;
            XMLElement *Native = Extras.NewElement("native");
            Extras.InsertEndChild(Native);
            XMLElement *RootExtras = Extras.NewElement("root");
            Native->InsertEndChild(RootExtras);
            for (XMLElement *E = Root->FirstChildElement(); E;)
            {
                XMLElement *Next = E->NextSiblingElement();
                if (std::strcmp(E->Name(), "worldbody") != 0)
                {
                    if (std::strcmp(E->Name(), "default") == 0)
                    {
                        CleanDefault(E);
                    }
                    RootExtras->InsertEndChild(E->DeepClone(&Extras));
                }
                E = Next;
            }
            for (int B = 0; B < M->nbody; ++B)
            {
                Body C;
                C.Id = C.Name = B ? Name(M.get(), mjOBJ_BODY, B) : "world";
                C.ParentId = B ? (M->body_parentid[B] ? Name(M.get(), mjOBJ_BODY, M->body_parentid[B]) : "world") : "";
                C.Pose = {Vec(M->body_pos + 3 * B), Quat(M->body_quat + 4 * B)};
                C.Mass = M->body_mass[B];
                C.CenterOfMass = Vec(M->body_ipos + 3 * B);
                C.DiagonalInertia = Vec(M->body_inertia + 3 * B);
                C.InertiaOrientation = Quat(M->body_iquat + 4 * B);
                C.Kinematic = M->body_mocapid[B] >= 0;
                C.ModelFrame = B == 0;
                V.Bodies.push_back(C);
            }
            for (int J = 0; J < M->njnt; ++J)
            {
                Joint C;
                C.Id = C.Name = Name(M.get(), mjOBJ_JOINT, J);
                C.BodyId = Name(M.get(), mjOBJ_BODY, M->jnt_bodyid[J]);
                C.Type = static_cast<JointType>(M->jnt_type[J]);
                C.Position = Vec(M->jnt_pos + 3 * J);
                C.Axis = Vec(M->jnt_axis + 3 * J);
                C.Limited = M->jnt_limited[J];
                C.LowerLimit = M->jnt_range[2 * J];
                C.UpperLimit = M->jnt_range[2 * J + 1];
                C.InitialPosition.assign(M->qpos0 + M->jnt_qposadr[J],
                                         M->qpos0 + M->jnt_qposadr[J] + NQ(M->jnt_type[J]));
                C.InitialVelocity.resize(NV(M->jnt_type[J]), 0);
                V.Joints.push_back(C);
            }
            for (int G = 0; G < M->ngeom; ++G)
            {
                Require(M->geom_type[G] >= mjGEOM_PLANE && M->geom_type[G] <= mjGEOM_MESH, "Unsupported geometry type");
                Geometry C;
                C.Id = C.Name = Name(M.get(), mjOBJ_GEOM, G);
                C.BodyId = M->geom_bodyid[G] ? Name(M.get(), mjOBJ_BODY, M->geom_bodyid[G]) : "world";
                // Native geometry enum uses hfield before mesh, unlike our stable plain boundary enum.
                switch (M->geom_type[G])
                {
                case mjGEOM_PLANE:
                    C.Type = GeometryType::Plane;
                    break;
                case mjGEOM_SPHERE:
                    C.Type = GeometryType::Sphere;
                    break;
                case mjGEOM_CAPSULE:
                    C.Type = GeometryType::Capsule;
                    break;
                case mjGEOM_ELLIPSOID:
                    C.Type = GeometryType::Ellipsoid;
                    break;
                case mjGEOM_CYLINDER:
                    C.Type = GeometryType::Cylinder;
                    break;
                case mjGEOM_BOX:
                    C.Type = GeometryType::Box;
                    break;
                case mjGEOM_HFIELD:
                    C.Type = GeometryType::HeightField;
                    break;
                default:
                    C.Type = GeometryType::Mesh;
                    break;
                }
                C.Pose = {Vec(M->geom_pos + 3 * G), Quat(M->geom_quat + 4 * G)};
                C.Size = Vec(M->geom_size + 3 * G);
                if (C.Type == GeometryType::Capsule || C.Type == GeometryType::Cylinder)
                {
                    C.Size.Z = C.Size.Y;
                    C.Size.Y = 0;
                }
                for (int K = 0; K < 3; ++K)
                {
                    C.Friction[K] = M->geom_friction[3 * G + K];
                }
                C.Collision = M->geom_contype[G] != 0 || M->geom_conaffinity[G] != 0;
                for (int K = 0; K < 4; ++K)
                {
                    C.Color[K] = M->geom_rgba[4 * G + K];
                }
                if (C.Type == GeometryType::Mesh || C.Type == GeometryType::HeightField)
                {
                    const int A = M->geom_dataid[G];
                    C.AssetName = Name(M.get(), C.Type == GeometryType::Mesh ? mjOBJ_MESH : mjOBJ_HFIELD, A);
                    if (C.Type == GeometryType::Mesh)
                    {
                        mjtNum InverseP[3], InverseQ[4], RawP[3], RawQ[4];
                        mju_negPose(InverseP, InverseQ, M->mesh_pos + 3 * A, M->mesh_quat + 4 * A);
                        mju_mulPose(RawP, RawQ, M->geom_pos + 3 * G, M->geom_quat + 4 * G, InverseP, InverseQ);
                        C.Pose = {Vec(RawP), Quat(RawQ)};
                        for (int I = 0; I < M->mesh_vertnum[A]; ++I)
                        {
                            const float *P = M->mesh_vert + 3 * (M->mesh_vertadr[A] + I);
                            mjtNum Processed[3]{P[0], P[1], P[2]}, Original[3];
                            mju_rotVecQuat(Original, Processed, M->mesh_quat + 4 * A);
                            C.Vertices.push_back({Original[0] + M->mesh_pos[3 * A],
                                                  Original[1] + M->mesh_pos[3 * A + 1],
                                                  Original[2] + M->mesh_pos[3 * A + 2]});
                        }
                        C.Indices.assign(M->mesh_face + 3 * M->mesh_faceadr[A],
                                         M->mesh_face + 3 * (M->mesh_faceadr[A] + M->mesh_facenum[A]));
                    }
                    else
                    {
                        const int Rows = M->hfield_nrow[A], Cols = M->hfield_ncol[A];
                        const mjtNum *Size = M->hfield_size + 4 * A;
                        for (int R = 0; R < Rows; ++R)
                        {
                            for (int K = 0; K < Cols; ++K)
                            {
                                C.Vertices.push_back({Size[0] * (2.0 * K / (Cols - 1) - 1),
                                                      Size[1] * (2.0 * R / (Rows - 1) - 1),
                                                      Size[2] * M->hfield_data[M->hfield_adr[A] + R * Cols + K]});
                            }
                        }
                        for (int R = 0; R < Rows - 1; ++R)
                        {
                            for (int K = 0; K < Cols - 1; ++K)
                            {
                                const int Vertex = R * Cols + K;
                                C.Indices.insert(C.Indices.end(), {Vertex, Vertex + 1, Vertex + Cols, Vertex + 1,
                                                                   Vertex + Cols + 1, Vertex + Cols});
                            }
                        }
                    }
                }
                V.Geometries.push_back(C);
            }
            // Split native-only attributes and non-physical body children from the canonical hierarchy.
            auto CopyNativeChildren = [&](auto &&Self, const XMLElement *From, XMLElement *To) -> void
            {
                for (const XMLElement *C = From->FirstChildElement(); C; C = C->NextSiblingElement())
                {
                    const std::string T = C->Name();
                    if (T == "body" || T == "joint" || T == "freejoint" || T == "geom" || T == "inertial" ||
                        T == "site" || T == "camera" || T == "light")
                    {
                        continue;
                    }
                    if (T == "frame")
                    {
                        XMLElement *Frame = Extras.NewElement("frame");
                        CopyAttributes(C, Frame);
                        Self(Self, C, Frame);
                        if (Frame->FirstChildElement())
                        {
                            To->InsertEndChild(Frame);
                        }
                        else
                        {
                            Extras.DeleteNode(Frame);
                        }
                    }
                    else
                    {
                        To->InsertEndChild(C->DeepClone(&Extras));
                    }
                }
            };
            auto Visit = [&](auto &&Self, XMLElement *Parent) -> void
            {
                for (XMLElement *E = Parent->FirstChildElement(); E; E = E->NextSiblingElement())
                {
                    const std::string T = E->Name();
                    if (T == "body" || T == "joint" || T == "freejoint" || T == "geom" || T == "worldbody")
                    {
                        const char *N = T == "worldbody" ? "world" : E->Attribute("name");
                        Require(N != nullptr, "Normalized element has no name");
                        XMLElement *X = Extras.NewElement(T == "worldbody"   ? "body"
                                                          : T == "freejoint" ? "joint"
                                                                             : T.c_str());
                        Native->InsertEndChild(X);
                        CopyAttributes(E, X);
                        X->DeleteAttribute("name");
                        X->SetAttribute("id", N);
                        if (T == "body" || T == "worldbody")
                        {
                            StripPose(X);
                            Remove(X, {"mocap"});
                        }
                        if (T == "joint" || T == "freejoint")
                        {
                            Remove(X, {"type", "pos", "axis", "limited", "range"});
                        }
                        if (T == "geom")
                        {
                            StripPose(X);
                            Remove(X, {"type", "size", "mesh", "hfield", "rgba", "mass", "density", "friction"});
                        }
                        CopyNativeChildren(CopyNativeChildren, E, X);
                        Self(Self, E);
                    }
                    else if (T == "frame")
                    {
                        Self(Self, E);
                    }
                }
            };
            if (XMLElement *WB = Root->FirstChildElement("worldbody"))
            {
                XMLElement *Wrapper = D.NewElement("wrapper");
                D.InsertEndChild(Wrapper);
                Wrapper->InsertEndChild(WB);
                Visit(Visit, Wrapper);
            }

            // Compiler normalization can align free-body inertial frames or fuse fixed bodies.
            // Native-only tree elements must use their compiled local frames, too; copying
            // their original XML poses would mix source and normalized body coordinates.
            auto FindTree = [&](auto &&Self, XMLNode *Parent, const char *Tag,
                                const std::string &ElementName) -> XMLElement *
            {
                for (XMLElement *E = Parent->FirstChildElement(); E; E = E->NextSiblingElement())
                {
                    if (std::strcmp(E->Name(), Tag) == 0 && E->Attribute("name") && ElementName == E->Attribute("name"))
                    {
                        return E;
                    }
                    if (XMLElement *Found = Self(Self, E, Tag, ElementName))
                    {
                        return Found;
                    }
                }
                return nullptr;
            };
            auto NativeBody = [&](int B) -> XMLElement *
            {
                const std::string Id = B ? Name(M.get(), mjOBJ_BODY, B) : "world";
                for (XMLElement *E = Native->FirstChildElement("body"); E; E = E->NextSiblingElement("body"))
                {
                    if (E->Attribute("id") && Id == E->Attribute("id"))
                    {
                        return E;
                    }
                }
                XMLElement *E = Extras.NewElement("body");
                E->SetAttribute("id", Id.c_str());
                Native->InsertEndChild(E);
                return E;
            };
            auto CopyTreeElement = [&](mjtObj Type, int Id, const char *Tag, int BodyId) -> XMLElement *
            {
                const std::string ElementName = Name(M.get(), Type, Id);
                XMLElement *SourceElement = FindTree(FindTree, &D, Tag, ElementName);
                Require(SourceElement != nullptr,
                        "Native normalized tree element is absent from source: " + ElementName);
                XMLElement *E = SourceElement->DeepClone(&Extras)->ToElement();
                StripPose(E);
                NativeBody(BodyId)->InsertEndChild(E);
                return E;
            };
            for (int A = 0; A < M->nsite; ++A)
            {
                XMLElement *E = CopyTreeElement(mjOBJ_SITE, A, "site", M->site_bodyid[A]);
                SetPose(E, {Vec(M->site_pos + 3 * A), Quat(M->site_quat + 4 * A)});
                E->SetAttribute(
                    "size", Numbers({M->site_size[3 * A], M->site_size[3 * A + 1], M->site_size[3 * A + 2]}).c_str());
            }
            for (int A = 0; A < M->ncam; ++A)
            {
                XMLElement *E = CopyTreeElement(mjOBJ_CAMERA, A, "camera", M->cam_bodyid[A]);
                SetPose(E, {Vec(M->cam_pos + 3 * A), Quat(M->cam_quat + 4 * A)});
            }
            for (int A = 0; A < M->nlight; ++A)
            {
                XMLElement *E = CopyTreeElement(mjOBJ_LIGHT, A, "light", M->light_bodyid[A]);
                E->SetAttribute(
                    "pos", Numbers({M->light_pos[3 * A], M->light_pos[3 * A + 1], M->light_pos[3 * A + 2]}).c_str());
                E->SetAttribute(
                    "dir", Numbers({M->light_dir[3 * A], M->light_dir[3 * A + 1], M->light_dir[3 * A + 2]}).c_str());
            }

            if (M->nkey)
            {
                if (XMLElement *Existing = RootExtras->FirstChildElement("keyframe"))
                {
                    RootExtras->DeleteChild(Existing);
                }
                XMLElement *Keys = Extras.NewElement("keyframe");
                RootExtras->InsertEndChild(Keys);
                auto Array = [](const mjtNum *Values, mjtSize Count)
                {
                    std::ostringstream Text;
                    Text << std::setprecision(17);
                    for (mjtSize K = 0; K < Count; ++K)
                    {
                        Require(std::isfinite(Values[K]), "Non-finite native keyframe");
                        Text << Values[K] << ' ';
                    }
                    return Text.str();
                };
                for (int K = 0; K < M->nkey; ++K)
                {
                    XMLElement *Key = Extras.NewElement("key");
                    Keys->InsertEndChild(Key);
                    const char *N = mj_id2name(M.get(), mjOBJ_KEY, K);
                    const std::string Generated = "ubot_key_" + std::to_string(K);
                    Key->SetAttribute("name", N ? N : Generated.c_str());
                    Key->SetAttribute("time", M->key_time[K]);
                    if (M->nq)
                    {
                        Key->SetAttribute("qpos", Array(M->key_qpos + K * M->nq, M->nq).c_str());
                    }
                    if (M->nv)
                    {
                        Key->SetAttribute("qvel", Array(M->key_qvel + K * M->nv, M->nv).c_str());
                    }
                    if (M->na)
                    {
                        Key->SetAttribute("act", Array(M->key_act + K * M->na, M->na).c_str());
                    }
                    if (M->nu)
                    {
                        Key->SetAttribute("ctrl", Array(M->key_ctrl + K * M->nu, M->nu).c_str());
                    }
                    if (M->nmocap)
                    {
                        Key->SetAttribute("mpos", Array(M->key_mpos + K * 3 * M->nmocap, 3 * M->nmocap).c_str());
                        Key->SetAttribute("mquat", Array(M->key_mquat + K * 4 * M->nmocap, 4 * M->nmocap).c_str());
                    }
                }
            }
            V.NativeData = Text(Native);
            Result.Messages.push_back("Imported rigid-body model; native sections retained in the backend extension");
            Out = std::move(Result);
        });
}

bool BuildXML(const Model &V, std::string &Out, std::string &Error)
{
    return Guard(
        Error,
        [&]
        {
            XMLDocument D;
            XMLElement *Root = D.NewElement("mujoco");
            D.InsertEndChild(Root);
            XMLDocument Extra;
            XMLElement *Native = nullptr;
            if (!V.NativeData.empty())
            {
                Require(Extra.Parse(V.NativeData.c_str()) == XML_SUCCESS, "Invalid native model extension");
                Native = Extra.FirstChildElement("native");
                Require(Native != nullptr, "Invalid native model extension root");
                for (XMLElement *E = Native->FirstChildElement(); E; E = E->NextSiblingElement())
                {
                    const std::string T = E->Name();
                    Require(T == "root" || T == "body" || T == "joint" || T == "geom",
                            "Unknown native extension element");
                    if (T == "root")
                    {
                        for (XMLElement *C = E->FirstChildElement(); C; C = C->NextSiblingElement())
                        {
                            const std::string Name = C->Name();
                            Require(Name != "worldbody" && Name != "body" && Name != "joint" && Name != "geom" &&
                                        Name != "option",
                                    "Common model/global options cannot be authored in native extension root");
                        }
                    }
                    else
                    {
                        Require(E->Attribute("id") != nullptr, "Native element has no common identity");
                        for (const char *A : {"name", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler", "fromto"})
                        {
                            Require(!E->Attribute(A), "Common transform/identity duplicated in native extension");
                        }
                        if (T == "body")
                        {
                            Require(!E->Attribute("mocap"), "Common motion kind duplicated in native extension");
                        }
                        if (T == "joint")
                        {
                            for (const char *A : {"type", "axis", "limited", "range"})
                            {
                                Require(!E->Attribute(A), "Common joint field duplicated in native extension");
                            }
                        }
                        if (T == "geom")
                        {
                            for (const char *A :
                                 {"type", "size", "mesh", "hfield", "rgba", "mass", "density", "friction"})
                            {
                                Require(!E->Attribute(A), "Common geometry field duplicated in native extension");
                            }
                        }
                    }
                }
            }
            if (Native)
            {
                if (XMLElement *R = Native->FirstChildElement("root"))
                {
                    for (XMLElement *E = R->FirstChildElement(); E; E = E->NextSiblingElement())
                    {
                        Root->InsertEndChild(E->DeepClone(&D));
                    }
                }
            }
            XMLElement *Compiler = Root->FirstChildElement("compiler");
            if (!Compiler)
            {
                Compiler = D.NewElement("compiler");
                Root->InsertFirstChild(Compiler);
            }
            Remove(Compiler, {"boundmass", "boundinertia", "settotalmass", "balanceinertia"});
            Compiler->SetAttribute("angle", "radian");
            Compiler->SetAttribute("inertiafromgeom", "false");
            Compiler->SetAttribute("fusestatic", "false");
            Compiler->SetAttribute("alignfree", "false");
            XMLElement *Option = D.NewElement("option");
            Root->InsertEndChild(Option);
            if (!V.SuggestedOptions.empty())
            {
                XMLDocument O;
                Require(O.Parse(V.SuggestedOptions.c_str()) == XML_SUCCESS, "Invalid solver options");
                CopyAttributes(O.RootElement(), Option);
                for (XMLElement *C = O.RootElement()->FirstChildElement(); C; C = C->NextSiblingElement())
                {
                    Option->InsertEndChild(C->DeepClone(&D));
                }
            }
            Option->SetAttribute("timestep", V.SuggestedStepSeconds);
            Option->SetAttribute("gravity",
                                 Numbers({V.SuggestedGravity.X, V.SuggestedGravity.Y, V.SuggestedGravity.Z}).c_str());
            XMLElement *WB = D.NewElement("worldbody");
            Root->InsertEndChild(WB);
            std::map<std::string, XMLElement *> Bodies;
            std::map<std::string, std::string> BodyNames, JointNames;
            std::map<std::string, std::string> Parents;
            for (const Body &B : V.Bodies)
            {
                Require(Parents.emplace(B.Id, B.ParentId).second, "Duplicate body identity");
            }
            for (const Body &B : V.Bodies)
            {
                std::set<std::string> Seen;
                std::string Current = B.Id;
                while (!Current.empty())
                {
                    Require(Seen.insert(Current).second, "Cyclic body hierarchy");
                    Require(Parents.count(Current) != 0, "Unknown parent body");
                    Current = Parents.at(Current);
                }
            }
            auto Apply = [&](const char *Type, const std::string &Id, XMLElement *E)
            {
                if (!Native)
                {
                    return;
                }
                for (XMLElement *X = Native->FirstChildElement(Type); X; X = X->NextSiblingElement(Type))
                {
                    if (X->Attribute("id") && Id == X->Attribute("id"))
                    {
                        CopyAttributes(X, E);
                        E->DeleteAttribute("id");
                        for (XMLElement *C = X->FirstChildElement(); C; C = C->NextSiblingElement())
                        {
                            E->InsertEndChild(C->DeepClone(&D));
                        }
                        break;
                    }
                }
            };
            for (const Body &B : V.Bodies)
            {
                Require(!B.Id.empty() && Bodies.find(B.Id) == Bodies.end(), "Duplicate/empty body identity");
                Require(std::isfinite(B.Mass) && B.Mass >= 0, "Body mass must be finite and nonnegative");
                XMLElement *E = B.ModelFrame ? WB : D.NewElement("body");
                if (E != WB)
                {
                    E->SetAttribute("name", B.Name.c_str());
                    SetPose(E, B.Pose);
                    if (B.Kinematic)
                    {
                        E->SetAttribute("mocap", "true");
                    }
                }
                Bodies[B.Id] = E;
                BodyNames[B.Id] = B.Name;
                Apply("body", B.Id, E);
                if (E != WB && B.Mass > 0)
                {
                    XMLElement *I = D.NewElement("inertial");
                    I->SetAttribute("mass", B.Mass);
                    I->SetAttribute("pos", Numbers({B.CenterOfMass.X, B.CenterOfMass.Y, B.CenterOfMass.Z}).c_str());
                    I->SetAttribute("quat", Numbers({B.InertiaOrientation.W, B.InertiaOrientation.X,
                                                     B.InertiaOrientation.Y, B.InertiaOrientation.Z})
                                                .c_str());
                    I->SetAttribute("diaginertia",
                                    Numbers({B.DiagonalInertia.X, B.DiagonalInertia.Y, B.DiagonalInertia.Z}).c_str());
                    E->InsertFirstChild(I);
                }
            }
            for (const Body &B : V.Bodies)
            {
                XMLElement *E = Bodies.at(B.Id);
                if (E == WB)
                {
                    continue;
                }
                XMLElement *P = WB;
                if (!B.ParentId.empty())
                {
                    Require(Bodies.count(B.ParentId) != 0, "Unknown parent body");
                    P = Bodies.at(B.ParentId);
                }
                Require(P != E, "Cyclic body parent");
                P->InsertEndChild(E);
            }
            // Detect cycles before traversing/serializing XML.
            for (const Body &B : V.Bodies)
            {
                std::set<const XMLNode *> Seen;
                for (const XMLNode *E = Bodies.at(B.Id); E; E = E->Parent())
                {
                    Require(Seen.insert(E).second, "Cyclic body hierarchy");
                }
            }
            for (const Joint &J : V.Joints)
            {
                Require(Bodies.count(J.BodyId) != 0, "Joint references unknown body");
                Require(JointNames.emplace(J.Id, J.Name).second, "Duplicate joint identity");
                XMLElement *E = D.NewElement("joint");
                Apply("joint", J.Id, E);
                E->SetAttribute("name", J.Name.c_str());
                E->SetAttribute("type", JointName(J.Type));
                if (J.Type != JointType::Free)
                {
                    E->SetAttribute("pos", Numbers({J.Position.X, J.Position.Y, J.Position.Z}).c_str());
                }
                if (J.Type == JointType::Slide || J.Type == JointType::Hinge)
                {
                    E->SetAttribute("axis", Numbers({J.Axis.X, J.Axis.Y, J.Axis.Z}).c_str());
                }
                E->SetAttribute("limited", J.Limited ? "true" : "false");
                if (J.Limited)
                {
                    E->SetAttribute("range", Numbers({J.LowerLimit, J.UpperLimit}).c_str());
                }
                Bodies.at(J.BodyId)->InsertEndChild(E);
            }
            for (const Geometry &G : V.Geometries)
            {
                Require(Bodies.count(G.BodyId) != 0, "Geometry references unknown body");
                XMLElement *E = D.NewElement("geom");
                Apply("geom", G.Id, E);
                E->SetAttribute("name", G.Name.c_str());
                E->SetAttribute("type", GeometryName(G.Type));
                SetPose(E, G.Pose);
                if (G.Type != GeometryType::Mesh && G.Type != GeometryType::HeightField)
                {
                    E->SetAttribute(
                        "size",
                        Numbers({G.Size.X,
                                 (G.Type == GeometryType::Capsule || G.Type == GeometryType::Cylinder) ? G.Size.Z
                                                                                                       : G.Size.Y,
                                 G.Size.Z})
                            .c_str());
                }
                E->SetAttribute("friction", Numbers({G.Friction[0], G.Friction[1], G.Friction[2]}).c_str());
                if (!G.Collision)
                {
                    E->SetAttribute("contype", 0);
                    E->SetAttribute("conaffinity", 0);
                }
                E->SetAttribute("rgba", Numbers({G.Color[0], G.Color[1], G.Color[2], G.Color[3]}).c_str());
                if (G.Type == GeometryType::Mesh || G.Type == GeometryType::HeightField)
                {
                    std::string Asset = G.AssetName;
                    if (Asset.empty() && !G.Vertices.empty())
                    {
                        Asset = "ubot_mesh_" + G.Id;
                        XMLElement *Assets = Root->FirstChildElement("asset");
                        if (!Assets)
                        {
                            Assets = D.NewElement("asset");
                            Root->InsertEndChild(Assets);
                        }
                        XMLElement *Mesh = D.NewElement("mesh");
                        Mesh->SetAttribute("name", Asset.c_str());
                        std::ostringstream Points;
                        Points << std::setprecision(17);
                        for (const Vector3 &P : G.Vertices)
                        {
                            Points << P.X << ' ' << P.Y << ' ' << P.Z << ' ';
                        }
                        Mesh->SetAttribute("vertex", Points.str().c_str());
                        if (!G.Indices.empty())
                        {
                            std::ostringstream Faces;
                            for (int I : G.Indices)
                            {
                                Faces << I << ' ';
                            }
                            Mesh->SetAttribute("face", Faces.str().c_str());
                        }
                        Assets->InsertEndChild(Mesh);
                    }
                    Require(!Asset.empty(), "Mesh or heightfield has no resource");
                    E->SetAttribute(G.Type == GeometryType::Mesh ? "mesh" : "hfield", Asset.c_str());
                }
                Bodies.at(G.BodyId)->InsertEndChild(E);
            }
            if (!V.Drives.empty())
            {
                XMLElement *A = Root->FirstChildElement("actuator");
                if (!A)
                {
                    A = D.NewElement("actuator");
                    Root->InsertEndChild(A);
                }
                for (const Drive &C : V.Drives)
                {
                    Require(JointNames.count(C.JointId) != 0, "Drive references unknown joint");
                    XMLElement *E = D.NewElement(C.Type == DriveType::Effort     ? "motor"
                                                 : C.Type == DriveType::Position ? "position"
                                                                                 : "velocity");
                    E->SetAttribute("name", C.Id.c_str());
                    E->SetAttribute("joint", JointNames.at(C.JointId).c_str());
                    if (C.Type == DriveType::Position)
                    {
                        E->SetAttribute("kp", C.Stiffness);
                        E->SetAttribute("kv", C.Damping);
                    }
                    else if (C.Type == DriveType::Velocity)
                    {
                        E->SetAttribute("kv", C.Damping);
                    }
                    A->InsertEndChild(E);
                }
            }
            CheckUnsupported(&D);
            Out = Text(Root);
        });
}
} // namespace UBot::MuJoCo::Native

namespace UBot::MuJoCo::Native
{
namespace
{
std::string Key(std::uint64_t Id, const std::string &Element)
{
    return std::to_string(Id) + ":" + Element;
}
void PrefixFiles(XMLElement *Node, const std::string &P, const std::string &MeshDir, const std::string &TextureDir)
{
    if (const char *File = Node->Attribute("file"))
    {
        std::string Path = File;
        const std::string T = Node->Name();
        const std::string &Dir = T == "texture" ? TextureDir : MeshDir;
        if (!Dir.empty() && Path.rfind(Dir, 0) != 0)
        {
            Path = Dir + "/" + Path;
        }
        std::replace(Path.begin(), Path.end(), '\\', '/');
        Node->SetAttribute("file", (P + Path).c_str());
    }
    for (XMLElement *C = Node->FirstChildElement(); C; C = C->NextSiblingElement())
    {
        PrefixFiles(C, P, MeshDir, TextureDir);
    }
}
std::string OptionMeaning(const std::string &XML)
{
    if (XML.empty())
    {
        return {};
    }
    XMLDocument D;
    Require(D.Parse(XML.c_str()) == XML_SUCCESS && D.RootElement(), "Invalid solver options");
    Remove(D.RootElement(), {"timestep", "gravity"});
    if (!D.RootElement()->FirstAttribute() && !D.RootElement()->FirstChildElement())
    {
        return {};
    }
    return Text(D.RootElement());
}
void ApplyInitial(mjModel *M, mjData *D, const std::vector<Instance> &Instances)
{
    for (const Instance &I : Instances)
    {
        const std::string P = Prefix(I.Id);
        for (const Joint &J : I.Value.Joints)
        {
            const int Id = mj_name2id(M, mjOBJ_JOINT, (P + J.Name).c_str());
            Require(Id >= 0, "Missing compiled joint: " + J.Name);
            const int Q = NQ(M->jnt_type[Id]), V = NV(M->jnt_type[Id]);
            Require(J.InitialPosition.empty() || J.InitialPosition.size() == static_cast<std::size_t>(Q),
                    "Invalid initial qpos dimension");
            Require(J.InitialVelocity.empty() || J.InitialVelocity.size() == static_cast<std::size_t>(V),
                    "Invalid initial qvel dimension");
            if (!J.InitialPosition.empty())
            {
                if (J.Type == JointType::Free)
                {
                    // qpos for a free root is a world transform; apply the model instance placement.
                    mjtNum LocalP[3]{J.InitialPosition[0], J.InitialPosition[1], J.InitialPosition[2]};
                    mjtNum LocalQ[4]{J.InitialPosition[3], J.InitialPosition[4], J.InitialPosition[5],
                                     J.InitialPosition[6]};
                    mjtNum PP[3]{I.Placement.Position.X, I.Placement.Position.Y, I.Placement.Position.Z};
                    mjtNum PQ[4]{I.Placement.Rotation.W, I.Placement.Rotation.X, I.Placement.Rotation.Y,
                                 I.Placement.Rotation.Z};
                    mju_mulPose(D->qpos + M->jnt_qposadr[Id], D->qpos + M->jnt_qposadr[Id] + 3, PP, PQ, LocalP, LocalQ);
                }
                else
                {
                    std::copy(J.InitialPosition.begin(), J.InitialPosition.end(), D->qpos + M->jnt_qposadr[Id]);
                }
            }
            if (!J.InitialVelocity.empty())
            {
                std::copy(J.InitialVelocity.begin(), J.InitialVelocity.end(), D->qvel + M->jnt_dofadr[Id]);
                if (J.Type == JointType::Free)
                {
                    const mjtNum PQ[4]{I.Placement.Rotation.W, I.Placement.Rotation.X, I.Placement.Rotation.Y,
                                       I.Placement.Rotation.Z};
                    const mjtNum Linear[3]{J.InitialVelocity[0], J.InitialVelocity[1], J.InitialVelocity[2]};
                    mju_rotVecQuat(D->qvel + M->jnt_dofadr[Id], Linear, PQ);
                }
            }
        }
    }
}
} // namespace
struct Scene::Implementation
{
    SpecPtr Spec{nullptr, mj_deleteSpec};
    ModelPtr M{nullptr, mj_deleteModel};
    DataPtr D{nullptr, mj_deleteData};
    std::vector<Instance> Instances;
    Configuration Config;
    bool Faulted = false, Published = false;
    double PublishedTime = 0;
    std::map<std::string, BodyState> BodyStates;
    std::map<std::string, JointState> JointStates;
    std::map<std::string, ActuatorState> ActuatorStates;
    std::map<std::string, SensorState> SensorStates;
    void Publish()
    {
        std::map<std::string, BodyState> NewBodies;
        std::map<std::string, JointState> NewJoints;
        std::map<std::string, ActuatorState> NewActuators;
        std::map<std::string, SensorState> NewSensors;
        for (const Instance &I : Instances)
        {
            const std::string P = Prefix(I.Id);
            for (const Body &B : I.Value.Bodies)
            {
                BodyState S;
                S.NativeTimeSeconds = D->time;
                if (B.ModelFrame)
                {
                    S.Pose = I.Placement;
                }
                else
                {
                    const int Id = mj_name2id(M.get(), mjOBJ_BODY, (P + B.Name).c_str());
                    Require(Id >= 0, "Missing body output");
                    S.Pose = {Vec(D->xpos + 3 * Id), Quat(D->xquat + 4 * Id)};
                    mjtNum V[6];
                    mj_objectVelocity(M.get(), D.get(), mjOBJ_XBODY, Id, V, 0);
                    S.AngularVelocity = Vec(V);
                    S.LinearVelocity = Vec(V + 3);
                }
                Require(std::isfinite(S.Pose.Position.X) && std::isfinite(S.Pose.Position.Y) &&
                            std::isfinite(S.Pose.Position.Z),
                        "Non-finite body state");
                NewBodies[Key(I.Id, B.Id)] = S;
            }
            for (const Joint &J : I.Value.Joints)
            {
                const int Id = mj_name2id(M.get(), mjOBJ_JOINT, (P + J.Name).c_str());
                Require(Id >= 0, "Missing joint output");
                JointState S;
                S.NativeTimeSeconds = D->time;
                const int Q = M->jnt_qposadr[Id], V = M->jnt_dofadr[Id];
                S.Position.assign(D->qpos + Q, D->qpos + Q + NQ(M->jnt_type[Id]));
                S.Velocity.assign(D->qvel + V, D->qvel + V + NV(M->jnt_type[Id]));
                for (double X : S.Position)
                {
                    Require(std::isfinite(X), "Non-finite joint qpos");
                }
                for (double X : S.Velocity)
                {
                    Require(std::isfinite(X), "Non-finite joint qvel");
                }
                NewJoints[Key(I.Id, J.Id)] = std::move(S);
            }
        }
        for (const Instance &I : Instances)
        {
            const std::string P = Prefix(I.Id);
            for (int A = 0; A < M->nactuator; ++A)
            {
                const char *Raw = mj_id2name(M.get(), mjOBJ_ACTUATOR, A);
                if (Raw && std::string(Raw).rfind(P, 0) == 0)
                {
                    ActuatorState S;
                    S.NativeTimeSeconds = D->time;
                    if (M->actuator_actadr[A] >= 0)
                    {
                        S.Activation.assign(D->act + M->actuator_actadr[A],
                                            D->act + M->actuator_actadr[A] + M->actuator_actnum[A]);
                    }
                    NewActuators[Key(I.Id, std::string(Raw).substr(P.size()))] = std::move(S);
                }
            }
            for (int A = 0; A < M->nsensor; ++A)
            {
                const char *Raw = mj_id2name(M.get(), mjOBJ_SENSOR, A);
                if (Raw && std::string(Raw).rfind(P, 0) == 0)
                {
                    SensorState S;
                    S.NativeTimeSeconds = D->time;
                    S.Values.assign(D->sensordata + M->sensor_adr[A],
                                    D->sensordata + M->sensor_adr[A] + M->sensor_dim[A]);
                    NewSensors[Key(I.Id, std::string(Raw).substr(P.size()))] = std::move(S);
                }
            }
        }
        BodyStates = std::move(NewBodies);
        JointStates = std::move(NewJoints);
        ActuatorStates = std::move(NewActuators);
        SensorStates = std::move(NewSensors);
        Published = true;
        PublishedTime = D->time;
    }
};
Scene::Scene() : Impl(std::make_unique<Implementation>())
{
}
Scene::~Scene() = default;
bool Scene::SetModels(const std::vector<Instance> &Instances, const Configuration &Config, bool Preserve,
                      std::string &Error)
{
    return Guard(Error,
                 [&]
                 {
                     Require(std::isfinite(Config.StepSeconds) && Config.StepSeconds > 0, "Invalid timestep");
                     const VFS Empty({});
                     SpecPtr Candidate =
                         Parse("<mujoco><compiler angle='radian' alignfree='false'/><worldbody/></mujoco>", Empty);
                     if (!Config.Options.empty())
                     {
                         SpecPtr Options = Parse("<mujoco>" + Config.Options + "<worldbody/></mujoco>", Empty);
                         Candidate->option = Options->option;
                     }
                     Candidate->compiler.usethread = false;
                     Candidate->option.timestep = Config.StepSeconds;
                     Candidate->option.gravity[0] = Config.Gravity.X;
                     Candidate->option.gravity[1] = Config.Gravity.Y;
                     Candidate->option.gravity[2] = Config.Gravity.Z;
                     std::vector<Resource> AllResources;
                     std::set<std::uint64_t> IDs;
                     for (const Instance &I : Instances)
                     {
                         Require(I.Id != 0 && IDs.insert(I.Id).second, "Instance IDs must be unique and nonzero");
                         const Model &V = I.Value;
                         if (!Config.AllowWorldOptionsOverride && !V.NativeData.empty())
                         {
                             Require(std::abs(V.SuggestedStepSeconds - Config.StepSeconds) < 1e-12 &&
                                         std::abs(V.SuggestedGravity.X - Config.Gravity.X) < 1e-12 &&
                                         std::abs(V.SuggestedGravity.Y - Config.Gravity.Y) < 1e-12 &&
                                         std::abs(V.SuggestedGravity.Z - Config.Gravity.Z) < 1e-12 &&
                                         OptionMeaning(V.SuggestedOptions) == OptionMeaning(Config.Options),
                                     "Imported global solver options conflict with the world configuration");
                         }
                         std::string XML, Why;
                         Require(BuildXML(V, XML, Why), Why);
                         XMLDocument D;
                         Require(D.Parse(XML.c_str()) == XML_SUCCESS, "Invalid model XML");
                         const std::string P = Prefix(I.Id);
                         XMLElement *C = D.RootElement()->FirstChildElement("compiler");
                         std::string MD = C && C->Attribute("meshdir") ? C->Attribute("meshdir") : "";
                         std::string TD = C && C->Attribute("texturedir") ? C->Attribute("texturedir") : "";
                         PrefixFiles(D.RootElement(), P, MD, TD);
                         if (C)
                         {
                             Remove(C, {"meshdir", "texturedir", "assetdir"});
                         }
                         std::vector<Resource> ChildFiles;
                         for (const Resource &R : V.Resources)
                         {
                             Resource F = R;
                             F.Name = P + R.Name;
                             ChildFiles.push_back(F);
                             AllResources.push_back(std::move(F));
                         }
                         VFS Files(ChildFiles);
                         SpecPtr Child = Parse(Text(D.RootElement()), Files);
                         mjsBody *World = mjs_findBody(Candidate.get(), "world");
                         mjsFrame *Frame = mjs_addFrame(World, nullptr);
                         Frame->pos[0] = I.Placement.Position.X;
                         Frame->pos[1] = I.Placement.Position.Y;
                         Frame->pos[2] = I.Placement.Position.Z;
                         Frame->quat[0] = I.Placement.Rotation.W;
                         Frame->quat[1] = I.Placement.Rotation.X;
                         Frame->quat[2] = I.Placement.Rotation.Y;
                         Frame->quat[3] = I.Placement.Rotation.Z;
                         Require(mjs_attach(Frame->element, Child->element, P.c_str(), "") != nullptr,
                                 mjs_getError(Candidate.get()));
                     }
                     VFS Files(AllResources);
                     ModelPtr M = Compile(Candidate.get(), Files);
                     DataPtr D(mj_makeData(M.get()), mj_deleteData);
                     Require(D != nullptr, "Could not allocate native state");
                     ApplyInitial(M.get(), D.get(), Instances);
                     if (Preserve && Impl->M && Impl->D)
                     {
                         Require(!Impl->Faulted, "Reset is required after a native fault");
                         // Compiler migration operates on isolated copies. It preserves all state categories the
                         // fixed MuJoCo version knows, including activation and history, by element identity.
                         ModelPtr OldM(mj_copyModel(nullptr, Impl->M.get()), mj_deleteModel);
                         DataPtr OldD(mj_copyData(nullptr, OldM.get(), Impl->D.get()), mj_deleteData);
                         Require(OldM && OldD, "Could not copy native state for migration");
                         Require(mj_recompile(Candidate.get(), &Files.Value, OldM.get(), OldD.get()) == 0,
                                 mjs_getError(Candidate.get()));
                         M = std::move(OldM);
                         D = std::move(OldD);
                         // A new instance's non-default initial state is applied only to its own coordinates.
                         std::vector<Instance> Added;
                         for (const Instance &I : Instances)
                         {
                             bool Exists = false;
                             for (const Instance &O : Impl->Instances)
                             {
                                 Exists |= O.Id == I.Id;
                             }
                             if (!Exists)
                             {
                                 Added.push_back(I);
                             }
                         }
                         ApplyInitial(M.get(), D.get(), Added);
                     }
                     mj_forward(M.get(), D.get());
                     Impl->Spec = std::move(Candidate);
                     Impl->M = std::move(M);
                     Impl->D = std::move(D);
                     Impl->Instances = Instances;
                     Impl->Config = Config;
                     Impl->Faulted = false;
                     // Published snapshots remain available for unchanged instances at the preceding actual time.
                     if (!Preserve)
                     {
                         Impl->Published = false;
                         Impl->BodyStates.clear();
                         Impl->JointStates.clear();
                         Impl->ActuatorStates.clear();
                         Impl->SensorStates.clear();
                         Impl->PublishedTime = Impl->D->time;
                     }
                     else
                     {
                         std::set<std::string> BodyKeys, JointKeys;
                         for (const Instance &I : Instances)
                         {
                             for (const Body &B : I.Value.Bodies)
                             {
                                 BodyKeys.insert(Key(I.Id, B.Id));
                             }
                             for (const Joint &J : I.Value.Joints)
                             {
                                 JointKeys.insert(Key(I.Id, J.Id));
                             }
                         }
                         for (auto P = Impl->BodyStates.begin(); P != Impl->BodyStates.end();)
                         {
                             if (!BodyKeys.count(P->first))
                             {
                                 P = Impl->BodyStates.erase(P);
                             }
                             else
                             {
                                 ++P;
                             }
                         }
                         for (auto P = Impl->JointStates.begin(); P != Impl->JointStates.end();)
                         {
                             if (!JointKeys.count(P->first))
                             {
                                 P = Impl->JointStates.erase(P);
                             }
                             else
                             {
                                 ++P;
                             }
                         }
                         for (auto P = Impl->ActuatorStates.begin(); P != Impl->ActuatorStates.end();)
                         {
                             const auto Colon = P->first.find(':');
                             if (!IDs.count(std::stoull(P->first.substr(0, Colon))))
                             {
                                 P = Impl->ActuatorStates.erase(P);
                             }
                             else
                             {
                                 ++P;
                             }
                         }
                         for (auto P = Impl->SensorStates.begin(); P != Impl->SensorStates.end();)
                         {
                             const auto Colon = P->first.find(':');
                             if (!IDs.count(std::stoull(P->first.substr(0, Colon))))
                             {
                                 P = Impl->SensorStates.erase(P);
                             }
                             else
                             {
                                 ++P;
                             }
                         }
                     }
                 });
}
bool Scene::Step(const Inputs &Input, std::string &Error)
{
    const bool Success = Guard(
        Error,
        [&]
        {
            Require(Impl->M && Impl->D && !Impl->Faulted, "Native scene is not ready; reset is required after a fault");
            mjModel *M = Impl->M.get();
            mjData *D = Impl->D.get();
            auto FindInstance = [&](std::uint64_t Id) -> const Instance &
            {
                for (const Instance &I : Impl->Instances)
                {
                    if (I.Id == Id)
                    {
                        return I;
                    }
                }
                throw NativeError("Unknown input instance");
            };
            auto BodyId = [&](std::uint64_t Id, const std::string &Stable) -> int
            {
                const Instance &I = FindInstance(Id);
                for (const Body &B : I.Value.Bodies)
                {
                    if (B.Id == Stable)
                    {
                        int N = mj_name2id(M, mjOBJ_BODY, (Prefix(Id) + B.Name).c_str());
                        Require(N >= 0, "Unknown native body");
                        return N;
                    }
                }
                throw NativeError("Unknown body identity");
            };
            auto JointId = [&](std::uint64_t Id, const std::string &Stable) -> int
            {
                const Instance &I = FindInstance(Id);
                for (const Joint &J : I.Value.Joints)
                {
                    if (J.Id == Stable)
                    {
                        int N = mj_name2id(M, mjOBJ_JOINT, (Prefix(Id) + J.Name).c_str());
                        Require(N >= 0, "Unknown native joint");
                        return N;
                    }
                }
                throw NativeError("Unknown joint identity");
            };
            std::fill(D->ctrl, D->ctrl + M->nu, 0);
            std::fill(D->qfrc_applied, D->qfrc_applied + M->nv, 0);
            std::fill(D->xfrc_applied, D->xfrc_applied + 6 * M->nbody, 0);
            for (const ActuatorInput &A : Input.Actuators)
            {
                FindInstance(A.InstanceId);
                int Id = mj_name2id(M, mjOBJ_ACTUATOR, (Prefix(A.InstanceId) + A.Name).c_str());
                Require(Id >= 0, "Unknown actuator: " + A.Name);
                Require(A.Values.size() == static_cast<std::size_t>(M->actuator_ctrlnum[Id]),
                        "Actuator input dimension mismatch");
                for (std::size_t K = 0; K < A.Values.size(); ++K)
                {
                    Require(std::isfinite(A.Values[K]), "Non-finite actuator input");
                    const int Address = M->actuator_ctrladr[Id] + static_cast<int>(K);
                    if (M->actuator_ctrllimited[Address])
                    {
                        Require(A.Values[K] >= M->actuator_ctrlrange[2 * Address] &&
                                    A.Values[K] <= M->actuator_ctrlrange[2 * Address + 1],
                                "Actuator input outside authored range");
                    }
                    D->ctrl[M->actuator_ctrladr[Id] + K] = A.Values[K];
                }
            }
            for (const JointForce &F : Input.JointForces)
            {
                int J = JointId(F.InstanceId, F.JointId);
                Require(F.Values.size() == static_cast<std::size_t>(NV(M->jnt_type[J])),
                        "Joint force dimension mismatch");
                for (std::size_t K = 0; K < F.Values.size(); ++K)
                {
                    Require(std::isfinite(F.Values[K]), "Non-finite joint force");
                    D->qfrc_applied[M->jnt_dofadr[J] + K] += F.Values[K];
                }
            }
            for (const BodyWrench &F : Input.Wrenches)
            {
                int B = BodyId(F.InstanceId, F.BodyId);
                const mjtNum *Origin = D->xpos + 3 * B;
                const mjtNum *COM = D->xipos + 3 * B;
                const Vector3 R{Origin[0] - COM[0], Origin[1] - COM[1], Origin[2] - COM[2]};
                const double Values[6]{F.Force.X,
                                       F.Force.Y,
                                       F.Force.Z,
                                       F.Torque.X + R.Y * F.Force.Z - R.Z * F.Force.Y,
                                       F.Torque.Y + R.Z * F.Force.X - R.X * F.Force.Z,
                                       F.Torque.Z + R.X * F.Force.Y - R.Y * F.Force.X};
                for (int K = 0; K < 6; ++K)
                {
                    Require(std::isfinite(Values[K]), "Non-finite body wrench");
                    D->xfrc_applied[6 * B + K] += Values[K];
                }
            }
            for (const KinematicTarget &T : Input.Kinematics)
            {
                int B = BodyId(T.InstanceId, T.BodyId);
                int A = M->body_mocapid[B];
                Require(A >= 0, "Kinematic input requires a mocap body");
                D->mocap_pos[3 * A] = T.Pose.Position.X;
                D->mocap_pos[3 * A + 1] = T.Pose.Position.Y;
                D->mocap_pos[3 * A + 2] = T.Pose.Position.Z;
                D->mocap_quat[4 * A] = T.Pose.Rotation.W;
                D->mocap_quat[4 * A + 1] = T.Pose.Rotation.X;
                D->mocap_quat[4 * A + 2] = T.Pose.Rotation.Y;
                D->mocap_quat[4 * A + 3] = T.Pose.Rotation.Z;
            }
            const double Before = D->time;
            int Bad[3]{D->warning[mjWARN_BADQPOS].number, D->warning[mjWARN_BADQVEL].number,
                       D->warning[mjWARN_BADQACC].number};
            mj_step(M, D);
            Require(std::abs(D->time - (Before + M->opt.timestep)) < 1e-10, "Native step reset or time discontinuity");
            Require(D->warning[mjWARN_BADQPOS].number == Bad[0] && D->warning[mjWARN_BADQVEL].number == Bad[1] &&
                        D->warning[mjWARN_BADQACC].number == Bad[2],
                    "Native numerical instability or automatic reset");
            mj_forward(M, D);
            Impl->Publish();
        });
    if (!Success)
    {
        Impl->Faulted = true;
    }
    return Success;
}
bool Scene::Reset(const std::string &Keyframe, std::string &Error)
{
    return Guard(
        Error,
        [&]
        {
            Require(Impl->M && Impl->D, "No native model to reset");
            if (Keyframe.empty())
            {
                mj_resetData(Impl->M.get(), Impl->D.get());
                ApplyInitial(Impl->M.get(), Impl->D.get(), Impl->Instances);
            }
            else
            {
                Require(Impl->Instances.size() == 1, "Named keyframe reset requires exactly one instance");
                const int K = mj_name2id(Impl->M.get(), mjOBJ_KEY, (Prefix(Impl->Instances[0].Id) + Keyframe).c_str());
                Require(K >= 0, "Unknown keyframe");
                mj_resetDataKeyframe(Impl->M.get(), Impl->D.get(), K);
                const Instance &I = Impl->Instances[0];
                XMLDocument Extra;
                Require(Extra.Parse(I.Value.NativeData.c_str()) == XML_SUCCESS, "Invalid native keyframe metadata");
                XMLElement *Root = Extra.RootElement() ? Extra.RootElement()->FirstChildElement("root") : nullptr;
                XMLElement *Keys = Root ? Root->FirstChildElement("keyframe") : nullptr;
                XMLElement *Authored = nullptr;
                if (Keys)
                {
                    for (XMLElement *E = Keys->FirstChildElement("key"); E; E = E->NextSiblingElement("key"))
                    {
                        if (E->Attribute("name") && Keyframe == E->Attribute("name"))
                        {
                            Authored = E;
                            break;
                        }
                    }
                }
                Require(Authored != nullptr, "Keyframe is absent from the model extension");
                const mjtNum PP[3]{I.Placement.Position.X, I.Placement.Position.Y, I.Placement.Position.Z};
                const mjtNum PQ[4]{I.Placement.Rotation.W, I.Placement.Rotation.X, I.Placement.Rotation.Y,
                                   I.Placement.Rotation.Z};
                for (const Joint &J : I.Value.Joints)
                {
                    if (J.Type != JointType::Free)
                    {
                        continue;
                    }
                    const int ID = mj_name2id(Impl->M.get(), mjOBJ_JOINT, (Prefix(I.Id) + J.Name).c_str());
                    mjtNum *Q = Impl->D->qpos + Impl->M->jnt_qposadr[ID];
                    mjtNum *V = Impl->D->qvel + Impl->M->jnt_dofadr[ID];
                    if (Authored->Attribute("qpos"))
                    {
                        const mjtNum RawP[3]{Q[0], Q[1], Q[2]}, RawQ[4]{Q[3], Q[4], Q[5], Q[6]};
                        mju_mulPose(Q, Q + 3, PP, PQ, RawP, RawQ);
                    }
                    const mjtNum RawV[3]{V[0], V[1], V[2]};
                    mju_rotVecQuat(V, RawV, PQ);
                }
                for (int A = 0; A < Impl->M->nmocap; ++A)
                {
                    mjtNum *P = Impl->D->mocap_pos + 3 * A;
                    mjtNum *Q = Impl->D->mocap_quat + 4 * A;
                    if (Authored->Attribute("mpos"))
                    {
                        const mjtNum Raw[3]{P[0], P[1], P[2]};
                        mju_rotVecQuat(P, Raw, PQ);
                        for (int Axis = 0; Axis < 3; ++Axis)
                        {
                            P[Axis] += PP[Axis];
                        }
                    }
                    if (Authored->Attribute("mquat"))
                    {
                        const mjtNum Raw[4]{Q[0], Q[1], Q[2], Q[3]};
                        mju_mulQuat(Q, PQ, Raw);
                    }
                }
            }
            mj_forward(Impl->M.get(), Impl->D.get());
            Impl->Faulted = false;
            Impl->Published = false;
            Impl->PublishedTime = Impl->D->time;
            Impl->BodyStates.clear();
            Impl->JointStates.clear();
            Impl->ActuatorStates.clear();
            Impl->SensorStates.clear();
        });
}
bool Scene::GetBodyState(std::uint64_t I, const std::string &B, BodyState &S) const
{
    auto P = Impl->BodyStates.find(Key(I, B));
    if (P == Impl->BodyStates.end())
    {
        return false;
    }
    S = P->second;
    return true;
}
bool Scene::GetJointState(std::uint64_t I, const std::string &J, JointState &S) const
{
    auto P = Impl->JointStates.find(Key(I, J));
    if (P == Impl->JointStates.end())
    {
        return false;
    }
    S = P->second;
    return true;
}
bool Scene::GetActuatorState(std::uint64_t I, const std::string &N, ActuatorState &S) const
{
    try
    {
        auto P = Impl->ActuatorStates.find(Key(I, N));
        if (P == Impl->ActuatorStates.end())
        {
            return false;
        }
        S = P->second;
        return true;
    }
    catch (...)
    {
        return false;
    }
}
bool Scene::GetSensorState(std::uint64_t I, const std::string &N, SensorState &S) const
{
    try
    {
        auto P = Impl->SensorStates.find(Key(I, N));
        if (P == Impl->SensorStates.end())
        {
            return false;
        }
        S = P->second;
        return true;
    }
    catch (...)
    {
        return false;
    }
}
std::vector<ActuatorInfo> Scene::GetActuators(std::uint64_t InstanceId) const
{
    std::vector<ActuatorInfo> Result;
    if (!Impl->M)
    {
        return Result;
    }
    const std::string P = Prefix(InstanceId);
    for (int A = 0; A < Impl->M->nactuator; ++A)
    {
        const char *N = mj_id2name(Impl->M.get(), mjOBJ_ACTUATOR, A);
        if (N && std::string(N).rfind(P, 0) == 0)
        {
            ActuatorInfo I;
            I.Name = std::string(N).substr(P.size());
            I.InputCount = Impl->M->actuator_ctrlnum[A];
            for (int K = 0; K < I.InputCount; ++K)
            {
                const int Address = Impl->M->actuator_ctrladr[A] + K;
                I.InputLimited.push_back(Impl->M->actuator_ctrllimited[Address]);
                I.InputMinimum.push_back(Impl->M->actuator_ctrlrange[2 * Address]);
                I.InputMaximum.push_back(Impl->M->actuator_ctrlrange[2 * Address + 1]);
            }
            Result.push_back(I);
        }
    }
    return Result;
}
double Scene::GetTimeSeconds() const
{
    return Impl->Faulted ? Impl->PublishedTime : Impl->D ? Impl->D->time : 0;
}
bool Scene::IsFaulted() const
{
    return Impl->Faulted;
}
} // namespace UBot::MuJoCo::Native
