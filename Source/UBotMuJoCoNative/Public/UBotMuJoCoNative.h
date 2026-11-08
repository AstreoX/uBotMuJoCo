#pragma once

#if !defined(UBOT_MUJOCO_STANDALONE)
#include "CoreTypes.h"
#endif

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifndef UBOTMUJOCONATIVE_API
#define UBOTMUJOCONATIVE_API
#endif

/** Plain numerical boundary. All vectors use right-handed Z-up axes and SI units.
 * Quaternions are XYZW, transforms are local-to-parent. Names are case-sensitive.
 * No function throws across this boundary. A Scene is confined to its calling thread.
 */
namespace UBot::MuJoCo::Native
{
struct Vector3
{
    double X = 0, Y = 0, Z = 0;
};
struct Quaternion
{
    double X = 0, Y = 0, Z = 0, W = 1;
};
struct Transform
{
    Vector3 Position;
    Quaternion Rotation;
};
enum class JointType
{
    Free,
    Ball,
    Slide,
    Hinge
};
enum class GeometryType
{
    Plane,
    Sphere,
    Capsule,
    Ellipsoid,
    Cylinder,
    Box,
    Mesh,
    HeightField
};
struct Resource
{
    std::string Name;
    std::vector<std::uint8_t> Bytes;
};
struct Body
{
    std::string Id, Name, ParentId;
    Transform Pose;
    double Mass = 0;
    Vector3 CenterOfMass, DiagonalInertia;
    Quaternion InertiaOrientation;
    bool Kinematic = false;
};
struct Joint
{
    std::string Id, Name, BodyId;
    JointType Type = JointType::Hinge;
    Vector3 Position, Axis{0, 0, 1};
    bool Limited = false;
    double LowerLimit = 0, UpperLimit = 0;
    /** Native generalized coordinates: ball WXYZ; free XYZ+WXYZ. Scalar qpos includes authored ref. */
    std::vector<double> InitialPosition, InitialVelocity;
};
struct Geometry
{
    std::string Id, Name, BodyId;
    GeometryType Type = GeometryType::Box;
    Transform Pose;
    Vector3 Size;
    std::string AssetName;
    bool Collision = true;
    double Color[4]{0.7, 0.7, 0.7, 1};
    std::vector<Vector3> Vertices;
    std::vector<int> Indices;
    /** Native sliding, torsional and rolling coefficients. Capsule/cylinder Size.Z is half length. */
    double Friction[3]{1, 0.005, 0.0001};
};
enum class DriveType
{
    Effort,
    Position,
    Velocity
};
struct Drive
{
    std::string Id, JointId;
    DriveType Type = DriveType::Effort;
    double Stiffness = 0, Damping = 0;
};
struct Model
{
    std::vector<Body> Bodies;
    std::vector<Joint> Joints;
    std::vector<Geometry> Geometries;
    std::vector<Drive> Drives;
    std::vector<Resource> Resources;
    /** Native-only XML sections/attributes referencing stable IDs. Contains no common body,
     * joint or geometry definitions. The source document is not an executable overlay. */
    std::string NativeData;
    /** Imported solver options. Select as world preset or explicitly allow a world override. */
    double SuggestedStepSeconds = 0.002;
    Vector3 SuggestedGravity{0, 0, -9.81};
    std::string SuggestedOptions;
};
struct ImportResult
{
    Model Value;
    std::vector<std::string> Messages;
};
/** Parse a UTF-8 filename from Resource entries. Missing resources and unsupported physics fail. */
UBOTMUJOCONATIVE_API bool Import(const std::string &Filename, const std::vector<Resource> &Resources,
                                 ImportResult &OutResult, std::string &OutError);
/** Serialize/reconstruct solely from common values and native-only data. Useful for diagnostics. */
UBOTMUJOCONATIVE_API bool BuildXML(const Model &Value, std::string &OutXML, std::string &OutError);
UBOTMUJOCONATIVE_API int Version();

struct Configuration
{
    double StepSeconds = 0.002;
    Vector3 Gravity{0, 0, -9.81};
    std::string Options;
    bool AllowWorldOptionsOverride = false;
};
struct Instance
{
    std::uint64_t Id = 0;
    Model Value;
    Transform Placement;
};
struct ActuatorInput
{
    std::uint64_t InstanceId = 0;
    std::string Name;
    std::vector<double> Values;
};
struct JointForce
{
    std::uint64_t InstanceId = 0;
    std::string JointId;
    std::vector<double> Values;
};
struct BodyWrench
{
    std::uint64_t InstanceId = 0;
    std::string BodyId;
    Vector3 Force, Torque;
};
struct KinematicTarget
{
    std::uint64_t InstanceId = 0;
    std::string BodyId;
    Transform Pose;
};
struct Inputs
{
    std::vector<ActuatorInput> Actuators;
    std::vector<JointForce> JointForces;
    std::vector<BodyWrench> Wrenches;
    std::vector<KinematicTarget> Kinematics;
};
struct BodyState
{
    Transform Pose;
    Vector3 LinearVelocity, AngularVelocity;
    double NativeTimeSeconds = 0;
};
/** Position follows raw native qpos layout (ball WXYZ, free XYZ+WXYZ); velocity follows native qvel. */
struct JointState
{
    std::vector<double> Position, Velocity;
    double NativeTimeSeconds = 0;
};
struct ActuatorState
{
    std::vector<double> Activation;
    double NativeTimeSeconds = 0;
};
struct SensorState
{
    std::vector<double> Values;
    double NativeTimeSeconds = 0;
};
struct ActuatorInfo
{
    std::string Name;
    int InputCount = 0;
    bool Limited = false;
    double Minimum = 0, Maximum = 0;
};

class UBOTMUJOCONATIVE_API Scene
{
  public:
    Scene();
    ~Scene();
    Scene(const Scene &) = delete;
    Scene &operator=(const Scene &) = delete;
    /** Transactional replacement. Preserve=true rejects unsupported migration instead of resetting.
     * No state is published until a successful Step. Instance IDs must be nonzero and unique. */
    bool SetModels(const std::vector<Instance> &Instances, const Configuration &Config, bool Preserve,
                   std::string &OutError);
    /** Complete input frame: unspecified controls are zero (subject to native ranges); forces are cleared.
     * Activation/history persist. A keyframe ctrl is superseded by the next explicit Step frame. */
    bool Step(const Inputs &Input, std::string &OutError);
    /** Reset the entire scene, optionally choosing a named keyframe in a single instance scene. */
    bool Reset(const std::string &Keyframe, std::string &OutError);
    bool GetBodyState(std::uint64_t InstanceId, const std::string &BodyId, BodyState &OutState) const;
    bool GetJointState(std::uint64_t InstanceId, const std::string &JointId, JointState &OutState) const;
    bool GetActuatorState(std::uint64_t InstanceId, const std::string &Name, ActuatorState &OutState) const;
    bool GetSensorState(std::uint64_t InstanceId, const std::string &Name, SensorState &OutState) const;
    std::vector<ActuatorInfo> GetActuators(std::uint64_t InstanceId) const;
    double GetTimeSeconds() const;
    bool IsFaulted() const;

  private:
    struct Implementation;
    std::unique_ptr<Implementation> Impl;
};
} // namespace UBot::MuJoCo::Native
