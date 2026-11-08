#include "UBotMuJoCoNative.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mujoco/mujoco.h>
#include <stdexcept>
using namespace UBot::MuJoCo::Native;
void Check(bool V, const std::string &E)
{
    if (!V)
    {
        throw std::runtime_error(E);
    }
}
#define Check(V, E)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(V))                                                                                                      \
        {                                                                                                              \
            throw std::runtime_error(E);                                                                               \
        }                                                                                                              \
    } while (false)
Resource Read(const std::filesystem::path &P)
{
    std::ifstream F(P, std::ios::binary);
    Check(F.good(), "fixture missing");
    Resource R;
    R.Name = P.filename().string();
    R.Bytes.assign(std::istreambuf_iterator<char>(F), {});
    return R;
}
void Reference(const std::filesystem::path &Path, const std::string &Motor, bool RequireContact = false)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), "Import: " + Error);
    Check(I.Value.NativeData.find("fromto=") == std::string::npos, "shared pose retained in extension");
    const bool OffsetBall = Path.filename() == "free_ball.xml";
    if (OffsetBall)
    {
        for (Joint &J : I.Value.Joints)
        {
            if (J.Type == JointType::Ball)
            {
                J.InitialPosition = {0.9659258262890683, 0.2588190451025207, 0, 0};
                J.InitialVelocity = {0.4, -0.2, 0.1};
            }
            else if (J.Type == JointType::Free)
            {
                J.InitialVelocity = {0.1, 0.2, 0.3, 0.3, 0.2, -0.1};
            }
        }
    }
    Scene S;
    Configuration C;
    C.StepSeconds = I.Value.SuggestedStepSeconds;
    C.Gravity = I.Value.SuggestedGravity;
    C.Options = I.Value.SuggestedOptions;
    Check(S.SetModels({Instance{1, I.Value, {}}}, C, false, Error), "SetModels: " + Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    const auto CheckJointDimensions = [M](int JointId, const JointState &State)
    {
        Check(JointId >= 0 && JointId < M->njnt, "joint is absent from the independent native model");
        const mjtSize PositionEnd = JointId + 1 < M->njnt ? M->jnt_qposadr[JointId + 1] : M->nq;
        const mjtSize VelocityEnd = JointId + 1 < M->njnt ? M->jnt_dofadr[JointId + 1] : M->nv;
        Check(State.Position.size() == static_cast<std::size_t>(PositionEnd - M->jnt_qposadr[JointId]),
              "published qpos dimension differs from the independent native layout");
        Check(State.Velocity.size() == static_cast<std::size_t>(VelocityEnd - M->jnt_dofadr[JointId]),
              "published qvel dimension differs from the independent native layout");
    };
    if (OffsetBall)
    {
        const int Ball = mj_name2id(M, mjOBJ_JOINT, "ball_joint"), Free = mj_name2id(M, mjOBJ_JOINT, "root");
        const double Q[4]{0.9659258262890683, 0.2588190451025207, 0, 0}, BallV[3]{0.4, -0.2, 0.1},
            FreeV[6]{0.1, 0.2, 0.3, 0.3, 0.2, -0.1};
        std::copy(Q, Q + 4, D->qpos + M->jnt_qposadr[Ball]);
        std::copy(BallV, BallV + 3, D->qvel + M->jnt_dofadr[Ball]);
        std::copy(FreeV, FreeV + 6, D->qvel + M->jnt_dofadr[Free]);
    }
    Inputs Input;
    bool SawContact = false;
    if (!Motor.empty())
    {
        Input.Actuators.push_back({1, Motor, {0.37}});
        D->ctrl[0] = 0.37;
    }
    // Same independent native input and integrator; compare every generalized coordinate after each step.
    for (int Step = 0; Step < 100; ++Step)
    {
        if (!Motor.empty())
        {
            const double Control = 0.37 * std::sin(Step * 0.1);
            Input.Actuators[0].Values[0] = Control;
            D->ctrl[0] = Control;
        }
        mj_step(M, D);
        mj_forward(M, D);
        SawContact |= D->ncon > 0;
        Check(S.Step(Input, Error), "Step: " + Error);
        for (const Joint &J : I.Value.Joints)
        {
            JointState V;
            Check(S.GetJointState(1, J.Id, V), "Missing joint output");
            int N = mj_name2id(M, mjOBJ_JOINT, J.Name.c_str());
            CheckJointDimensions(N, V);
            for (std::size_t K = 0; K < V.Position.size(); ++K)
            {
                Check(std::abs(V.Position[K] - D->qpos[M->jnt_qposadr[N] + K]) < 1e-8,
                      "qpos differs for " + J.Name + " K=" + std::to_string(K) +
                          " actual=" + std::to_string(V.Position[K]) +
                          " expected=" + std::to_string(D->qpos[M->jnt_qposadr[N] + K]));
            }
            for (std::size_t K = 0; K < V.Velocity.size(); ++K)
            {
                Check(std::abs(V.Velocity[K] - D->qvel[M->jnt_dofadr[N] + K]) < 1e-8, "qvel differs for " + J.Name);
            }
        }
    }
    Check(!RequireContact || SawContact, "contact fixture never produced a native contact before migration");
    const double Before = S.GetTimeSeconds();
    auto Models = std::vector<Instance>{{1, I.Value, {}}, {2, I.Value, {{3, 0, 0}, {}}}};
    Check(S.SetModels(Models, C, true, Error), "Preserve adding instance: " + Error);
    Check(std::abs(S.GetTimeSeconds() - Before) < 1e-12, "migration changed time");
    bool SawMigratedContact = false;
    for (int Step = 100; Step < 120; ++Step)
    {
        if (!Motor.empty())
        {
            const double Control = 0.37 * std::sin(Step * 0.1);
            Input.Actuators[0].Values[0] = Control;
            D->ctrl[0] = Control;
        }
        Check(S.Step(Input, Error), "post migration step: " + Error);
        mj_step(M, D);
        mj_forward(M, D);
        SawMigratedContact |= D->ncon > 0;
        for (const Joint &J : I.Value.Joints)
        {
            JointState V;
            Check(S.GetJointState(1, J.Id, V), "missing joint after migration");
            int N = mj_name2id(M, mjOBJ_JOINT, J.Name.c_str());
            CheckJointDimensions(N, V);
            for (std::size_t K = 0; K < V.Position.size(); ++K)
            {
                Check(std::abs(V.Position[K] - D->qpos[M->jnt_qposadr[N] + K]) < 1e-8,
                      "migration lost native dynamic state");
            }
            for (std::size_t K = 0; K < V.Velocity.size(); ++K)
            {
                Check(std::abs(V.Velocity[K] - D->qvel[M->jnt_dofadr[N] + K]) < 1e-8,
                      "migration changed native velocity");
            }
        }
        if (M->na)
        {
            ActuatorState A;
            Check(S.GetActuatorState(1, Motor, A), "missing actuator state");
            for (std::size_t K = 0; K < A.Activation.size(); ++K)
            {
                Check(std::abs(A.Activation[K] - D->act[K]) < 1e-10, "migration lost activation/history");
            }
        }
        for (int A = 0; A < M->nsensor; ++A)
        {
            SensorState V;
            Check(S.GetSensorState(1, mj_id2name(M, mjOBJ_SENSOR, A), V), "missing native sensor");
            for (std::size_t K = 0; K < V.Values.size(); ++K)
            {
                Check(std::abs(V.Values[K] - D->sensordata[M->sensor_adr[A] + K]) < 1e-8, "native sensor mismatch");
            }
        }
    }
    Check(!RequireContact || SawMigratedContact, "contact fixture lost all native contact after migration");
    // An invalid candidate must leave the current scene available and time unchanged.
    const double GoodTime = S.GetTimeSeconds();
    Models[1].Value.Bodies[1].Mass = -1;
    Check(!S.SetModels(Models, C, true, Error), "bad candidate accepted");
    Check(S.GetTimeSeconds() == GoodTime, "failed compilation altered scene time");
    Check(S.SetModels({{1, I.Value, {}}}, C, true, Error), Error);
    BodyState GoneBody;
    JointState GoneJoint;
    Check(!S.GetBodyState(2, I.Value.Bodies.back().Id, GoneBody) &&
              !S.GetJointState(2, I.Value.Joints.back().Id, GoneJoint),
          "removed instance leaked published state");
    Check(S.SetModels({{1, I.Value, {}}, {2, I.Value, {{3, 0, 0}, {}}}}, C, true, Error), Error);
    Check(!S.GetBodyState(2, I.Value.Bodies.back().Id, GoneBody) &&
              !S.GetJointState(2, I.Value.Joints.back().Id, GoneJoint),
          "reused instance inherited old publication");
    Check(S.Reset("", Error), "Reset: " + Error);
    Check(S.GetTimeSeconds() == 0, "reset time");
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << Path.filename().string() << ": reference, migration, rollback, reset passed\n";
}

void FramesAndForces(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    I.Value.Joints[0].InitialVelocity = {1, 2, 3, 0.3, 0.4, 0.5};
    const double Half = std::sqrt(0.5);
    Transform Placement{{3, 4, 0}, {0, 0, Half, Half}};
    Scene S;
    Configuration C;
    C.Gravity = {0, 0, 0};
    Check(S.SetModels({{7, I.Value, Placement}}, C, false, Error), Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    // Analytic 90-degree yaw: x/y position (0,0)->(3,4), world linear velocity (1,2,3)->(-2,1,3).
    D->qpos[0] = 3;
    D->qpos[1] = 4;
    D->qpos[2] = 1;
    D->qpos[3] = Half;
    D->qpos[4] = 0;
    D->qpos[5] = 0;
    D->qpos[6] = Half;
    const double Velocity[6]{-2, 1, 3, 0.3, 0.4, 0.5};
    std::copy(Velocity, Velocity + 6, D->qvel);
    mj_forward(M, D);
    const int B = mj_name2id(M, mjOBJ_BODY, "offset");
    const mjtNum Force[3]{2, 3, 4}, Torque[3]{0.1, 0.2, 0.3};
    mj_applyFT(M, D, Force, Torque, D->xpos + 3 * B, B, D->qfrc_applied);
    Inputs Input;
    Input.Wrenches.push_back({7, "offset", {2, 3, 4}, {0.1, 0.2, 0.3}});
    Check(S.Step(Input, Error), Error);
    mj_step(M, D);
    mj_forward(M, D);
    JointState J;
    Check(S.GetJointState(7, "free_joint", J), "missing offset joint");
    for (int K = 0; K < 7; ++K)
    {
        Check(std::abs(J.Position[K] - D->qpos[K]) < 1e-10, "placement/wrench qpos mismatch");
    }
    for (int K = 0; K < 6; ++K)
    {
        Check(std::abs(J.Velocity[K] - D->qvel[K]) < 1e-10, "origin wrench conversion mismatch");
    }
    BodyState Body;
    Check(S.GetBodyState(7, "offset", Body), "missing offset body");
    mjtNum COMVelocity[6];
    mj_objectVelocity(M, D, mjOBJ_BODY, B, COMVelocity, 0);
    const mjtNum *Origin = D->xpos + 3 * B;
    const mjtNum *COM = D->xipos + 3 * B;
    const double Offset[3]{Origin[0] - COM[0], Origin[1] - COM[1], Origin[2] - COM[2]};
    const double Expected[3]{COMVelocity[3] + COMVelocity[1] * Offset[2] - COMVelocity[2] * Offset[1],
                             COMVelocity[4] + COMVelocity[2] * Offset[0] - COMVelocity[0] * Offset[2],
                             COMVelocity[5] + COMVelocity[0] * Offset[1] - COMVelocity[1] * Offset[0]};
    Check(std::abs(Body.LinearVelocity.X - Expected[0]) < 1e-10 &&
              std::abs(Body.LinearVelocity.Y - Expected[1]) < 1e-10 &&
              std::abs(Body.LinearVelocity.Z - Expected[2]) < 1e-10,
          "body-origin velocity mismatch");
    const double Published = S.GetTimeSeconds();
    Input.Actuators.push_back({7, "missing", {1}});
    Check(!S.Step(Input, Error), "invalid input accepted");
    Check(S.IsFaulted() && S.GetTimeSeconds() == Published, "failed step advanced publication");
    BodyState Retained;
    Check(S.GetBodyState(7, "offset", Retained) && Retained.NativeTimeSeconds == Published,
          "failed step lost snapshot");
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << "origin velocity, origin wrench, rotated placement and fault snapshot passed\n";
}
void MeshNormalization(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    std::string XML;
    Check(BuildXML(I.Value, XML, Error), Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjSpec *Spec = mj_parseXMLString(XML.c_str(), nullptr, E, sizeof(E));
    Check(Spec != nullptr, E);
    mjModel *Rebuilt = mj_compile(Spec, nullptr);
    Check(Rebuilt != nullptr, mjs_getError(Spec));
    Check(M->ngeom == Rebuilt->ngeom, "mesh geometry count mismatch");
    for (int G = 0; G < M->ngeom; ++G)
    {
        for (int K = 0; K < 3; ++K)
        {
            Check(std::abs(M->geom_pos[3 * G + K] - Rebuilt->geom_pos[3 * G + K]) < 1e-8,
                  "mesh recentering applied twice");
        }
        for (int K = 0; K < 4; ++K)
        {
            Check(std::abs(M->geom_quat[4 * G + K] - Rebuilt->geom_quat[4 * G + K]) < 1e-8,
                  "mesh orientation applied twice");
        }
    }
    if (Path.filename() == "mesh.xml")
    {
        Check(!I.Value.Geometries[0].Vertices.empty(), "decoded visual mesh missing");
    }
    mj_deleteModel(M);
    mj_deleteModel(Rebuilt);
    mj_deleteSpec(Spec);
    std::cout << "mesh resource normalization and decoded visuals passed\n";
}

void URDF(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    Check(I.Value.Resources.empty(), "original source document leaked into runtime resources");
    bool Reported = false, Converted = false;
    for (const auto &M : I.Messages)
    {
        Reported |= M.find("gazebo") != std::string::npos;
        Converted |= M.find("mimic") != std::string::npos;
    }
    Check(Reported && Converted, "URDF conversion/ignored-extension report missing");
    std::string XML;
    Check(BuildXML(I.Value, XML, Error), Error);
    char E[2048]{};
    mjSpec *S = mj_parseXMLString(XML.c_str(), nullptr, E, sizeof(E));
    Check(S != nullptr, E);
    mjModel *M = mj_compile(S, nullptr);
    Check(M != nullptr, mjs_getError(S));
    Check(M->neq == 1 && M->eq_type[0] == mjEQ_JOINT, "URDF mimic equality missing count=" + std::to_string(M->neq));
    Check(std::string(mj_id2name(M, mjOBJ_JOINT, M->eq_obj1id[0])) == "b" &&
              std::string(mj_id2name(M, mjOBJ_JOINT, M->eq_obj2id[0])) == "a",
          "mimic relation reversed");
    Check(M->eq_data[0] == 0.2 && M->eq_data[1] == -1.5, "mimic polynomial does not match authored affine equation");
    mj_deleteModel(M);
    mj_deleteSpec(S);
    std::cout << "URDF mimic equation and loss report passed\n";
}

void BundledResources()
{
    const std::string XML =
        u8"<mujoco><asset><mesh name='m' file='网格.obj'/></asset><worldbody><body name='object'><freejoint "
        u8"name='root'/><geom name='g' type='mesh' mesh='m'/></body></worldbody></mujoco>";
    const std::string OBJ = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";
    Resource Main{u8"机器人.xml", {XML.begin(), XML.end()}}, Mesh{u8"网格.obj", {OBJ.begin(), OBJ.end()}};
    ImportResult I;
    std::string Error;
    Check(!Import(Main.Name, {Main}, I, Error), "Missing asset dependency accepted");
    Check(Import(Main.Name, {Main, Mesh}, I, Error), Error);
    Check(I.Value.Resources.size() == 1 && I.Value.Resources[0].Name == Mesh.Name, "Source XML or asset bytes lost");
    Scene S;
    Configuration C;
    Check(S.SetModels({{1, I.Value, {}}}, C, false, Error), Error);
    Check(S.Step({}, Error), Error);
    std::cout << "UTF-8 bundled resources and missing dependency handling passed\n";
}

void ClearInputs(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    Scene S;
    Configuration C;
    C.Gravity = I.Value.SuggestedGravity;
    C.Options = I.Value.SuggestedOptions;
    Check(S.SetModels({{1, I.Value, {}}}, C, false, Error), Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    Inputs On;
    On.Actuators.push_back({1, "filtered", {0.5}});
    for (int K = 0; K < 10; ++K)
    {
        D->ctrl[0] = 0.5;
        mj_step(M, D);
        mj_forward(M, D);
        Check(S.Step(On, Error), Error);
    }
    for (int K = 0; K < 10; ++K)
    {
        D->ctrl[0] = 0;
        mj_step(M, D);
        mj_forward(M, D);
        Check(S.Step({}, Error), Error);
        ActuatorState A;
        Check(S.GetActuatorState(1, "filtered", A), "missing activation");
        Check(std::abs(A.Activation[0] - D->act[0]) < 1e-10, "empty frame retained old control or erased activation");
    }
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << "complete input replacement clears controls and preserves activation passed\n";
}

void PlacedKeyframe(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    Scene S;
    Configuration C;
    C.Gravity = {0, 0, 0};
    const double Half = std::sqrt(0.5);
    Transform P{{3, 4, 0}, {0, 0, Half, Half}};
    Check(S.SetModels({{1, I.Value, P}}, C, false, Error), Error);
    Check(S.Reset("placed", Error), Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    mj_resetDataKeyframe(M, D, 0);
    // Rotate authored position (1,2,3) and velocity (1,2,3) 90 degrees, then translate position.
    D->qpos[0] = 1;
    D->qpos[1] = 5;
    D->qpos[2] = 3;
    D->qpos[3] = Half;
    D->qpos[4] = 0;
    D->qpos[5] = 0;
    D->qpos[6] = Half;
    D->qvel[0] = -2;
    D->qvel[1] = 1;
    D->qvel[2] = 3;
    Check(S.Step({}, Error), Error);
    mj_step(M, D);
    mj_forward(M, D);
    JointState J;
    Check(S.GetJointState(1, "free_joint", J), "missing placed keyframe joint");
    for (int K = 0; K < 7; ++K)
    {
        Check(std::abs(J.Position[K] - D->qpos[K]) < 1e-10, "keyframe placement mismatch K=" + std::to_string(K) +
                                                                " native=" + std::to_string(J.Position[K]) +
                                                                " expected=" + std::to_string(D->qpos[K]));
    }
    for (int K = 0; K < 6; ++K)
    {
        Check(std::abs(J.Velocity[K] - D->qvel[K]) < 1e-10, "keyframe velocity placement mismatch");
    }
    Check(std::abs(S.GetTimeSeconds() - 0.502) < 1e-12, "keyframe native time lost");
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << "placed free-root keyframe pose velocity and time passed\n";
}

void MultiInput(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    Configuration C;
    C.Gravity = {0, 0, 0};
    Scene S;
    Check(S.SetModels({{1, I.Value, {}}}, C, false, Error), Error);
    const auto Info = S.GetActuators(1);
    Check(Info.size() == 2 && Info[0].InputCount == 3 && Info[1].InputCount == 1, "multi-input layout wrong");
    Check(Info[0].InputLimited[0] && Info[0].InputLimited[1] && Info[0].InputLimited[2] &&
              Info[0].InputMaximum[0] == 1 && Info[0].InputMaximum[1] == 2 && Info[0].InputMaximum[2] == 3 &&
              Info[1].InputMaximum[0] == 4,
          "per-channel ranges wrong");
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    Inputs Input;
    Input.Actuators = {{1, "multi", {0.2, 1.7, 2.7}}, {1, "following", {3.7}}};
    for (int Step = 0; Step < 20; ++Step)
    {
        D->ctrl[0] = 0.2;
        D->ctrl[1] = 1.7;
        D->ctrl[2] = 2.7;
        D->ctrl[3] = 3.7;
        mj_step(M, D);
        mj_forward(M, D);
        Check(S.Step(Input, Error), Error);
        for (const Joint &J : I.Value.Joints)
        {
            JointState V;
            Check(S.GetJointState(1, J.Id, V), "missing multi-input joint");
            const int N = mj_name2id(M, mjOBJ_JOINT, J.Name.c_str());
            Check(std::abs(V.Position[0] - D->qpos[M->jnt_qposadr[N]]) < 1e-10, "multi-input dynamics differ");
        }
    }
    Input.Actuators[0].Values[1] = 2.1;
    const double Last = S.GetTimeSeconds();
    Check(!S.Step(Input, Error) && S.GetTimeSeconds() == Last, "per-channel invalid input accepted");
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << "multi-input ranges and following actuator native differential passed\n";
}
void UnreservedWorldName()
{
    Model V;
    Body B;
    B.Id = "world";
    B.Name = "body_0";
    B.Mass = 1;
    B.DiagonalInertia = {1, 1, 1};
    V.Bodies.push_back(B);
    Joint J;
    J.Id = "joint";
    J.Name = "joint";
    J.BodyId = "world";
    J.Type = JointType::Free;
    V.Joints.push_back(J);
    Geometry G;
    G.Id = "shape";
    G.Name = "shape";
    G.BodyId = "world";
    G.Type = GeometryType::Sphere;
    G.Size = {0.1, 0, 0};
    V.Geometries.push_back(G);
    Scene S;
    Configuration C;
    std::string Error;
    Check(S.SetModels({{1, V, {}}}, C, false, Error), Error);
    Check(S.Step({}, Error), Error);
    BodyState State;
    Check(S.GetBodyState(1, "world", State) && State.Pose.Position.Z < 0, "generic world name treated as model frame");
    std::cout << "generic world identity is not reserved passed\n";
}

void AlignedKeyframe(const std::filesystem::path &Path)
{
    Resource R = Read(Path);
    ImportResult I;
    std::string Error;
    Check(Import(R.Name, {R}, I, Error), Error);
    Scene S;
    Configuration C;
    Check(S.SetModels({{1, I.Value, {}}}, C, false, Error), Error);
    Check(S.Reset("pose", Error), Error);
    char E[2048]{};
    mjModel *M = mj_loadXML(Path.string().c_str(), nullptr, E, sizeof(E));
    Check(M != nullptr, E);
    mjData *D = mj_makeData(M);
    mj_resetDataKeyframe(M, D, 0);
    for (int Step = 0; Step < 20; ++Step)
    {
        mj_step(M, D);
        mj_forward(M, D);
        Check(S.Step({}, Error), Error);
        JointState J;
        Check(S.GetJointState(1, "aligned_free", J), "missing aligned keyframe joint");
        for (int K = 0; K < 7; ++K)
        {
            Check(std::abs(J.Position[K] - D->qpos[K]) < 1e-10, "aligned keyframe qpos mismatch");
        }
        for (int K = 0; K < 6; ++K)
        {
            Check(std::abs(J.Velocity[K] - D->qvel[K]) < 1e-10, "aligned keyframe qvel mismatch");
        }
        SensorState V;
        Check(S.GetSensorState(1, "site_pos", V), "missing aligned site observation");
        for (int K = 0; K < 3; ++K)
        {
            Check(std::abs(V.Values[K] - D->sensordata[K]) < 1e-10, "aligned keyframe native site mismatch");
        }
    }
    mj_deleteData(D);
    mj_deleteModel(M);
    std::cout << "alignfree source keyframe and site native differential passed\n";
}
int main(int argc, char **argv)
{
    try
    {
        Check(argc == 2, "fixture path required");
        Check(Version() == 4000000, "SDK mismatch");
        std::filesystem::path Root = argv[1];
        Reference(Root / "pendulum.xml", "motor");
        Reference(Root / "coupled.xml", "filtered");
        Reference(Root / "free_ball.xml", "");
        Reference(Root / "contact.xml", "", true);
        FramesAndForces(Root / "offset.xml");
        MeshNormalization(Root / "mesh.xml");
        URDF(Root / "mimic.urdf");
        Reference(Root / "mimic.urdf", "");
        BundledResources();
        ClearInputs(Root / "coupled.xml");
        PlacedKeyframe(Root / "offset.xml");
        MultiInput(Root / "multi_input.xml");
        UnreservedWorldName();
        Reference(Root / "aligned.xml", "");
        Reference(Root / "fused.xml", "");
        MeshNormalization(Root / "aligned.xml");
        MeshNormalization(Root / "fused.xml");
        AlignedKeyframe(Root / "aligned.xml");
        ImportResult I;
        std::string Error;
        Check(!Import("missing.xml", {}, I, Error), "Missing input accepted");
        const std::string Unsupported =
            "<mujoco><worldbody/><deformable><flex name='unsupported'/></deformable></mujoco>";
        Resource R{"unsupported.xml", {Unsupported.begin(), Unsupported.end()}};
        Check(!Import(R.Name, {R}, I, Error), "Unsupported model accepted");
        std::cout << "All native tests passed\n";
        return 0;
    }
    catch (const std::exception &E)
    {
        std::cerr << E.what() << '\n';
        return 1;
    }
}
