#include "UBotMuJoCoNative.h"
#include <mujoco/mujoco.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <filesystem>
using namespace UBot::MuJoCo::Native;
void Check(bool V,const std::string& E){if(!V){throw std::runtime_error(E);}}
#define Check(V,E) do { if (!(V)) { throw std::runtime_error(E); } } while(false)
Resource Read(const std::filesystem::path& P){std::ifstream F(P,std::ios::binary);Check(F.good(),"fixture missing");Resource R;R.Name=P.filename().string();R.Bytes.assign(std::istreambuf_iterator<char>(F),{});return R;}
void Reference(const std::filesystem::path& Path,const std::string& Motor)
{
    Resource R=Read(Path);ImportResult I;std::string Error;
    Check(Import(R.Name,{R},I,Error),"Import: "+Error);
    Check(I.Value.NativeData.find("fromto=")==std::string::npos,"shared pose retained in extension");
    Scene S;Configuration C;C.StepSeconds=I.Value.SuggestedStepSeconds;C.Gravity=I.Value.SuggestedGravity;C.Options=I.Value.SuggestedOptions;
    Check(S.SetModels({Instance{1,I.Value,{}}},C,false,Error),"SetModels: "+Error);
    char E[2048]{};mjModel* M=mj_loadXML(Path.string().c_str(),nullptr,E,sizeof(E));Check(M!=nullptr,E);mjData* D=mj_makeData(M);
    Inputs Input;if(!Motor.empty()){Input.Actuators.push_back({1,Motor,{0.37}});D->ctrl[0]=0.37;}
    // Same independent native input and integrator; compare every generalized coordinate after each step.
    for(int Step=0;Step<100;++Step)
    {
        mj_step(M,D);mj_forward(M,D);Check(S.Step(Input,Error),"Step: "+Error);
        for(const Joint& J:I.Value.Joints)
        {
            JointState V;Check(S.GetJointState(1,J.Id,V),"Missing joint output");int N=mj_name2id(M,mjOBJ_JOINT,J.Name.c_str());
            for(std::size_t K=0;K<V.Position.size();++K){Check(std::abs(V.Position[K]-D->qpos[M->jnt_qposadr[N]+K])<1e-8,"qpos differs for "+J.Name);}
            for(std::size_t K=0;K<V.Velocity.size();++K){Check(std::abs(V.Velocity[K]-D->qvel[M->jnt_dofadr[N]+K])<1e-8,"qvel differs for "+J.Name);}
        }
    }
    const double Before=S.GetTimeSeconds();auto Models=std::vector<Instance>{{1,I.Value,{}},{2,I.Value,{{3,0,0},{}}}};
    Check(S.SetModels(Models,C,true,Error),"Preserve adding instance: "+Error);Check(std::abs(S.GetTimeSeconds()-Before)<1e-12,"migration changed time");
    Check(S.Step(Input,Error),"post migration step: "+Error);mj_step(M,D);mj_forward(M,D);
    for(const Joint& J:I.Value.Joints){JointState V;S.GetJointState(1,J.Id,V);int N=mj_name2id(M,mjOBJ_JOINT,J.Name.c_str());for(std::size_t K=0;K<V.Position.size();++K){Check(std::abs(V.Position[K]-D->qpos[M->jnt_qposadr[N]+K])<1e-8,"migration lost native dynamic state");}}
    // An invalid candidate must leave the current scene available and time unchanged.
    const double GoodTime=S.GetTimeSeconds();Models[1].Value.Bodies[1].Mass=-1;Check(!S.SetModels(Models,C,true,Error),"bad candidate accepted");Check(S.GetTimeSeconds()==GoodTime,"failed compilation altered scene time");
    Check(S.Reset("",Error),"Reset: "+Error);Check(S.GetTimeSeconds()==0,"reset time");
    mj_deleteData(D);mj_deleteModel(M);std::cout<<Path.filename().string()<<": reference, migration, rollback, reset passed\n";
}
int main(int argc,char** argv)
{
    try
    {
        Check(argc==2,"fixture path required");Check(Version()==4000000,"SDK mismatch");std::filesystem::path Root=argv[1];
        Reference(Root/"pendulum.xml","motor");Reference(Root/"coupled.xml","filtered");Reference(Root/"free_ball.xml","");
        ImportResult I;std::string Error;Check(!Import("missing.xml",{},I,Error),"Missing input accepted");
        const std::string Unsupported="<mujoco><worldbody/><deformable><flex name='unsupported'/></deformable></mujoco>";Resource R{"unsupported.xml",{Unsupported.begin(),Unsupported.end()}};Check(!Import(R.Name,{R},I,Error),"Unsupported model accepted");
        std::cout<<"All native tests passed\n";return 0;
    }
    catch(const std::exception& E){std::cerr<<E.what()<<'\n';return 1;}
}
