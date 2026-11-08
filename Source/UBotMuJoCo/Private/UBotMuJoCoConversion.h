#pragma once

#include "UBotMuJoCoNative.h"
#include "UBotPhysicsModel.h"

class UUBotMuJoCoSolverPreset;

namespace UBot::MuJoCo::Conversion
{
namespace N = UBot::MuJoCo::Native;
N::Vector3 Polar(const FVector &UEValue);
N::Vector3 Axial(const FVector &UEValue);
FVector Polar(const N::Vector3 &NativeValue);
FVector Axial(const N::Vector3 &NativeValue);
N::Transform Pose(const FTransform &UEMeters);
FTransform Pose(const N::Transform &NativeMeters);
bool ToNative(const FUBotPhysicsModel &Model, N::Model &OutModel, FText &OutError);
bool FromNative(const N::ImportResult &Import, UObject *Outer, UUBotMuJoCoSolverPreset *Preset,
                FUBotPhysicsModel &OutModel, FText &OutError);
} // namespace UBot::MuJoCo::Conversion
