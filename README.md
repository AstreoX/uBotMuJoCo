<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot MuJoCo" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBot MuJoCo</h1>
<p align="center"><b>为 Unreal Engine 提供原生 CPU MuJoCo 物理求解。</b></p>

Unreal Engine 5.5 的原生 CPU MuJoCo 求解器，作为 uBot Physics 后端运行。开发版本为 `0.1.0`，依赖 uBot Core 与 uBot Physics `0.2.x`。目前尚未发布。

## 使用方式

1. 启用 `UBotCore`、`UBotPhysics` 和 `UBotMuJoCo`，统一重编译宿主。
2. 在内容浏览器导入 MJCF `.xml` 或 URDF `.urdf`。导入器生成公共 Physics 模型资产和建议的 MuJoCo Solver Preset。
3. 将 Physics Model Actor 放入关卡并选择模型资产，或者在自己的 Actor 上配置 Physics Binding 的模型与刚体映射。后端名称为 `MuJoCo`。
4. 场景可放置一个 MuJoCo World Settings，选择导入的 Solver Preset。没有配置时使用 `0.002 s` 步长和 UE World 重力。重复配置会拒绝创建 solver。
5. 通过 Physics 的 C++ 物理输入接口提交模型定义的 actuator 输入、广义力、外力或运动学目标，并读取刚体／关节状态。机器人控制器和轨迹逻辑由调用方负责。

同一个 World 的全部 MuJoCo 实例进入同一个原生场景，能够相互接触。其余 UE 碰撞体作为静态或运动学几何交付；跨物理后端保持单向运动学作用。需要双向动态接触的对象应交给同一个后端。

## 模型与资源

公共模型是刚体、惯性、几何、关节及基础驱动的唯一数据来源。原生扩展保存 tendon、equality、原生 actuator、接触参数和引用等专属数据；运行时从二者编译，不使用源 MJCF 覆盖公共字段。

导入模型的物理属性通过修改源文件后重新导入更新。运行中的实例保留自己的定义，需移除并重新绑定才使用新版。成功重导入产生新版本，源元素标识保持稳定；失败保留现有资产。

导入器仅收集显式引用的文件，包括相对路径的 include、mesh、texture 和 heightfield。资源随资产打包，运行时通过虚拟文件系统加载。绝对引用和 `package://` URI 应先改为相对文件。重新导入的完整路径只保留在当前编辑器会话；资产仅保存源文件名，重启后使用 **Reimport With New File** 重新定位源文件。

当前接入面向 MuJoCo 的原生刚体／关节模型，保留原生约束、tendon、参考值、keyframe 及内置 actuator 的相关语义。URDF 以固定版本 MuJoCo 解析器的能力为准，不承诺任意 ROS URDF 扩展。flex、skin、外部插件和用户回调会拒绝导入。

MuJoCo 的网格碰撞采用凸包。UE 导出的三角网格默认拒绝；可在 World Settings 显式允许凸包近似。原生接触使用 MuJoCo 的顺应性模型，UE 材质的 restitution 系数不能保证等效。

## 时间、状态与失败

每个 World 的后端按 uBot 仿真时钟固定步进，完成积分后回写组件。位置、速度和外力接口使用 SI 单位与 UE 轴向；SDK 的右手系转换只发生在 MuJoCo 适配边界。关节位置与速度具有独立布局，球关节为 `4/3`，自由关节为 `7/6`；公共四元数为 `XYZW`。

`mjData` 是唯一积分状态；公共状态是带真实积分终点时间戳的只读输出。首版不声明公共刚体加速度、可微性或跨平台确定性。

MuJoCo 使用连续时间模式。超出每帧步数预算后暂停后端并报告 `Overrun`，不会通过跳过原生时间继续运行。需要显式重置整个 World 的 MuJoCo 后端建立新时间起点。模型或场景更新失败、数值故障同样暂停该后端；其他后端可继续运行。

## 构建与验证

插件固定使用 MuJoCo `4.0.0` Win64 运行库，原生 SDK 保持私有。第三方源码、二进制和许可证位于 `ThirdParty/`；Native 模块隔离错误和异常，不跨 UE 接口抛出异常。

在加载此插件的 UE 5.5 宿主中构建编辑器，然后运行 `Automation RunTests UBotMuJoCo`。原生数值测试位于 `Tests/Native`，可独立使用 CMake 与 Visual Studio 构建；测试以直接调用官方 MuJoCo 的输出作为对照。实际执行和结果应以对应构建日志为准。

Win64 Cook 后的无渲染验证使用 `-NullRHI -sm5`，显式选择 SM5 可避开 UE 5.5 默认 SM6 NullRHI 的 Nanite 资源析构路径。真实渲染可使用 `-d3d11` 验证。

许可证：本插件使用 [LICENSE](LICENSE)，第三方依赖遵循各自附带许可证。
