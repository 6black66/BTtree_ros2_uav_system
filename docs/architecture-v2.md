# UAV System Architecture

## 目录边界

```text
uav_system/
|-- config/
|   |-- behavior_trees/        # 每个状态一棵 BehaviorTree.CPP XML
|   |-- executors/             # 阶段到多个能力插件的映射
|   |-- transitions/           # 顶层状态机转换
|   `-- node/                  # <节点名>/<节点名>.xml，各节点运行配置
|-- master_ws/src/
|   |-- behavior/              # PhaseManager、行为执行器和公共 BT ROS 适配
|   |-- mission/               # 任务调度
|   |-- controller/            # 独立 ROS 2 包；飞机/吊舱指令和状态反馈
|   |-- scene/                 # 实时视频/场景数据入口
|   |-- px4_adapter/           # 独立 uXRCE-DDS 飞控适配器
|   |-- core/                  # 领域模型与无 ROS 框架
|   `-- platform/ros2/         # 消息、公共支持和 bringup
|-- exector_ws/src/
|   |-- bt_plugins/            # 按能力拆分的动态 BT 插件
|   |-- find/
|   |-- observe/
|   |-- track/
|   |-- refind/
|   `-- report/
|-- thirdparty/
|   `-- behaviortree_cpp/
`-- external/                  # 可选，当前为空
```

`master_ws` 中的节点可被多个任务阶段共同使用；`exector_ws` 中的节点只表达阶段
执行能力。ROS 2 进程由 launch 管理，行为树不负责启动或停止进程。

## 状态机和行为树

顶层状态机只负责切换阶段树：

```text
standby -> approch -> onstation -> find -> track -> report -> standby
                                      `-> refind -> track
```

- `standby`：系统自检并等待新任务；
- `approch`：导航规划并控制飞机接近任务区；
- `onstation`：建立盘旋、吊舱搜索位和场景同步；
- `find`：Find 规划吊舱搜索，Observe 提供检测/跟踪框；
- `track`：Track 规划跟踪控制，Observe 持续提供框；
- `refind`：目标丢失后的局部重新发现；
- `report`：整理并上报任务结果。

配置、插件加载、建树或 tick 异常统一回到 `standby`。Track 执行器明确报告目标丢失
时，行为节点写入 `LOST` 结果并进入 `refind`；通信、超时等 `ERROR` 直接回
`standby`，`refind` 失败也回 `standby`。

## 多插件阶段

阶段清单的 `libs` 是有序数组，可以包含任意数量的能力库。例如：

```json
{
  "kind": "phase",
  "name": "phase_find",
  "tree": "phase_find.xml",
  "libs": [
    "libbt_find_nodes.so",
    "libbt_observe_nodes.so",
    "libbt_controller_nodes.so",
    "libbt_scene_nodes.so"
  ]
}
```

XML 只描述业务控制流，不写动态库路径。`PhaseManager` 先加载清单中的所有能力库，
再通过 BehaviorTree.CPP factory 创建 XML 指定的树。同一能力库被多个阶段引用时只
注册一次。

## 依赖规则

```text
ROS 2 entrypoint -> adapter -> executor core -> domain
```

- `core` 不依赖 `rclcpp` 或 ROS 2 生成类型；
- 阶段执行器不直接包含其他执行器实现；
- 飞机、吊舱和场景设备访问集中在 `master_ws` 的公共节点；
- 图像帧不进入行为树黑板，行为树只传递句柄、状态和轻量结果；
- 第三方源码放 `thirdparty`；将来如接入独立二进制、SDK 工程或不宜 ROS 2 化的软件，
  再放入当前为空的 `external`。

## 当前实现范围

controller 已接入 QLET UDP 吊舱控制，scene 已接入 FFmpeg 视频解码和共享内存。
其余节点主要为可配置占位代码，用于演示插件动态加载和状态切换。
真实路径规划、检测、跟踪和飞机控制应在对应边界内替换，不应写入行为树 XML。

## 吊舱控制与视频边界

`controller` 通过 UDP 实现 QLET 单次请求式控制和状态反馈，对项目内提供 ROS 2
命令、状态、结果和链路话题。导航接口仍为模拟实现。
`scene` 独立接收 RTSP/UDP 视频，解码后写入公共 `uav_shared_video` 环形共享内存，
发布帧引用和视频流状态；消费者使用公共读取器校验帧会话和序号。
两者不复用控制 socket，也不以控制状态代替视频流状态。

所有节点运行配置统一放置在 `config/node/<节点名>/<节点名>.xml`，由
`system.launch.py` 加载为 ROS 参数；可通过 `config_dir` 指定配置根目录。
controller、scene 单独运行也支持 `config_file` 指向 XML。详见
[controller.md](controller.md) 和 [scene.md](scene.md)。

## PX4 适配器

`px4_adapter_ros2` 通过官方 Micro XRCE-DDS Agent 与 PX4 v1.17 原生消息连接，
对内提供 `px4/setpoint`、`px4/command`、`px4/state`、结果和链路话题。
配置位于 `config/node/px4_adapter` 与 `config/node/xrce_agent`，启动入口为
`bringup/px4.launch.py`。原 controller 的模拟导航接口尚未迁移；真实飞控控制
使用新适配器接口。详见 [px4_adapter.md](px4_adapter.md)。
