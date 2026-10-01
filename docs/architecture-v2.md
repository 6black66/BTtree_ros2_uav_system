# UAV System Architecture

## 目录边界

```text
uav_system/
|-- config/
|   |-- behavior_trees/        # 每个状态一棵 BehaviorTree.CPP XML
|   |-- executors/             # 阶段到多个能力插件的映射
|   `-- transitions/           # 顶层状态机转换
|-- master_ws/src/
|   |-- behavior/              # PhaseManager、行为执行器和公共 BT ROS 适配
|   |-- mission/               # 任务调度
|   |-- controller/            # 独立 ROS 2 包；飞机/吊舱指令和状态反馈
|   |-- scene/                 # 实时视频/场景数据入口
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

当前节点为可配置占位代码，用于演示插件动态加载和状态切换。真实路径规划、视频
解码、检测、跟踪、飞机控制和吊舱控制应在对应边界内替换，不应写入行为树 XML。
