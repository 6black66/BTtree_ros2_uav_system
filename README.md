# UAV System

该项目不具有任何实际功能，单纯尝试构建一次项目框架
项目参考 BehaviorTree.CPP 4.9 构建行为树，实现各阶段不同节点的动态加载，依赖ros2工作空间
公共运行能力放在 `master_ws`，阶段执行能力放在 `exector_ws`（名称按项目约定保留），配置、第三方源码和外部程序分别隔离。

## 主目录

```text
uav_system/
|-- config/                    # 状态转换、行为树、阶段插件和部署配置
|-- thirdparty/                # 随工程源码构建的第三方依赖
|-- exector_ws/                # find/observe/track/refind/report 等阶段能力
|   `-- src/
|-- master_ws/                 # 调度、状态机、行为执行器、控制器、场景及公共接口
|   `-- src/
`-- external/                  # 可选；当前为空，供非 ROS 外部程序接入
```

现有 `build*`、`install*`、`log*` 是此前构建产生的历史目录，不属于源码架构。
`external` 当前没有实际依赖；只有接入独立 Navigator、厂商程序或 SDK 工程时才使用。

## 阶段状态机

```text
standby -> approch -> onstation -> find -> track -> report -> standby
                                      |
                                      +-- lost/failure -> refind -> track
                                                            |
                                                            +-- failure -> standby
```

`approch` 按当前接口约定使用此拼写。加载插件、创建行为树或 tick 抛出异常时，
`PhaseManager` 统一回到 `standby`。普通阶段执行失败也回 `standby`；Track 执行器
明确返回目标丢失时产生 `LOST` 结果，才会进入 `refind`。

## 能力插件

插件按能力拆分并跨阶段复用，不再按阶段生成单一动态库：

```text
libbt_mission_nodes.so
libbt_navigation_nodes.so
libbt_controller_nodes.so
libbt_scene_nodes.so
libbt_find_nodes.so
libbt_observe_nodes.so
libbt_track_nodes.so
libbt_refind_nodes.so
libbt_report_nodes.so
```

阶段与能力库的关系由 `config/executors/phase_*.json` 配置。例如：

- `approch`：navigation、controller、scene、mission；
- `find`：find、observe、controller、scene；
- `track`：track、observe、controller、scene；
- `refind`：refind、observe、controller、scene。

`PhaseManager` 进入阶段时遍历清单中的 `libs`，动态注册尚未加载的插件，然后读取
该阶段 XML 创建树。插件在进程中只加载一次。

当前所有执行节点均为占位实现，仅用于验证动态装载、阶段切换、ROS 2 通信和失败
回退，不包含真实检测、跟踪、路径规划或飞控算法。

详细目录和职责见 [docs/architecture-v2.md](docs/architecture-v2.md)。
# BTtree_ros2_uav_system
