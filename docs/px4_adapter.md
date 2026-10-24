# PX4 uXRCE-DDS 适配器

本节点独立实现，只依赖项目公共 XML 加载器、ROS 2 和官方 PX4 消息定义。
未复用其他本地项目的适配器代码。目标固件为 PX4 最新稳定版 v1.17.0，
消息快照来自官方 px4_msgs 的 release/1.17，具体提交及逐文件哈希见
`thirdparty/px4_msgs/UPSTREAM.md` 和 `SHA256SUMS`。当前仅编译所用的九种官方消息，定义内容未修改。

ROS 包：`px4_adapter_ros2`；程序：`px4_adapter_node`。

```text
其他项目节点 ↔ 项目 ROS 2 接口 ↔ px4_adapter
                                  ↕ 原生 /fmu/* ROS 2 话题
                            Micro XRCE-DDS Agent
                                  ↕ UART / UDP
                            PX4 uxrce_dds_client
```

当前提供多旋翼及 VTOL 多旋翼阶段的局部位置、速度、位置与速度前馈控制，
以及解锁/上锁和模式请求、位置速度姿态及全局位置反馈。
固定翼阶段使用不同的 PX4 控制接口：本版明确拒绝固定翼/过渡阶段的 TrajectorySetpoint，
不将多旋翼速度控制假装成固定翼控制。全局位置目前用于反馈，不提供经纬度目标控制。

现有 controller 的 goto/orbit/nav_state 仍是模拟接口，未自动切换至本节点。
真实控制必须使用下述 px4/* 接口。吊舱控制和 scene 视频处理均保持独立。

## 安装与启动

已在 `master_ws/install` 安装适配器和 Agent。重建时，从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base master_ws/log build \
  --base-paths thirdparty/px4_msgs master_ws/src/platform/ros2/interfaces \
    master_ws/src/platform/ros2/support master_ws/src/px4_adapter \
    master_ws/src/platform/ros2/launch \
  --build-base master_ws/build --install-base master_ws/install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
bash scripts/build_xrce_agent.sh
source master_ws/install/setup.bash
```

Agent 是官方 eProsima v2.4.3，源码及来源记录位于 `thirdparty/micro_xrce_dds_agent`。
其版本按 PX4 官方 ROS 2 Jazzy 对照表选择。构建使用本机 ROS DDS 库，不安装到系统目录；
产物在 `master_ws/install/xrce_agent`。本机构建关闭了可选的 Agent 内部 logger 和 P2P
配置，保留串口及 UDP、DDS 桥接能力；运行诊断以适配器的链路和错误回执为准。
脚本默认两路编译，可用 `UAV_BUILD_JOBS` 调整。

只启动适配器，使用外部已运行的 Agent：

```bash
ros2 launch bringup px4.launch.py config_dir:="$PWD/config"
```

由本次 launch 一并启动 Agent：

```bash
ros2 launch bringup px4.launch.py config_dir:="$PWD/config" start_agent:=true
```

`start_agent` 默认 false，避免接管已有 Agent。true 时只管理自己创建的进程，
不会停止旧服务或杀掉其他 Agent；启动失败会结束本次 launch，不会抢占端口。
`system.launch.py` 也会启动适配器，但不额外启动 Agent；可复用独立 Agent。
不要同时运行两份指向同一 /fmu 输入的适配器。

## 配置目录

```text
config/node/
├── px4_adapter/px4_adapter.xml   # 控制接口、时序、范围和原生 PX4 命名空间
└── xrce_agent/xrce_agent.xml     # 串口/UDP、端口、设备、波特率、可执行程序
```

Agent 默认配置是 serial、`/dev/ttyUSB0`、921600；这是配置示例，不代表已探测到实机串口。
部署前将 device 改为实际设备，优先使用稳定的 `/dev/serial/by-id/...` 路径。
Agent 不与节点交换自定义串口报文，串口由官方 Agent 独占。

```xml
<node name="xrce_agent">
  <param name="transport" type="string" value="serial"/>
  <param name="port" type="int" value="8888"/>
  <param name="device" type="string" value="/dev/serial/by-id/实际设备"/>
  <param name="baudrate" type="int" value="921600"/>
  <param name="executable" type="string" value=""/>
</node>
```

SITL/网络连接改 transport 为 udp4，port 默认 8888。executable 为空时优先选择本项目
安装的 Agent，否则查找 PATH；使用其他安装布局时可填写 Agent 绝对路径。
所有 XML 都在启动时读取，修改后需重启。显式 ROS 参数覆盖 XML，例如只读联调：

```bash
ros2 run px4_adapter_ros2 px4_adapter_node --ros-args -r __ns:=/uav1 \
  -p config_file:="$PWD/config/node/px4_adapter/px4_adapter.xml" \
  -p command_enabled:=false
```

适配器主要配置：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| command_enabled | true | false 拒绝全部控制请求；启动本身不发送控制、不自动解锁 |
| fmu_namespace | 空 | 空对应 /fmu；例如 /uav_1 对应 /uav_1/fmu |
| source_system / source_component | 245 / 191 | 命令源标识，用于匹配 ACK |
| target_system / target_component | 1 / 1 | 必须与 VehicleStatus 的设备标识匹配 |
| control_period_ms / state_period_ms | 50 / 100 | 20Hz 控制会话心跳、10Hz 项目状态 |
| status_timeout_ms / local_timeout_ms | 1000 / 500 | 状态和局部估计的有效时间 |
| setpoint_ttl_ms / max_setpoint_ttl_ms | 500 / 5000 | 设定值默认/最大租期 |
| offboard_warmup_ms | 1500 | 连续 OffboardControlMode 预发送，最小 1100ms |
| ack_timeout_ms / command_timeout_ms | 500 / 5000 | ACK 重试间隔和事务总截止时间；Offboard 另加预热时间 |
| command_retries | 2 | 首次发送之外的最大重试次数 |
| max_position_m | 10000.0 | 每个位置轴相对局部原点的绝对值上限 |
| max_speed_m_s | 20.0 | 速度向量模长上限 |
| max_yaw_rate_rad_s | 2.0 | 偏航角速度绝对值上限 |

范围检查是接口边界，不替代 PX4 的机型配置、定位健康、地理围栏或限速。

## PX4 端准备

固件版本基线来自 [官方 v1.17.0 稳定发布](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0)。
串口、域、命名空间及 Agent 设置以 [官方 uXRCE-DDS 文档](https://docs.px4.io/main/en/middleware/uxrce_dds) 为准。

普通飞控接 TELEM2 时，在 QGroundControl 中配置：

- 确认固件含 uxrce_dds_client，能找到 UXRCE_DDS_CFG 参数。
- 将 UXRCE_DDS_CFG 设为使用的 TELEM 端口，并设置对应 SER_TEL2_BAUD 与 Agent 一致。
- 将占用同一 UART 的 MAVLink 实例解绑；同一端口不能同时承载两个驱动。
- UXRCE_DDS_DOM_ID 与电脑 ROS_DOMAIN_ID 一致；保留正常的桥接时间同步配置。
- 参数重启生效，设备接线和电平按具体飞控手册确认。本代码不修改飞控参数或刷写固件。

程序的项目命名空间默认是 `/uav1`，原生 PX4 命名空间默认是空，二者独立。
若 PX4 使用 `/uav_1/fmu/*`，需将适配器 fmu_namespace 改为 `/uav_1`；
只修改 launch 的 uav_namespace 不会修改飞控原生话题。

## 项目接口

以下均相对于节点命名空间，例如 `/uav1/px4/state`。

| 名称 | 类型 | 方向 / QoS |
| --- | --- | --- |
| px4/setpoint | msgs/msg/FlightSetpoint | 订阅，reliable、深度 1、volatile |
| px4/command | msgs/msg/FlightCommand | 订阅，reliable、深度 10、volatile |
| px4/command_service | msgs/srv/FlightCommand | 异步事务受理入口 |
| px4/state | msgs/msg/FlightState | 发布，best effort、深度 5 |
| px4/link_state | msgs/msg/FlightLinkState | 发布，reliable、深度 1、transient local |
| px4/command_result | msgs/msg/FlightCommandResult | 发布，reliable、深度 50 |

服务返回 accepted=true 只表示受理，最终结果查询 command_result。
所有名称可在适配器 XML 中修改。

### 位置与速度

FlightSetpoint 包括：

- mode：POSITION=0、VELOCITY=1、POSITION_VELOCITY=2。
- coordinate_frame：ENU=0（默认）或 NED=1；不接受机体系目标或自动 TF 转换。
- x/y/z、vx/vy/vz：米、米/秒。
- use_yaw/yaw、use_yaw_rate/yaw_rate：可选偏航角和角速度，单位弧度、弧度/秒。
- source、command_id：逻辑控制源和关联标识；同一 source 可持续刷新目标。
- ttl_ms：零使用默认值；header.stamp 必须为调用者当前 ROS 时间，未来超过 100ms 或已过期均拒绝。

ENU 为东、北、上；NED 为北、东、下。ENU 偏航角零指向东、逆时针为正，
NED 零指向北、顺时针为正。局部原点是 PX4 估计器原点，不一定是 Home 或起飞位置；
例如 ENU z=3 表示局部原点上方 3 米，不能直接解释为距地 3 米。
非空 header.frame_id 必须匹配配置的 ENU frame_id（默认 map），或 NED 的 map_ned。

映射到 TrajectorySetpoint：

| 上层模式 | OffboardControlMode | 原生数据 |
| --- | --- | --- |
| POSITION | position=true | position 有效；velocity/acceleration/jerk 为 NaN |
| VELOCITY | velocity=true | velocity 有效；position/acceleration/jerk 为 NaN |
| POSITION_VELOCITY | position=true | position 和前馈 velocity 有效；acceleration/jerk 为 NaN |

不使用的 yaw/yawspeed 同样设为 NaN，避免隐式朝向零或位置零点。
位置与速度前馈是“位置 + 速度作为前馈”，本版不提供加速度目标接口。

### 命令和 Offboard 生命周期

command 支持 arm、disarm、offboard、hold、land、rtl、position_mode。
不提供强制解锁、强制上锁或绕过 PX4 检查的参数。

1. 上层以至少能够覆盖 TTL 的频率发布有效设定值，建议 20Hz。适配器先缓存，不直接控制飞控。
2. 同一 source 显式请求 offboard，开始 20Hz OffboardControlMode 预发送；预热期间继续刷新目标。
3. 达到预热时间后发 VehicleCommand 请求 Offboard。只有真实 VehicleStatus 报告 Offboard，才开始发 TrajectorySetpoint。
4. 如需解锁，由上层另行显式请求 arm，并观察真实结果。节点不自动解锁或自动起飞。
5. 到期、估计失效、失联或人工切模式时撤销会话；恢复消息不会自动重新进入 Offboard。

手动请求示例（前提是目标源 demo 正在持续发有效设定值）：

```bash
ros2 service call /uav1/px4/command_service msgs/srv/FlightCommand \
  '{command_id: offboard-1, source: demo, command: offboard}'
ros2 topic echo /uav1/px4/command_result
ros2 topic echo /uav1/px4/state --qos-reliability best_effort
```

事务全程异步，不在 ROS 回调中等待 ACK。每次仅跟踪一个 VehicleCommand，
ACK 必须匹配命令字、源系统/组件和 external 标记，并排除重复旧 ACK。
accepted、sent、acknowledged、in_progress 均不等于状态已改变；
confirmed 需要 PX4 接受 ACK 和请求之后收到的实际目标状态同时成立。
例如 land 的 confirmed 表示进入 LAND 模式，不表示已经落地；落地查询 landed_valid/landed。

超时会停止重试并报告 timeout。取消本地事务不撤回已交给 PX4 的命令；
最终行为始终以真实状态为准。PX4 ACK 没有本项目的 command_id，串行跟踪和真实状态确认
用于限制错误关联，不宣称协议具有逐条全局唯一事务号。

设定值仅允许一个 source 在租期内控制，其他 source 被拒绝。
过期后停止目标和心跳；若状态仍为 Offboard 且链路正常，会尝试请求 HOLD。
失联时无法保证发送 HOLD，最终失效处理由 PX4 的 Offboard 丢失配置决定。
本节点正常退出同样停止续发，未实现进程退出后的自主控制。
不要另起控制器同时向相同 /fmu/in/* 写入。

### 真实状态

FlightState 从 PX4 原生状态、局部位置、全局位置、姿态和落地检测消息汇总。
局部位置/速度转换成 ENU，姿态转换到 ENU/FLU。各 valid 标记同时检查设备标记、
数值和消息新鲜度，未取得或过期的数值发布 NaN；connected=false 时状态标志不代表已确认的机体状态。
frame.header.stamp 是适配器发布时间，*_age_s 为本机单调时钟计算的接收龄。
原生位置/航向重置会撤销旧目标，避免沿用改变前的局部参考。

### 原生接口与版本

消息版本大于零时按 PX4 规则追加 `_vN`，不硬编码无版本名：

```text
发布 /fmu/in/offboard_control_mode
发布 /fmu/in/trajectory_setpoint
发布 /fmu/in/vehicle_command
订阅 /fmu/out/vehicle_status_v1
订阅 /fmu/out/vehicle_local_position_v1
订阅 /fmu/out/vehicle_global_position
订阅 /fmu/out/vehicle_attitude
订阅 /fmu/out/vehicle_land_detected
订阅 /fmu/out/vehicle_command_ack
```

原生端点采用 best effort、volatile；项目控制接口不使用 latched 控制请求。
消息与固件升级时应一起核对，不能仅修改 topic 后缀来兼容不同数据结构。

## 测试

本版没有接入实机或实际 PX4 SITL。测试使用隔离 ROS 域中的独立 PX4 话题模拟器；
Agent 另外验证了临时 UDP、伪串口打开及 launch 启停，不将其声称为实机 XRCE 全链路验证。

```bash
ctest --test-dir master_ws/build/px4_adapter_ros2 --output-on-failure
ROS_LOG_DIR=/tmp/uav-px4-test-log /usr/bin/python3 \
  master_ws/src/platform/ros2/launch/test/test_xml_config.py
ROS_LOG_DIR=/tmp/uav-px4-test-log ROS_DOMAIN_ID=186 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  /usr/bin/python3 master_ws/src/px4_adapter/test/test_adapter_integration.py \
  --executable "$PWD/master_ws/install/px4_adapter_ros2/lib/px4_adapter_ros2/px4_adapter_node" \
  --config "$PWD/config/node/px4_adapter/px4_adapter.xml"
ROS_LOG_DIR=/tmp/uav-px4-test-log ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  /usr/bin/python3 master_ws/src/px4_adapter/test/test_agent_smoke.py \
  --agent "$PWD/master_ws/install/xrce_agent/bin/MicroXRCEAgent" --config-dir "$PWD/config"
```

测试覆盖坐标和 NaN 语义、控制源租约、仅心跳预热、显式解锁、ACK 与状态确认、
命令拒绝/丢失/重试、目标过期、人工模式接管、状态新鲜度、机型限制和只读配置。
