# controller、scene、PX4 适配器与 ros2_uavs_8.0.0 的代码对比

本次核查日期：2026-10-15。对比目录 `/home/vinci/opencode_pg/00changchun/ros2_uavs_8.0.0`。
范围是代码、配置、接口及测试设施的静态核查；未启动对比项目的飞控或吊舱控制程序，
未验证对比项目所述实机记录，也未重新进行当前项目的 SITL/实机验收。
下面“已有”指源码中存在实现，不代表已经在当前硬件上验证。
本次仅新增本报告，没有修改节点运行逻辑。

当前项目已具备基础收发、状态发布和部分超时保护。主要差距是具体设备兼容性、
任务层接入、视频低延迟和可追溯能力，以及飞控诊断、部署与实机验证。

## 最高优先级

### 1. 行为树仍通过模拟导航接口判断到位

- 当前 `master_ws/src/controller/src/controller_node.cpp:242` 的 `setup_navigation_stub()`
  无条件创建 `controller/goto`、`controller/orbit`、`controller/nav_state`。
  goto 不使用目标坐标，定时器到时就设置 arrived=true。
- `master_ws/src/behavior/ros2/bt_nodes/src/nav_nodes.cpp:35` 的 GoTo 仍调用上述服务；
  `:74` 依据 nav_state.arrived 返回 SUCCESS。Orbit 同样调用模拟接口。
- 对比项目 `external/navigator/src/navigator_node.cpp`、`navigator_route.cpp`、
  `flight_position_core.cpp` 提供实际飞行接口接入、航线执行和状态判断。
- 影响：项目虽然启动了 PX4 适配器，行为树仍可能在飞机没有移动时进入“已到位”阶段。
- 建议：生产模式禁用模拟导航；由独立导航执行器把任务转成 px4/setpoint，基于真实
  位置误差、速度、稳定持续时间判定完成。controller 保持只负责吊舱。

### 2. 吊舱速度/停止指令硬编码为 0x82，存在型号兼容风险

- 当前 `master_ws/src/controller/src/qlet_protocol.cpp:36` 对 velocity 使用 0x82、
  0.01°/s；stop 也使用 0x82。TTL 超时和退出路径同样硬编码此指令。
- 对比项目 `external/controller_node/src/QL130PodControlIntent.cpp:103` 使用
  0x70 粗调，整度/秒；其 README 记载 QL-130TM 实测 0x82 不响应。
- 影响：若用户设备与该记录相符，运动与停止帧都可能被忽略；本地 UDP send 成功
  不能证明设备执行。当前模拟器使用相同协议假设，无法排除这一风险。
- 建议：增加明确的型号/固件协议 profile，统一配置速度和停止指令及量化方式，
  根据该台设备抓包验证，不能只修改正常 velocity 分支而漏掉看门狗和退出路径。
- 边界：对比项目的实机描述尚未独立复验；这是高优先级兼容风险，不是所有 QLET
  吊舱都不支持 0x82 的结论。

### 3. 状态反馈缺少协议布局/profile 选择

- 当前 `qlet_protocol.cpp:69` 只接受 `FC 2C 55` 变长帧，接收后只处理命令 0x0014，
  `decode_state()` 按文档表14读取字段，经纬度为 INT32 ×1e-7。
- 对比项目 `external/controller_node/src/QL130PodFeedback.cpp:87` 同时接受
  64B 的 `FC 2C ... F0` 和 67B 的 `FC 2C 55 ...` 两种封装；其共用字段布局
  仍把经纬度按 FLOAT32 解析（`:182` 附近）。
- 原始 QLET PDF 表6与表14的经纬度类型确实不同：前者 FLOAT，后者 INT32。
  因此不能把两者只视为帧头差异，也不能简单把当前 INT32 一律改成 FLOAT。
- 影响：旧格式设备无法上线；即使封装相同，设备实际布局不同也可能造成定位数值错误。
- 建议：区分封装与字段布局，建立有来源的抓包金样，测试经纬度、角度、焦距等实际值。
  仅校验帧长与 XOR 通过不足以证明字段解释正确。

## Controller 功能及闭环差距

### 4. 缺少基于实际伺服状态的切模式顺序

- 对比项目 `external/controller_node/src/QL130PodExternalAdapter.cpp:239` 在跟踪态
  切其他模式前补取消跟踪 0x3B，在手动态切非惯性模式前补零速，并等待一定间隔。
- 当前 submit()/transmit() 主要负责验证、排队和发送，没有同等状态迁移流程。
- 影响：上层发了新指令，吊舱可能仍保留跟踪或手动模式语义。
- 建议：增加显式的取消旧模式、确认/等待、执行新模式的事务，超时失败反馈给调用者。

### 5. 控制权和设备完成状态还不完整

- 当前已有 source + TTL 租约、停止优先和队列上限，这是应保留的能力。
  但先占租约的自动控制源可以使人工普通指令被拒绝，缺少明确的人工接管模式与优先级。
- 对比项目外部适配层有 Manual/Programmatic 控制权模式与反馈；但其“自动跟踪绕过
  控制权过滤”也需要按当前需求重新审查，不能不加判断地照搬。
- 当前 GimbalCommandResult 仅能报告本地受理/发送/失败，不将 last_command 回显
  纳入事务，也没有“到位”“模式已进入”“跟踪已锁定”的完成判据。
- 对比项目 CommService 有挂起命令与回显计数；**其 README 声称超时重试，实际
  源码 QL130PodCommService.cpp:331 是超时 2×周期释放槽位，不重发。**
- 建议：区分 received/sent/echoed/state_confirmed，使用新鲜反馈判断模式/到位。
  对无序列号的回显明确关联限制；仅对适合重复执行的命令设计有界重试。

### 6. 型号专用功能和跨节点遥测尚未接齐

- 对比项目有地理指引/跟踪 0x1B/0x1C、跟踪模板、红外相关控制、激光强制关闭等
  型号接口。当前通用命令集尚未暴露这些能力；这些需要按设备需求补充，不要求全部照搬。
- 当前 controller 可订阅 AircraftTelemetry 并编码 0x0402，但默认关闭；仓库生产代码
  没有从 px4/state 转换并发布 AircraftTelemetry 的桥接，只有测试发布器。
- PX4 当前状态也缺 GPS UTC 时间、卫星数等遥测字段，不能仅改话题名称接通。
- 影响：真实飞控数据尚未贯通到吊舱；涉及姿态补偿、地理指引的功能缺少完整输入链。
- 建议：扩展所需原生遥测，建立独立转换层，明确坐标系、海拔基准、UTC 和有效性。

## Scene 差距

### 7. 低延迟措施和视频年龄指标不足

- 对比项目 `external/episode_node/src/QL130RTSPStreamer.cpp:455` 有低延迟参数、
  不同 UDP/TCP 配置及 auto 模式路径；转换输出处选择最新解码帧。
- 当前 `master_ws/src/scene/src/video_decoder.cpp:105` 主要配置传输及读写超时，
  逐帧读解码，没有同等的积压清理、自动传输策略和低延迟调优接口。
- 当前 max_publish_fps 是在解码和像素转换后丢发布帧，不能直接降低这些处理成本。
- 当前 scene_node.cpp:129 才生成 header 时间，属于解码后本地时间。
  下游能检查发布后的年龄，但不能据此判断图像是否已在网络/解码队列里积压很久。
- 建议：先测量网络接收→解码→共享内存→算法的延迟分布，再增加可配置的最新帧策略、
  队列/解码耗时指标。保留 source PTS，并明确它到统一时钟的映射和重连后的连续性。
  低延迟 FFmpeg 选项需按实际链路验证，不能认为“禁缓冲”必然改善所有场景。

### 8. 缺少视频与吊舱/飞控状态快照的关联、录像及回放

- 对比项目 QL130StreamService.cpp 可挂载 QL130EpisodeRecorder；其 recorder、
  flight/pod/target cache 实现视频码流、每帧状态快照 SEI、索引、文件轮转与回放工具。
- 当前 scene 只输出帧与流状态，不订阅飞控/吊舱状态，也没有上述落盘和回放设施。
- 影响：算法框、吊舱姿态、飞机位置难以按同一帧复盘；定位错误和现场失败难以复现。
- 建议：建立按时间/帧引用的状态匹配与样本年龄字段，录像服务可独立实现，避免拖慢拉流。
  对比项目保存的是状态快照，不据此宣称已实现精确硬件时间同步。

### 9. 多路流管理和内存带宽优化不足

- 对比项目 QL130StreamService.cpp:69 为每路流创建独立上下文、共享内存与 recorder。
  当前一个 scene 实例只有一个 source_url；可起多个实例，但没有统一的多路配置与管理。
- 对比项目 QL130RTSPStreamer.cpp:403 把颜色转换结果直接写入预留共享内存槽。
  当前先转换到 pixels，再由 frame_ring.cpp:136 memcpy 到共享内存。
  消费端为一致性会再复制；不能把“使用共享内存”等同于端到端零拷贝。
- 建议：需要可见光/红外同时消费时增加 stream_id 和独立状态；用 prepare/commit 槽位
  接口减少生产端中间复制，仍保留当前帧覆盖检测与 robust mutex 的一致性保护。
- 硬解补充：当前未显式提供硬解设备配置，但本次也没有发现对比项目在线拉流主路径
  使用硬解的充分证据；其离线转码工具支持 GPU 不应被写成在线拉流优势。

## PX4 适配器差距

### 10. 遥测和诊断覆盖不足

- 当前原生订阅为 VehicleStatus、LocalPosition、GlobalPosition、Attitude、LandDetected
  和 CommandAck，提供位置/速度/姿态与基本状态。
- 对比项目 `external/navigator/r12/px4_flight_adapter/px4_flight_adapter/adapter_node.py`
  额外接入 HomePosition、SensorGps、BatteryStatus、FailsafeFlags、EstimatorStatusFlags、
  SensorCombined，并发布导航/诊断等接口（populate_diagnostics_state 在 :407）。
- 影响：当前可以知道命令被拒绝或 failsafe=true，但缺少电量、GNSS/EKF/IMU、具体
  failsafe 条件等诊断，现场难以判断“为何不能进入 Offboard”。
- 建议：补充带 valid/fresh/age 的规范遥测与诊断，输出心跳间隔、预热计数、ACK 和
  模式确认等关键审计信息，避免无界打印每帧消息。

### 11. 高度基准和高层飞行动作接口有限

- 当前接口明确支持 ENU/NED 局部目标，原点是估计器原点；没有 Home/解锁点相对高度
  接口、经纬度目标转换、姿态设定值和显式 takeoff 动作。
- 对比项目 R12 适配器有垂直参考锁定/转换、姿态目标及独立动作服务；Navigator
  负责经纬度/航线、悬停、到位与交接。因此不能把这些全部算成适配器内部功能。
- 影响：“起飞后上升20米”不能直接等同于当前 ENU z=20，任务调用者需要自行处理。
- 建议：对外明确区分局部坐标、相对起飞点、相对 Home 和 AMSL；导航及轨迹生成
  放在任务/导航层，适配器维持协议边界。补充真实落地/到位完成语义，避免把进入模式
  的 confirmed 理解为整个任务已经完成。

### 12. 多机配置与部署一致性检查不足

- 当前项目节点默认 namespace=/uav1，但原生 fmu_namespace 默认空，即 /fmu。
  只改 ROS 项目 namespace 不会隔离飞控原生话题。
- 当前 system.launch 启动适配器但不启动 Agent；独立 px4.launch 可显式启动 Agent。
  这是已有设计，缺少的是启动前的整链检查和多机配置一致性验证，不是完全没有启动工具。
- 对比项目有分层启停、状态查询、环境审计、机号配置和 namespace isolation 验证脚本。
- 建议：检查 Agent/端口、固件消息匹配、原生前缀、system/component ID、遥测新鲜度及
  是否存在重复控制发布者，失败时给出具体诊断。保留“只管理自己启动的进程”的边界。

### 13. 验证深度不足

- 当前 PX4 集成测试明确使用 mock 原生 DDS 话题；Agent 测试验证 UDP/伪串口打开
  和生命周期。它们不能替代实际 PX4 uxrce_dds_client→Agent→适配器的完整联调。
- 对比项目提供 verify_flight_actions_sitl_suite.sh、verify_offboard_tracking_sitl_suite.sh、
  verify_namespace_isolation.sh 等脚本；本次仅确认设施存在，没有运行或认证其结果。
- 当前吊舱与 scene 的本地模拟/文件视频测试也不能证明当前实物命令、反馈布局和
  RTSP 长时间表现正确。
- 建议：增加固定固件版本的 SITL、金样抓包回放、实际 RTSP 弱网/重连长稳测试，
  再验收硬件链路。优先覆盖人工接管、估计器重置、断流、Agent 重启及重复控制源。

## 实施顺序

1. P0：生产环境隔离模拟导航；验证吊舱速度/停止指令和反馈布局。
2. P1：真实导航执行器与行为树接入；飞控→吊舱遥测；伺服模式迁移及人工接管；
   补充飞控诊断，并完成 SITL 与设备链路验证。
3. P2：视频延迟指标与积压处理；帧/状态关联、录像回放；按需要扩展多路流与内存优化。

当前应保留的设计：controller 的 TTL 看门狗/租约/停止优先，scene 独立共享内存库
及 session/序号覆盖校验，PX4 的 ENU/NED 与 NaN 语义、ACK+真实状态确认、失效后
不自动恢复旧控制。这些不应为了“看起来和对比项目一样”而退化。
