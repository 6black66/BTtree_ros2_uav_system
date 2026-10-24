# 吊舱控制网关

`master_ws/src/controller` 的 ROS 2 包名为 `flight_controller_ros2`，可执行程序为
`controller_node`。本次实现 QLET V1.4 单次请求式 UDP 控制和状态反馈。

## 职责边界

- controller：QLET 控制帧、状态反馈、命令调度、控制源租约、链路监测；不接收视频。
- scene：独立负责 RTSP / UDP 视频接入、解码、共享内存写入和帧通知，
  详见 [scene.md](scene.md)。
- 飞机导航的 `controller/goto`、`controller/orbit`、`controller/nav_state`
  仍为原有模拟功能，不通过吊舱 UDP 链路控制飞机。

## 配置与启动

所有节点的配置源文件位于 `config/node/<节点名>/<节点名>.xml`，包内不放配置副本。
`bringup` 在安装时将根目录 config 打包进 share/bringup/config。

XML 示例：

```xml
<node name="controller">
  <param name="local_ip" type="string" value="0.0.0.0"/>
  <param name="local_port" type="int" value="10000"/>
  <param name="gimbal_ip" type="string" value="192.168.1.160"/>
  <param name="gimbal_port" type="int" value="10000"/>
</node>
```

`param` 支持 string、bool（true/false）、int、double。统一 launch 会校验 XML 并转换为
ROS 参数；支持 `${config_dir}`、`${uav_namespace}`、`${plugin_dir}` 路径变量。
变量替换由 launch 完成。配置在启动时加载，修改后需重启节点。

统一启动（工作区构建并 source 后）：

```bash
ros2 launch bringup system.launch.py role:=uav uav_namespace:=uav1 \
  config_dir:=/home/vinci/opencode_pg/00changchun/uav_system/config
```

controller 也支持单独读取 XML；显式 ROS 参数覆盖 XML 默认值：

```bash
ros2 run flight_controller_ros2 controller_node --ros-args \
  -r __ns:=/uav1 \
  -p config_file:=/home/vinci/opencode_pg/00changchun/uav_system/config/node/controller/controller.xml
```

默认接收地址绑定 `0.0.0.0:10000`，发送到 `192.168.1.160:10000`。
文档指定设备反馈目的地址为 `192.168.1.121:10000`，需让本机网卡地址与设备反馈配置匹配。
绑定 `0.0.0.0` 只是监听本机所有地址，不会改变设备的反馈目的地址。
收包严格检查吊舱源 IP 和端口。

## ROS 接口

以下名称相对于无人机命名空间，例如 `/uav1/controller/gimbal/command`。

| 话题 | 类型（msgs/msg/） | 方向 | QoS |
| --- | --- | --- | --- |
| controller/gimbal/command | GimbalCommand | 订阅 | reliable，深度由 queue_limit 指定，volatile |
| controller/gimbal/state | GimbalState | 发布 | best effort，深度 5，volatile |
| controller/gimbal/command_result | GimbalCommandResult | 发布 | reliable，深度 20，volatile |
| controller/gimbal/link_state | GimbalLinkState | 发布 | reliable，深度 1，transient local |
| controller/aircraft/telemetry | AircraftTelemetry | 订阅，可选 | best effort，深度 1 |

发布、订阅名称可通过 controller.xml 中对应 `*_topic` 参数修改。
`state.header.stamp` 为本地接收时间，框架角与姿态角分别发布，角度沿用厂家约定：
方位向右为正，俯仰向上为正。设备测距或定位字段的有效性需结合设备工作状态判断，
消息不将零值解释为有效测量。

兼容原有 `controller/gimbal` 服务：与话题使用相同的队列和校验，`ok=true` 仅表示入队。
原 `controller/gimbal_state` JSON 话题保留，发布实际反馈角度与伺服模式，不再回显指令。
新消费者应使用结构化状态话题；该状态话题仅在合法反馈到达时发布，是否过期查询链路话题。

## 控制指令

| command | QLET 命令字 | 使用字段 |
| --- | --- | --- |
| point | 0x72 | yaw_deg [-179,179]，pitch_deg [-120,90] |
| attitude_point | 0x7C | 同上，姿态指引 |
| velocity | 0x82 | yaw_rate_deg_s、pitch_rate_deg_s，均 [-50,50]，分辨率 0.01°/s |
| stop | 0x82，必要时追加 0x45 | 停止轴运动；若本节点启动了变倍，也停止镜头 |
| center / stow / down | 0x71 / 0x74 / 0x73 | 无 |
| lock / freeze / follow | 0x7A / 0x7A / 0x7B | 无 |
| scan / preset_search | 0x79 | 无，使用设备当前扫描参数 |
| zoom | 0x45 | zoom_direction：-1 缩小、0 停止、1 放大；zoom_speed：0..7 |
| focal_length | 0x4C | focal_length_mm [0.1,6500]，分辨率 0.1mm |
| display | 0x25 | display_mode：0..5，见协议表 2 |
| track | 0x3A | target_x [-960,960]、target_y [-540,540]，相对图像中心的像素偏移 |
| cancel_track | 0x3B | 无 |
| photo / record_start / record_stop | 0x32 / 0x33 / 0x33 | 无 |
| range_once / range_start / range_stop | 0x3D / 0x3E / 0x3F | 无；连续测距采用 1Hz |

具体型号的机械范围、扫描和姿态模式支持情况以实机为准。上述范围为协议边界。
`preset_search` 当前兼容映射为扫描，不自动设置扫描范围、角度或焦距。
原 GimbalCmd.srv 的 `zoom` 倍率没有通用的厂家焦距映射，因此明确拒绝旧服务的 zoom 命令；
使用新话题的方向变倍或绝对焦距，不将倍率静默解释为毫米。

例如，指向方位 5°、俯仰 -30°：

```bash
ros2 topic pub --once /uav1/controller/gimbal/command msgs/msg/GimbalCommand \
  '{command_id: point-1, source: manual, command: point, yaw_deg: 5.0, pitch_deg: -30.0}'
ros2 topic echo /uav1/controller/gimbal/state --qos-reliability best_effort
ros2 topic echo /uav1/controller/gimbal/link_state --qos-durability transient_local
```

## 调度与反馈语义

- 默认需要收到新鲜状态反馈才接收运动指令，`require_online=false` 可用于无反馈联调。
- 默认 TTL 为 500ms，最大 2000ms，可配置；零 header 时间戳按接收时刻计算。
  非零时间戳须与 ROS 时钟一致，过期或提前超过 100ms 的命令会被拒绝。
- 一个 source 在命令有效期内持有控制租约；其他 source 的普通命令被拒绝。
  stop 和镜头停止可跨控制源提交；stop 清空待发送队列并释放租约。
  source 是逻辑仲裁标识，不是认证身份；没有填写时归为 anonymous。
- 同类型连续命令在发送队列中保留最新值，离散指令保序；队列有上限。
- 控制帧实际发送间隔不少于 20ms；UDP 收包通过 5ms 非阻塞定时器处理，每次最多 64 包。
  使用单线程 ROS executor，所有状态访问串行，避免 socket 阻塞回调。
- velocity 和方向 zoom 需要持续刷新，TTL 到期分别发送轴停止和镜头停止。
  point 是有限目标指令，TTL 限制发送时效，不表示到期必须中断已开始的定位。
- 状态反馈超过默认 500ms 未更新即离线；清除待发命令，停止本节点启动的连续动作，
  恢复后不重放旧指令。首次反馈之前 feedback_age_s=-1。
- 结果包含 accepted、sent、rejected、expired、superseded、send_failed。
  `sent` 仅为本机 sendto 成功。协议无通用请求序列号，状态里的 last_command
  不能严格关联每次请求；本实现不伪造逐条设备确认或到位结果。
- 正常退出时发送停止帧，停止本节点启动的连续转动和镜头变倍。
- 离散命令不自动重试，以免重复拍照等操作。UDP 无法保证交付，应用需观察真实状态。

`telemetry_enabled=true` 时订阅 AircraftTelemetry，按表 10 编码 0x0402，使用最多
500ms 内的新鲜样本，需包含有效时间戳，发送周期配置为 20ms。该消息的 gps_year
为从 2000 年起的偏移，GPS 时间按 UTC 输入；速度为北、东、地向（向下为正）。
不使用模拟 NavState 填充真实遥测。

反馈按 0x0014、表 14 的至少 60 字节解析。帧长取命令字加数据长度，帧总长为
length+5，校验为长度字节到最后一个数据字节的 XOR。文档表 8/12 的个别下标不一致，
实现按长度定义和控制示例、表 15 的 SN 反馈示例核对。收到损坏帧后可重新寻找帧头，
支持单个 UDP 报文中多帧；不会拼接不同 UDP 报文的残帧。

## 验证

使用系统 Python 构建，避免 Conda Python 与 ROS Jazzy 的 Python 3.12 依赖冲突：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths master_ws/src/platform/ros2/interfaces master_ws/src/controller \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ctest --test-dir build/flight_controller_ros2 --output-on-failure
```

本地集成测试会启动 controller 和回环 UDP 模拟器，使用临时端口，不连接实机：

```bash
ROS_DOMAIN_ID=183 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  master_ws/src/controller/test/test_gateway_integration.py \
  --executable "$PWD/install/flight_controller_ros2/lib/flight_controller_ros2/controller_node"
/usr/bin/python3 master_ws/src/platform/ros2/launch/test/test_xml_config.py
```

手动模拟器：`/usr/bin/python3 master_ws/src/controller/test/fake_gimbal.py`，默认绑定
127.0.0.1:19001，向 127.0.0.1:19000 发送状态。单独启动 controller 时覆盖 local_port=19000、
gimbal_ip=127.0.0.1、gimbal_port=19001 即可。
