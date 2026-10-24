# scene 视频网关

`master_ws/src/scene` 的 ROS 2 包为 `video_gateway_ros2`，主程序保留原有名称
`episode_node`。它独立负责吊舱视频接收、解码、共享内存写入和 ROS 帧通知；
不收发 QLET 控制指令，不复用 controller 的 UDP socket。

## 设备资料与默认地址

根据用户提供的 `QL-130TM吊舱` 资料目录：

- `2025-QL_096A系列吊舱用户手册-V1.0.pdf` 第 5.1.1 节列出主流、可见光、红外地址。
- 配套 `ViewControlStudio.../appCfg.ini` 的 video1_play_address、video2_play_address
  同样使用 track0 和 track1。
- `QL-ET-XX130TM三轴三光吊舱(33X+35+3).pdf` 明确支持
  IP 视频（RTSP/UDP/TS、H.264/H.265、1080p 30Hz）。

| 视频 | 默认地址 |
| --- | --- |
| 主流，默认配置 | rtsp://192.168.1.160:554/live/track0 |
| 可见光 | rtsp://192.168.1.160:554/live/track1 |
| 红外 | rtsp://192.168.1.160:554/live/track2 |

主流是设备当前输出画面，可能由 controller 切换显示模式。要始终消费可见光或红外，
可修改 source_url 使用 track1 或 track2。一次 scene 实例接收一路视频；多路并行时
使用不同命名空间、话题和配置实例，不要在一个工作线程中轮流读取多路。

UDP 单播需在设备配置页启用，并设置视频目的 IP/端口。显控配置中 video1_port=10008，
仅作为部署参考，不能据此保证设备当前 UDP 端口。控制端口 10000 与视频端口独立。

## 构建和启动

本机已经在 `master_ws/install` 完成编译安装。后续重建可从项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base master_ws/log build \
  --base-paths master_ws/src/platform/ros2/interfaces \
    master_ws/src/platform/shared_video master_ws/src/platform/ros2/support \
    master_ws/src/controller master_ws/src/scene master_ws/src/platform/ros2/launch \
  --build-base master_ws/build --install-base master_ws/install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source master_ws/install/setup.bash
```

依赖 FFmpeg 开发库 libavformat、libavcodec、libavutil、libswscale，以及 tinyxml2。
使用库接口，不依赖 ffmpeg 命令行或 OpenCV。当前实现使用 CPU 软件解码。

单独启动 scene：

```bash
source /opt/ros/jazzy/setup.bash
source master_ws/install/setup.bash
ros2 launch bringup scene.launch.py uav_namespace:=uav1 \
  config_dir:="$PWD/config"
```

统一 system.launch.py 也会加载同一份 scene 配置。独立启动不会启动 controller 或任务节点。
默认配置源文件是 `config/node/scene/scene.xml`；不传 config_dir 时使用安装时的配置快照。
配置在启动时读取，修改源配置后使用上述 config_dir 路径启动，或重新安装后重启。

也可以直接运行并覆盖单个参数：

```bash
ros2 run video_gateway_ros2 episode_node --ros-args -r __ns:=/uav1 \
  -p config_file:="$PWD/config/node/scene/scene.xml" \
  -p rtsp_transport:=udp
```

## 配置

配置格式为 `<node name="scene"><param name="..." type="..." value="..."/></node>`。
ROS 显式参数覆盖 XML。scene 单独运行支持 config_file，不需要 YAML。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| enabled | true | false 时仅发布 disabled 状态，不创建共享内存或视频连接 |
| input_type | rtsp | rtsp、udp 或 file（本地回放/联调） |
| source_url | 主码流地址 | RTSP URL、UDP URL 或本地文件路径 |
| input_format | 空 | FFmpeg 自动探测；UDP/TS 可明确指定 mpegts |
| rtsp_transport | tcp | RTSP 视频承载方式 tcp/udp，与直接 UDP 输入不同 |
| open_timeout_ms / read_timeout_ms | 3000 / 2000 | 建链/探测超时和单次网络读取超时 |
| reconnect_delay_ms | 1000 | 视频失败后的重试间隔 |
| frame_timeout_ms | 1500 | 超过该时间无新解码帧即 offline/stalled |
| output_encoding | bgr8 | bgr8、rgb8、mono8 |
| max_width / max_height | 1920 / 1080 | 共享内存容量上限，分辨率超限会报错并重试 |
| slot_count | 3 | 环形槽位数，范围 2..32 |
| shm_name | 空 | 自动按 ROS 命名空间生成基名，实际名称附加进程/会话标识 |
| max_publish_fps | 0.0 | 0 为发布全部解码帧；大于零时丢弃超额输出，不积压 |
| file_loop | false | 本地文件 EOF 后是否重播；文件按码流帧率回放 |
| frame_id | gimbal_camera | 帧坐标标识 |
| frame_topic / state_topic | scene/frame、scene/stream_state | 相对 ROS 话题名 |
| status_period_ms | 200 | 视频状态发布周期 |
| publish_legacy_frame_stamp | true | 兼容发布 episode/frame_stamp JSON 通知 |
| frame_stamp_topic | episode/frame_stamp | 兼容通知话题 |

直接 UDP/TS 的 XML 例子（需与设备的实际视频目的配置一致）：

```xml
<param name="input_type" type="string" value="udp"/>
<param name="input_format" type="string" value="mpegts"/>
<param name="source_url" type="string"
       value="udp://0.0.0.0:10008?fifo_size=4096&amp;overrun_nonfatal=1"/>
```

`fifo_size` 的单位遵循 FFmpeg UDP 协议，为 188 字节数据包数量。
文件、MPEG-TS 和 RTSP 由 FFmpeg 解复用和解码，H.264/H.265 需本机 FFmpeg 包含对应解码器。
像素格式转换保留输入色彩范围，并根据帧标记处理 BT.709/BT.2020 矩阵。

## ROS 接口

相对话题会带无人机命名空间，例如 `/uav1/scene/frame`。

| 话题 | 消息类型 | QoS | 内容 |
| --- | --- | --- | --- |
| scene/frame | msgs/msg/VideoFrameRef | best effort、深度 1、volatile | 共享内存名称、会话、槽位、帧序号、宽高、步长、像素格式、时间信息 |
| scene/stream_state | msgs/msg/VideoStreamState | reliable、深度 1、transient local | 在线状态、连接阶段、最后错误、帧龄、发布帧率及统计 |
| episode/frame_stamp | std_msgs/msg/String | reliable、深度 1 | 兼容帧通知，只有真实帧写入成功后才发布 |

帧 header.stamp 为本地解码完成后的 ROS 时钟时间，**不是相机曝光时间**。
source_pts_ns 有效时为输入流的展示时间，时间原点由码流决定，不能直接当作 UTC/ROS 时间。
文件重播或断流重连时 PTS 可能回退，消费端应以会话和 sequence 标识共享内存帧。

连接阶段包括 disabled、connecting、waiting_frame、streaming、stalled、reconnecting、ended。
连接成功不等于视频在线，online 需要收到新鲜解码帧。frame_age_s=-1 表示尚无帧。
fps 为最近状态周期的成功发布速率；dropped_frames 统计限频和共享内存槽位忙导致的丢帧，
不代表网络丢包数；errors 统计建链、读取、解码等失败，reconnects 统计重试/重播次数。

## 消费共享内存

公共包 `uav_shared_video` 位于 `master_ws/src/platform/shared_video`，不依赖 ROS。
消费节点依赖该包并链接 `uav_shared_video::uav_shared_video`，使用：

```cpp
#include <uav_shared_video/frame_ring.hpp>
// 收到 VideoFrameRef，shm_name 变化时重新创建 reader。
uav::video::FrameReader reader(message.shm_name);
uav::video::FrameReference ref;
ref.shm_name = message.shm_name;
ref.session_id = message.session_id;
ref.slot = message.slot;
ref.metadata.sequence = message.sequence;
uav::video::Frame frame;
if(reader.read(ref, frame)) {
  // frame.pixels 是当前进程持有的稳定副本；metadata 包含实际宽高、步长和格式。
}
```

实际节点应复用 reader，避免每帧重新映射。完整示例是
`master_ws/src/scene/src/frame_consumer.cpp`：

```bash
ros2 run video_gateway_ros2 scene_frame_consumer --ros-args \
  -r __ns:=/uav1 -p max_frames:=10
ros2 topic echo /uav1/scene/stream_state --qos-durability transient_local
```

共享内存是本机 Linux POSIX 共享内存，消费者需位于同一 IPC 命名空间、具有访问权限，
使用相同的库 ABI。ROS 话题可跨机器传递引用，但远端机器不能据此直接读取本机像素。

每个槽位用跨进程 robust mutex 保护复制过程。生产者不等待消费者：槽位全忙时丢帧。
消费者验证 session、slot 和 sequence，旧引用被覆盖或槽位忙时 read 返回 false；
调用者应等待新引用，不把过期引用作为故障重试。消费端可以并行读取不同帧，
同一个 FrameWriter 仅由一个生产线程使用。

共享内存有 ABI 魔数/版本/容量校验，固定容量内可接收分辨率变化。默认 3 个 1080p BGR
槽位约占 18MiB；分配时预留 /dev/shm 空间，空间不足会在启动时报错。
正常退出 unlink 自己的共享内存；重启生成新名称，已有 reader 映射不被重新初始化。
SIGKILL/断电不能执行清理，可能留下旧对象；确认相关生产者已退出后再清理对应
`/dev/shm/<名称>`。不要删除仍在使用中的对象。

## 验证

```bash
ctest --test-dir master_ws/build/uav_shared_video --output-on-failure
ctest --test-dir master_ws/build/video_gateway_ros2 --output-on-failure
ROS_DOMAIN_ID=184 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  master_ws/src/scene/test/test_scene_integration.py \
  --executable "$PWD/master_ws/install/video_gateway_ros2/lib/video_gateway_ros2/episode_node" \
  --consumer "$PWD/master_ws/install/video_gateway_ros2/lib/video_gateway_ros2/scene_frame_consumer" \
  --fixture "$PWD/master_ws/build/video_gateway_ros2/scene_fixture.ts" \
  --config "$PWD/config/node/scene/scene.xml"
```

集成测试使用确定颜色的本地 MPEG-TS 文件与回环 UDP，验证独立进程读取的像素、XML
覆盖、限频、断流恢复、共享内存清理和禁用模式，不依赖实机。
解码单测还覆盖延迟帧排空、PTS、mono8、分辨率超限。
配套显控软件目录中的 1920×1080 MP4 录像也可用 scene_decoder_test 的 `--probe` 模式验证。
