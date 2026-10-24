# Observe 检测和快速目标跟踪

observe 使用 scene 发布的 `VideoFrameRef` 从同机共享内存复制图片，在本节点运行推理。
scene 负责 RTSP/UDP 解码和共享内存写入；controller 负责吊舱协议控制/反馈；
observe 负责视觉结果，不直接拉流、不控制吊舱、不控制飞控。

## 已实现的算法

- 慢检测：OpenCV DNN + YOLOX-S ONNX，COCO 80 类。包括 person、car、bus、truck、
  motorcycle、bicycle、boat 等。“其他”指其余 COCO 类，不代表能识别任意未知物体。
  权重和类别文件已经放入 `config/node/observe/models/`，不需要运行时下载。
- 快算法：KCF 单目标跟踪。调用者从慢检测结果中选一个目标，发请求后，在该检测
  **原始图片及其目标框**上初始化，再处理新帧。保留原目标 ID，不重新做全图分类。
  丢失后停止并输出 lost，需重新从慢检测选择目标，不自动换人或换车。
- 两个独立工作线程，慢检测默认周期 500 ms，快跟踪默认最小间隔 33 ms；实际频率
  受图片、目标尺寸、模型耗时和硬件限制，配置值不是帧率保证。两者读取最新帧，
  不排队处理过时图片。慢检测期间快跟踪可以继续运行。
- 慢检测使用类别 + IoU 匹配相邻检测结果的 ID；间隔超时、分辨率变化或 scene
  session 变化后清空关联。ID 不复用，但不保证遮挡重现/目标交叉后的物理身份一致。
  快跟踪的 ID 是调用方选定的那一个，后续慢检测的关联不会修改它。

通用模型的实际准确率需要用吊舱画面评估。高空小目标、红外和特殊车辆通常需要
专用数据微调。KCF 也可能漂移，`tracking` 是跟踪器判断，并不提供身份正确性保证。

模型来源、SHA256 和 Apache-2.0 许可见 `config/node/observe/models/UPSTREAM.md`。
官方资料：https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox 。
此后处理明确支持该模型的 RGB 0..255 输入和 `[1,8400,85]` YOLOX 输出；
不能直接把 YOLOv8/v11、嵌入 NMS 的模型或其他输出布局改个文件名替换进来。

## 构建和启动

需要 ROS 2 Jazzy、OpenCV >=4.8（含 dnn、tracking）、jsoncpp、tinyxml2。
当前环境已具备这些依赖。仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths master_ws/src/platform master_ws/src/scene exector_ws/src/observe/ros2 \
  --build-base master_ws/build --install-base master_ws/install \
  --packages-up-to video_gateway_ros2 observe_executor_ros2 bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source master_ws/install/setup.bash
ros2 launch bringup scene.launch.py config_dir:="$PWD/config"
```

另开终端，同样 source ROS 和 overlay 后启动：

```bash
ros2 launch bringup observe.launch.py config_dir:="$PWD/config"
```

两者默认命名空间 `/uav1`，可以分别用 `uav_namespace:=...` 修改，必须一致。
`system.launch.py` 已包含 observe，无需再单独启动第二份。
单独运行 executable 时，必须传 `config_file` 或绝对 `model_dir`：

```bash
ros2 run observe_executor_ros2 observe_executor_node --ros-args \
  -r __ns:=/uav1 -p config_file:="$PWD/config/node/observe/observe.xml"
```

模型路径默认相对 observe.xml 所在目录解析。配置随 bringup 安装，省略 config_dir
时也可使用安装目录中的模型。参数在启动时读取，修改 XML 后重启节点。

## ROS 接口

下列话题均在 UAV 命名空间下；所有名称可在 observe.xml 中配置。

| 方向 | 话题 | 类型 | 作用 |
| --- | --- | --- | --- |
| 输入 | scene/frame | msgs/msg/VideoFrameRef | 最新帧共享内存引用，best effort/depth 1 |
| 输入 | observe/fast/request | msgs/msg/ObserveTrackRequest | 选择目标/停止，reliable |
| 输出 | observe/slow/detections | msgs/msg/Observation | 全图慢检测，reliable |
| 输出 | observe/fast/result | msgs/msg/Observation | 所选目标快跟踪、请求拒绝和丢失状态，reliable |

`Observation.header.stamp` 是 scene 接收/解码时间；`frame_sequence`、`session_id`
对应源图片，`processing_ms` 是本次检测/跟踪耗时。框统一使用**原图像素坐标**
`x,y,width,height`，原点左上，单位像素，需按 image_width/image_height 归一化。
慢检测正常空结果为 `status=ok, detections=[]`，不等同于算法故障。
慢检测 confidence 是模型分数；KCF 不提供可比较的分类置信度，快结果固定 `-1`。

其他节点的交互步骤：

1. 订阅慢话题，在 `status=ok` 的结果中选目标。
2. 保存该结果的 `session_id`、`observation_id` 和目标的 `target_id`。
3. 尽快向请求话题发 `command=start`，携带上述三个 ID 以及自己的 request_id。
4. 订阅快结果，按 request_id 关联；收到 initialized 后，接收 tracking 的单个目标框。
   检查状态和帧时间，只有 tracking 帧才是新的跟踪结果。
5. 收到 lost 后重新选目标；需要主动停止时发 `command=stop`。

示例（用实际慢结果替换所有 ID）：

```bash
ros2 topic echo /uav1/observe/slow/detections
ros2 topic pub --once /uav1/observe/fast/request msgs/msg/ObserveTrackRequest \
  '{request_id: "track-1", command: "start", session_id: 123, observation_id: 456, target_id: 1}'
ros2 topic echo /uav1/observe/fast/result
ros2 topic pub --once /uav1/observe/fast/request msgs/msg/ObserveTrackRequest \
  '{request_id: "stop-1", command: "stop"}'
```

默认缓存最近 4 个慢检测原始帧；初始化帧最大年龄 1500 ms。超过缓存或年龄的请求
会被 rejected。请求必须指向准确的慢结果快照，而不是把旧目标框套在最新图片上。
一个节点同时跟踪一个目标；合法的新 start 会替换旧目标，初始化失败保留旧目标。
一次只接受一个待处理请求，繁忙请求会明确 rejected，不静默丢弃。

快状态：idle、initialized、tracking、lost、rejected、stopped。
错误请求的 rejected 不代表当前目标丢失，应按 request_id 区分。
慢状态：ok、no_frame、stale、error。模型加载失败会持续输出 error，不返回模拟目标；
需要修复配置后重启。断流后快跟踪输出 lost，源 session 改变不会自动接续旧目标。

同机进程、同一 IPC namespace 且具有共享内存读写权限才可消费。
输入元数据需与共享内存实际帧一致；忙碌/覆盖帧跳过，不阻塞 scene。

## 原有行为树 Action

保留 `behaviors/observe` 的 `msgs/action/Task`，替换了原先固定车辆框的桩。
空 JSON goal 返回当前新鲜慢检测中置信度最高的目标；可指定 `target_id`
筛选，也可提供 session_id/observation_id 精确查询缓存快照；没有匹配结果时
等待至 action_timeout_ms 后 abort，取消立即完成。
此 Action 查询慢检测结果，不代替快跟踪请求话题。
结果仍包含 target_id、target_type、confidence、**归一化** bbox `[x,y,w,h]`，
另包含 session_id 和 observation_id。像素坐标的实时接口请使用强类型话题。
find/track/refind 等其他执行器目前的桩不由本次实现自动改造成视觉控制闭环。

## 验证

C++ 测试覆盖真实模型加载、输出布局拒绝、网格解码、类别感知 NMS、相邻帧 ID 关联
与过期、KCF 移动跟踪及丢失：

```bash
ctest --test-dir master_ws/build/observe_executor_ros2 --output-on-failure
```

真实图检测可传额外图片（测试内额外校验含 basketball/dog416 名称的官方测试图）：

```bash
master_ws/build/observe_executor_ros2/observe_perception_test config/node/observe/models picture.jpg
```

ROS 集成测试使用真实 scene 解码和真实 YOLOX，不用假检测器。输入需要是一段包含人
且能够连续跟踪的测试视频；自动验证检测、请求、ID、快慢同时输出、错误选择、Action、
断流、scene 重启、旧 session 拒绝与停止，并关闭由测试启动的所有进程：

```bash
export ROS_DOMAIN_ID=167 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/tmp/observe_test_logs
/usr/bin/python3 exector_ws/src/observe/ros2/test/test_observe_integration.py "$PWD" /tmp/observe_test.avi
```
