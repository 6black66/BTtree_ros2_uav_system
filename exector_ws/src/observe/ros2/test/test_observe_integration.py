#!/usr/bin/env python3
"""Real scene -> shared memory -> YOLOX -> requested KCF integration.
Run after building with sourced ROS/overlay, isolated ROS domain and local IPC.
Arguments: repository root, test video containing people. No fake detector.
"""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from msgs.action import Task
from msgs.msg import Observation, ObserveTrackRequest

root = Path(sys.argv[1]).resolve()
video = str(Path(sys.argv[2]).resolve())
scene_binary = root / 'master_ws/install/video_gateway_ros2/lib/video_gateway_ros2/episode_node'
observe_binary = root / 'master_ws/install/observe_executor_ros2/lib/observe_executor_ros2/observe_executor_node'
processes = []
logs = []
rclpy.init()
node = rclpy.create_node('observe_integration')
slow, fast = [], []
node.create_subscription(Observation, '/observe_test/observe/slow/detections', slow.append, 100)
node.create_subscription(Observation, '/observe_test/observe/fast/result', fast.append, 100)
publisher = node.create_publisher(ObserveTrackRequest, '/observe_test/observe/fast/request', 10)
action = ActionClient(node, Task, '/observe_test/behaviors/observe')


def start(binary, label, params):
    log = open('/tmp/observe_integration_' + label + '.log', 'w')
    logs.append(log)
    command = [str(binary), '--ros-args', '-r', '__ns:=/observe_test']
    for key, value in params.items():
        command += ['-p', f'{key}:={value}']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    processes.append(process)
    return process


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            raise AssertionError('node failed graceful shutdown')


def until(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        for process in processes:
            assert process.poll() is None or process.returncode == 0, 'node crashed; inspect /tmp/observe_integration_*.log'
        if predicate():
            return
    raise AssertionError('timeout: ' + str([(x.status, x.detail) for x in fast[-5:]]) + ' slow=' + str([(x.status, x.detail) for x in slow[-3:]]))


def request(name, command, observation=None, target=None):
    message = ObserveTrackRequest(request_id=name, command=command)
    if observation:
        message.session_id = observation.session_id
        message.observation_id = observation.observation_id
        message.target_id = target.target_id
    publisher.publish(message)


try:
    scene_params = {'input_type': 'file', 'source_url': video, 'file_loop': 'true', 'max_publish_fps': '15.0'}
    scene = start(scene_binary, 'scene', scene_params)
    observe = start(observe_binary, 'observe', {'config_file': root / 'config/node/observe/observe.xml'})
    until(lambda: any(x.status == 'ok' and any(d.class_id == 0 for d in x.detections) for x in slow))
    snapshot = next(x for x in reversed(slow) if x.status == 'ok' and any(d.class_id == 0 for d in x.detections))
    target = next(d for d in snapshot.detections if d.class_id == 0)
    assert snapshot.image_width > 0 and snapshot.frame_sequence > 0
    assert 0 <= target.x < snapshot.image_width and target.confidence > 0.45
    request('select-person', 'start', snapshot, target)
    until(lambda: sum(x.request_id == 'select-person' and x.status == 'tracking' for x in fast) >= 4)
    tracks = [x for x in fast if x.request_id == 'select-person' and x.status == 'tracking']
    assert all(x.observation_id == snapshot.observation_id and x.session_id == snapshot.session_id and
               len(x.detections) == 1 and x.detections[0].target_id == target.target_id and
               x.detections[0].confidence == -1 for x in tracks)
    assert all(a.frame_sequence < b.frame_sequence for a, b in zip(tracks, tracks[1:]))
    slow_count, fast_count = len(slow), len(tracks)
    until(lambda: len(slow) > slow_count and len([x for x in fast if x.status == 'tracking']) > fast_count + 3)
    print('PASS: real YOLOX person detection, stable selected ID, independent slow/fast output')
    # Invalid selection must not switch or kill the current valid target.
    bad = ObserveTrackRequest(request_id='bad-id', command='start', session_id=snapshot.session_id,
                              observation_id=snapshot.observation_id, target_id=2**63)
    publisher.publish(bad)
    until(lambda: any(x.request_id == 'bad-id' and x.status == 'rejected' for x in fast))
    previous = len(fast)
    until(lambda: any(x.request_id == 'select-person' and x.status == 'tracking' for x in fast[previous:]))
    assert action.wait_for_server(timeout_sec=3)
    future = action.send_goal_async(Task.Goal(behavior='observe', goal_json='{}'))
    until(future.done)
    handle = future.result()
    assert handle.accepted
    result_future = handle.get_result_async()
    until(result_future.done)
    result = result_future.result().result
    assert result.ok and json.loads(result.result_json)['target_type'] == 'person'
    print('PASS: invalid request rejection and compatible action returning real detection')
    stop(scene)
    until(lambda: any(x.request_id == 'select-person' and x.status == 'lost' for x in fast))
    until(lambda: any(x.status == 'stale' and x.session_id == snapshot.session_id for x in slow))
    scene = start(scene_binary, 'scene_restart', scene_params)
    until(lambda: any(x.status == 'ok' and x.session_id != snapshot.session_id and x.detections for x in slow))
    new = next(x for x in reversed(slow) if x.status == 'ok' and x.session_id != snapshot.session_id and x.detections)
    request('old-session', 'start', snapshot, target)
    until(lambda: any(x.request_id == 'old-session' and x.status == 'rejected' for x in fast))
    request('restart-select', 'start', new, new.detections[0])
    until(lambda: any(x.request_id == 'restart-select' and x.status == 'tracking' for x in fast))
    request('stop-track', 'stop')
    until(lambda: any(x.request_id == 'stop-track' and x.status == 'stopped' for x in fast))
    boundary = len(fast)
    end = time.monotonic() + 0.4
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert not any(x.status == 'tracking' for x in fast[boundary:])
    print('PASS: frame timeout, scene restart, old-session rejection and explicit stop')
    stop(observe)
    boundary = len(slow)
    broken = start(observe_binary, 'missing_model', {
        'config_file': root / 'config/node/observe/observe.xml',
        'model_path': '/tmp/observe_nonexistent_model.onnx', 'action_timeout_ms': 300})
    until(lambda: any(x.status == 'error' for x in slow[boundary:]))
    assert not any(x.status == 'ok' for x in slow[boundary:])
    until(lambda: action.server_is_ready())
    future = action.send_goal_async(Task.Goal(behavior='observe', goal_json='{}'))
    until(future.done)
    handle = future.result()
    assert handle.accepted
    result_future = handle.get_result_async()
    until(result_future.done)
    assert not result_future.result().result.ok
    print('PASS: missing model reports error and action cannot fabricate successful detections')
finally:
    for process in reversed(processes):
        stop(process)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
