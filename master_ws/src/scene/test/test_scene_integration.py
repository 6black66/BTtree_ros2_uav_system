#!/usr/bin/env python3
"""End-to-end file/UDP -> decoded shared pixels -> independent ROS consumer test."""
import argparse
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from msgs.msg import VideoFrameRef, VideoStreamState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ('executable', 'consumer', 'fixture', 'config'):
        parser.add_argument('--' + argument, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('scene_integration_test')
    frames, states = [], []
    subscriptions = [
        node.create_subscription(VideoFrameRef, '/scene_test/scene/frame', frames.append,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(VideoStreamState, '/scene_test/scene/stream_state', states.append,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    processes, resources = [], []
    sender_stop, sender_enabled = threading.Event(), threading.Event()
    sender_thread, process = None, None
    with tempfile.TemporaryDirectory(prefix='scene-integration-') as temp:
        def start(name, **params):
            nonlocal process
            frames.clear(); states.clear()
            log = open(Path(temp) / f'{name}.log', 'w+')
            resources.append(log)
            command = [args.executable, '--ros-args', '-r', '__ns:=/scene_test',
                       '-p', f'config_file:={args.config}', '-p', 'max_width:=64', '-p', 'max_height:=48',
                       '-p', 'status_period_ms:=100', '-p', 'frame_timeout_ms:=300',
                       '-p', 'open_timeout_ms:=1000', '-p', 'read_timeout_ms:=500',
                       '-p', 'reconnect_delay_ms:=100', '-p', f'shm_name:=/scene_test_{name}']
            for key, value in params.items():
                text = str(value).lower() if isinstance(value, bool) else str(value)
                command += ['-p', f'{key}:={text}']
            process = subprocess.Popen(command, stdout=log, stderr=log)
            processes.append((process, log))
        def wait(predicate, timeout=8):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if process.poll() is not None:
                    raise AssertionError(f'scene exited with {process.returncode}')
                if predicate():
                    return
            raise AssertionError('timeout waiting for scene behavior')
        def stop():
            name = frames[-1].shm_name if frames else ''
            process.send_signal(signal.SIGINT)
            process.wait(timeout=4)
            assert process.returncode == 0
            if name:
                assert not (Path('/dev/shm') / name.lstrip('/')).exists(), 'shared memory not unlinked'
        def consumer_check():
            log = open(Path(temp) / f'consumer-{len(processes)}.log', 'w+')
            resources.append(log)
            consumer = subprocess.Popen([args.consumer, '--ros-args', '-r', '__ns:=/scene_test',
                                         '-p', 'max_frames:=5'], stdout=log, stderr=log)
            processes.append((consumer, log))
            wait(lambda: consumer.poll() is not None)
            log.flush(); log.seek(0)
            rows = [line.split() for line in log.read().splitlines() if line.startswith('FRAME ')]
            assert consumer.returncode == 0 and len(rows) == 5, rows
            for row in rows:
                assert tuple(map(int, row[3:6])) == (64, 48, 9216), row
                assert row[6] == 'bgr8'
                blue, green, red = map(int, row[7:10])
                assert blue < 10 and green < 10 and red > 240, row
        try:
            start('file', input_type='file', source_url=args.fixture, file_loop=True, max_publish_fps=10.0)
            wait(lambda: len(frames)>=3 and states and states[-1].online)
            assert frames[-1].width == 64 and frames[-1].height == 48 and frames[-1].source_pts_valid
            consumer_check()
            wait(lambda: states[-1].dropped_frames > 0)
            stop()
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1', 0))
                port = reserve.getsockname()[1]
            data = Path(args.fixture).read_bytes()
            chunks = [data[i:i+1316] for i in range(0, len(data), 1316)]
            def send_video():
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    index = 0
                    while not sender_stop.is_set():
                        if sender_enabled.is_set():
                            sock.sendto(chunks[index], ('127.0.0.1', port))
                            index = (index+1) % len(chunks)
                        sender_stop.wait(4.0 / len(chunks))
            sender_enabled.set()
            sender_thread = threading.Thread(target=send_video)
            sender_thread.start()
            start('udp', input_type='udp', input_format='mpegts', max_publish_fps=20.0,
                  source_url=f'udp://127.0.0.1:{port}?fifo_size=4096&overrun_nonfatal=1')
            wait(lambda: len(frames)>=3 and states and states[-1].online)
            consumer_check()
            old_session = frames[-1].session_id
            sender_enabled.clear()
            wait(lambda: not states[-1].online and states[-1].errors > 0 and states[-1].reconnects > 0)
            count = states[-1].published_frames
            sender_enabled.set()
            wait(lambda: states[-1].online and states[-1].published_frames > count)
            assert frames[-1].session_id == old_session
            sender_enabled.clear()
            stop()
            sender_stop.set(); sender_thread.join(timeout=2)
            start('disabled', enabled=False)
            wait(lambda: bool(states) and not states[-1].enabled)
            assert not states[-1].online and states[-1].state == 'disabled' and not states[-1].shm_name
            stop()
            print('Scene integration passed: XML overrides, file/UDP decode, independent shared-memory consumer, pixel colors, throttling, stall/reconnect, bounded shutdown, cleanup, disabled mode')
        except BaseException:
            for child, log in processes:
                log.flush(); log.seek(0); print(log.read())
            raise
        finally:
            sender_stop.set()
            if sender_thread is not None:
                sender_thread.join(timeout=2)
            for child, _ in processes:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        child.kill(); child.wait()
            for resource in resources:
                resource.close()
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
