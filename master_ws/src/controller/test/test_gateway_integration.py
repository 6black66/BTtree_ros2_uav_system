#!/usr/bin/env python3
"""Exercise the real ROS 2 gateway against loopback UDP. Requires sourced build."""
import argparse
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from msgs.msg import GimbalCommand, GimbalState, GimbalCommandResult, GimbalLinkState, AircraftTelemetry
from msgs.srv import GimbalCmd
from fake_gimbal import state_packet, decode_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gateway_integration_test')
    results, states, links, received = [], [], [], []
    subscriptions = [
        node.create_subscription(GimbalState, '/qlet_test/controller/gimbal/state', states.append, qos_profile_sensor_data),
        node.create_subscription(GimbalCommandResult, '/qlet_test/controller/gimbal/command_result', results.append, 100),
        node.create_subscription(GimbalLinkState, '/qlet_test/controller/gimbal/link_state', links.append, 100),
    ]
    pub = node.create_publisher(GimbalCommand, '/qlet_test/controller/gimbal/command', 10)
    telemetry_pub = node.create_publisher(AircraftTelemetry, '/qlet_test/controller/aircraft/telemetry', qos_profile_sensor_data)
    service = node.create_client(GimbalCmd, '/qlet_test/controller/gimbal')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.setblocking(False)
    remote_port = sock.getsockname()[1]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1', 0))
        local_port = reserve.getsockname()[1]
    process = None
    feedback = True
    with tempfile.TemporaryDirectory(prefix='qlet-integration-') as temp:
        config = Path(temp) / 'controller.xml'
        # The CLI override must win over XML gimbal_port=1.
        config.write_text(f'''<node name="controller">
          <param name="local_ip" type="string" value="127.0.0.1"/>
          <param name="gimbal_ip" type="string" value="127.0.0.1"/>
          <param name="local_port" type="int" value="{local_port}"/>
          <param name="gimbal_port" type="int" value="1"/>
          <param name="feedback_timeout_ms" type="int" value="250"/>
          <param name="telemetry_enabled" type="bool" value="true"/>
        </node>''')
        log = open(Path(temp) / 'gateway.log', 'w+')
        last_feedback = 0
        def step():
            nonlocal last_feedback
            t = time.monotonic()
            if feedback and t-last_feedback >= 0.02:
                sock.sendto(state_packet(), ('127.0.0.1', local_port))
                last_feedback = t
            rclpy.spin_once(node, timeout_sec=0.005)
            while True:
                try:
                    packet, _ = sock.recvfrom(65536)
                except BlockingIOError:
                    break
                command, payload = decode_command(packet)
                received.append((time.monotonic(), command, payload))
            if process.poll() is not None:
                raise AssertionError('gateway exited unexpectedly')
        def until(predicate, timeout=5):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                step()
                if predicate():
                    return
            raise AssertionError('timed out waiting for gateway behavior')
        def send(command_id, command, source='test', **fields):
            message = GimbalCommand(command_id=command_id, source=source, command=command, **fields)
            message.header.stamp = node.get_clock().now().to_msg()
            pub.publish(message)
        def status(command_id, expected):
            return any(r.command_id==command_id and r.status==expected for r in results)
        try:
            process = subprocess.Popen([args.executable, '--ros-args', '-r', '__ns:=/qlet_test',
                '-p', f'config_file:={config}', '-p', f'gimbal_port:={remote_port}'], stdout=log, stderr=log)
            until(lambda: bool(states) and pub.get_subscription_count()>0 and service.service_is_ready())
            assert states[-1].yaw_deg==5 and states[-1].pitch_deg==-30
            assert states[-1].visible_focal_length_mm==25
            send('point', 'point', yaw_deg=5., pitch_deg=5., ttl_ms=200)
            until(lambda: status('point','sent'))
            assert any(c==0x72 and p==bytes.fromhex('f4 01 f4 01') for _,c,p in received)
            send('busy', 'center', source='other')
            until(lambda: status('busy','rejected'))
            send('stop', 'stop', source='other')
            until(lambda: status('stop','sent'))
            send('invalid', 'point', yaw_deg=180.)
            until(lambda: status('invalid','rejected'))
            bad_stamp=GimbalCommand(command_id='bad_stamp',source='test',command='center')
            bad_stamp.header.stamp.sec=-1
            pub.publish(bad_stamp)
            until(lambda: status('bad_stamp','rejected'))
            stale=GimbalCommand(command_id='stale',source='test',command='center',ttl_ms=100)
            stale.header.stamp=node.get_clock().now().to_msg()
            stale.header.stamp.sec-=2
            pub.publish(stale)
            until(lambda: status('stale','rejected'))
            send('velocity','velocity',yaw_rate_deg_s=-10.25,ttl_ms=150)
            until(lambda: status('velocity','sent'))
            until(lambda: any(c==0x82 and p==b'\0'*4 and t>next(t for t,c,p in received if c==0x82 and p!=b'\0'*4) for t,c,p in received))
            assert any(c==0x82 and p==bytes.fromhex('ff fb 00 00') for _,c,p in received)
            # Allow source lease to expire before testing the compatibility service.
            start=time.monotonic()
            until(lambda: time.monotonic()-start>0.22)
            request=GimbalCmd.Request(command='center')
            future=service.call_async(request)
            until(future.done)
            assert future.result().ok and 'not device completion' in future.result().message
            future=service.call_async(GimbalCmd.Request(command='zoom',zoom=2.))
            until(future.done)
            assert not future.result().ok
            send('release','stop')
            until(lambda: status('release','sent'))
            send('zoom','zoom',zoom_direction=1,zoom_speed=4,ttl_ms=150)
            until(lambda: status('zoom','sent'))
            until(lambda: any(c==0x45 and p==b'\0'*4 for _,c,p in received))
            assert any(c==0x45 and p==bytes.fromhex('01 04 00 00') for _,c,p in received)
            # Telemetry table 10: signed values, little endian and 30-byte payload.
            until(lambda: telemetry_pub.get_subscription_count()>0)
            m=AircraftTelemetry(pitch_deg=-2.,roll_deg=3.,heading_deg=90.,gps_year=26,gps_month=10,gps_day=15,
                longitude_deg=120.,latitude_deg=30.,satellites=12,altitude_m=100.,velocity_down_m_s=-1.25)
            m.header.stamp=node.get_clock().now().to_msg()
            telemetry_pub.publish(m)
            until(lambda: any(c==0x0402 for _,c,_ in received))
            p=next(p for _,c,p in received if c==0x0402)
            assert len(p)==30 and struct.unpack_from('<h',p,0)[0]==-200
            assert struct.unpack_from('<i',p,13)[0]==1200000000 and struct.unpack_from('<h',p,28)[0]==-125
            # Foreign-source packets must not be accepted as gimbal feedback.
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as foreign:
                foreign.sendto(state_packet(99),('127.0.0.1',local_port))
            until(lambda: links[-1].foreign_datagrams>0)
            assert states[-1].yaw_deg==5
            feedback=False
            until(lambda: bool(links) and not links[-1].online, timeout=3)
            send('offline','center')
            until(lambda: status('offline','rejected'))
            corrupt=bytearray(state_packet()); corrupt[-1]^=1
            for _ in range(3):
                sock.sendto(corrupt,('127.0.0.1',local_port)); step()
            until(lambda: links[-1].invalid_frames>0)
            assert not links[-1].online
            # Valid checksum but short state payload must not make the link online.
            short=bytearray([0xFC,0x2C,0x55,2,0x14,0,0x16])
            sock.sendto(short,('127.0.0.1',local_port))
            start=time.monotonic()
            until(lambda: time.monotonic()-start>0.15)
            assert not links[-1].online
            feedback=True
            until(lambda: links[-1].online)
            # Actual wire control spacing is at least 20ms (receiver timestamps have jitter).
            controls=[t for t,c,p in received if c!=0x0402]
            assert all(b-a>=0.018 for a,b in zip(controls,controls[1:])), controls
            send('final_release','stop')
            until(lambda: status('final_release','sent'))
            send('shutdown_velocity','velocity',yaw_rate_deg_s=2.,ttl_ms=1500)
            until(lambda: status('shutdown_velocity','sent'))
            send('shutdown_zoom','zoom',zoom_direction=1,ttl_ms=1500)
            until(lambda: status('shutdown_zoom','sent'))
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            shutdown_frames=[]
            while True:
                try:
                    data,_=sock.recvfrom(65536)
                    shutdown_frames.append(decode_command(data))
                except BlockingIOError:
                    break
            assert (0x82,b'\0'*4) in shutdown_frames and (0x45,b'\0'*4) in shutdown_frames, shutdown_frames
            assert process.returncode==0
            print('ROS/UDP integration passed: XML overrides, real feedback, commands, arbitration, TTL stop, service, telemetry, corruption, offline/recovery')
        except BaseException:
            log.flush(); log.seek(0); print(log.read()); raise
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            sock.close(); log.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
