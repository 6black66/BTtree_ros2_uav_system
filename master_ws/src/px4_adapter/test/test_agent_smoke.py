#!/usr/bin/env python3
"""Start the official Agent on temporary UDP/PTY endpoints and exercise launch ownership."""
import argparse
import errno
import os
from pathlib import Path
import pty
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from uav_bringup.xrce import agent_command


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agent',required=True)
    parser.add_argument('--config-dir',required=True)
    args=parser.parse_args()
    process=None
    master_fd=slave_fd=None
    with tempfile.TemporaryDirectory(prefix='xrce-agent-test-') as temporary:
        root=Path(temporary)
        directory=root/'node/xrce_agent'; directory.mkdir(parents=True)
        adapter_dir=root/'node/px4_adapter'; adapter_dir.mkdir(parents=True)
        (adapter_dir/'px4_adapter.xml').write_text((Path(args.config_dir)/'node/px4_adapter/px4_adapter.xml').read_text())
        config_path=directory/'xrce_agent.xml'
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
            reserve.bind(('127.0.0.1',0)); port=reserve.getsockname()[1]
        def config(transport,device='/dev/unused'):
            element=ET.Element('node',name='xrce_agent')
            for name,kind,value in [('transport','string',transport),('port','int',port),('device','string',device),
                                    ('baudrate','int',921600),('executable','string',args.agent)]:
                ET.SubElement(element,'param',name=name,type=kind,value=str(value))
            ET.ElementTree(element).write(config_path,encoding='unicode')
        log=open(root/'agent.log','w+')
        def wait(predicate,timeout=5):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                if process.poll() is not None: raise AssertionError(f'Agent/launch exited with {process.returncode}')
                if predicate(): return
                time.sleep(.02)
            raise AssertionError('Agent startup timed out')
        def port_owned():
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
                try: probe.bind(('0.0.0.0',port)); return False
                except OSError as error:
                    if error.errno==errno.EADDRINUSE: return True
                    raise
        def stop():
            process.send_signal(signal.SIGINT); process.wait(timeout=8)
            assert process.returncode in (0,-signal.SIGINT), process.returncode
        try:
            config('udp4')
            process=subprocess.Popen(agent_command(str(root)),stdout=log,stderr=log)
            wait(port_owned); stop()
            master_fd,slave_fd=pty.openpty()
            device=os.ttyname(slave_fd)
            config('serial',device)
            process=subprocess.Popen(agent_command(str(root)),stdout=log,stderr=log)
            def serial_owned():
                for fd in Path(f'/proc/{process.pid}/fd').iterdir():
                    try:
                        if os.readlink(fd)==device: return True
                    except FileNotFoundError: pass
                return False
            wait(serial_owned); stop()
            os.close(master_fd); os.close(slave_fd); master_fd=slave_fd=None
            config('udp4')
            process=subprocess.Popen(['/opt/ros/jazzy/bin/ros2','launch','bringup','px4.launch.py',
                                      'start_agent:=true',f'config_dir:={root}','uav_namespace:=agent_smoke'],stdout=log,stderr=log)
            wait(port_owned); time.sleep(.5)
            assert process.poll() is None
            stop()
            assert not port_owned(), 'owned Agent survived launch shutdown'
            print('Agent smoke passed: official 2.4.3 UDP bind, serial PTY open, XML configuration, owned launch startup/shutdown')
        except BaseException:
            log.flush(); log.seek(0); print(log.read()); raise
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            if master_fd is not None: os.close(master_fd)
            if slave_fd is not None: os.close(slave_fd)
            log.close()


if __name__=='__main__': main()
