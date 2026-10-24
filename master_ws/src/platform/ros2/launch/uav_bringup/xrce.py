"""Build Agent actions without taking over existing agents, services or serial ports."""
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_prefix
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from .xml_config import load_node_config


def agent_command(config_dir):
    config = load_node_config(config_dir, 'xrce_agent', {})
    transport = config['transport']
    executable = config.get('executable', '')
    if not executable:
        local = Path(get_package_prefix('px4_adapter_ros2')).parent / 'xrce_agent/bin/MicroXRCEAgent'
        executable = str(local) if local.is_file() else shutil.which('MicroXRCEAgent')
    if not executable or not Path(executable).is_file():
        raise ValueError('MicroXRCEAgent not found; run bash scripts/build_xrce_agent.sh or set executable in xrce_agent.xml')
    if transport == 'serial':
        device, baudrate = config['device'], config['baudrate']
        if not isinstance(baudrate, int) or not 9600 <= baudrate <= 4000000:
            raise ValueError('invalid Agent serial baudrate')
        if not str(device).startswith('/dev/') or not Path(device).exists():
            raise ValueError('Agent serial device does not exist; configure the actual /dev/serial/by-id/... path in xrce_agent.xml')
        return [executable, 'serial', '--dev', device, '-b', str(baudrate)]
    if transport == 'udp4':
        port = config['port']
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError('invalid Agent UDP port')
        return [executable, 'udp4', '-p', str(port)]
    raise ValueError('Agent transport must be serial or udp4')


def agent_actions(config_dir):
    agent = ExecuteProcess(cmd=agent_command(config_dir), output='screen')
    # A failed owned Agent terminates this launch; external Agent processes are untouched.
    on_exit = RegisterEventHandler(OnProcessExit(target_action=agent, on_exit=[
        EmitEvent(event=Shutdown(reason='owned MicroXRCEAgent exited'))]))
    return [on_exit, agent]
