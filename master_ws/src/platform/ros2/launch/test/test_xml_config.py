#!/usr/bin/env python3
"""Read source configuration and validate composition without starting ROS nodes."""
import importlib.util
from pathlib import Path
import tempfile
import sys
import unittest

from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[6]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
SPEC = importlib.util.spec_from_file_location('system_launch', Path(__file__).resolve().parents[1] / 'launch/system.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class XmlConfigurationTest(unittest.TestCase):
    def test_all_node_configs(self):
        variables = {'config_dir': str(ROOT / 'config'), 'uav_namespace': 'test_uav', 'plugin_dir': '/tmp/plugins/lib'}
        configs = {name: MODULE.load_node_config(ROOT / 'config', name, variables) for name in
                   ('controller', 'scene', 'mission', 'behavior', 'find', 'observe', 'track', 'refind', 'report', 'px4_adapter', 'xrce_agent')}
        self.assertEqual(configs['controller']['send_period_ms'], 20)
        self.assertIs(configs['controller']['require_online'], True)
        self.assertEqual(configs['mission']['target_namespace'], 'test_uav')
        self.assertEqual(configs['behavior']['manifests_dir'], str(ROOT / 'config/executors'))

    def test_rejects_malformed_config(self):
        bodies = [
            '<node name="test"><param name="x" type="bool" value="yes"/></node>',
            '<node name="test"><param name="x" type="int" value="2.1"/></node>',
            '<node name="test"><param name="x" type="int" value="9223372036854775808"/></node>',
            '<node name="test"><param name="x" type="double" value="nan"/></node>',
            '<node name="test"><param name="x" type="string" value="${missing}"/></node>',
            '<node name="test"><param name="x" type="string" value="a"/><param name="x" type="int" value="1"/></node>',
            '<node name="wrong"/>',
        ]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'node/test/test.xml'
            path.parent.mkdir(parents=True)
            for body in bodies:
                with self.subTest(body=body):
                    path.write_text(body)
                    with self.assertRaises(ValueError):
                        MODULE.load_node_config(temp, 'test', {})

    def test_composition(self):
        MODULE.get_package_prefix = lambda _: '/tmp/plugins'
        for role, expected in [('master', 1), ('uav', 1), ('master_uav', 2)]:
            context = LaunchContext()
            context.launch_configurations.update(role=role, uav_namespace='uav1', config_dir=str(ROOT / 'config'))
            self.assertEqual(len(MODULE.compose(context)), expected)


if __name__ == '__main__':
    unittest.main()
