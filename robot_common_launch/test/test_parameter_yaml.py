import os
import unittest

import rclpy
import yaml
from yaml.events import AliasEvent

from robot_common_launch.common.robot_utils import write_temp_ros2_control_yaml
from robot_common_launch.common.controller_utils import write_spawner_controller_param_file


class ParameterYamlTest(unittest.TestCase):
    def test_shared_objects_with_rcl_parser(self):
        shared = [1.0, 2.0]
        params = {'left_gains': shared, 'right_gains': shared}
        payload = {'left': {'ros__parameters': params}, 'right': {'ros__parameters': params}}
        paths = [write_temp_ros2_control_yaml(payload, quiet=True),
                 write_spawner_controller_param_file('left', params, quiet=True)]
        try:
            for path, expected in zip(paths, [payload, {'left': {'ros__parameters': params}}]):
                with open(path) as f:
                    text = f.read()
                self.assertFalse(any(isinstance(e, AliasEvent) for e in yaml.parse(text)))
                self.assertEqual(yaml.safe_load(text), expected)
                context = rclpy.Context()
                try:
                    # Exercise real rcl YAML argument parsing; creates no robot nodes.
                    rclpy.init(args=['--ros-args', '--params-file', path], context=context)
                finally:
                    if context.ok():
                        context.shutdown()
            self.assertIs(params['left_gains'], params['right_gains'])
        finally:
            for path in paths:
                os.unlink(path)


if __name__ == '__main__':
    unittest.main()
