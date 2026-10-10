"""Check sensor ownership and switches without starting hardware processes."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


@unittest.skipUnless(importlib.util.find_spec('launch_ros'), 'requires ROS 2 launch')
class SensorLaunchTest(unittest.TestCase):
    def test_platform_and_switches(self):
        from launch import LaunchContext, LaunchDescription
        from launch.actions import GroupAction, IncludeLaunchDescription

        spec = importlib.util.spec_from_file_location(
            'k9_launch', Path(__file__).resolve().parents[1] / 'launch/k9.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cases = [
            ('pi', 'true', 'true', 2),
            ('all', 'true', 'true', 2),
            ('jetson', 'true', 'true', 0),
            ('pi', 'false', 'true', 0),
            ('pi', 'true', 'false', 1),
            ('pi', 'false', 'false', 0),
        ]
        for platform, camera, floor, expected in cases:
            with self.subTest(platform=platform, camera=camera, floor=floor):
                context = LaunchContext()
                context.launch_configurations.update({
                    'platform': platform, 'log_level': 'info', 'enable_chess': 'false',
                    'enable_ld06': 'false', 'enable_robot_description': 'false',
                    'enable_oak': camera, 'enable_oak_floor_filter': floor,
                })
                with patch.object(module, 'get_package_share_directory',
                                  side_effect=lambda name: '/test/' + name) as lookup:
                    actions = module.launch_nodes(context)
                groups = [a for a in actions if isinstance(a, GroupAction)]
                includes = [a for g in groups for a in g.get_sub_entities()
                            if isinstance(a, IncludeLaunchDescription)]
                self.assertEqual(len(includes), expected)
                looked_up = [call.args[0] for call in lookup.call_args_list]
                self.assertEqual('k9_ros2_nav' in looked_up, bool(expected))
                if expected:
                    paths = []
                    for action in includes:
                        source = action.launch_description_source
                        with patch.object(source, '_get_launch_description',
                                          return_value=LaunchDescription()):
                            source.get_launch_description(context)
                        paths.append(source.location)
                    self.assertTrue(any('oak.launch.py' in p for p in paths))
                    self.assertEqual(any('oak_floor_filter.launch.py' in p for p in paths),
                                     expected == 2)


if __name__ == '__main__':
    unittest.main()
