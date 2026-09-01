"""A controller this spawner's configuration never named must not take the spawner down.

The controller manager lists everything it holds, including whatever another actor loaded. The
startup sequence looks each listed controller up in its own configuration, and an absent one throws
out of the sequence with every controller already deactivated, leaving the robot with none and no
process left to bring them back.
"""

import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner_foreign.yaml"
    )
    fake_cm = os.path.join(pkg_share, "test", "fake_controller_manager.py")

    for path in [spawner_config, fake_cm]:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing test file: {path}")

    controller_manager = launch.actions.ExecuteProcess(
        cmd=[
            "python3",
            fake_cm,
            "--ros-args",
            "-p",
            # Active, and named nowhere in the spawner's configuration.
            "foreign_active:=['someone_elses_controller']",
            "-p",
            "hardware:=['athena_arm_interface']",
        ],
        output="screen",
    )

    spawner_node = launch_ros.actions.Node(
        package="hector_controller_spawner",
        executable="hector_controller_spawner",
        output="screen",
        parameters=[spawner_config],
    )

    return (
        launch.LaunchDescription(
            [
                controller_manager,
                launch.actions.TimerAction(period=3.0, actions=[spawner_node]),
                launch.actions.TimerAction(
                    period=5.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"controller_manager": controller_manager, "spawner_node": spawner_node},
    )


class TestSpawnerToleratesForeignControllers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_foreign_controller")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sequence_completes_alongside_a_controller_it_does_not_know(
        self, proc_output, spawner_node
    ):
        proc_output.assertWaitFor(
            "Multi Controller Spawner complete",
            process=spawner_node,
            timeout=40,
        )


@launch_testing.post_shutdown_test()
class TestSpawnerExit(unittest.TestCase):
    def test_spawner_did_not_die(self, proc_info):
        # An uncaught std::out_of_range leaves SIGABRT here, not a clean exit.
        self.assertEqual(proc_info["hector_controller_spawner"].returncode, 0)
