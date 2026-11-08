"""An abandoned attempt has to be retried whether or not an e-stop is being tracked.

Without an e-stop topic there is nothing to restart the sequence, so an attempt that gave up on a
call the manager did not answer was the only one that ever ran. The process then reported success
having activated nothing.
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
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner_retry_no_estop.yaml"
    )
    fake_cm = os.path.join(pkg_share, "test", "fake_controller_manager.py")

    for path in [spawner_config, fake_cm]:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing test file: {path}")

    # The first listing is swallowed, which abandons the first attempt after the hardware is up.
    # Every later call is answered, so a retry has everything it needs to finish.
    controller_manager = launch.actions.ExecuteProcess(
        cmd=[
            "python3",
            fake_cm,
            "--ros-args",
            "-p",
            "stall_first:=['list_controllers']",
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
                    period=6.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"controller_manager": controller_manager, "spawner_node": spawner_node},
    )


class TestRetryWithoutEStop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_retry_no_estop")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_the_attempt_that_gave_up_is_run_again(
        self, proc_output, proc_info, spawner_node
    ):
        proc_output.assertWaitFor(
            "Retrying the startup sequence", process=spawner_node, timeout=30
        )
        proc_output.assertWaitFor(
            "Multi Controller Spawner complete", process=spawner_node, timeout=30
        )

        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        self.assertIsNotNone(response)

        states = {c.name: c.state for c in response.controller}
        self.assertEqual(
            states.get("joint_state_broadcaster"),
            "active",
            "The spawner stopped after the attempt it abandoned, so nothing was activated: "
            f"{states}",
        )
        # Shutting the launch down while the spawner still exits on its own interrupts it
        proc_info.assertWaitForShutdown(process=spawner_node, timeout=10)


@launch_testing.post_shutdown_test()
class TestSpawnerExitCode(unittest.TestCase):
    def test_exits_cleanly(self, proc_info, spawner_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0], process=spawner_node
        )
