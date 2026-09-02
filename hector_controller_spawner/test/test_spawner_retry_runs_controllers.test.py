"""A retry after an abandoned attempt has to run the controller phase.

The restart shortcut asks whether the hardware is still active and, if it is, decides there is
nothing to redo. Hardware left active by an attempt that was abandoned before it ever reached the
controllers does not mean that: the robot is energised with no controllers running, and the
spawner reported itself complete.
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner_degraded_cm.yaml"
    )
    fake_cm = os.path.join(pkg_share, "test", "fake_controller_manager.py")

    for path in [spawner_config, fake_cm]:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing test file: {path}")

    # Swallowing the first listing abandons the first attempt with the hardware already active,
    # which is the state the restart shortcut misreads.
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


class TestRetryRunsControllers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_retry_runs_controllers")
        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.estop_pub = cls.node.create_publisher(
            Bool, "estop_board/hard_estop", estop_qos
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_a_retry_does_not_mistake_live_hardware_for_a_finished_sequence(
        self, proc_output, spawner_node
    ):
        msg = Bool()
        msg.data = False
        for _ in range(5):
            self.estop_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

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
            "The retry took the shortcut for hardware that was still up and skipped the "
            f"controllers entirely: {states}",
        )

        shortcut = "No need to redo hw interface and controller start sequence"
        self.assertFalse(
            any(
                shortcut in line.text.decode(errors="replace")
                for line in proc_output[spawner_node]
            ),
            "The retry decided there was nothing to redo, which is the line that names the bug.",
        )
