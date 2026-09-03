"""A restart has to look at the controllers, not at the hardware components.

The dynamixel interface's own e-stop and its offset service deactivate controllers without touching
the lifecycle state of the hardware component they belong to. A restart that asks only whether the
components are still ACTIVE therefore sees a healthy robot and skips the controller phase, leaving
the hardware up with nothing driving it and reporting that the bring-up succeeded.
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
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def generate_test_description():
    pkg_share = get_package_share_directory("hector_controller_spawner")
    spawner_config = os.path.join(
        pkg_share, "test", "config", "controller_spawner_torn_down.yaml"
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


class TestControllersTornDownBehindTheSpawner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_torn_down")
        estop_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.estop_pub = cls.node.create_publisher(
            Bool, "estop_board/hard_estop", estop_qos
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_estop(self, engaged):
        msg = Bool()
        msg.data = engaged
        for _ in range(5):
            self.estop_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _controller_states(self):
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        self.assertIsNotNone(response)
        return {c.name: c.state for c in response.controller}

    def test_a_restart_reactivates_controllers_torn_down_underneath_it(
        self, proc_output, spawner_node
    ):
        self._publish_estop(False)
        proc_output.assertWaitFor(
            "Multi Controller Spawner complete", process=spawner_node, timeout=30
        )
        self.assertEqual(
            self._controller_states().get("joint_state_broadcaster"), "active"
        )

        # Somebody else stops the controller. The hardware component is untouched and stays ACTIVE,
        # which is exactly what the dynamixel interface's e-stop leaves behind.
        switch = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.assertTrue(switch.wait_for_service(timeout_sec=10.0))
        request = SwitchController.Request()
        request.deactivate_controllers = ["joint_state_broadcaster"]
        request.strictness = SwitchController.Request.BEST_EFFORT
        future = switch.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertNotEqual(
            self._controller_states().get("joint_state_broadcaster"), "active"
        )

        self._publish_estop(True)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self._publish_estop(False)

        deadline = 30
        proc_output.assertWaitFor(
            "Multi Controller Spawner complete", process=spawner_node, timeout=deadline
        )
        self.assertEqual(
            self._controller_states().get("joint_state_broadcaster"),
            "active",
            "The restart saw the hardware still active and skipped the controllers, so the "
            "controller somebody else stopped was never brought back.",
        )
