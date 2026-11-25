"""The startup sequence has to survive a controller_manager that stops answering.

Right after a physical e-stop the manager can hold its resource lock for seconds, so the calls the
sequence makes can time out. A timed-out call's answer never arrives, so waiting on it would block
the process for good and the robot would never be brought back. The sequence has to give up on the
attempt and start another one instead.
"""

import os
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
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

    # Answers everything the first pass needs, then swallows the call the restart path makes.
    controller_manager = launch.actions.ExecuteProcess(
        cmd=[
            "python3",
            fake_cm,
            "--ros-args",
            "-p",
            "never_answer:=['set_hardware_component_state']",
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


class TestSpawnerSurvivesUnansweredCall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_degraded_cm")
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

    def test_unanswered_call_does_not_wedge_the_sequence(
        self, proc_output, spawner_node
    ):
        # The fake manager receives the hardware activation and never answers it, which is what its
        # resource lock looks like from outside.
        self._publish_estop(False)

        marker = "retrying the sequence in"
        proc_output.assertWaitFor(marker, process=spawner_node, timeout=30)

        # One occurrence is only the call timing out. A second is the sequence having been
        # abandoned and started over, which a process parked in a wait that can never finish
        # cannot do.
        deadline = time.time() + 30
        occurrences = 0
        while time.time() < deadline and occurrences < 2:
            time.sleep(1.0)
            occurrences = sum(
                line.text.decode(errors="replace").count(marker)
                for line in proc_output[spawner_node]
            )
        self.assertGreaterEqual(
            occurrences,
            2,
            "The startup sequence never came back after the call that was never answered",
        )

        # And it does not retry for ever: a manager that never answers is not a busy one, so the
        # attempt count runs out and the process can report the failure.
        proc_output.assertWaitFor("Gave up after", process=spawner_node, timeout=60)
