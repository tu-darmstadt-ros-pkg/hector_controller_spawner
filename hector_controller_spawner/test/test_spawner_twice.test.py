import os
import sys
import unittest

import launch_testing
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(__file__))
from spawner_test_utils import (  # noqa: E402
    get_controllers,
    make_test_description,
    poll_until,
    run_spawner_to_completion,
)

ROS_DOMAIN_ID = 95


def generate_test_description():
    # No spawner in the description - this test starts it itself, twice.
    return make_test_description(
        controller_config="controllers.yaml",
        spawner_config=None,
        domain_id=ROS_DOMAIN_ID,
    )


class TestControllerSpawnerIdempotency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_controller_spawner_idempotency")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_spawner_run_twice_idempotent(self):
        """Running the spawner a second time must leave the controller states unchanged."""

        pkg_share = get_package_share_directory("hector_controller_spawner")
        spawner_config = os.path.join(
            pkg_share, "test", "config", "controller_spawner.yaml"
        )

        cmd = [
            "ros2",
            "run",
            "hector_controller_spawner",
            "hector_controller_spawner",
            "--ros-args",
            "--params-file",
            spawner_config,
        ]

        expected_active = {
            "joint_state_broadcaster",
            "flipper_velocity_controller",
            "gripper_trajectory_controller",
            "arm_trajectory_controller",
            "vel_to_pos_controller",
        }

        # 1. Initial state - the manager has nothing loaded yet.
        initial_count = len(self._controller_states())
        self.node.get_logger().info(f"Initial controller count: {initial_count}")

        # 2. First run. The subprocess inherits ROS_DOMAIN_ID from this process, so it talks to
        #    the manager launched for this test.
        self.node.get_logger().info("Running spawner first time...")
        return_code, stdout, stderr = run_spawner_to_completion(self, cmd)
        self.assertEqual(
            return_code,
            0,
            f"First spawner run failed with return code {return_code}\n"
            f"stdout:\n{stdout}\nstderr:\n{stderr}",
        )

        # 3. State after the first run. The spawner exits once the switch is through, but the
        #    manager applies it in its own update loop, so wait for it to land.
        first_run = poll_until(
            self._controller_states,
            lambda states: expected_active.issubset(
                {n for n, s in states.items() if s == "active"}
            ),
        )
        self.node.get_logger().info(f"State after first run: {first_run}")
        self.assertGreater(
            len(first_run),
            initial_count,
            "Controllers should be loaded after the first spawner run",
        )
        for controller in expected_active:
            self.assertEqual(
                first_run.get(controller),
                "active",
                f"Controller {controller} should be active after the first run. Seen: {first_run}",
            )

        # 4. Second run against a manager that already has everything loaded and running. This is
        #    the case that used to hang: load_controller reports failure for an already-loaded
        #    controller, so a spawner that cannot tell the two apart never terminates.
        self.node.get_logger().info("Running spawner second time...")
        return_code, stdout, stderr = run_spawner_to_completion(self, cmd)
        self.assertEqual(
            return_code,
            0,
            f"Second spawner run failed with return code {return_code}\n"
            f"stdout:\n{stdout}\nstderr:\n{stderr}",
        )

        # 5. Nothing may have changed. Poll for a state that equals the first run's rather than
        #    sampling once, so a transient mid-switch snapshot cannot fail the comparison.
        second_run = poll_until(
            self._controller_states, lambda states: states == first_run
        )
        self.node.get_logger().info(f"State after second run: {second_run}")
        self.assertEqual(
            second_run,
            first_run,
            "Controller states should be identical after the second spawner run",
        )

    def _controller_states(self):
        return {c.name: c.state for c in get_controllers(self.node)}


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The spawner processes are started by the test itself and checked there; nothing in the
        # launch description is asserted on here.
        pass
