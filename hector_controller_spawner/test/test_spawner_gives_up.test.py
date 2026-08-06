"""The spawner must give up on a manager it cannot talk to, rather than retry forever.

Every startup step used to sit in a `while (rclcpp::ok())` loop with no attempt cap. A service
call that timed out client-side was retried indefinitely, and for load_controller it could never
succeed again: the manager may well have carried out the request we stopped waiting for, and it
answers a second load of the same controller with a plain failure. That turned one slow call on
a loaded machine into a spawner that never terminated - which is what made the CI runs hang until
the launch-test timeout instead of failing in seconds.

Driving every service call into a client-side timeout reproduces that deterministically, without
depending on how fast the machine is.
"""

import os
import sys
import unittest

import launch_testing
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(__file__))
from spawner_test_utils import (  # noqa: E402
    make_test_description,
    run_spawner_to_completion,
    wait_for_controller_manager,
)

ROS_DOMAIN_ID = 98


def generate_test_description():
    return make_test_description(
        controller_config="controllers.yaml",
        spawner_config=None,
        domain_id=ROS_DOMAIN_ID,
    )


class TestSpawnerGivesUp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_spawner_gives_up")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_spawner_terminates_when_every_call_times_out(self):
        wait_for_controller_manager(self, self.node)

        pkg_share = get_package_share_directory("hector_controller_spawner")
        spawner_config = os.path.join(
            pkg_share, "test", "config", "controller_spawner.yaml"
        )

        # A 1 ms budget times out every reply while the manager itself is perfectly healthy, so
        # the spawner sees exactly the failure mode that used to wedge it.
        cmd = [
            "ros2",
            "run",
            "hector_controller_spawner",
            "hector_controller_spawner",
            "--ros-args",
            "--params-file",
            spawner_config,
            "-p",
            "service_call_timeout_ms:=1",
            "-p",
            "max_attempts:=3",
            "-p",
            "retry_delay:=0.5",
        ]

        # Fails the test with the spawner's own output if it does not exit in time.
        return_code, stdout, stderr = run_spawner_to_completion(
            self, cmd, timeout_sec=60.0
        )

        # Exactly 1, not just non-zero: a negative code would mean the process died on a signal.
        # The e-stop test used to see -6 (SIGABRT) from an unguarded future.get() on a call that
        # had already been reported as failed.
        self.assertEqual(
            return_code,
            1,
            "A spawner that never reached its configured state must exit(1), cleanly.\n"
            f"stdout:\n{stdout}\nstderr:\n{stderr}",
        )
        self.assertIn(
            "giving up",
            stderr,
            "The spawner should say which step exhausted its retries.\n"
            f"stdout:\n{stdout}\nstderr:\n{stderr}",
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The spawner is started by the test itself and checked there.
        pass
