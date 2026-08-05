import os
import sys
import unittest

import launch_testing
import rclpy
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(__file__))
from spawner_test_utils import (  # noqa: E402
    assert_controller_states,
    assert_hardware_components_loaded,
    make_test_description,
)

ROS_DOMAIN_ID = 91


def generate_test_description():
    return make_test_description(
        controller_config="controllers.yaml",
        spawner_config="controller_spawner.yaml",
        domain_id=ROS_DOMAIN_ID,
    )


class TestControllerSpawner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_controller_spawner")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_hardware_interfaces_loaded(self):
        assert_hardware_components_loaded(
            self, self.node, ["athena_flipper_interface", "athena_arm_interface"]
        )

    def test_controllers_loaded_and_activated(self):
        assert_controller_states(
            self,
            self.node,
            {
                "joint_state_broadcaster": "active",
                "flipper_velocity_controller": "active",
                "gripper_trajectory_controller": "active",
                "arm_trajectory_controller": "active",
                "vel_to_pos_controller": "active",
                "flipper_trajectory_controller": "inactive",
            },
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The spawner exits non-zero when a startup step exhausts its retries or a controller
        # misses the state the config asks for, so this is a real assertion about the run.
        self.assertEqual(proc_info["hector_controller_spawner"].returncode, 0)
