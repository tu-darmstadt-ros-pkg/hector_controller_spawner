import os
import sys
import time
import unittest

import launch_testing
import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from spawner_test_utils import (  # noqa: E402
    assert_active_hardware,
    assert_controller_states,
    assert_nothing_active,
    call_service,
    make_test_description,
    wait_for_controller_manager,
)

ROS_DOMAIN_ID = 96

EXPECTED_HARDWARE = ["athena_flipper_interface", "athena_arm_interface"]
EXPECTED_ACTIVE_CONTROLLERS = {
    "joint_state_broadcaster": "active",
    "flipper_velocity_controller": "active",
    "gripper_trajectory_controller": "active",
    "arm_trajectory_controller": "active",
    "vel_to_pos_controller": "active",
}
ALL_CONTROLLERS = list(EXPECTED_ACTIVE_CONTROLLERS) + ["flipper_trajectory_controller"]


def generate_test_description():
    return make_test_description(
        controller_config="controllers.yaml",
        spawner_config="controller_spawner_with_estop.yaml",
        domain_id=ROS_DOMAIN_ID,
    )


class TestEStopFunctionality(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_estop_functionality")
        cls.estop_pub = cls.node.create_publisher(
            Bool,
            "estop_board/hard_estop",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_complete_estop_workflow(self):
        """Engaged e-stop blocks startup; releasing it starts everything, twice over."""

        # The negative assertions below read an empty list as "nothing is active", so the manager
        # has to be answering before any of them run.
        wait_for_controller_manager(self, self.node)

        # Step 1: e-stop engaged - the spawner must not bring anything up.
        self.node.get_logger().info("Step 1: engaging e-stop")
        self._publish_estop(True)
        assert_nothing_active(
            self, self.node, "Nothing may be active while the e-stop is engaged."
        )

        # Step 2: release it and let the spawner run its sequence.
        self.node.get_logger().info("Step 2: releasing e-stop")
        self._publish_estop(False)
        assert_active_hardware(self, self.node, EXPECTED_HARDWARE)
        assert_controller_states(self, self.node, EXPECTED_ACTIVE_CONTROLLERS)

        # Step 3: engage it again and tear the stack down the way the e-stop board would.
        self.node.get_logger().info("Step 3: engaging e-stop and tearing down")
        self._publish_estop(True)
        self._deactivate_all_controllers()
        self._deactivate_hardware_interfaces()
        assert_nothing_active(
            self, self.node, "Nothing may be active after the e-stop was engaged again."
        )

        # Step 4: release again. The spawner has to notice the hardware went away and redo the
        # whole sequence, this time against controllers that are already loaded.
        self.node.get_logger().info("Step 4: releasing e-stop a second time")
        self._publish_estop(False)
        assert_active_hardware(self, self.node, EXPECTED_HARDWARE)
        assert_controller_states(self, self.node, EXPECTED_ACTIVE_CONTROLLERS)

        self.node.get_logger().info("E-stop workflow test completed successfully")

    def _publish_estop(self, engaged):
        """Publish the e-stop state. Repeated because the spawner may still be subscribing."""
        msg = Bool()
        msg.data = engaged
        for _ in range(10):
            self.estop_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

    def _deactivate_hardware_interfaces(self):
        for name in EXPECTED_HARDWARE:
            self.node.get_logger().info(f"Deactivating hardware interface: {name}")
            request = SetHardwareComponentState.Request()
            request.name = name
            request.target_state = State()
            request.target_state.id = State.PRIMARY_STATE_INACTIVE
            request.target_state.label = "inactive"
            response = call_service(
                self.node,
                SetHardwareComponentState,
                "/controller_manager/set_hardware_component_state",
                request,
            )
            self.assertIsNotNone(
                response, f"set_hardware_component_state failed for {name}"
            )

    def _deactivate_all_controllers(self):
        self.node.get_logger().info("Deactivating all controllers")
        request = SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = ALL_CONTROLLERS
        # BEST_EFFORT: some of these may already be inactive, which must not fail the switch.
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = False
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        response = call_service(
            self.node,
            SwitchController,
            "/controller_manager/switch_controller",
            request,
        )
        self.assertIsNotNone(response, "switch_controller call did not complete")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        self.assertEqual(proc_info["hector_controller_spawner"].returncode, 0)
