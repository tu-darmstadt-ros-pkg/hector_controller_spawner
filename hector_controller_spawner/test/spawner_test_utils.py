"""Shared helpers for the hector_controller_spawner launch tests.

The spawner is asynchronous: it waits for the controller manager, then loads, configures and
switches controllers. How long that takes depends entirely on how loaded the machine is - the
manager serialises those service calls behind one mutex and the first load of each plugin type
pays for the pluginlib scan. Asserting after a fixed sleep therefore encodes the speed of the
machine the test was written on, which is why the suite passed on a workstation and failed in
CI. Everything here polls until the expected state is reached or a generous deadline expires.

The launch description builder also pins a per-test ROS_DOMAIN_ID. Every test in this package
runs a node called /controller_manager, and so does hector_controller_orchestrator's gtest;
colcon is free to schedule them concurrently, so without isolation a client can bind to the
wrong manager.
"""

import os
import subprocess
import time

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents

# Generous on purpose. A correct spawner reaches its final state long before this; the deadline
# only bounds how long a genuinely broken run takes to report itself.
DEFAULT_TIMEOUT_SEC = 90.0
POLL_PERIOD_SEC = 0.5


def make_test_description(
    controller_config,
    domain_id,
    spawner_config=None,
    spawner_delay=3.0,
    ready_delay=4.0,
):
    """Build the launch description shared by the spawner tests.

    controller_config and spawner_config are file names inside test/config. Passing
    spawner_config=None leaves the spawner out, for tests that start it themselves.

    Returns the (LaunchDescription, context) pair that generate_test_description must return.
    """
    pkg_share = get_package_share_directory("hector_controller_spawner")
    config_dir = os.path.join(pkg_share, "test", "config")

    controller_config = os.path.join(config_dir, controller_config)
    robot_description_file = os.path.join(config_dir, "athena.urdf")
    required = [controller_config, robot_description_file]
    if spawner_config is not None:
        spawner_config = os.path.join(config_dir, spawner_config)
        required.append(spawner_config)
    for path in required:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing test file: {path}")

    with open(robot_description_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controller_config],
    )

    actions = [
        # Executed before anything else, so the launched processes and this very test process
        # (rclpy.init runs later, in setUpClass) all end up on the same isolated domain.
        launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain_id)),
        launch.actions.SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"
        ),
        robot_state_publisher,
        controller_manager,
    ]
    context = {"controller_manager": controller_manager}

    if spawner_config is not None:
        spawner_node = launch_ros.actions.Node(
            package="hector_controller_spawner",
            executable="hector_controller_spawner",
            output="screen",
            parameters=[spawner_config],
        )
        actions.append(
            launch.actions.TimerAction(period=spawner_delay, actions=[spawner_node])
        )
        context["spawner_node"] = spawner_node

    actions.append(
        launch.actions.TimerAction(
            period=ready_delay, actions=[launch_testing.actions.ReadyToTest()]
        )
    )

    return launch.LaunchDescription(actions), context


def call_service(node, srv_type, srv_name, request=None, timeout_sec=10.0):
    """Make a one-shot service call. Returns the response, or None if it did not complete."""
    client = node.create_client(srv_type, srv_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn(f"Service {srv_name} is not available")
            return None
        future = client.call_async(
            request if request is not None else srv_type.Request()
        )
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        return future.result()
    finally:
        node.destroy_client(client)


def get_controllers(node, timeout_sec=10.0):
    """Return the manager's controller list, or [] if it could not be queried."""
    response = call_service(
        node,
        ListControllers,
        "/controller_manager/list_controllers",
        timeout_sec=timeout_sec,
    )
    return list(response.controller) if response else []


def get_controller_states(node, timeout_sec=10.0):
    """Return the manager's controllers as a {name: state} mapping."""
    return {c.name: c.state for c in get_controllers(node, timeout_sec)}


def get_hardware_components(node, timeout_sec=10.0):
    """Return the manager's hardware components, or [] if they could not be queried."""
    response = call_service(
        node,
        ListHardwareComponents,
        "/controller_manager/list_hardware_components",
        timeout_sec=timeout_sec,
    )
    return list(response.component) if response else []


def get_active_hardware(node, timeout_sec=10.0):
    """Return the names of the hardware components currently reported as active."""
    return [
        hw.name
        for hw in get_hardware_components(node, timeout_sec)
        if hw.state.label == "active"
    ]


def poll_until(probe, done, timeout_sec=DEFAULT_TIMEOUT_SEC):
    """Call probe() until done(result) holds or the deadline passes. Returns the last result.

    The caller asserts on that last result, so a timeout produces the same message as an
    outright wrong state - including what was actually observed.
    """
    deadline = time.monotonic() + timeout_sec
    result = probe()
    while not done(result) and time.monotonic() < deadline:
        time.sleep(POLL_PERIOD_SEC)
        result = probe()
    return result


def assert_controller_states(
    test_case, node, expected, timeout_sec=DEFAULT_TIMEOUT_SEC
):
    """Wait for every controller in `expected` ({name: state}) to reach its state, then assert."""
    states = poll_until(
        lambda: get_controller_states(node),
        lambda seen: all(seen.get(name) == state for name, state in expected.items()),
        timeout_sec,
    )
    for name, state in expected.items():
        test_case.assertIn(
            name,
            states,
            f"Controller '{name}' was never loaded within {timeout_sec}s. Seen: {states}",
        )
        test_case.assertEqual(
            states[name],
            state,
            f"Controller '{name}' did not reach '{state}' within {timeout_sec}s. Seen: {states}",
        )
    return states


def assert_active_hardware(test_case, node, expected, timeout_sec=DEFAULT_TIMEOUT_SEC):
    """Wait for every hardware component in `expected` to become active, then assert."""
    active = poll_until(
        lambda: get_active_hardware(node),
        lambda seen: all(name in seen for name in expected),
        timeout_sec,
    )
    for name in expected:
        test_case.assertIn(
            name,
            active,
            f"Hardware '{name}' did not become active within {timeout_sec}s. Active: {active}",
        )
    return active


def assert_hardware_components_loaded(
    test_case, node, expected, timeout_sec=DEFAULT_TIMEOUT_SEC
):
    """Wait for every hardware component in `expected` to be known to the manager, then assert."""
    names = poll_until(
        lambda: [hw.name for hw in get_hardware_components(node)],
        lambda seen: all(name in seen for name in expected),
        timeout_sec,
    )
    test_case.assertGreater(len(names), 0, "No hardware interfaces loaded")
    for name in expected:
        test_case.assertIn(
            name,
            names,
            f"Hardware '{name}' was not loaded within {timeout_sec}s. Loaded: {names}",
        )
    return names


def wait_for_controller_manager(test_case, node, timeout_sec=DEFAULT_TIMEOUT_SEC):
    """Block until the manager answers list_controllers.

    Worth doing explicitly before any negative assertion: the query helpers return an empty list
    when the service is unavailable, which would otherwise read as "nothing is active".
    """
    ready = poll_until(
        lambda: (
            call_service(
                node,
                ListControllers,
                "/controller_manager/list_controllers",
                timeout_sec=5.0,
            )
            is not None
        ),
        lambda seen: seen,
        timeout_sec,
    )
    test_case.assertTrue(
        ready, f"Controller manager did not come up within {timeout_sec}s"
    )


def assert_nothing_active(
    test_case, node, message, settle_sec=DEFAULT_TIMEOUT_SEC, hold_sec=5.0
):
    """Wait for the manager to report nothing active, then confirm it stays that way.

    Both directions matter here: whatever the test just switched off needs a moment to land, and
    the spawner must not quietly bring anything back up while the e-stop is engaged. A single
    sample cannot tell "not yet started" from "already stopped again".
    """

    def probe():
        return (
            get_active_hardware(node),
            [c.name for c in get_controllers(node) if c.state == "active"],
        )

    poll_until(probe, lambda seen: not seen[0] and not seen[1], settle_sec)

    deadline = time.monotonic() + hold_sec
    while True:
        active_hw, active_ctrl = probe()
        test_case.assertEqual(
            active_hw, [], f"{message} Still-active hardware: {', '.join(active_hw)}"
        )
        test_case.assertEqual(
            active_ctrl,
            [],
            f"{message} Still-active controllers: {', '.join(active_ctrl)}",
        )
        if time.monotonic() >= deadline:
            return
        time.sleep(POLL_PERIOD_SEC)


def run_spawner_to_completion(test_case, cmd, timeout_sec=DEFAULT_TIMEOUT_SEC):
    """Run the spawner as a subprocess and return (returncode, stdout, stderr).

    On timeout the process is killed and whatever it printed is attached to the failure - a bare
    TimeoutExpired says nothing about where the spawner got stuck.
    """
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        stdout, stderr = process.communicate(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate()
        test_case.fail(
            f"Spawner did not exit within {timeout_sec}s.\n"
            f"stdout:\n{stdout.decode(errors='replace')}\n"
            f"stderr:\n{stderr.decode(errors='replace')}"
        )
    return (
        process.returncode,
        stdout.decode(errors="replace"),
        stderr.decode(errors="replace"),
    )
