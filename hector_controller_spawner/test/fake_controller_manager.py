#!/usr/bin/env python3
"""A controller_manager stand-in whose answers can be delayed, dropped or pre-seeded.

The real controller_manager always answers promptly and only ever holds the controllers the
spawner's own configuration names. Neither is guaranteed on a robot: right after a physical e-stop
the resource manager can hold its lock for seconds, and controllers loaded by another actor sit in
the same list. Tests drive those states from here.

Parameters:
  never_answer      list of service base names that receive the request and never reply
  answer_delay_sec  seconds to stall every service before replying
  drop_first_list   answer the first list_controllers call with nothing at all
  foreign_active    controllers reported active that no spawner config names
  hardware          hardware component names reported
  hardware_active   whether those components report ACTIVE
"""

import threading

import rclpy
from lifecycle_msgs.msg import State
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    LoadController,
    SetHardwareComponentState,
    SwitchController,
)


class FakeControllerManager(Node):
    def __init__(self):
        super().__init__("controller_manager")

        # An empty default is inferred as a byte array, and the declared type then refuses the
        # string list a test passes in, so these are left dynamically typed.
        string_array = ParameterDescriptor(dynamic_typing=True)

        self.never_answer = self.declare_parameter(
            "never_answer", [], string_array
        ).value
        self.answer_delay_sec = self.declare_parameter("answer_delay_sec", 0.0).value
        self.drop_first_list = self.declare_parameter("drop_first_list", False).value
        self.foreign_active = self.declare_parameter(
            "foreign_active", [], string_array
        ).value
        self.hardware = self.declare_parameter(
            "hardware", ["athena_arm_interface"], string_array
        ).value
        self.hardware_active = self.declare_parameter("hardware_active", True).value

        self._lock = threading.Lock()
        self._list_calls = 0
        # name -> state string, seeded with the controllers no spawner config knows about
        self._controllers = {name: "active" for name in self.foreign_active}
        self._stalled = []  # keeps never-answered requests alive

        self.create_service(
            ListHardwareComponents,
            "~/list_hardware_components",
            self._list_hardware_components,
        )
        self.create_service(
            ListControllers, "~/list_controllers", self._list_controllers
        )
        self.create_service(LoadController, "~/load_controller", self._load_controller)
        self.create_service(
            ConfigureController, "~/configure_controller", self._configure_controller
        )
        self.create_service(
            SwitchController, "~/switch_controller", self._switch_controller
        )
        self.create_service(
            SetHardwareComponentState,
            "~/set_hardware_component_state",
            self._set_hardware_component_state,
        )
        self.get_logger().info(
            f"fake controller_manager up: never_answer={self.never_answer} "
            f"delay={self.answer_delay_sec} foreign_active={self.foreign_active}"
        )

    def _gate(self, service_name):
        """Returns False when this call must never be answered."""
        if service_name in self.never_answer:
            self.get_logger().warn(f"swallowing a {service_name} request")
            return False
        if self.answer_delay_sec > 0.0:
            threading.Event().wait(self.answer_delay_sec)
        return True

    def _list_hardware_components(self, request, response):
        del request
        if not self._gate("list_hardware_components"):
            self._stalled.append(response)
            # Blocks this callback for good, which is what a controller_manager holding the
            # resource manager lock looks like from outside.
            threading.Event().wait()
        for name in self.hardware:
            component = HardwareComponentState()
            component.name = name
            component.type = "system"
            component.state.id = (
                State.PRIMARY_STATE_ACTIVE
                if self.hardware_active
                else State.PRIMARY_STATE_INACTIVE
            )
            component.state.label = "active" if self.hardware_active else "inactive"
            response.component.append(component)
        return response

    def _list_controllers(self, request, response):
        del request
        if not self._gate("list_controllers"):
            threading.Event().wait()
        with self._lock:
            self._list_calls += 1
            if self.drop_first_list and self._list_calls == 1:
                self.get_logger().warn(
                    "answering the first list_controllers with nothing"
                )
                return response
            for name, state in self._controllers.items():
                controller = ControllerState()
                controller.name = name
                controller.state = state
                controller.type = "fake/Controller"
                response.controller.append(controller)
        return response

    def _load_controller(self, request, response):
        if not self._gate("load_controller"):
            threading.Event().wait()
        with self._lock:
            # The real manager refuses to load a controller it already holds.
            if request.name in self._controllers:
                response.ok = False
            else:
                self._controllers[request.name] = "unconfigured"
                response.ok = True
        return response

    def _configure_controller(self, request, response):
        if not self._gate("configure_controller"):
            threading.Event().wait()
        with self._lock:
            if request.name in self._controllers:
                self._controllers[request.name] = "inactive"
                response.ok = True
            else:
                response.ok = False
        return response

    def _switch_controller(self, request, response):
        if not self._gate("switch_controller"):
            threading.Event().wait()
        with self._lock:
            for name in request.deactivate_controllers:
                if name in self._controllers:
                    self._controllers[name] = "inactive"
            for name in request.activate_controllers:
                if name in self._controllers:
                    self._controllers[name] = "active"
            response.ok = True
        return response

    def _set_hardware_component_state(self, request, response):
        if not self._gate("set_hardware_component_state"):
            threading.Event().wait()
        response.ok = True
        response.state = request.target_state
        return response


def main():
    rclpy.init()
    node = FakeControllerManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
