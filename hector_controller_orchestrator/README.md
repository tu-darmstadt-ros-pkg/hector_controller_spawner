# hector_controller_orchestrator

The `hector_controller_orchestrator` package provides a C++ library to simplify complex controller management tasks in ROS 2 `ros2_control` systems. It specializes in handling controller switching while automatically resolving resource conflicts and dependency chains.

## Key Features

- **Smart Switching**: Automatically identifies and stops active controllers that conflict with requested ones.
- **Dependency Resolution**: Handles chained controllers by ensuring that when a parent controller is stopped, its dependents are also stopped, and vice versa for starting.
- **Async & Sync API**: Offers both blocking (synchronous) and non-blocking (asynchronous) methods for integration into various node execution models.
- **State Caching**: Maintains an internal cache of controller states (updated via `controller_manager/activity` or manual refresh) to minimize service call latency.

## Usage

### ⚠️ Important: Executor Requirements for Blocking Functions

**Most functions in this library are blocking** and internally wait for service responses. They **cannot be called from within a callback** if your node uses a `SingleThreadedExecutor`, as this will cause a deadlock (the executor cannot process the service response while waiting in the callback).

**Solutions:**
- Use `MultiThreadedExecutor` or `StaticMultiThreadedExecutor` for your node
- Use the asynchronous variants (e.g., `smartSwitchControllerAsync`) which are safe for single-threaded executors
- Call blocking functions from a separate thread

**Safe for SingleThreadedExecutor:** `smartSwitchControllerAsync`, `refreshControllerStatesAsync`


### Initialization

Include the header and initialize the `ControllerOrchestrator` within your node:

```cpp
#include <controller_orchestrator/controller_orchestrator.hpp>

// Inside your node class
auto orchestrator = std::make_shared<controller_orchestrator::ControllerOrchestrator>(shared_from_this());
```

### Smart Switching

The primary function of this library is `smartSwitchController`. You specify which controllers you *want* to be active, and the orchestrator figures out the rest.

#### Synchronous (Blocking)
```cpp
std::vector<std::string> target_controllers = {"joint_trajectory_controller"};
bool success = orchestrator->smartSwitchController(target_controllers);
```

**⚠️ Warning**: This is a blocking call. Do not use in SingleThreadedExecutor callbacks.

#### Asynchronous (Non-Blocking)
```cpp
orchestrator->smartSwitchControllerAsync(target_controllers,
    [](bool success, const std::string &message) {
        if (success) {
            RCLCPP_INFO(rclcpp::get_logger("my_node"), "Switch successful!");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("my_node"), "Switch failed: %s", message.c_str());
        }
    });
```

### How "Smart Switch" Works

1. **Dependency Expansion**: If you request controller A, and A depends on B (chained), the orchestrator automatically adds B to the activation list.
2. **Conflict Detection**: It checks which resources (joints/interfaces) the requested controllers need. If an currently active controller C uses any of those resources, C is marked for deactivation.
3. **Ripple Deactivation**: If C is marked for deactivation, any controller D that depends on C is also marked for deactivation.
4. **Topological Ordering**: The orchestrator calculates the correct sequence for stopping and starting. For example, it stops dependents *before* dependencies and starts dependencies *before* dependents.

### Manual Controller Activation/Deactivation

For direct control without the smart switching logic, you can use:

#### Activate Controllers
```cpp
std::vector<std::string> controllers = {"controller1", "controller2"};
bool success = orchestrator->activateControllers(controllers, timeout_s);
```

#### Deactivate Controllers
```cpp
std::vector<std::string> controllers = {"controller1", "controller2"};
bool success = orchestrator->deactivateControllers(controllers, timeout_s);
```

**⚠️ Warning**: These are blocking calls. Do not use in SingleThreadedExecutor callbacks.

**Note**: These methods do not handle dependencies or conflicts—they simply activate or deactivate the specified controllers in the order provided.

### Hardware Interface Management

#### Get Active Controllers for Hardware Interface
Find all active controllers that claim a specific hardware interface:

```cpp
std::string hardware_interface = "joint1";
std::vector<std::string> active_controllers =
    orchestrator->getActiveControllerOfHardwareInterface(hardware_interface, timeout_s);

for (const auto& ctrl : active_controllers) {
    RCLCPP_INFO(rclcpp::get_logger("my_node"),
                "Controller %s is using %s", ctrl.c_str(), hardware_interface.c_str());
}
```

**⚠️ Warning**: This is a blocking call. Do not use in SingleThreadedExecutor callbacks.

This is useful for determining which controllers need to be stopped before performing maintenance or switching to a different control strategy for a specific joint.

#### Unload Controllers for a Joint
Automatically deactivate all controllers claiming a specific joint, including their dependents:

```cpp
std::string joint_name = "shoulder_pan_joint";
bool success = orchestrator->unloadControllersOfJoint(joint_name, timeout_s);
```

**⚠️ Warning**: This is a blocking call. Do not use in SingleThreadedExecutor callbacks.

This function:
1. Finds all active controllers that claim the specified joint
2. Identifies any controllers that depend on those controllers (upstream dependents)
3. Deactivates them in the correct topological order to avoid breaking dependencies

This is particularly useful when you need to free up a joint for emergency stops, maintenance, or switching to a completely different control mode.

## Dependencies

- `rclcpp`
- `controller_manager_msgs`
- `lifecycle_msgs`
