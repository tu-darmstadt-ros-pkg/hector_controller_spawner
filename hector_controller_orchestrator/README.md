# hector_controller_orchestrator

The `hector_controller_orchestrator` package is a thin C++ client of the controller manager's
`switch_controller` service. A switch is one FORCE_AUTO request that names the controllers to
activate. The controller manager (`hector_controller_manager`) adds the providers they chain into,
stops what conflicts with them, and answers with its reason when it refuses, for example a
protected controller in the way or an engaged e-stop. The controller states come from the manager's
latest `~/activity` message.

## Usage

### Class Integration (as a Node Member)

Pass your node and optionally the controller manager name (defaults to `controller_manager`). The orchestrator does not keep the node alive, so the node can own it. It works as a plain member, a `shared_ptr` or on the stack.

```cpp
#include <controller_orchestrator/controller_orchestrator.hpp>

class MyNode : public rclcpp::Node {
public:
    MyNode() : rclcpp::Node("my_node") {}

    // shared_from_this() is not available in the constructor
    void init() {
        orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
                shared_from_this());
    }

private:
    std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
};
```

When the destructor returns, no callback of the orchestrator is running or will run. It waits for callbacks in progress on other threads, so a callback may capture `this` of the owner. A callback may also destroy the orchestrator itself.

### ⚠️ Important: Executor Requirements for Blocking Functions

`smartSwitchController` is blocking and waits for the service response. It **cannot be called from within a callback** if your node uses a `SingleThreadedExecutor`, as this will cause a deadlock (the executor cannot process the service response while waiting in the callback).

**Solutions:**
- Use `MultiThreadedExecutor` or `StaticMultiThreadedExecutor` for your node
- Use the asynchronous variants (e.g., `smartSwitchControllerAsync`) which are safe for single-threaded executors
- Call blocking functions from a separate thread

**Safe for SingleThreadedExecutor:** `smartSwitchControllerAsync`, `isControllerActive`, `areControllersActive`


### Initialization

Include the header and initialize the `ControllerOrchestrator` within your node:

```cpp
#include <controller_orchestrator/controller_orchestrator.hpp>

// Inside your node class
auto orchestrator = std::make_shared<controller_orchestrator::ControllerOrchestrator>(shared_from_this());
```

### Smart Switching

The primary function of this library is `smartSwitchController`. You name the controllers you *want* to be active, and the controller manager figures out the rest in one FORCE_AUTO switch.

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

The callback fires exactly once, unless the orchestrator is destroyed first, with the controller
manager's answer and message. `timeout_s` (default 5 s) bounds the whole switch: a service not yet
discovered is waited for within it, the request carries what is left as its timeout, and if no
answer arrives within one second more, the callback gets `false`, also when the service never
appeared.

The blocking call waits up to `timeout_s` for the service and then up to `timeout_s` plus one
second for the answer.

A node destroyed while a switch still waits for the service never runs that switch's timers again.
The pending switch, its timers and its callback are then never freed, a few hundred bytes per such
switch at shutdown, and the callback never runs.

### When a switch fails

`false` does not always mean that nothing changed. A switch the controller manager refuses before
it starts changes nothing. A switch that reaches its deadline keeps what it already stopped, and a
start still under way can reach the hardware later (hcm's "Deadlines" rule). Without an answer the
outcome is unknown. Read the states from `isControllerActive` or `areControllersActive` before you
retry.

### Controller States

`isControllerActive` and `areControllersActive` answer from the latest `~/activity` message, which
the controller manager latches, so an orchestrator built after bring-up knows the states at once.

## Dependencies

- `rclcpp`
- `controller_manager_msgs`
