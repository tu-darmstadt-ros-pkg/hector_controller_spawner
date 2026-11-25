# hector_controller_orchestrator

The `hector_controller_orchestrator` package is a C++ client of the controller manager's
`switch_controller` service. You name the controllers you want active. The controllers they chain
into are started with them, and every active controller that claims an interface they need is
stopped, together with the controllers chained into it. The controller states come from the
manager's latest `~/activity` message.

## Supported controller managers

Both `hector_controller_manager` and the stock ros2_control controller manager work. Before the
first switch the orchestrator reads the manager's `capabilities` parameter through
`<manager>/get_parameters` and logs which mode it uses.

- **`capabilities` contains `force_auto`** (hector_controller_manager). A switch is one FORCE_AUTO
  request. The manager resolves chains and conflicts and answers with its reason when it refuses,
  for example a protected controller in the way or an engaged e-stop.
- **Anything else, or no such parameter** (stock ros2_control, which runs FORCE_AUTO as
  BEST_EFFORT). The orchestrator calls `list_controllers`, plans the activations and deactivations
  itself and sends them as one STRICT request, which the manager checks as a whole and refuses
  if any part fails.

The parameter is read again when a restarted manager publishes its activity.

On stock ros2_control a few things are worth knowing:

- A requested controller that is not loaded or not configured fails the switch before anything is
  sent.
- The plan comes from one listing. If the manager changes in between, for example through another
  switch still in flight, the STRICT request is refused and the switch fails. Retry it if that
  matters.
- A requested controller that is already active is not sent again. If someone stops it between the
  listing and the switch, the switch can still answer ok.

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

The primary function of this library is `smartSwitchController`. You name the controllers you *want* to be active, and the rest is figured out as described above.

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
manager's answer and message, or with the reason the orchestrator refused. `timeout_s` (default
5 s) bounds the whole switch. Services not yet discovered are waited for within it, the switch
request carries what is left as its timeout, and if no answer arrives within one second more, the
callback gets `false`, also when the services never appeared.

The blocking call runs the same switch and waits for its answer.

A node destroyed while a switch still waits for the service never runs that switch's timers again.
The pending switch, its timers and its callback are then never freed, a few hundred bytes per such
switch at shutdown, and the callback never runs.

### When a switch fails

`false` does not always mean that nothing changed. A switch the controller manager refuses before
it starts changes nothing. A switch that reaches its deadline keeps what it already stopped, and a
start still under way can reach the hardware later. Without an answer the outcome is unknown.
Read the states from `isControllerActive` or `areControllersActive` before you retry.

### Controller States

`isControllerActive` and `areControllersActive` answer from the latest `~/activity` message, which
the controller manager latches, so an orchestrator built after bring-up knows the states at once.

## Dependencies

- `rclcpp`
- `rcl_interfaces`
- `controller_manager_msgs`
