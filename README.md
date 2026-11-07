# Hector Controller Tools
![Lint](https://github.com/tu-darmstadt-ros-pkg/hector_controller_spawner/actions/workflows/lint_build_test.yaml/badge.svg)

Toolkit containing:
- **Hector Controller Spawner (Multispawner):** One-shot launcher that loads/activates hardware and controllers with chaining, retries, and optional e-stop gating.
- **Hector Controller Orchestrator:** C++ client that activates controllers together with their chain and stops what conflicts, on `hector_controller_manager` or stock ros2_control. See [hector_controller_orchestrator/README.md](hector_controller_orchestrator/README.md).
---
## Hector Controller Spawner – **Multispawner**

**ROS2 Hardware & Controller Launcher for `ros2_control`**

**Multispawner** is a minimal ROS2 node that launches an entire `ros2_control` setup in a single coordinated pass.
It robustly manages hardware interfaces and controllers, ensuring everything is loaded, activated (if required), and
ready to go with minimal configuration.

## 🚀 Features

* **Wait-for-safety (e-stop):** Optionally blocks on an `std_msgs/Bool` topic (e.g., emergency stop) before starting.
  Useful when motors can't be activated while safety is engaged.
* **Re-activation:** Automatically reactivates hardware interfaces and controllers after releasing the e-stop (if they
  became inactive).
* **Controller Manager Synchronization:** Waits for the controller manager to become available before proceeding.
* **Hardware-first strategy:** Ensures all listed hardware interfaces are both *loaded* and *activated* (with automatic
  retries on failure).
* **Intelligent controller loading:** Skips controllers already present - only loads and activates what’s missing.
* **Automatic chaining:** Automatically detects and starts *chained controllers* together - no additional config
  required.
* **Single-node simplicity:** No need to spawn one spawner per controller - Multispawner handles everything.
* **Robust retry logic:** Retries failed hardware/controller activations with configurable delays.


## 🔧 Key Parameters

| Name                             | Type       | Default | Description                                                               |
|----------------------------------|------------|---------|---------------------------------------------------------------------------|
| `hardware_interfaces`            | `string[]` | -       | Ordered list of hardware interface names to activate.                     |
| `controllers`                    | `string[]` | -       | Ordered list of controller names to load and manage.                      |
| `<ctrl>.activate`                | `bool`     | `true`  | Should the controller be activated after loading?                         |
| `retry_delay`                    | `double`   | `5.0`   | Delay (in seconds) between retry attempts.                                |
| `start_delay`                     | `double`   | `0.0`   | Initial delay (in seconds) before starting the spawner process.           |
| `service_call_timeout_ms`         | `int`      | `5000`  | Timeout (in milliseconds) for service calls to the controller manager.    |
| `load_groups_one_by_one`         | `bool`     | `true`  | Load controllers in groups (chained controllers together) or all at once. |
| `estop_topic`                    | `string`   | `""`    | Topic to wait on (false ⇒ proceed). Leave empty to disable e-stop gating. |
| `restart_after_estop_deactivation` | `bool`     | `true`  | Restart hardware and controllers after e-stop deactivation                |

📄 See [`athena.yaml`](config/athena.yaml) for a complete configuration example.


## 🧪 Example Usage

```bash
ros2 launch hector_controller_spawner hector_controller_spawner_launch.yml
```

* Include **only once** in your launch setup.
* No need for individual `spawner` calls per controller.


---
# Hector Controller Orchestrator
C++ client of the controller manager's `switch_controller` service. You name the controllers you
want active, and the controllers they chain into are started and whatever conflicts is stopped.

- Works with `hector_controller_manager` and with stock ros2_control. A manager that lists `force_auto` in its `capabilities` parameter gets one FORCE_AUTO request and resolves everything itself. Any other gets one STRICT request that the orchestrator planned from `list_controllers`.
- Provides synchronous and asynchronous APIs; the async variant is safe for single-threaded executors.
- Reports controller states from the manager's latest `~/activity` message.

Read the full guide and examples: [hector_controller_orchestrator/README.md](hector_controller_orchestrator/README.md)
