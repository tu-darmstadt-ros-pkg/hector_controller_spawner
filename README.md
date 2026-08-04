# Hector Controller Tools
![Lint](https://github.com/tu-darmstadt-ros-pkg/hector_controller_spawner/actions/workflows/lint_build_test.yaml/badge.svg)

Toolkit containing:
- **Hector Controller Spawner (Multispawner):** One-shot launcher that loads/activates hardware and controllers with chaining, retries, and optional e-stop gating.
- **Hector Controller Orchestrator:** C++ helper library that performs dependency-aware, resource-safe controller switching. See [hector_controller_orchestrator/README.md](hector_controller_orchestrator/README.md).
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
* **Automatic chaining:** Chain dependencies and resource conflicts are resolved by the controller manager via the
  `FORCE_AUTO` strictness of `switch_controller`, the whole desired state is applied in a single update iteration,
  no additional config required.
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
| `srv_call_timeout_ms`             | `int`      | `5000`  | Timeout (in milliseconds) for service calls to the controller manager.    |
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
C++ utility library that makes controller switching safe and predictable in `ros2_control` systems. It:
- Resolves chained dependencies and resource conflicts through the controller manager's `FORCE_AUTO` strictness.
- Provides synchronous and asynchronous APIs; async variants are safe for single-threaded executors.
- Caches controller state from `controller_manager/activity` to avoid repeated service calls.
- Exposes helper utilities to query active controllers for a hardware interface or unload all controllers claiming a joint.

Read the full guide and examples: [hector_controller_orchestrator/README.md](hector_controller_orchestrator/README.md)
