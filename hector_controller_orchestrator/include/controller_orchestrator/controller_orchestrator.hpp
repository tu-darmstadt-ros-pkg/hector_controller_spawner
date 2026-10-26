#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>

#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace controller_orchestrator
{

/**
 * @class ControllerOrchestrator
 * @brief Orchestrates controller switching with dependency and conflict resolution.
 *
 * This class provides high-level functions to manage controllers in a ROS 2 system.
 * It handles:
 * - Smart switching: Automatically deactivates conflicting controllers and their dependents.
 * - Chain management: Resolves downstream dependencies and upstream dependents for chained controllers.
 * - Async/Sync operations: Provides both blocking and non-blocking interfaces for common tasks.
 * - State caching: Maintains a local cache of controller states for faster access.
 *
 * Responses are delivered by the executor spinning the node. The blocking calls wait for one, so
 * they need another thread of that executor to be free.
 */
class ControllerOrchestrator
{
public:
  using Callback = std::function<void( bool success, const std::string &message )>;

  /**
   * @brief Construct a new Controller Orchestrator object.
   * @param node The node to talk to the controller manager through. Not kept alive by the
   *        orchestrator, so the node may own it.
   * @param controller_manager_name Name of the controller manager node.
   */
  explicit ControllerOrchestrator(
      const rclcpp::Node::SharedPtr &node,
      const std::string &controller_manager_name = "controller_manager" );

  /// Waits for callbacks of this orchestrator running on other threads. None runs afterwards.
  ~ControllerOrchestrator();

  ControllerOrchestrator( const ControllerOrchestrator & ) = delete;
  ControllerOrchestrator &operator=( const ControllerOrchestrator & ) = delete;

  /**
   * @brief Tries to activate the given controllers, deactivating all conflicting controllers.
   *
   * Conflicting controllers are those that claim the same hardware resources or are part of a
   * dependency chain that must be stopped. This call is blocking.
   *
   * @param activate_controllers List of controllers to activate.
   * @param timeout_s Timeout in seconds for the operation (default 10s).
   * @return true if the switch operation was successful.
   */
  bool smartSwitchController( const std::vector<std::string> &activate_controllers,
                              int timeout_s = 10 ) const;

  /**
   * @brief Asynchronous version of smartSwitchController.
   *
   * Analyzes dependencies and conflicts, then performs the switch operation non-blockingly.
   *
   * @param activate_controllers List of controllers to activate.
   * @param callback Function to call upon completion with (success, message). Invoked exactly once
   *        unless the orchestrator is destroyed first, including when a newer request takes the
   *        controllers over before this chain finishes. A newer request answers this one with false
   *        straight away, and so does a step the controller manager leaves unanswered for
   *        @p step_timeout_s.
   * @param step_timeout_s How long each service call of the chain may go unanswered.
   */
  void smartSwitchControllerAsync( const std::vector<std::string> &activate_controllers,
                                   const Callback &callback, int step_timeout_s = 5 ) const;

  /**
   * @brief Get a list of currently active controllers that claim a specific hardware interface.
   * @param hardware_interface Name of the hardware component (e.g., "athena_arm_interface").
   * @param timeout_s Timeout in seconds for the whole operation.
   * @return Active controllers claiming a command interface of @p hardware_interface. Empty if
   *         none is active. nullopt if the services did not answer or the component is not loaded.
   */
  std::optional<std::vector<std::string>>
  getActiveControllerOfHardwareInterface( const std::string &hardware_interface,
                                          int timeout_s = 10 ) const;

  /**
   * @brief Deactivate the given list of controllers.
   *
   * This is a "dumb" deactivation and does not check for dependency violations.
   * @param controllers_to_deactivate Controllers to stop.
   * @param timeout_s Timeout in seconds.
   * @return true if the deactivation was successful.
   */
  bool deactivateControllers( const std::vector<std::string> &controllers_to_deactivate,
                              int timeout_s = 10 ) const;

  /**
   * @brief Activate the given list of controllers.
   *
   * This is a "dumb" activation and does not resolve conflicts or dependencies.
   * @param controllers_to_activate Controllers to start.
   * @param timeout_s Timeout in seconds.
   * @return true if the activation was successful.
   */
  bool activateControllers( const std::vector<std::string> &controllers_to_activate,
                            int timeout_s = 10 ) const;

  /**
   * @brief Deactivate all active controllers claiming any interface of a specific joint, together
   *        with the controllers chained to them.
   * @param joint_name Name of the joint.
   * @param timeout_s Timeout in seconds for the whole operation.
   * @return true if the controllers were deactivated successfully.
   */
  bool deactivateControllersOfJoint( const std::string &joint_name, int timeout_s = 10 ) const;

  [[deprecated( "Use deactivateControllersOfJoint, which does the same." )]]
  bool unloadControllersOfJoint( const std::string &joint_name, int timeout_s = 10 ) const
  {
    return deactivateControllersOfJoint( joint_name, timeout_s );
  }

  /**
   * @brief Query the controller manager and update the local state cache (blocking).
   * @param timeout_s Timeout in seconds.
   * @return true if the refresh was successful.
   */
  bool refreshControllerStates( int timeout_s = 10 ) const;

  /**
   * @brief Query the controller manager and update the local state cache (asynchronous).
   * @param callback Function to call upon completion. Invoked exactly once unless the
   *        orchestrator is destroyed first, with false if no answer arrives within @p timeout_s.
   * @param timeout_s Timeout in seconds.
   */
  void refreshControllerStatesAsync( const Callback &callback, int timeout_s = 10 ) const;

  /**
   * @brief Checks if the controllers is currently active.
   * @param controller_name controller names to check
   * @return true if the controller is active, false otherwise
   */
  bool isControllerActive( const std::string &controller_name ) const;

  /**
   * @brief Checks if all given controllers are currently active.
   * @param controller_names List of controller names to check
   * @return true if all controllers are active, false otherwise
   */
  bool areControllersActive( const std::vector<std::string> &controller_names ) const;

private:
  struct Core;
  /// Shared with the callbacks in flight, which hold it weakly and so cannot outlive its state.
  std::shared_ptr<Core> core_;
};

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
