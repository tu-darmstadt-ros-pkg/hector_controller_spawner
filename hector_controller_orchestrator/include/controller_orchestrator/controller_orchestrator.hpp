#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP

#include <memory>
#include <mutex>
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
 * - Smart switching: Delegates dependency expansion and conflict resolution to the controller
 *   manager via the "FORCE_AUTO" strictness of the switch_controller service, so the whole switch
 *   is applied atomically in a single update iteration.
 * - Async/Sync operations: Provides both blocking and non-blocking interfaces for common tasks.
 * - State caching: Maintains a local cache of controller states for faster access.
 *
 * @note "FORCE_AUTO" requires a controller manager that implements it. Older versions accept the
 * value but silently fall back to "BEST_EFFORT", in which case conflicting controllers are not
 * deactivated.
 */
class ControllerOrchestrator
{
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;

public:
  /**
   * @brief Construct a new Controller Orchestrator object.
   * @param node Shared pointer to the parent node.
   * @param controller_manager_name Name of the controller manager node (default: "controller_manager").
   */
  explicit ControllerOrchestrator(
      const rclcpp::Node::SharedPtr &node,
      const std::string &controller_manager_name = "controller_manager" );

  /**
   * @brief Tries to activate the given controllers, deactivating all conflicting controllers.
   *
   * Issues a single "FORCE_AUTO" switch: the controller manager pulls in the chain dependencies of
   * the requested controllers and deactivates every active controller claiming a conflicting
   * command interface, together with everything depending on it. All controllers in the resolved
   * chain must already be configured ('inactive' state) - the switch does not configure them.
   * This call is blocking.
   *
   * @param activate_controllers List of controllers to activate.
   * @param timeout_s Timeout in seconds for the operation (default 2s).
   * @return true if the switch operation was successful.
   */
  bool smartSwitchController( std::vector<std::string> &activate_controllers,
                              int timeout_s = 2 ) const;

  /**
   * @brief Asynchronous version of smartSwitchController.
   *
   * @param activate_controllers List of controllers to activate.
   * @param callback Function to call upon completion with (success, message). On failure the
   * message is the one reported by the controller manager.
   */
  void smartSwitchControllerAsync(
      const std::vector<std::string> &activate_controllers,
      const std::function<void( bool success, const std::string &message )> &callback ) const;

  /**
   * @brief Get a list of currently active controllers that claim a specific hardware interface.
   * @param hardware_interface Name of the hardware interface (e.g., "joint1").
   * @param timeout_s Timeout in seconds for the operation.
   * @return Vector of active controller names.
   */
  std::vector<std::string> getActiveControllerOfHardwareInterface( const std::string &hardware_interface,
                                                                   int timeout_s = 2 ) const;

  /**
   * @brief Deactivate the given list of controllers.
   *
   * This is a "dumb" deactivation and does not check for dependency violations.
   * @param controllers_to_deactivate Controllers to stop.
   * @param timeout_s Timeout in seconds.
   * @return true if the deactivation was successful.
   */
  bool deactivateControllers( const std::vector<std::string> &controllers_to_deactivate,
                              int timeout_s = 2 ) const;

  /**
   * @brief Activate the given list of controllers.
   *
   * This is a "dumb" activation and does not resolve conflicts or dependencies.
   * @param controllers_to_activate Controllers to start.
   * @param timeout_s Timeout in seconds.
   * @return true if the activation was successful.
   */
  bool activateControllers( const std::vector<std::string> &controllers_to_activate,
                            int timeout_s = 2 ) const;

  /**
   * @brief Unload all active controllers claiming any interface of a specific joint.
   * @param joint_name Name of the joint.
   * @param timeout_s Timeout in seconds.
   * @return true if the controllers were deactivated successfully.
   */
  bool unloadControllersOfJoint( const std::string &joint_name, int timeout_s = 2 );

  /**
   * @brief Query the controller manager and update the local state cache (blocking).
   * @param timeout_s Timeout in seconds.
   * @return true if the refresh was successful.
   */
  bool refreshControllerStates( int timeout_s = 2 ) const;

  /**
   * @brief Query the controller manager and update the local state cache (asynchronous).
   * @param callback Function to call upon completion.
   * @param timeout_s Timeout in seconds.
   */
  void refreshControllerStatesAsync(
      const std::function<void( bool success, const std::string &message )> &callback,
      int timeout_s = 2 ) const;
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
  // ============================================================================
  // State Management
  // ============================================================================

  /**
   * @brief Updates the internal controller states cache from a list_controllers response.
   * @param res Response from the list_controllers service
   */
  void updateControllerStatesFromList(
      const controller_manager_msgs::srv::ListControllers_Response &res ) const;

  rclcpp::Node::SharedPtr node_;
  std::string controller_manager_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
      list_hardware_components_client_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
      activity_subscription_;
  mutable std::mutex controller_states_mutex_;
  mutable std::unordered_map<std::string, std::string> controller_states_;
};

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
