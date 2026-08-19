#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP

#include <memory>
#include <mutex>
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
   * Conflicting controllers are those that claim the same hardware resources or are part of a
   * dependency chain that must be stopped. This call is blocking.
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
   * Analyzes dependencies and conflicts, then performs the switch operation non-blockingly.
   *
   * @param activate_controllers List of controllers to activate.
   * @param callback Function to call upon completion with (success, message).
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

  // ============================================================================
  // Controller Chain Analysis
  // ============================================================================

  /**
   * @brief Builds forward and reverse chain connection maps from controller information.
   *
   * Forward map: controller -> list of controllers it chains to
   * Reverse map: controller -> list of controllers that chain to it
   *
   * @param controllers List of controller states from list_controllers response
   * @param forward_connections Output: forward chain connections
   * @param reverse_connections Output: reverse chain connections
   */
  void buildChainConnectionMaps(
      const std::vector<controller_manager_msgs::msg::ControllerState> &controllers,
      std::unordered_map<std::string, std::vector<std::string>> &forward_connections,
      std::unordered_map<std::string, std::vector<std::string>> &reverse_connections ) const;

  /**
   * @brief Get all downstream dependencies of the given controllers.
   * @param seed_controllers List of controller names to start from
   * @param forward_connections Forward chain connection map (use buildChainConnectionMaps to create)
   * @return Set of all downstream dependent controller names
   */
  std::unordered_set<std::string> getDownstreamDependencies(
      const std::vector<std::string> &seed_controllers,
      const std::unordered_map<std::string, std::vector<std::string>> &forward_connections ) const;

  /**
   * @brief Get all upstream dependents of the given controllers.
   * @param seed_controllers List of controller names to start from
   * @param reverse_connections Reverse chain connection map (use buildChainConnectionMaps to create)
   * @return Set of all upstream dependent controller names
   */
  std::unordered_set<std::string> getUpstreamDependents(
      const std::vector<std::string> &seed_controllers,
      const std::unordered_map<std::string, std::vector<std::string>> &reverse_connections ) const;

  /**
   * @brief Performs topological sort on active controllers in a chain.
   *
   * Returns controllers ordered such that dependencies are satisfied:
   * - Controllers earlier in the list should be deactivated first
   * - Controllers later in the list should be activated first
   *
   * @param active_controllers Set of active controllers to sort
   * @param forward_connections Forward chain connection map (use buildChainConnectionMaps to create)
   * @return Topologically sorted list of controllers, or empty vector if cycle detected
   */
  std::vector<std::string> topologicalSortControllers(
      const std::unordered_set<std::string> &active_controllers,
      const std::unordered_map<std::string, std::vector<std::string>> &forward_connections ) const;

  /**
   * @brief Builds a map of controller names to their claimed command interfaces.
   * @param controllers List of controller states from list_controllers response
   * @return Map from controller name to vector of claimed interface names
   */
  std::unordered_map<std::string, std::vector<std::string>> buildControllerResourceMap(
      const std::vector<controller_manager_msgs::msg::ControllerState> &controllers ) const;

  // ============================================================================
  // Smart Switch Analysis
  // ============================================================================

  /**
   * @brief Analyzes which controllers to activate and deactivate for a smart switch.
   *
   * This method:
   * 1. Validates requested controllers exist
   * 2. Removes already-active controllers from activation list
   * 3. Adds all chain-connected controllers to activation list
   * 4. Identifies resource conflicts with currently active controllers
   * 5. Adds conflicting controllers and their dependents to deactivation list
   *
   * @param to_activate Input/Output: controllers to activate (modified in place)
   * @param to_deactivate Output: controllers that must be deactivated
   * @param res Response from list_controllers service
   * @return true if analysis succeeded, false on error
   */
  bool smartSwitchControllerAnalysis(
      std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
      const controller_manager_msgs::srv::ListControllers_Response &res ) const;

  // ============================================================================
  // Async Operations
  // ============================================================================

  /**
   * @brief Recursively activates controllers in reverse order (last to first).
   *
   * This is part of the async switching mechanism. Controllers are activated
   * one at a time, starting from the end of the list.
   *
   * @param controllers_to_activate Shared pointer to list of controllers
   * @param index Current index to activate (decrements with each recursion)
   * @param callback Callback to invoke when all activations complete or on error
   */
  void recursiveActivateControllers(
      std::shared_ptr<std::vector<std::string>> controllers_to_activate, size_t index,
      const std::function<void( bool success, const std::string &message )> &callback ) const;

  /**
   * @brief Recursively deactivates controllers in reverse order, then activates.
   *
   * This is part of the async switching mechanism. Controllers are deactivated
   * one at a time from last to first, then activation begins.
   *
   * @param controllers_to_activate Shared pointer to list of controllers to activate after deactivation
   * @param controllers_to_deactivate Shared pointer to list of controllers to deactivate
   * @param index Current index to deactivate (decrements with each recursion)
   * @param callback Callback to invoke when all operations complete or on error
   */
  void recursiveDeactivateControllers(
      std::shared_ptr<std::vector<std::string>> controllers_to_activate,
      std::shared_ptr<std::vector<std::string>> controllers_to_deactivate, size_t index,
      const std::function<void( bool success, const std::string &message )> &callback ) const;

  /**
   * @brief Replace the whole state cache in one step.
   *
   * Takes the new map by value so the caller builds it outside the lock: the critical section is
   * then a move rather than a clear plus one allocation per controller, and no reader can observe
   * a half-rebuilt cache.
   */
  void replaceControllerStates( std::unordered_map<std::string, std::string> states ) const;

  /**
   * @brief Whether @p controller_name is cached as active.
   * @pre controller_states_mutex_ is held.
   *
   * Split out from the public accessors because std::shared_lock is not recursive: a public entry
   * point calling another would deadlock, and a name carrying the precondition is what stops
   * someone writing that.
   */
  bool isActiveLocked( const std::string &controller_name ) const;

  rclcpp::Node::SharedPtr node_;
  std::string controller_manager_name_;

  /// Serves the activity subscription. Mutually exclusive: the callback replaces the cache
  /// wholesale, so dispatching it to several threads at once only makes them queue on the mutex.
  rclcpp::CallbackGroup::SharedPtr cache_callback_group_;
  /// Serves the service clients. Reentrant by necessity - a blocking call parks one thread of this
  /// group waiting for a response another thread of the same group has to deliver.
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
      list_hardware_components_client_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
      activity_subscription_;

  /// Shared, because reads dominate: areControllersActive() is on the e-stop path and asks about
  /// several controllers at once, while writes happen only when the activity topic ticks.
  mutable std::shared_mutex controller_states_mutex_;
  mutable std::unordered_map<std::string, std::string> controller_states_;
};

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
