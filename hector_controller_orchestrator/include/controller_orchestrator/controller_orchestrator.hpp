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

class ControllerOrchestrator
{
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;

public:
  explicit ControllerOrchestrator(
      const rclcpp::Node::SharedPtr &node,
      const std::string &controller_manager_name = "controller_manager" );

  /**
   * Tries to activate the given controllers, deactivating all conflicting controllers (all
   * controllers that are currently active and claim the same resources).
   *
   * Can be replaced with strictness FORCE_AUTO, after implementing
   * @param activate_controllers list of controllers to activate
   * @param timeout_s timeout in seconds for the operation, defaults to 2.0
   * of the controllers to be activated is already active.
   * @return
   */
  bool smartSwitchController( std::vector<std::string> &activate_controllers, int timeout_s = 2,
                              bool refresh_ctrl_status = false ) const;

  void smartSwitchControllerAsync(
      const std::vector<std::string> &activate_controllers,
      const std::function<void( bool success, const std::string &message )> &callback,
      bool refresh_ctrl_status = false ) const;

  std::vector<std::string> getActiveControllerOfHardwareInterface( const std::string &hardware_interface,
                                                                   int timeout_s = 2 ) const;

  /**
   * @brief Deactivate the given controllers. Does not check for conflicts (e.g. if dependent controllers must be deactivated first).
   * @param controllers_to_deactivate  controllers to deactivate
   * @param timeout_s
   * @return
   */
  bool deactivateControllers( const std::vector<std::string> &controllers_to_deactivate,
                              int timeout_s = 2 ) const;

  bool activateControllers( const std::vector<std::string> &controllers_to_activate,
                            int timeout_s = 2 ) const;

  /*bool activateControllersOfHardwareInterface( const std::string &hardware_interface,
                                               const std::vector<std::string>
     &controllers_to_activate, int timeout_s = 2 );*/
  bool unloadControllersOfJoint( const std::string &joint_name, int timeout_s = 2 );
  bool refreshControllerStates( int timeout_s = 2 ) const;

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

  /**
   * @brief Checks if all given controllers are currently active.
   * @param controllers List of controller names to check
   * @return true if all controllers are active, false otherwise
   */
  bool areControllersActive( const std::vector<std::string> &controllers ) const;

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

  std::unordered_set<std::string> getDownstreamDependencies(
      const std::vector<std::string> &seed_controllers,
      const std::unordered_map<std::string, std::vector<std::string>> &forward_connections ) const;

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
   * @param forward_connections Forward chain connection map
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
