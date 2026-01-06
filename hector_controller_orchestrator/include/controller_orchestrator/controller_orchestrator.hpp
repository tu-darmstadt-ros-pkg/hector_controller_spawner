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
  void updateControllerStatesFromList(
      const controller_manager_msgs::srv::ListControllers_Response &res ) const;
  bool areControllersActive( const std::vector<std::string> &controllers ) const;
  bool smartSwitchControllerAnalysis(
      std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
      const controller_manager_msgs::srv::ListControllers_Response &req ) const;
  void recursiveActivateControllers(
      std::shared_ptr<std::vector<std::string>> controllers_to_activate, size_t index,
      const std::function<void( bool success, const std::string &message )> &callback ) const;
  void recursiveDeactivateControllers(
      std::shared_ptr<std::vector<std::string>> controllers_to_activate,
      std::shared_ptr<std::vector<std::string>> controllers_to_deactivate, size_t index,
      const std::function<void( bool success, const std::string &message )> &callback ) const;
  rclcpp::Node::SharedPtr node_;
  std::string controller_manager_name_;
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
