#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace controller_orchestrator
{

/**
 * @class ControllerOrchestrator
 * @brief Activates controllers together with the controllers they chain into, and stops what
 * conflicts with them.
 *
 * A controller manager that declares "force_auto" in its capabilities parameter resolves that
 * itself, so a switch is one FORCE_AUTO request. For any other, such as stock ros2_control, the
 * orchestrator lists the controllers and sends one STRICT request with the activations and
 * deactivations it planned. The capabilities are read once per controller manager. The controller
 * states come from the manager's latest ~/activity message.
 *
 * Responses are delivered by the executor spinning the node. The blocking call waits for one, so
 * it needs another thread of that executor to be free.
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
   * @brief Activates the given controllers. Blocking.
   * @param activate_controllers List of controllers to activate.
   * @param timeout_s The timeout of the whole switch in seconds, as for the async version.
   * @return true if the controller manager carried the switch out. False does not mean nothing
   * changed, see the README.
   */
  bool smartSwitchController( const std::vector<std::string> &activate_controllers,
                              int timeout_s = 10 ) const;

  /**
   * @brief Asynchronous version of smartSwitchController.
   * @param activate_controllers List of controllers to activate.
   * @param callback Called exactly once with (success, the controller manager's message), unless
   * the orchestrator is destroyed first. Called with false if no answer arrives in time.
   * @param timeout_s The timeout of the whole switch in seconds. Services not yet discovered are
   * waited for within it, and the switch request carries what is left as its timeout. The
   * callback gets false if no answer arrives within one second more.
   */
  void smartSwitchControllerAsync( const std::vector<std::string> &activate_controllers,
                                   const Callback &callback, int timeout_s = 5 ) const;

  /// Whether the latest activity message reports @p controller_name active.
  bool isControllerActive( const std::string &controller_name ) const;

  /// Whether the latest activity message reports all of @p controller_names active.
  bool areControllersActive( const std::vector<std::string> &controller_names ) const;

private:
  struct Core;
  /// Shared with the callbacks in flight, which hold it weakly and so cannot outlive its state.
  std::shared_ptr<Core> core_;
};

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
