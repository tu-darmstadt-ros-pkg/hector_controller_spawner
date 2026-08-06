#include "controller_orchestrator/controller_orchestrator.hpp"

#include <algorithm>
#include <chrono>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <functional>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>

namespace controller_orchestrator
{

std::string lifecycleStateLabel( const lifecycle_msgs::msg::State &state )
{
  if ( !state.label.empty() ) {
    return state.label;
  }
  switch ( state.id ) {
  case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    return "active";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    return "inactive";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
    return "unconfigured";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
    return "finalized";
  default:
    return "unknown";
  }
}

using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

ControllerOrchestrator::ControllerOrchestrator( const rclcpp::Node::SharedPtr &node,
                                                const std::string &controller_manager_name )
    : node_( node ), controller_manager_name_( controller_manager_name )
{
  callback_group_ = node_->create_callback_group( rclcpp::CallbackGroupType::Reentrant );

  list_controllers_client_ = node_->create_client<ListControllers>(
      controller_manager_name_ + "/list_controllers", rclcpp::ServicesQoS(), callback_group_ );
  switch_controller_client_ = node_->create_client<SwitchController>(
      controller_manager_name_ + "/switch_controller", rclcpp::ServicesQoS(), callback_group_ );
  list_hardware_components_client_ =
      node_->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          controller_manager_name_ + "/list_hardware_components", rclcpp::ServicesQoS(),
          callback_group_ );
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  activity_subscription_ =
      node_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
          controller_manager_name_ + "/activity", rclcpp::QoS( 10 ),
          [this]( const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr msg ) {
            if ( !msg ) {
              return;
            }
            std::lock_guard<std::mutex> lock( controller_states_mutex_ );
            controller_states_.clear();
            for ( const auto &controller : msg->controllers ) {
              controller_states_[controller.name] = lifecycleStateLabel( controller.state );
            }
          },
          subscription_options );
}

/**
 * @brief Asynchronous implementation of the smart switch logic.
 *
 * The controller manager resolves the request itself: FORCE_AUTO expands the chain dependencies of
 * the requested controllers, deactivates whatever claims a conflicting command interface together
 * with everything depending on it, and applies the result in a single update iteration.
 */
void ControllerOrchestrator::smartSwitchControllerAsync(
    const std::vector<std::string> &activate_controllers,
    const std::function<void( bool success, const std::string &message )> &callback ) const
{
  if ( activate_controllers.empty() ) {
    callback( true, "No controllers requested" );
    return;
  }

  // optimization: skip if all requested controllers are already cached as active
  if ( areControllersActive( activate_controllers ) ) {
    callback( true, "Controller are already active. If not call refreshControllerStates." );
    return;
  }

  if ( !switch_controller_client_->service_is_ready() ) {
    callback( false, "switch_controller service not ready" );
    return;
  }

  auto req = std::make_shared<SwitchController::Request>();
  req->activate_controllers = activate_controllers;
  req->strictness = SwitchController::Request::FORCE_AUTO;

  switch_controller_client_->async_send_request(
      req, [callback]( rclcpp::Client<SwitchController>::SharedFuture future ) {
        const auto resp = future.get();
        if ( !resp ) {
          callback( false, "switch_controller returned null response" );
          return;
        }
        callback( resp->ok, resp->message );
      } );
}

/**
 * @brief Refreshes the internal controller state cache by querying the controller manager.
 * @note This is a blocking call. Do not call this from within a SingleThreadedExecutor callback.
 * @param timeout_s Timeout in seconds for the operation.
 * @return true if the refresh was successful.
 */
bool ControllerOrchestrator::refreshControllerStates( int timeout_s ) const
{
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "list_controllers service not available" );
    return false;
  }
  auto list_future =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() );
  if ( list_future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout )
    return false;
  auto list_resp = list_future.get();
  if ( !list_resp )
    return false;
  updateControllerStatesFromList( *list_resp );
  return true;
}

/**
 * @brief Asynchronous version of refreshControllerStates.
 * @param callback Function to call upon completion with (success, message).
 * @param timeout_s Timeout in seconds for the operation.
 */
void ControllerOrchestrator::refreshControllerStatesAsync(
    const std::function<void( bool success, const std::string &message )> &callback,
    int timeout_s ) const
{
  if ( !list_controllers_client_->service_is_ready() ) {
    callback( false, "list_controllers service not ready" );
    return;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  list_controllers_client_->async_send_request(
      list_req, [this, callback]( rclcpp::Client<ListControllers>::SharedFuture list_future ) {
        const auto list_resp = list_future.get();
        if ( !list_resp ) {
          callback( false, "list_controllers returned null response" );
          return;
        }
        updateControllerStatesFromList( *list_resp );
        callback( true, "Refresh successful" );
      } );
}

/**
 * @brief Synchronous version of smartSwitchController.
 *
 * This function blocks until the switch operation is complete or times out.
 * It internally calls the asynchronous version and waits for the result.
 * @note This is a blocking call. Do not call this from within a SingleThreadedExecutor callback.
 * @param activate_controllers List of controllers to activate.
 * @param timeout_s Timeout in seconds for the operation (default 2s).
 * @return true if the switch operation was successful.
 */
bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s ) const
{
  // if controllers already active do nothing
  if ( areControllersActive( activate_controllers ) ) {
    RCLCPP_INFO( node_->get_logger(), "[ControllerOrchestrator] Controllers are already active. If "
                                      "not call refreshControllerStates." );
    return true;
  }

  auto promise = std::make_shared<std::promise<std::pair<bool, std::string>>>();
  auto future = promise->get_future();

  // Call the Async version, but bind the result to our promise
  smartSwitchControllerAsync( activate_controllers,
                              [promise]( bool success, const std::string &message ) {
                                promise->set_value( { success, message } );
                              } );

  // Wait for the result (Blocking)
  auto status = future.wait_for( std::chrono::seconds( timeout_s ) );

  if ( status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] Synchronous Switch Timed Out after %d seconds. "
                  "Likely Cause: The ROS Executor is not spinning to process the response.",
                  timeout_s );
    return false;
  }

  auto result = future.get();
  if ( !result.first ) {
    RCLCPP_ERROR( node_->get_logger(), "[ControllerOrchestrator] Switch failed: %s",
                  result.second.c_str() );
  }
  return result.first;
}

void ControllerOrchestrator::updateControllerStatesFromList(
    const controller_manager_msgs::srv::ListControllers_Response &res ) const
{
  std::lock_guard<std::mutex> lock( controller_states_mutex_ );
  controller_states_.clear();
  for ( const auto &ctrl : res.controller ) { controller_states_[ctrl.name] = ctrl.state; }
}

/**
 * @brief Get a list of currently active controllers that claim a specific hardware interface and
 * the controllers that depend on them (recursively).
 * @param hardware_interface Name of the hardware interface (e.g., "joint1").
 * @param timeout_s Timeout in seconds for the operation.
 * @return Vector of active controller names.
 */
std::vector<std::string> ControllerOrchestrator::getActiveControllerOfHardwareInterface(
    const std::string &hardware_interface, int timeout_s ) const
{
  if ( !list_hardware_components_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) )
    return {};
  auto hw_resp = list_hardware_components_client_
                     ->async_send_request( std::make_shared<ListHardwareComponents::Request>() )
                     .get();
  if ( !hw_resp )
    return {};

  std::unordered_set<std::string> target_ifs;
  for ( const auto &comp : hw_resp->component ) {
    if ( comp.name == hardware_interface ) {
      for ( const auto &hw_if : comp.command_interfaces ) target_ifs.insert( hw_if.name );
      break;
    }
  }
  if ( target_ifs.empty() )
    return {};

  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) )
    return {};
  auto list_resp =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() ).get();
  if ( !list_resp )
    return {};

  std::vector<std::string> active_controllers;
  for ( const auto &ctrl : list_resp->controller ) {
    if ( ctrl.state != "active" )
      continue;
    for ( const auto &claimed : ctrl.claimed_interfaces ) {
      for ( const auto &target : target_ifs ) {
        if ( claimed.size() >= target.size() &&
             claimed.compare( claimed.size() - target.size(), target.size(), target ) == 0 ) {
          active_controllers.push_back( ctrl.name );
          goto next_ctrl;
        }
      }
    }
  next_ctrl:;
  }
  return active_controllers;
}

/**
 * @brief Deactivate a list of controllers.
 * @note This is a blocking call. Do not call this from within a SingleThreadedExecutor callback.
 * @param controllers_to_deactivate List of controller names to deactivate.
 * @param timeout_s Timeout in seconds for the operation.
 * @return true if the deactivation was successful.
 */
bool ControllerOrchestrator::deactivateControllers(
    const std::vector<std::string> &controllers_to_deactivate, int timeout_s ) const
{
  auto req = std::make_shared<SwitchController::Request>();
  req->deactivate_controllers = controllers_to_deactivate;
  req->strictness = SwitchController::Request::BEST_EFFORT;
  auto resp = switch_controller_client_->async_send_request( req ).get();
  if ( !resp || !resp->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Deactivate failed: %s",
                  resp ? resp->message.c_str() : "null" );
    return false;
  }
  return true;
}

/**
 * @brief Activate a list of controllers.
 * @note This is a blocking call. Do not call this from within a SingleThreadedExecutor callback.
 * @param controllers_to_activate List of controller names to activate.
 * @param timeout_s Timeout in seconds for the operation.
 * @return true if the activation was successful.
 */
bool ControllerOrchestrator::activateControllers(
    const std::vector<std::string> &controllers_to_activate, int timeout_s ) const
{
  auto req = std::make_shared<SwitchController::Request>();
  req->activate_controllers = controllers_to_activate;
  req->strictness = SwitchController::Request::BEST_EFFORT;
  auto resp = switch_controller_client_->async_send_request( req ).get();
  if ( !resp || !resp->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Activate failed: %s", resp ? resp->message.c_str() : "null" );
    return false;
  }
  return true;
}

/**
 * @brief Unload controllers that claim a specific joint, including their dependents.
 * @note This is a blocking call. Do not call this from within a SingleThreadedExecutor callback.
 * @param joint_name Name of the joint.
 * @param timeout_s Timeout in seconds for the operation.
 * @return true if the unload was successful.
 */
bool ControllerOrchestrator::unloadControllersOfJoint( const std::string &joint_name, int timeout_s )
{
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) )
    return false;
  auto list_resp =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() ).get();
  if ( !list_resp )
    return false;

  std::vector<std::string> joint_claimers;
  for ( const auto &ctrl : list_resp->controller ) {
    if ( ctrl.state != "active" )
      continue;
    for ( const auto &claimed : ctrl.claimed_interfaces ) {
      if ( claimed.find( joint_name + "/" ) == 0 ) {
        joint_claimers.push_back( ctrl.name );
        break;
      }
    }
  }
  if ( joint_claimers.empty() )
    return true;

  // FORCE_AUTO seeds the forced-deactivation walk with the requested controllers, so every
  // controller depending on a joint claimer is stopped along with it, in the correct order.
  auto req = std::make_shared<SwitchController::Request>();
  req->deactivate_controllers = joint_claimers;
  req->strictness = SwitchController::Request::FORCE_AUTO;
  auto resp = switch_controller_client_->async_send_request( req ).get();
  if ( !resp || !resp->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Deactivating controllers of joint '%s' failed: %s",
                  joint_name.c_str(), resp ? resp->message.c_str() : "null" );
    return false;
  }
  return true;
}

bool ControllerOrchestrator::isControllerActive( const std::string &controller_name ) const
{
  std::lock_guard<std::mutex> lock( controller_states_mutex_ );
  const auto it = controller_states_.find( controller_name );
  return it != controller_states_.end() && it->second == "active";
}

bool ControllerOrchestrator::areControllersActive( const std::vector<std::string> &controller_names ) const
{
  return std::all_of( controller_names.begin(), controller_names.end(),
                      [&]( const auto &name ) { return isControllerActive( name ); } );
}

} // namespace controller_orchestrator
