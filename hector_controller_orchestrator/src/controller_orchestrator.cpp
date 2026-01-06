#include "controller_orchestrator/controller_orchestrator.hpp"

#include <algorithm>
#include <chrono>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <functional>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>

namespace controller_orchestrator
{

std::string vecToString( const std::vector<std::string> &vec )
{
  std::string result;
  for ( const auto &item : vec ) {
    if ( !result.empty() ) {
      result += ", ";
    }
    result += item;
  }
  return result;
}

inline bool in( const std::vector<std::string> &vec, const std::string &item )
{
  return std::find( vec.begin(), vec.end(), item ) != vec.end();
}

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

void ControllerOrchestrator::smartSwitchControllerAsync(
    const std::vector<std::string> &activate_controllers,
    const std::function<void( bool success, const std::string &message )> &callback,
    bool refresh_ctrl_status ) const
{
  if ( activate_controllers.empty() ) {
    callback( true, "No controllers requested" );
    return;
  }

  if ( refresh_ctrl_status && !refreshControllerStates() ) {
    callback( false, "refreshControllerStates failed" );
    return;
  }

  if ( areControllersActive( activate_controllers ) ) {
    callback( true, "Requested controllers are already active" );
    return;
  }

  if ( !list_controllers_client_->service_is_ready() ) {
    callback( false, "list_controllers service not ready" );
    return;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  list_controllers_client_->async_send_request(
      list_req, [=]( rclcpp::Client<ListControllers>::SharedFuture list_future ) {
        const auto list_resp = list_future.get();
        if ( !list_resp ) {
          callback( false, "list_controllers returned null response" );
          return;
        }
        updateControllerStatesFromList( *list_resp );

        auto to_deactivate = std::make_shared<std::vector<std::string>>();
        auto to_activate = std::make_shared<std::vector<std::string>>( activate_controllers );

        if ( !smartSwitchControllerAnalysis( *to_activate, *to_deactivate, *list_resp ) ) {
          callback( false, "smartSwitchControllerAnalysis failed" );
          return;
        }

        if ( to_activate->empty() && to_deactivate->empty() ) {
          callback( true, "No controllers to activate or deactivate" );
          return;
        }

        if ( !switch_controller_client_->service_is_ready() ) {
          callback( false, "switch_controller service not ready" );
          return;
        }

        if ( !to_deactivate->empty() ) {
          // Deactivate from last to first, then activate
          recursiveDeactivateControllers( to_activate, to_deactivate, to_deactivate->size() - 1,
                                          callback );
        } else {
          // Nothing to deactivate, only activate
          recursiveActivateControllers( to_activate, to_activate->size() - 1, callback );
        }
      } );
}

bool ControllerOrchestrator::smartSwitchControllerAnalysis(
    std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
    const controller_manager_msgs::srv::ListControllers_Response &res ) const
{
  // Build a map: controller name → vector of claimed interfaces
  std::unordered_map<std::string, std::vector<std::string>> controller_to_resources;
  std::unordered_map<std::string, std::vector<std::string>> chain_connections;
  std::vector<std::string> currently_active;

  // Remove controllers from to_activate that do not exist
  for ( auto it = to_activate.begin(); it != to_activate.end(); ) {
    if ( std::find_if( res.controller.begin(), res.controller.end(), [&it]( const auto &ctrl ) {
           return ctrl.name == *it;
         } ) == res.controller.end() ) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[ControllerOrchestrator] Controller '%s' not found in list_controllers response; "
          "removing from activation list.",
          it->c_str() );
      it = to_activate.erase( it );
    } else {
      ++it;
    }
  }

  // Remove controllers from the activation list that are already active
  for ( const auto &ctrl_state : res.controller ) {
    const std::string &name = ctrl_state.name;
    if ( ctrl_state.state == "active" ) {
      currently_active.push_back( name );
      if ( in( to_activate, name ) ) {
        RCLCPP_DEBUG(
            node_->get_logger(), "[ControllerOrchestrator] Controller '%s' is already active; removing from activation list.",
            name.c_str() );
        to_activate.erase( std::remove( to_activate.begin(), to_activate.end(), name ),
                           to_activate.end() );
      }
    }
    controller_to_resources[name] = { ctrl_state.required_command_interfaces.begin(),
                                      ctrl_state.required_command_interfaces.end() };
    for ( const auto &chain : ctrl_state.chain_connections ) {
      chain_connections[name].push_back( chain.name );
    }
  }

  if ( to_activate.empty() ) {
    RCLCPP_DEBUG(
        node_->get_logger(),
        "[ControllerOrchestrator] No new controllers to activate; all are already active." );
    return true;
  }

  // Recursively, add all controllers of a group with at least one to_activate controller
  bool added = true;
  while ( added ) {
    added = false;
    std::vector<std::string> new_members;
    for ( const auto &name : to_activate ) {
      for ( const auto &chain_connection : chain_connections[name] ) {
        if ( !in( to_activate, chain_connection ) && !in( new_members, chain_connection ) ) {
          new_members.push_back( chain_connection );
        }
      }
    }
    if ( !new_members.empty() ) {
      to_activate.insert( to_activate.end(), new_members.begin(), new_members.end() );
      added = true;
    }
  }

  // 3) Build a set of all interfaces needed by the controllers we will activate
  std::unordered_set<std::string> needed_resources;
  needed_resources.reserve( to_activate.size() * 4 );

  for ( const auto &name : to_activate ) {
    const auto &resources = controller_to_resources.at( name );
    for ( const auto &res_name : resources ) {
      if ( needed_resources.count( res_name ) ) {
        RCLCPP_ERROR( node_->get_logger(),
                      "Resource conflict among requested controllers: "
                      "interface '%s' requested by more than one controller.",
                      res_name.c_str() );
        return false;
      }
      needed_resources.insert( res_name );
    }
  }

  // 4) Compare needed_resources against interfaces held by currently active controllers
  for ( const auto &active_name : currently_active ) {
    const auto &active_resources = controller_to_resources.at( active_name );
    bool conflict = false;
    for ( const auto &res_name : active_resources ) {
      if ( needed_resources.count( res_name ) ) {
        conflict = true;
        break;
      }
    }
    if ( conflict ) {
      RCLCPP_DEBUG( node_->get_logger(),
                    "Active controller '%s' will be stopped due to resource conflict.",
                    active_name.c_str() );
      to_deactivate.push_back( active_name );
    }
  }

  // Recursively, add all active controllers that depend on a controller in to_deactivate
  added = true;
  while ( added ) {
    added = false;
    std::vector<std::string> new_members;
    for ( const auto &name : currently_active ) {
      for ( const auto &chain_connection : chain_connections[name] ) {
        if ( in( to_deactivate, chain_connection ) && !in( new_members, name ) &&
             !in( to_deactivate, name ) ) {
          new_members.push_back( name );
        }
      }
    }
    if ( !new_members.empty() ) {
      to_deactivate.insert( to_deactivate.end(), new_members.begin(), new_members.end() );
      added = true;
    }
  }

  RCLCPP_DEBUG( node_->get_logger(),
                "[ControllerOrchestrator] Analysis complete: "
                "\nActivating %s controllers, \nDeactivating %s controllers.",
                vecToString( to_activate ).c_str(), vecToString( to_deactivate ).c_str() );
  return true;
}

bool ControllerOrchestrator::refreshControllerStates( int timeout_s ) const
{
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service not available" );
    return false;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request( list_req );

  const auto status = list_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ControllerOrchestrator] list_controllers service call failed or returned null" );
    return false;
  }

  updateControllerStatesFromList( *list_resp );
  return true;
}

void ControllerOrchestrator::recursiveActivateControllers(
    std::shared_ptr<std::vector<std::string>> controllers_to_activate, size_t index,
    const std::function<void( bool success, const std::string &message )> &callback ) const
{
  if ( !controllers_to_activate || index >= controllers_to_activate->size() ) {
    // All activations complete
    callback( true, "Weird error should not happen." );
    return;
  }

  auto act_switch_req = std::make_shared<SwitchController::Request>();
  act_switch_req->activate_controllers = { ( *controllers_to_activate )[index] };
  act_switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  act_switch_req->activate_asap = false;
  act_switch_req->timeout.sec = 0;
  act_switch_req->timeout.nanosec = 0;

  switch_controller_client_->async_send_request(
      act_switch_req, [this, controllers_to_activate, index,
                       callback]( rclcpp::Client<SwitchController>::SharedFuture switch_future ) {
        const auto act_switch_resp = switch_future.get();
        if ( !act_switch_resp || !act_switch_resp->ok ) {
          const std::string msg = act_switch_resp ? act_switch_resp->message : "null response";
          callback( false, "Failed to activate controller '" + ( *controllers_to_activate )[index] +
                               "': " + msg );
          return;
        }
        if ( index == 0 ) {
          // All activations complete
          callback( true, "Switched controllers successfully" );
          return;
        }
        // Recursively activate the next controller
        recursiveActivateControllers( controllers_to_activate, index - 1, callback );
      } );
}

void ControllerOrchestrator::recursiveDeactivateControllers(
    std::shared_ptr<std::vector<std::string>> controllers_to_activate,
    std::shared_ptr<std::vector<std::string>> controllers_to_deactivate, size_t index,
    const std::function<void( bool success, const std::string &message )> &callback ) const
{
  if ( !controllers_to_deactivate || controllers_to_deactivate->empty() ) {
    // Nothing to deactivate, just activate
    recursiveActivateControllers( controllers_to_activate, 0, callback );
    return;
  }

  if ( index >= controllers_to_deactivate->size() ) {
    // Done deactivating, now activate
    recursiveActivateControllers( controllers_to_activate, 0, callback );
    return;
  }

  auto deact_switch_req = std::make_shared<SwitchController::Request>();
  deact_switch_req->deactivate_controllers = { ( *controllers_to_deactivate )[index] };
  deact_switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  deact_switch_req->activate_asap = false;
  deact_switch_req->timeout.sec = 0;
  deact_switch_req->timeout.nanosec = 0;

  switch_controller_client_->async_send_request(
      deact_switch_req, [this, controllers_to_activate, controllers_to_deactivate, index,
                         callback]( rclcpp::Client<SwitchController>::SharedFuture switch_future ) {
        const auto deact_switch_resp = switch_future.get();
        if ( !deact_switch_resp || !deact_switch_resp->ok ) {
          const std::string msg = deact_switch_resp ? deact_switch_resp->message : "null response";
          callback( false, "Failed to deactivate controller '" +
                               ( *controllers_to_deactivate )[index] + "': " + msg );
          return;
        }

        // Recursively deactivate the next controller (reverse order)
        if ( index == 0 ) {
          // All deactivations complete, now activate
          recursiveActivateControllers( controllers_to_activate,
                                        controllers_to_activate->size() - 1, callback );
          return;
        }
        recursiveDeactivateControllers( controllers_to_activate, controllers_to_deactivate,
                                        index - 1, callback );
      } );
}

bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s, bool refresh_ctrl_status ) const
{
  if ( refresh_ctrl_status && !refreshControllerStates( timeout_s ) ) {
    return false;
  }

  if ( areControllersActive( activate_controllers ) ) {
    RCLCPP_DEBUG( node_->get_logger(),
                  "[ControllerOrchestrator] Requested controllers are already active." );
    return true;
  }

  // 1) Get controller information from controller manager (blocking call)
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service not available" );
    return false;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request( list_req );

  // Block until either the future is ready or we exceed timeout_s
  const auto status = list_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ControllerOrchestrator] list_controllers service call failed or returned null" );
    return false;
  }
  updateControllerStatesFromList( *list_resp );

  // 2) Analyze which controllers to activate and which to deactivate
  std::vector<std::string> to_deactivate;
  if ( !smartSwitchControllerAnalysis( activate_controllers, to_deactivate, *list_resp ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] smartSwitchControllerAnalysis failed" );
    return false;
  }
  if ( activate_controllers.empty() && to_deactivate.empty() ) {
    RCLCPP_DEBUG( node_->get_logger(),
                  "[ControllerOrchestrator] No controllers to activate or deactivate." );
    return true;
  }

  // 3) Finally, call switch_controller with start=to_activate, stop=to_deactivate (blocking call)
  if ( !switch_controller_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service not available" );
    return false;
  }
  const auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = activate_controllers;
  switch_req->deactivate_controllers = to_deactivate;
  switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_req->activate_asap = false;
  switch_req->timeout.sec = 0; // zero = infinite
  switch_req->timeout.nanosec = 0u;

  auto switch_future = switch_controller_client_->async_send_request( switch_req );

  // Block until either the future is ready or we exceed timeout_s
  auto switch_status = switch_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( switch_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto switch_resp = switch_future.get();
  if ( !switch_resp || !switch_resp->ok ) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }

  RCLCPP_INFO( node_->get_logger(),
               "Successfully switched controllers. Activated [%zu], deactivated [%zu].",
               activate_controllers.size(), to_deactivate.size() );
  return true;
}

void ControllerOrchestrator::updateControllerStatesFromList(
    const controller_manager_msgs::srv::ListControllers_Response &res ) const
{
  std::lock_guard<std::mutex> lock( controller_states_mutex_ );
  controller_states_.clear();
  for ( const auto &ctrl : res.controller ) { controller_states_[ctrl.name] = ctrl.state; }
}

bool ControllerOrchestrator::areControllersActive( const std::vector<std::string> &controllers ) const
{
  if ( controllers.empty() ) {
    return true;
  }

  std::lock_guard<std::mutex> lock( controller_states_mutex_ );
  for ( const auto &name : controllers ) {
    const auto it = controller_states_.find( name );
    if ( it == controller_states_.end() || it->second != "active" ) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> ControllerOrchestrator::getActiveControllerOfHardwareInterface(
    const std::string &hardware_interface, int timeout_s ) const
{
  // 1) Wait for and call "list_hardware_components"
  if ( !list_hardware_components_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] getActiveControllerOfHardwareInterface: "
                  "list_hardware_components service not available after %d s",
                  timeout_s );
    return {};
  }
  auto hw_req = std::make_shared<ListHardwareComponents::Request>();
  RCLCPP_INFO( node_->get_logger(),
               "getActiveControllerOfHardwareInterface: calling %s/list_hardware_components...",
               controller_manager_name_.c_str() );
  auto hw_future = list_hardware_components_client_->async_send_request( hw_req );

  if ( hw_future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_hardware_components call timed out (>%d s)",
        timeout_s );
    return {};
  }
  auto hw_resp = hw_future.get();
  if ( !hw_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_hardware_components returned null" );
    return {};
  }

  // 2) Find the component matching hardware_interface and collect its command interfaces
  std::unordered_set<std::string> target_command_interfaces;
  bool found_component = false;
  for ( const auto &comp : hw_resp->component ) {
    if ( comp.name == hardware_interface ) {
      found_component = true;
      for ( const auto &hw_if : comp.command_interfaces ) {
        target_command_interfaces.insert( hw_if.name );
      }
      break;
    }
  }
  if ( !found_component ) {
    RCLCPP_WARN( node_->get_logger(),
                 "getActiveControllerOfHardwareInterface: hardware component '%s' not found",
                 hardware_interface.c_str() );
    return {};
  }
  if ( target_command_interfaces.empty() ) {
    RCLCPP_INFO( node_->get_logger(),
                 "getActiveControllerOfHardwareInterface: hardware '%s' has no command interfaces",
                 hardware_interface.c_str() );
    return {};
  }

  // 3) Wait for and call "list_controllers"
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_controllers service not available after %d s",
        timeout_s );
    return {};
  }
  auto list_req = std::make_shared<ListControllers::Request>();
  RCLCPP_INFO( node_->get_logger(),
               "getActiveControllerOfHardwareInterface: calling %s/list_controllers...",
               controller_manager_name_.c_str() );
  auto list_future = list_controllers_client_->async_send_request( list_req );

  if ( list_future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "getActiveControllerOfHardwareInterface: list_controllers call timed out (>%d s)",
                  timeout_s );
    return {};
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "getActiveControllerOfHardwareInterface: list_controllers returned null" );
    return {};
  }

  // 4) Iterate through each controller; if active and any claimed_interface ∈ target_command_interfaces,
  //    add its name to the result; also add all controllers that are chained to it
  std::vector<std::string> active_controllers;
  for ( const auto &ctrl : list_resp->controller ) {
    if ( ctrl.state != "active" ) {
      continue;
    }
    bool added_controller = false;
    for ( const auto &claimed_if : ctrl.claimed_interfaces ) {
      for ( const auto &target_if : target_command_interfaces ) {
        // check if the target interface is equal to the last part of the claimed interface
        if ( claimed_if.size() >= target_if.size() &&
             claimed_if.compare( claimed_if.size() - target_if.size(), target_if.size(),
                                 target_if ) == 0 ) {
          active_controllers.push_back( ctrl.name );
          added_controller = true;
          break;
        }
      }
      if ( added_controller )
        break;
    }
  }
  return active_controllers;
}

bool ControllerOrchestrator::deactivateControllers(
    const std::vector<std::string> &controllers_to_deactivate, int timeout_s ) const
{
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->deactivate_controllers = controllers_to_deactivate;
  switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_req->activate_asap = false;
  switch_req->timeout.sec = 0; // zero = infinite
  switch_req->timeout.nanosec = 0u;

  auto switch_future = switch_controller_client_->async_send_request( switch_req );

  // Block until either the future is ready or we exceed timeout_s
  auto switch_status = switch_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( switch_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto switch_resp = switch_future.get();
  if ( !switch_resp || !switch_resp->ok ) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }
  return true;
}

bool ControllerOrchestrator::activateControllers(
    const std::vector<std::string> &controllers_to_activate, int timeout_s ) const
{
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = controllers_to_activate;
  switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_req->activate_asap = false;
  switch_req->timeout.sec = 0; // zero = infinite
  switch_req->timeout.nanosec = 0u;

  auto switch_future = switch_controller_client_->async_send_request( switch_req );

  // Block until either the future is ready or we exceed timeout_s
  auto switch_status = switch_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( switch_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto switch_resp = switch_future.get();
  if ( !switch_resp || !switch_resp->ok ) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }
  return true;
}

bool ControllerOrchestrator::unloadControllersOfJoint( const std::string &joint_name, int timeout_s )
{
  // Step 1: Request list of controllers
  auto list_request = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request( list_request );
  auto list_status = list_future.wait_for( std::chrono::seconds( timeout_s ) );

  if ( list_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service call timed out (>%d s)",
                  timeout_s );
    return false;
  }

  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service returned null response" );
    return false;
  }

  std::unordered_map<std::string, std::vector<std::string>> chain_connections;
  std::unordered_map<std::string, std::vector<std::string>> reverse_connections;
  std::unordered_map<std::string, bool> controller_active;
  std::vector<std::string> controllers_to_deactivate;

  auto controller_claims_joint = [&joint_name]( const auto &ctrl ) {
    for ( const auto &claimed_if : ctrl.claimed_interfaces ) {
      const auto sep_pos = claimed_if.find( '/' );
      if ( sep_pos != std::string::npos ) {
        if ( claimed_if.substr( 0, sep_pos ) == joint_name ) {
          return true;
        }
      }
    }
    return false;
  };

  // Step 2: Iterate through active controllers and check if they claim the joint
  for ( const auto &ctrl : list_resp->controller ) {
    for ( const auto &chain : ctrl.chain_connections ) {
      chain_connections[ctrl.name].push_back( chain.name );
      reverse_connections[chain.name].push_back( ctrl.name );
    }
    controller_active[ctrl.name] = ( ctrl.state == "active" );

    if ( ctrl.state == "active" ) {
      if ( controller_claims_joint( ctrl ) ) {
        controllers_to_deactivate.push_back( ctrl.name );
      }
    }
  }

  // Step 3: If none to deactivate, return early
  if ( controllers_to_deactivate.empty() ) {
    RCLCPP_INFO( node_->get_logger(),
                 "[ControllerOrchestrator] No active controllers claiming joint '%s'",
                 joint_name.c_str() );
    return true;
  }

  // Step 4: Collect all connected chain controllers and order by chain precedence.
  std::unordered_set<std::string> visited;
  std::vector<std::string> queue;
  queue.reserve( controllers_to_deactivate.size() );
  for ( const auto &name : controllers_to_deactivate ) {
    if ( visited.insert( name ).second ) {
      queue.push_back( name );
    }
  }

  for ( size_t i = 0; i < queue.size(); ++i ) {
    const std::string name = queue[i];
    auto forward_it = chain_connections.find( name );
    if ( forward_it != chain_connections.end() ) {
      for ( const auto &neighbor : forward_it->second ) {
        if ( visited.insert( neighbor ).second ) {
          queue.push_back( neighbor );
        }
      }
    }
    auto reverse_it = reverse_connections.find( name );
    if ( reverse_it != reverse_connections.end() ) {
      for ( const auto &neighbor : reverse_it->second ) {
        if ( visited.insert( neighbor ).second ) {
          queue.push_back( neighbor );
        }
      }
    }
  }

  std::unordered_set<std::string> active_chain;
  for ( const auto &name : visited ) {
    auto active_it = controller_active.find( name );
    if ( active_it != controller_active.end() && active_it->second ) {
      active_chain.insert( name );
    }
  }

  if ( active_chain.empty() ) {
    RCLCPP_INFO( node_->get_logger(),
                 "[ControllerOrchestrator] No active controllers found in chain for joint '%s'",
                 joint_name.c_str() );
    return true;
  }

  std::unordered_map<std::string, size_t> indegree;
  indegree.reserve( active_chain.size() );
  for ( const auto &name : active_chain ) { indegree[name] = 0; }

  for ( const auto &pair : chain_connections ) {
    const auto &src = pair.first;
    if ( active_chain.count( src ) == 0U ) {
      continue;
    }
    for ( const auto &dst : pair.second ) {
      if ( active_chain.count( dst ) == 0U ) {
        continue;
      }
      ++indegree[dst];
    }
  }

  std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>> ready;
  for ( const auto &pair : indegree ) {
    if ( pair.second == 0U ) {
      ready.push( pair.first );
    }
  }

  std::vector<std::string> ordered_controllers;
  ordered_controllers.reserve( active_chain.size() );
  while ( !ready.empty() ) {
    const auto name = ready.top();
    ready.pop();
    ordered_controllers.push_back( name );
    auto forward_it = chain_connections.find( name );
    if ( forward_it != chain_connections.end() ) {
      for ( const auto &neighbor : forward_it->second ) {
        if ( active_chain.count( neighbor ) == 0U ) {
          continue;
        }
        auto indeg_it = indegree.find( neighbor );
        if ( indeg_it != indegree.end() && indeg_it->second > 0U ) {
          --indeg_it->second;
          if ( indeg_it->second == 0U ) {
            ready.push( neighbor );
          }
        }
      }
    }
  }

  if ( ordered_controllers.size() != active_chain.size() ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] Unable to compute chain order for joint '%s'",
                  joint_name.c_str() );
    return false;
  }

  // Step 5: Deactivate controllers from top to joint.
  for ( const auto &name : ordered_controllers ) {
    if ( !deactivateControllers( { name }, timeout_s ) ) {
      RCLCPP_ERROR( node_->get_logger(),
                    "[ControllerOrchestrator] Failed to deactivate controller '%s'", name.c_str() );
      return false;
    }
  }
  return true;
}

} // namespace controller_orchestrator
