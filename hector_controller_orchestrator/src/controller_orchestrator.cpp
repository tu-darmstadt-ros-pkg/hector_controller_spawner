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
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
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

/**
 * @brief Wait for a service response with a deadline, and drop the request if it never arrives.
 *
 * Never plain get(). The response is delivered by an executor thread of this node, so a caller that
 * is itself a callback occupying the last free thread waits for something that can no longer
 * happen - get() then blocks for the life of the process. A deadline turns that into a failed call
 * the caller can report. The pending request has to be removed explicitly on timeout, or the client
 * keeps it forever.
 */
template<typename ClientT>
std::optional<typename ClientT::element_type::SharedResponse>
awaitResponse( const ClientT &client, typename ClientT::element_type::FutureAndRequestId &future,
               std::chrono::seconds timeout )
{
  if ( future.wait_for( timeout ) != std::future_status::ready ) {
    client->remove_pending_request( future );
    return std::nullopt;
  }
  return future.get();
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
  // Two groups, because the clients and the subscription need opposite things. The clients need
  // reentrancy: a blocking call parks a thread of the group waiting for a response another thread
  // of the same group must deliver. The subscription needs the opposite - it replaces the cache
  // wholesale, so dispatching it to several threads at once only makes them queue on the mutex.
  client_callback_group_ = node_->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
  cache_callback_group_ =
      node_->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );

  list_controllers_client_ =
      node_->create_client<ListControllers>( controller_manager_name_ + "/list_controllers",
                                             rclcpp::ServicesQoS(), client_callback_group_ );
  switch_controller_client_ =
      node_->create_client<SwitchController>( controller_manager_name_ + "/switch_controller",
                                              rclcpp::ServicesQoS(), client_callback_group_ );
  list_hardware_components_client_ =
      node_->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          controller_manager_name_ + "/list_hardware_components", rclcpp::ServicesQoS(),
          client_callback_group_ );
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = cache_callback_group_;
  activity_subscription_ =
      node_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
          controller_manager_name_ + "/activity", rclcpp::QoS( 10 ),
          [this]( controller_manager_msgs::msg::ControllerManagerActivity::ConstSharedPtr msg ) {
            if ( !msg ) {
              return;
            }
            std::unordered_map<std::string, std::string> states;
            states.reserve( msg->controllers.size() );
            for ( const auto &controller : msg->controllers ) {
              states.emplace( controller.name, lifecycleStateLabel( controller.state ) );
            }
            replaceControllerStates( std::move( states ) );
          },
          subscription_options );
}

/**
 * @brief Asynchronous implementation of the smart switch logic.
 *
 * This function initiates a three-step process:
 * 1. Fetch current controller states from the controller manager.
 * 2. Analyze dependencies and conflicts to determine which controllers to start/stop.
 * 3. Execute the switch through recursive async service calls to ensure correct ordering.
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

        // Step 2: Analysis of dependencies and resource conflicts
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

        // Activation: Tail -> Head (Reverse Topo)
        // smartSwitchControllerAnalysis returns Head -> Tail, so we reverse it for activation
        // such that dependencies are started before the controllers that depend on them.
        std::reverse( to_activate->begin(), to_activate->end() );

        if ( !to_deactivate->empty() ) {
          // Deactivation: Head -> Tail (Topo)
          // Start deactivation process, which will trigger activation once finished.
          recursiveDeactivateControllers( to_activate, to_deactivate, 0, callback );
        } else {
          // No deactivations needed, start activation directly.
          recursiveActivateControllers( to_activate, 0, callback );
        }
      } );
}

/**
 * @brief Analyzes dependencies and resource conflicts to determine switch actions.
 *
 * The logic follows these steps:
 * 1. Build maps of controller chains (who depends on whom).
 * 2. Identify currently active controllers and their claimed resources.
 * 3. Expand the 'to_activate' set to include all downstream dependencies.
 * 4. Check for resource conflicts between the new set and currently active controllers.
 * 5. Expand the 'to_deactivate' set to include all upstream dependents of conflicting controllers.
 * 6. Perform a topological sort to ensure controllers are stopped/started in the correct order.
 */
bool ControllerOrchestrator::smartSwitchControllerAnalysis(
    std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
    const controller_manager_msgs::srv::ListControllers_Response &res ) const
{
  // 1. Build Chain Maps
  std::unordered_map<std::string, std::vector<std::string>> forward_chain; // A->B (A depends on B)
  std::unordered_map<std::string, std::vector<std::string>> reverse_chain; // B->A (B is used by A)
  buildChainConnectionMaps( res.controller, forward_chain, reverse_chain );

  auto resource_map = buildControllerResourceMap( res.controller );

  // 2. Identify currently active
  std::unordered_set<std::string> currently_active_set;
  std::vector<std::string> available_controllers;

  for ( const auto &ctrl : res.controller ) {
    available_controllers.push_back( ctrl.name );
    if ( ctrl.state == "active" ) {
      currently_active_set.insert( ctrl.name );
    }
  }

  // 3. Filter valid requests
  std::vector<std::string> valid_requests;
  for ( const auto &req : to_activate ) {
    if ( in( available_controllers, req ) ) {
      valid_requests.push_back( req );
    } else {
      RCLCPP_WARN( node_->get_logger(), "Requested controller '%s' not found.", req.c_str() );
    }
  }

  // 4. Expand Activation Set (Downstream Dependencies Only)
  // We must ensure that if we start A, we also start everything A depends on.
  std::unordered_set<std::string> expanded_activation_set =
      getDownstreamDependencies( valid_requests, forward_chain );

  // 5. Detect Conflicts -> Build Deactivation Candidates
  std::unordered_set<std::string> needed_resources;
  std::unordered_set<std::string> deactivation_candidates;

  // Map resources claimed by new controllers
  for ( const auto &name : expanded_activation_set ) {
    for ( const auto &res_name : resource_map[name] ) {
      if ( needed_resources.count( res_name ) ) {
        RCLCPP_ERROR( node_->get_logger(), "Conflict: Resource '%s' claimed by multiple requested.",
                      res_name.c_str() );
        return false;
      }
      needed_resources.insert( res_name );
    }
  }

  // Check against active controllers for resource overlaps
  for ( const auto &active_name : currently_active_set ) {
    // If we plan to activate/reuse this controller, don't flag it as a conflict against itself
    if ( expanded_activation_set.count( active_name ) )
      continue;

    bool conflict = false;
    for ( const auto &res_name : resource_map[active_name] ) {
      if ( needed_resources.count( res_name ) ) {
        conflict = true;
        break;
      }
    }
    if ( conflict ) {
      deactivation_candidates.insert( active_name );
    }
  }

  // 6. Expand Deactivation Set (Upstream Dependents Only)
  // If we stop B because of a conflict, we MUST stop A if A depends on B.
  std::vector<std::string> deactivation_seed( deactivation_candidates.begin(),
                                              deactivation_candidates.end() );
  std::unordered_set<std::string> expanded_deactivation_set =
      getUpstreamDependents( deactivation_seed, reverse_chain );

  // Filter deactivation set to only those currently running
  std::unordered_set<std::string> final_deactivation_set;
  for ( const auto &name : expanded_deactivation_set ) {
    if ( currently_active_set.count( name ) )
      final_deactivation_set.insert( name );
  }

  // 7. Finalize Activation List
  // We activate everything in 'expanded_activation_set' that is:
  // a) Not currently active, OR
  // b) Currently active BUT scheduled for deactivation (i.e., needs a restart because one of its dependencies is restarting)
  std::unordered_set<std::string> final_activation_set;
  for ( const auto &name : expanded_activation_set ) {
    if ( !currently_active_set.count( name ) || final_deactivation_set.count( name ) ) {
      final_activation_set.insert( name );
    }
  }

  // 8. Topological Sort (Head -> Tail)
  // Ensures that controllers are returned in an order that satisfies dependency constraints.
  to_activate = topologicalSortControllers( final_activation_set, forward_chain );
  to_deactivate = topologicalSortControllers( final_deactivation_set, forward_chain );

  RCLCPP_DEBUG( node_->get_logger(), "Analysis: Activating [%s], Deactivating [%s]",
                vecToString( to_activate ).c_str(), vecToString( to_deactivate ).c_str() );

  return true;
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
  const auto list_resp =
      awaitResponse( list_controllers_client_, list_future, std::chrono::seconds( timeout_s ) );
  if ( !list_resp || !*list_resp )
    return false;
  updateControllerStatesFromList( **list_resp );
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

void ControllerOrchestrator::recursiveActivateControllers(
    std::shared_ptr<std::vector<std::string>> controllers_to_activate, size_t index,
    const std::function<void( bool success, const std::string &message )> &callback ) const
{
  if ( !controllers_to_activate || index >= controllers_to_activate->size() ) {
    callback( true, "Activation chain complete" );
    return;
  }
  auto req = std::make_shared<SwitchController::Request>();
  req->activate_controllers = { ( *controllers_to_activate )[index] };
  req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_controller_client_->async_send_request(
      req, [this, controllers_to_activate, index,
            callback]( rclcpp::Client<SwitchController>::SharedFuture f ) {
        if ( !f.get() || !f.get()->ok ) {
          callback( false, "Failed to activate " + ( *controllers_to_activate )[index] );
          return;
        }
        recursiveActivateControllers( controllers_to_activate, index + 1, callback );
      } );
}

void ControllerOrchestrator::recursiveDeactivateControllers(
    std::shared_ptr<std::vector<std::string>> controllers_to_activate,
    std::shared_ptr<std::vector<std::string>> controllers_to_deactivate, size_t index,
    const std::function<void( bool success, const std::string &message )> &callback ) const
{
  if ( !controllers_to_deactivate || index >= controllers_to_deactivate->size() ) {
    recursiveActivateControllers( controllers_to_activate, 0, callback );
    return;
  }
  auto req = std::make_shared<SwitchController::Request>();
  req->deactivate_controllers = { ( *controllers_to_deactivate )[index] };
  req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_controller_client_->async_send_request(
      req, [this, controllers_to_activate, controllers_to_deactivate, index,
            callback]( rclcpp::Client<SwitchController>::SharedFuture f ) {
        if ( !f.get() || !f.get()->ok ) {
          callback( false, "Failed to deactivate " + ( *controllers_to_deactivate )[index] );
          return;
        }
        recursiveDeactivateControllers( controllers_to_activate, controllers_to_deactivate,
                                        index + 1, callback );
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
  // Safety Check: Avoid running this directly inside a SingleThreadedExecutor callback

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
  std::unordered_map<std::string, std::string> states;
  states.reserve( res.controller.size() );
  for ( const auto &ctrl : res.controller ) { states.emplace( ctrl.name, ctrl.state ); }
  replaceControllerStates( std::move( states ) );
}

void ControllerOrchestrator::replaceControllerStates(
    std::unordered_map<std::string, std::string> states ) const
{
  // The critical section is one move. Building the replacement outside keeps the allocations off
  // the lock, and no reader can observe a half-rebuilt cache.
  const std::lock_guard lock( controller_states_mutex_ );
  controller_states_ = std::move( states );
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
  auto hw_future = list_hardware_components_client_->async_send_request(
      std::make_shared<ListHardwareComponents::Request>() );
  const auto hw_resp = awaitResponse( list_hardware_components_client_, hw_future,
                                      std::chrono::seconds( timeout_s ) );
  if ( !hw_resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "list_hardware_components did not answer within %ds. If this recurs, the calling "
                  "executor has no thread left to deliver the response on.",
                  timeout_s );
    return {};
  }
  if ( !*hw_resp )
    return {};

  std::unordered_set<std::string> target_ifs;
  for ( const auto &comp : ( *hw_resp )->component ) {
    if ( comp.name == hardware_interface ) {
      for ( const auto &hw_if : comp.command_interfaces ) target_ifs.insert( hw_if.name );
      break;
    }
  }
  if ( target_ifs.empty() )
    return {};

  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) )
    return {};
  auto list_future =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() );
  const auto list_resp =
      awaitResponse( list_controllers_client_, list_future, std::chrono::seconds( timeout_s ) );
  if ( !list_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "list_controllers did not answer within %ds. If this recurs, the calling executor "
        "has no thread left to deliver the response on.",
        timeout_s );
    return {};
  }
  if ( !*list_resp )
    return {};

  std::vector<std::string> active_controllers;
  for ( const auto &ctrl : ( *list_resp )->controller ) {
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
  auto future = switch_controller_client_->async_send_request( req );
  const auto resp =
      awaitResponse( switch_controller_client_, future, std::chrono::seconds( timeout_s ) );
  if ( !resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "Deactivate failed: switch_controller did not answer within %ds. If this recurs, "
                  "the calling executor has no thread left to deliver the response on.",
                  timeout_s );
    return false;
  }
  if ( !*resp || !( *resp )->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Deactivate failed: %s",
                  *resp ? ( *resp )->message.c_str() : "null" );
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
  auto future = switch_controller_client_->async_send_request( req );
  const auto resp =
      awaitResponse( switch_controller_client_, future, std::chrono::seconds( timeout_s ) );
  if ( !resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "Activate failed: switch_controller did not answer within %ds. If this recurs, "
                  "the calling executor has no thread left to deliver the response on.",
                  timeout_s );
    return false;
  }
  if ( !*resp || !( *resp )->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Activate failed: %s",
                  *resp ? ( *resp )->message.c_str() : "null" );
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
  auto list_future =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() );
  const auto list_response =
      awaitResponse( list_controllers_client_, list_future, std::chrono::seconds( timeout_s ) );
  if ( !list_response ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "list_controllers did not answer within %ds. If this recurs, the calling executor "
        "has no thread left to deliver the response on.",
        timeout_s );
    return false;
  }
  if ( !*list_response )
    return false;
  const auto &list_resp = *list_response;

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

  std::unordered_map<std::string, std::vector<std::string>> fwd, rev;
  buildChainConnectionMaps( list_resp->controller, fwd, rev );

  // Use reverse (dependents) to identify what needs to stop if joint claimer stops
  auto affected_set = getUpstreamDependents( joint_claimers, rev );

  std::vector<std::string> to_deactivate;
  for ( const auto &name : affected_set ) {
    auto it = std::find_if( list_resp->controller.begin(), list_resp->controller.end(),
                            [&]( const auto &c ) { return c.name == name && c.state == "active"; } );
    if ( it != list_resp->controller.end() )
      to_deactivate.push_back( name );
  }

  // Deactivate: Head -> Tail
  auto sorted = topologicalSortControllers( { to_deactivate.begin(), to_deactivate.end() }, fwd );
  return deactivateControllers( sorted, timeout_s );
}

void ControllerOrchestrator::buildChainConnectionMaps(
    const std::vector<controller_manager_msgs::msg::ControllerState> &controllers,
    std::unordered_map<std::string, std::vector<std::string>> &forward_connections,
    std::unordered_map<std::string, std::vector<std::string>> &reverse_connections ) const
{
  forward_connections.clear();
  reverse_connections.clear();
  for ( const auto &ctrl : controllers ) {
    for ( const auto &chain : ctrl.chain_connections ) {
      forward_connections[ctrl.name].push_back( chain.name );
      reverse_connections[chain.name].push_back( ctrl.name );
    }
  }
}

// -------------------------  HELPER: Downstream Only (Activation) ----------------------
std::unordered_set<std::string> ControllerOrchestrator::getDownstreamDependencies(
    const std::vector<std::string> &seed_controllers,
    const std::unordered_map<std::string, std::vector<std::string>> &forward_connections ) const
{
  std::unordered_set<std::string> visited;
  std::vector<std::string> queue;
  for ( const auto &name : seed_controllers ) {
    if ( visited.insert( name ).second )
      queue.push_back( name );
  }

  size_t head = 0;
  while ( head < queue.size() ) {
    std::string curr = queue[head++];
    if ( forward_connections.count( curr ) ) {
      for ( const auto &child : forward_connections.at( curr ) ) {
        if ( visited.insert( child ).second )
          queue.push_back( child );
      }
    }
  }
  return visited;
}

// -------------------------  HELPER: Upstream Only (Deactivation) ----------------------
std::unordered_set<std::string> ControllerOrchestrator::getUpstreamDependents(
    const std::vector<std::string> &seed_controllers,
    const std::unordered_map<std::string, std::vector<std::string>> &reverse_connections ) const
{
  std::unordered_set<std::string> visited;
  std::vector<std::string> queue;
  for ( const auto &name : seed_controllers ) {
    if ( visited.insert( name ).second )
      queue.push_back( name );
  }

  size_t head = 0;
  while ( head < queue.size() ) {
    std::string curr = queue[head++];
    if ( reverse_connections.count( curr ) ) {
      for ( const auto &parent : reverse_connections.at( curr ) ) {
        if ( visited.insert( parent ).second )
          queue.push_back( parent );
      }
    }
  }
  return visited;
}

std::vector<std::string> ControllerOrchestrator::topologicalSortControllers(
    const std::unordered_set<std::string> &active_controllers,
    const std::unordered_map<std::string, std::vector<std::string>> &forward_connections ) const
{
  if ( active_controllers.empty() )
    return {};
  std::unordered_map<std::string, size_t> indegree;
  for ( const auto &name : active_controllers ) indegree[name] = 0;
  for ( const auto &pair : forward_connections ) {
    if ( active_controllers.count( pair.first ) == 0 )
      continue;
    for ( const auto &dst : pair.second ) {
      if ( active_controllers.count( dst ) )
        indegree[dst]++;
    }
  }
  std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>> ready;
  for ( const auto &pair : indegree ) {
    if ( pair.second == 0 )
      ready.push( pair.first );
  }
  std::vector<std::string> sorted;
  while ( !ready.empty() ) {
    std::string u = ready.top();
    ready.pop();
    sorted.push_back( u );
    if ( forward_connections.count( u ) ) {
      for ( const auto &v : forward_connections.at( u ) ) {
        if ( active_controllers.count( v ) ) {
          indegree[v]--;
          if ( indegree[v] == 0 )
            ready.push( v );
        }
      }
    }
  }
  return sorted;
}

std::unordered_map<std::string, std::vector<std::string>>
ControllerOrchestrator::buildControllerResourceMap(
    const std::vector<controller_manager_msgs::msg::ControllerState> &controllers ) const
{
  std::unordered_map<std::string, std::vector<std::string>> resource_map;
  for ( const auto &ctrl : controllers ) {
    resource_map[ctrl.name] = { ctrl.required_command_interfaces.begin(),
                                ctrl.required_command_interfaces.end() };
  }
  return resource_map;
}

bool ControllerOrchestrator::isActiveLocked( const std::string &controller_name ) const
{
  // find(), not a scan of every entry: this is a hash map and the question is about one key.
  const auto it = controller_states_.find( controller_name );
  return it != controller_states_.end() && it->second == "active";
}

bool ControllerOrchestrator::isControllerActive( const std::string &controller_name ) const
{
  const std::shared_lock lock( controller_states_mutex_ );
  return isActiveLocked( controller_name );
}

bool ControllerOrchestrator::areControllersActive( const std::vector<std::string> &controller_names ) const
{
  // One lock for the whole question, so the answer describes a single moment. Locking per name
  // answered each one against a different snapshot: a switch landing mid-loop could produce an
  // answer true of no state that ever existed, and the caller acts on it by skipping a switch it
  // needed.
  const std::shared_lock lock( controller_states_mutex_ );
  return std::all_of( controller_names.begin(), controller_names.end(),
                      [this]( const std::string &name ) { return isActiveLocked( name ); } );
}

} // namespace controller_orchestrator
