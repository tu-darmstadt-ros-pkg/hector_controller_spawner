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

        // Activation: Tail -> Head (Reverse Topo)
        // smartSwitchControllerAnalysis returns Head -> Tail
        std::reverse( to_activate->begin(), to_activate->end() );

        if ( !to_deactivate->empty() ) {
          // Deactivation: Head -> Tail (Topo)
          recursiveDeactivateControllers( to_activate, to_deactivate, 0, callback );
        } else {
          recursiveActivateControllers( to_activate, 0, callback );
        }
      } );
}

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
  // FIX: Unidirectional search! Only grab things we NEED (A->B), do not climb back up to competitors.
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

  // Check against active controllers
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
  // FIX: Unidirectional search! If we stop B, we must stop A (A->B).
  std::vector<std::string> deactivation_seed( deactivation_candidates.begin(),
                                              deactivation_candidates.end() );
  std::unordered_set<std::string> expanded_deactivation_set =
      getUpstreamDependents( deactivation_seed, reverse_chain );

  // Filter deactivation set to only active ones
  std::unordered_set<std::string> final_deactivation_set;
  for ( const auto &name : expanded_deactivation_set ) {
    if ( currently_active_set.count( name ) )
      final_deactivation_set.insert( name );
  }

  // 7. Finalize Activation List
  // We activate everything in 'expanded_activation_set' that is:
  // a) Not currently active, OR
  // b) Currently active BUT scheduled for deactivation (Needs Restart)
  std::unordered_set<std::string> final_activation_set;
  for ( const auto &name : expanded_activation_set ) {
    if ( !currently_active_set.count( name ) || final_deactivation_set.count( name ) ) {
      final_activation_set.insert( name );
    }
  }

  // 8. Topological Sort (Head -> Tail)
  to_activate = topologicalSortControllers( final_activation_set, forward_chain );
  to_deactivate = topologicalSortControllers( final_deactivation_set, forward_chain );

  RCLCPP_DEBUG( node_->get_logger(), "Analysis: Activating [%s], Deactivating [%s]",
                vecToString( to_activate ).c_str(), vecToString( to_deactivate ).c_str() );

  return true;
}

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

bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s, bool refresh_ctrl_status ) const
{
  if ( refresh_ctrl_status && !refreshControllerStates( timeout_s ) )
    return false;
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) )
    return false;
  auto list_resp =
      list_controllers_client_->async_send_request( std::make_shared<ListControllers::Request>() ).get();
  if ( !list_resp )
    return false;
  updateControllerStatesFromList( *list_resp );

  std::vector<std::string> to_deactivate;
  // Analysis returns Head -> Tail
  if ( !smartSwitchControllerAnalysis( activate_controllers, to_deactivate, *list_resp ) )
    return false;
  if ( activate_controllers.empty() && to_deactivate.empty() )
    return true;

  // 1. Deactivate: Head -> Tail (Topo)
  if ( !to_deactivate.empty() ) {
    RCLCPP_INFO( node_->get_logger(), "Deactivating: %s", vecToString( to_deactivate ).c_str() );
    if ( !deactivateControllers( to_deactivate, timeout_s ) )
      return false;
  }

  // 2. Activate: Tail -> Head (Reverse Topo)
  if ( !activate_controllers.empty() ) {
    std::reverse( activate_controllers.begin(), activate_controllers.end() );
    RCLCPP_INFO( node_->get_logger(), "Activating: %s", vecToString( activate_controllers ).c_str() );
    if ( !activateControllers( activate_controllers, timeout_s ) )
      return false;
  }
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
  std::lock_guard<std::mutex> lock( controller_states_mutex_ );
  for ( const auto &name : controllers ) {
    const auto it = controller_states_.find( name );
    if ( it == controller_states_.end() || it->second != "active" )
      return false;
  }
  return true;
}

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

// ---------------------- NEW HELPER: Downstream Only (Activation) ----------------------
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

// ---------------------- NEW HELPER: Upstream Only (Deactivation) ----------------------
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

// NOTE: Replaced bidirectional 'findControllersInChain' with the specific ones above.
// Removed 'findControllersInChain' to prevent misuse.

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

} // namespace controller_orchestrator
