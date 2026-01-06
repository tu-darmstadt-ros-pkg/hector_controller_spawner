#ifndef CONTROLLER_ORCHESTRATOR_TEST_HELPERS_HPP
#define CONTROLLER_ORCHESTRATOR_TEST_HELPERS_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace controller_orchestrator_test
{

template<typename ExecutorT, typename FuncT>
auto spin_while_executing( ExecutorT &executor, FuncT &&func ) -> decltype( func() )
{
  std::atomic<bool> spinning{ true };
  std::thread spin_thread( [&executor, &spinning]() {
    while ( spinning.load() ) {
      executor.spin_some();
      std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
    }
  } );

  if constexpr ( std::is_void_v<decltype( func() )> ) {
    func();
    spinning = false;
    spin_thread.join();
  } else {
    auto result = func();
    spinning = false;
    spin_thread.join();
    return result;
  }
}

inline std::string load_file( const std::string &path )
{
  std::ifstream stream( path );
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

inline std::string lifecycle_state_label( const lifecycle_msgs::msg::State &state )
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

inline std::unordered_map<std::string, std::string>
states_from_activity( const controller_manager_msgs::msg::ControllerManagerActivity &msg )
{
  std::unordered_map<std::string, std::string> states;
  states.reserve( msg.controllers.size() );
  for ( const auto &controller : msg.controllers ) {
    states[controller.name] = lifecycle_state_label( controller.state );
  }
  return states;
}

inline std::unordered_map<std::string, std::string>
states_from_list( const controller_manager_msgs::srv::ListControllers::Response &resp )
{
  std::unordered_map<std::string, std::string> states;
  states.reserve( resp.controller.size() );
  for ( const auto &controller : resp.controller ) { states[controller.name] = controller.state; }
  return states;
}

inline bool overlaps( const std::vector<std::string> &left,
                      const std::unordered_set<std::string> &right )
{
  for ( const auto &value : left ) {
    if ( right.count( value ) != 0U ) {
      return true;
    }
  }
  return false;
}

inline std::unordered_set<std::string>
compute_expected_deactivation( const controller_manager_msgs::srv::ListControllers::Response &resp,
                               const std::vector<std::string> &requested )
{
  std::unordered_map<std::string, std::vector<std::string>> controller_resources;
  std::unordered_map<std::string, std::vector<std::string>> chain_connections;
  std::vector<std::string> active_controllers;

  controller_resources.reserve( resp.controller.size() );
  chain_connections.reserve( resp.controller.size() );

  for ( const auto &ctrl : resp.controller ) {
    controller_resources[ctrl.name] = std::vector<std::string>(
        ctrl.required_command_interfaces.begin(), ctrl.required_command_interfaces.end() );
    if ( ctrl.state == "active" ) {
      active_controllers.push_back( ctrl.name );
    }
    for ( const auto &chain : ctrl.chain_connections ) {
      chain_connections[ctrl.name].push_back( chain.name );
    }
  }

  std::vector<std::string> to_activate = requested;
  bool added = true;
  while ( added ) {
    added = false;
    for ( size_t i = 0; i < to_activate.size(); ++i ) {
      const auto &name = to_activate[i];
      for ( const auto &linked : chain_connections[name] ) {
        if ( std::find( to_activate.begin(), to_activate.end(), linked ) == to_activate.end() ) {
          to_activate.push_back( linked );
          added = true;
        }
      }
    }
  }

  std::unordered_set<std::string> needed_resources;
  for ( const auto &name : to_activate ) {
    const auto &resources = controller_resources[name];
    needed_resources.insert( resources.begin(), resources.end() );
  }

  std::unordered_set<std::string> to_deactivate;
  for ( const auto &active_name : active_controllers ) {
    if ( overlaps( controller_resources[active_name], needed_resources ) ) {
      to_deactivate.insert( active_name );
    }
  }

  added = true;
  while ( added ) {
    added = false;
    for ( const auto &active_name : active_controllers ) {
      for ( const auto &linked : chain_connections[active_name] ) {
        if ( to_deactivate.count( linked ) != 0U && to_deactivate.count( active_name ) == 0U ) {
          to_deactivate.insert( active_name );
          added = true;
        }
      }
    }
  }

  for ( const auto &name : to_activate ) { to_deactivate.erase( name ); }

  return to_deactivate;
}

} // namespace controller_orchestrator_test

#endif // CONTROLLER_ORCHESTRATOR_TEST_HELPERS_HPP
