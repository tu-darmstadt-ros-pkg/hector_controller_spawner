#include "switch_plan.hpp"

#include <unordered_map>
#include <unordered_set>

namespace controller_orchestrator
{

namespace
{

using ControllerState = controller_manager_msgs::msg::ControllerState;
using Links = std::unordered_map<std::string, std::vector<std::string>>;

/// The seeds and everything reachable from them along @p links.
std::unordered_set<std::string> closure( const std::vector<std::string> &seeds, const Links &links )
{
  std::unordered_set<std::string> reached( seeds.begin(), seeds.end() );
  std::vector<std::string> queue( seeds.begin(), seeds.end() );
  while ( !queue.empty() ) {
    const std::string name = std::move( queue.back() );
    queue.pop_back();
    const auto it = links.find( name );
    if ( it == links.end() )
      continue;
    for ( const auto &next : it->second ) {
      if ( reached.insert( next ).second )
        queue.push_back( next );
    }
  }
  return reached;
}

std::string join( const std::vector<std::string> &names )
{
  std::string joined;
  for ( const auto &name : names ) joined += ( joined.empty() ? "" : ", " ) + name;
  return joined;
}

} // namespace

SwitchPlan planSwitch( const std::vector<std::string> &requested,
                       const std::vector<ControllerState> &controllers )
{
  std::unordered_map<std::string, const ControllerState *> by_name;
  std::unordered_map<std::string, std::string> exported_by;
  for ( const auto &controller : controllers ) {
    by_name[controller.name] = &controller;
    for ( const auto &name : controller.reference_interfaces )
      exported_by[controller.name + "/" + name] = controller.name;
    for ( const auto &name : controller.exported_state_interfaces )
      exported_by[controller.name + "/" + name] = controller.name;
  }

  // The manager refuses to start a controller whose followers stay inactive, and to stop one
  // whose preceders stay active, for command and state interfaces alike
  Links followers;
  Links preceders;
  for ( const auto &controller : controllers ) {
    for ( const auto *interfaces :
          { &controller.required_command_interfaces, &controller.required_state_interfaces } ) {
      for ( const auto &interface : *interfaces ) {
        const auto it = exported_by.find( interface );
        if ( it == exported_by.end() )
          continue;
        followers[controller.name].push_back( it->second );
        preceders[it->second].push_back( controller.name );
      }
    }
  }

  std::vector<std::string> unavailable;
  for ( const auto &name : requested ) {
    const auto it = by_name.find( name );
    if ( it == by_name.end() ||
         ( it->second->state != "inactive" && it->second->state != "active" ) )
      unavailable.push_back( name );
  }
  if ( !unavailable.empty() )
    return { {}, {}, "Not loaded or not configured: " + join( unavailable ) };

  const auto needed = closure( requested, followers );
  std::unordered_map<std::string, std::string> claimed_by;
  for ( const auto &name : needed ) {
    for ( const auto &interface : by_name.at( name )->required_command_interfaces ) {
      const auto [it, inserted] = claimed_by.emplace( interface, name );
      if ( !inserted )
        return { {}, {}, it->second + " and " + name + " both claim " + interface };
    }
  }

  std::vector<std::string> conflicting;
  for ( const auto &controller : controllers ) {
    if ( controller.state != "active" || needed.count( controller.name ) != 0 )
      continue;
    for ( const auto &interface : controller.required_command_interfaces ) {
      if ( claimed_by.count( interface ) != 0 ) {
        conflicting.push_back( controller.name );
        break;
      }
    }
  }
  const auto stopping = closure( conflicting, preceders );

  // One request, so the manager checks and applies all of it at once and the order is free
  SwitchPlan plan;
  for ( const auto &controller : controllers ) {
    const bool active = controller.state == "active";
    if ( !active && needed.count( controller.name ) != 0 )
      plan.activate.push_back( controller.name );
    if ( active && stopping.count( controller.name ) != 0 )
      plan.deactivate.push_back( controller.name );
  }
  return plan;
}

} // namespace controller_orchestrator
