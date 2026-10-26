#include "controller_graph.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>

#include <rclcpp/logging.hpp>

namespace controller_orchestrator
{

namespace
{

using ConnectionMap = std::unordered_map<std::string, std::vector<std::string>>;

/// Chain connections in both directions.
struct ChainMaps {
  ConnectionMap forward; ///< controller -> controllers it chains to
  ConnectionMap reverse; ///< controller -> controllers that chain to it
};

ChainMaps buildChainMaps( const ControllerStates &controllers )
{
  ChainMaps maps;
  for ( const auto &ctrl : controllers ) {
    for ( const auto &chain : ctrl.chain_connections ) {
      maps.forward[ctrl.name].push_back( chain.name );
      maps.reverse[chain.name].push_back( ctrl.name );
    }
  }
  return maps;
}

/// The seeds and everything reachable from them along @p connections.
std::unordered_set<std::string> closure( const std::vector<std::string> &seeds,
                                         const ConnectionMap &connections )
{
  std::unordered_set<std::string> visited;
  std::vector<std::string> queue;
  for ( const auto &name : seeds ) {
    if ( visited.insert( name ).second )
      queue.push_back( name );
  }

  size_t head = 0;
  while ( head < queue.size() ) {
    const std::string curr = queue[head++];
    const auto it = connections.find( curr );
    if ( it == connections.end() )
      continue;
    for ( const auto &next : it->second ) {
      if ( visited.insert( next ).second )
        queue.push_back( next );
    }
  }
  return visited;
}

/**
 * @brief Orders @p controllers so that each comes before the controllers it chains to.
 *
 * Deactivate in this order, activate in reverse.
 *
 * @return The order, or nullopt if a cycle leaves it undefined. An empty list is a valid order.
 */
std::optional<std::vector<std::string>>
topologicalSort( const std::unordered_set<std::string> &controllers, const ConnectionMap &forward,
                 const rclcpp::Logger &logger )
{
  if ( controllers.empty() )
    return std::vector<std::string>{};
  std::unordered_map<std::string, size_t> indegree;
  for ( const auto &name : controllers ) indegree[name] = 0;
  for ( const auto &pair : forward ) {
    if ( controllers.count( pair.first ) == 0 )
      continue;
    for ( const auto &dst : pair.second ) {
      if ( controllers.count( dst ) )
        indegree[dst]++;
    }
  }
  // Ordered by name, so the same listing always gives the same switch sequence.
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
    const auto it = forward.find( u );
    if ( it == forward.end() )
      continue;
    for ( const auto &v : it->second ) {
      if ( controllers.count( v ) && --indegree[v] == 0 )
        ready.push( v );
    }
  }
  if ( sorted.size() != controllers.size() ) {
    RCLCPP_ERROR( logger,
                  "The controller chain contains a cycle: %zu of %zu controllers could not be "
                  "ordered. Refusing to switch.",
                  controllers.size() - sorted.size(), controllers.size() );
    return std::nullopt;
  }
  return sorted;
}

std::unordered_map<std::string, std::vector<std::string>>
buildControllerResourceMap( const ControllerStates &controllers )
{
  std::unordered_map<std::string, std::vector<std::string>> resource_map;
  for ( const auto &ctrl : controllers ) {
    resource_map[ctrl.name] = { ctrl.required_command_interfaces.begin(),
                                ctrl.required_command_interfaces.end() };
  }
  return resource_map;
}

/// Whether @p claimed names @p target, either exactly or as its last path segments.
bool claimsInterface( const std::string &claimed, const std::string &target )
{
  if ( claimed.size() < target.size() )
    return false;
  const size_t offset = claimed.size() - target.size();
  return claimed.compare( offset, target.size(), target ) == 0 &&
         ( offset == 0 || claimed[offset - 1] == '/' );
}

} // namespace

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

std::optional<SwitchPlan> planSwitch( const std::vector<std::string> &requested,
                                      const ControllerStates &controllers,
                                      const rclcpp::Logger &logger )
{
  const ChainMaps chains = buildChainMaps( controllers );
  auto resource_map = buildControllerResourceMap( controllers );

  std::unordered_set<std::string> currently_active_set;
  for ( const auto &ctrl : controllers ) {
    if ( ctrl.state == "active" ) {
      currently_active_set.insert( ctrl.name );
    }
  }

  // Held and inactive, not merely held. The manager drops an unconfigured controller from a
  // best-effort switch and answers ok for the empty request that is left, so asking for one reads
  // as a switch that succeeded while the joint stays with whatever controller had it.
  std::vector<std::string> valid_requests;
  std::vector<std::string> unknown_requests;
  for ( const auto &req : requested ) {
    const auto state = std::find_if( controllers.begin(), controllers.end(),
                                     [&req]( const auto &c ) { return c.name == req; } );
    if ( state != controllers.end() &&
         ( state->state == "inactive" || state->state == "active" ) ) {
      valid_requests.push_back( req );
    } else {
      unknown_requests.push_back( req );
    }
  }
  if ( !unknown_requests.empty() ) {
    RCLCPP_WARN( logger, "Not held by the manager, or not configured, so not activatable: %s",
                 vecToString( unknown_requests ).c_str() );
  }
  // Nothing the manager holds was asked for, so there is no switch to make. Answering success for
  // that reports a robot in a state nobody arranged, which is what an empty listing after a manager
  // restart used to produce. A request only partly available still switches what it can, since
  // refusing it outright would drop the controllers that were there.
  if ( valid_requests.empty() ) {
    RCLCPP_ERROR( logger,
                  "None of the requested controllers can be activated (not held by the manager, or "
                  "not configured): %s",
                  vecToString( unknown_requests ).c_str() );
    return std::nullopt;
  }

  // Starting a controller starts everything it chains to.
  const std::unordered_set<std::string> expanded_activation_set =
      closure( valid_requests, chains.forward );

  std::unordered_set<std::string> needed_resources;
  for ( const auto &name : expanded_activation_set ) {
    for ( const auto &res_name : resource_map[name] ) {
      if ( needed_resources.count( res_name ) ) {
        RCLCPP_ERROR( logger, "Conflict: Resource '%s' claimed by multiple requested.",
                      res_name.c_str() );
        return std::nullopt;
      }
      needed_resources.insert( res_name );
    }
  }

  std::vector<std::string> deactivation_seed;
  for ( const auto &active_name : currently_active_set ) {
    // A controller that stays active is no conflict with itself.
    if ( expanded_activation_set.count( active_name ) )
      continue;
    const auto &claimed = resource_map[active_name];
    if ( std::any_of( claimed.begin(), claimed.end(),
                      [&]( const auto &res_name ) { return needed_resources.count( res_name ); } ) ) {
      deactivation_seed.push_back( active_name );
    }
  }

  // Stopping a controller stops everything chained to it.
  std::unordered_set<std::string> final_deactivation_set;
  for ( const auto &name : closure( deactivation_seed, chains.reverse ) ) {
    if ( currently_active_set.count( name ) )
      final_deactivation_set.insert( name );
  }

  // A controller that is active and stays in the plan restarts if one of its dependencies does.
  std::unordered_set<std::string> final_activation_set;
  for ( const auto &name : expanded_activation_set ) {
    if ( !currently_active_set.count( name ) || final_deactivation_set.count( name ) ) {
      final_activation_set.insert( name );
    }
  }

  auto sorted_activation = topologicalSort( final_activation_set, chains.forward, logger );
  auto sorted_deactivation = topologicalSort( final_deactivation_set, chains.forward, logger );
  if ( !sorted_activation || !sorted_deactivation ) {
    return std::nullopt;
  }
  RCLCPP_DEBUG( logger, "Analysis: Activating [%s], Deactivating [%s]",
                vecToString( *sorted_activation ).c_str(),
                vecToString( *sorted_deactivation ).c_str() );
  return SwitchPlan{ std::move( *sorted_activation ), std::move( *sorted_deactivation ) };
}

std::vector<std::string> activeClaimersOf( const ControllerStates &controllers,
                                           const std::unordered_set<std::string> &interfaces )
{
  std::vector<std::string> claimers;
  for ( const auto &ctrl : controllers ) {
    if ( ctrl.state != "active" )
      continue;
    const bool claims = std::any_of(
        ctrl.claimed_interfaces.begin(), ctrl.claimed_interfaces.end(), [&]( const auto &claimed ) {
          return std::any_of( interfaces.begin(), interfaces.end(), [&]( const auto &target ) {
            return claimsInterface( claimed, target );
          } );
        } );
    if ( claims )
      claimers.push_back( ctrl.name );
  }
  return claimers;
}

std::optional<std::vector<std::string>> deactivationOrderForJoint( const ControllerStates &controllers,
                                                                   const std::string &joint,
                                                                   const rclcpp::Logger &logger )
{
  std::vector<std::string> joint_claimers;
  for ( const auto &ctrl : controllers ) {
    if ( ctrl.state != "active" )
      continue;
    for ( const auto &claimed : ctrl.claimed_interfaces ) {
      if ( claimed.find( joint + "/" ) == 0 ) {
        joint_claimers.push_back( ctrl.name );
        break;
      }
    }
  }
  if ( joint_claimers.empty() )
    return std::vector<std::string>{};

  const ChainMaps chains = buildChainMaps( controllers );
  std::unordered_set<std::string> to_deactivate;
  for ( const auto &name : closure( joint_claimers, chains.reverse ) ) {
    const auto it = std::find_if( controllers.begin(), controllers.end(),
                                  [&]( const auto &c ) { return c.name == name; } );
    if ( it != controllers.end() && it->state == "active" )
      to_deactivate.insert( name );
  }
  return topologicalSort( to_deactivate, chains.forward, logger );
}

} // namespace controller_orchestrator
