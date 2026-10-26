#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include "controller_graph.hpp"

using controller_manager_msgs::msg::ControllerState;
using controller_orchestrator::ControllerStates;

namespace
{

ControllerState controller( const std::string &name, const std::string &state,
                            const std::vector<std::string> &claimed = {},
                            const std::vector<std::string> &chains_to = {} )
{
  ControllerState result;
  result.name = name;
  result.state = state;
  result.claimed_interfaces = claimed;
  for ( const auto &target : chains_to ) {
    controller_manager_msgs::msg::ChainConnection connection;
    connection.name = target;
    result.chain_connections.push_back( connection );
  }
  return result;
}

const rclcpp::Logger kLogger = rclcpp::get_logger( "test_controller_graph" );

} // namespace

TEST( ControllerGraph, ASwitchIntoAChainCycleIsRefused )
{
  const ControllerStates controllers = { controller( "a", "inactive", {}, { "b" } ),
                                         controller( "b", "inactive", {}, { "a" } ) };
  EXPECT_FALSE( controller_orchestrator::planSwitch( { "a" }, controllers, kLogger ).has_value() );
}

TEST( ControllerGraph, AnInterfaceIsClaimedOnlyByWholePathSegments )
{
  const ControllerStates controllers = {
      controller( "chained", "active", { "vel_to_pos/arm_joint_1/position" } ),
      controller( "lookalike", "active", { "other_arm_joint_1/position" } ),
      controller( "inactive", "inactive", { "arm_joint_1/position" } ) };
  EXPECT_EQ( controller_orchestrator::activeClaimersOf( controllers, { "arm_joint_1/position" } ),
             std::vector<std::string>{ "chained" } );
}

TEST( ControllerGraph, AJointsControllersStopBeforeTheControllersTheyChainTo )
{
  const ControllerStates controllers = {
      controller( "base", "active", { "flipper_joint/velocity" } ),
      controller( "head", "active", { "base/flipper_joint/velocity" }, { "base" } ) };
  EXPECT_EQ(
      controller_orchestrator::deactivationOrderForJoint( controllers, "flipper_joint", kLogger ),
      ( std::vector<std::string>{ "head", "base" } ) );
}
