#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_GRAPH_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_GRAPH_HPP

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <rclcpp/logger.hpp>

namespace controller_orchestrator
{

using ControllerStates = std::vector<controller_manager_msgs::msg::ControllerState>;

std::string vecToString( const std::vector<std::string> &vec );

/// The switches a smart switch makes, each list ordered head to tail.
struct SwitchPlan {
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
};

/**
 * @brief Plans a smart switch to @p requested on the controllers of one listing.
 *
 * Adds every controller a requested one chains to, and deactivates each active controller that
 * claims a needed resource together with everything chained to it.
 *
 * @return The plan, or nullopt if none of the requested controllers can be activated, two of them
 *         claim the same resource, or a chain cycle leaves the order undefined.
 */
std::optional<SwitchPlan> planSwitch( const std::vector<std::string> &requested,
                                      const ControllerStates &controllers,
                                      const rclcpp::Logger &logger );

/// Active controllers claiming one of @p interfaces, matched on whole path segments from the end.
std::vector<std::string> activeClaimersOf( const ControllerStates &controllers,
                                           const std::unordered_set<std::string> &interfaces );

/**
 * @brief Active controllers claiming an interface of @p joint and every active controller chained
 *        to them, in deactivation order.
 * @return The controllers, or nullopt if a chain cycle leaves the order undefined.
 */
std::optional<std::vector<std::string>> deactivationOrderForJoint( const ControllerStates &controllers,
                                                                   const std::string &joint,
                                                                   const rclcpp::Logger &logger );

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_GRAPH_HPP
