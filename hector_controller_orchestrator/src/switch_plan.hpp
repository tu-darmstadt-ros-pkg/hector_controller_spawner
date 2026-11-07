#ifndef CONTROLLER_ORCHESTRATOR_SWITCH_PLAN_HPP
#define CONTROLLER_ORCHESTRATOR_SWITCH_PLAN_HPP

#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>

namespace controller_orchestrator
{

/// One switch for a controller manager that resolves neither chains nor conflicts itself.
struct SwitchPlan {
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
  /// Why no switch can satisfy the request. Empty if the plan is valid.
  std::string error;
};

/**
 * @brief Plans the switch that makes @p requested active, from one list_controllers answer.
 *
 * Adds every controller a requested one chains into, through reference or exported state
 * interfaces. Deactivates each active controller that claims a command interface one of them
 * needs, together with every active controller chained into it.
 */
SwitchPlan planSwitch( const std::vector<std::string> &requested,
                       const std::vector<controller_manager_msgs::msg::ControllerState> &controllers );

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_SWITCH_PLAN_HPP
