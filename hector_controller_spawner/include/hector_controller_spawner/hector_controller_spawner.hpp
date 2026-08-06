#ifndef HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP
#define HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <rclcpp/parameter_client.hpp>
namespace hector_controller_spawner
{

inline std::string vecToString( const std::vector<std::string> &vec )
{
  std::string result;
  for ( const auto &s : vec ) {
    if ( !result.empty() )
      result += ", ";
    result += s;
  }
  result += " (size: " + std::to_string( vec.size() ) + ")";
  return result;
}

/**
 *  @brief  Multispawner waits for an (optional) e‑stop, then loads & activates
 *          hardware interfaces followed by controllers .
 */
class MultiSpawner final : public rclcpp::Node
{
public:
  explicit MultiSpawner();
  explicit MultiSpawner( const rclcpp::NodeOptions &options );
  void initialize();
  /// Bring hardware and controllers into the configured state.
  /// @return true when every step succeeded. False means a step exhausted its retries or the
  ///         context was shut down mid-sequence - the caller decides whether that is fatal.
  bool start_sequence( bool initial_init );
  bool is_tracking_estop() const noexcept { return !estop_topic_.empty(); }
  bool estop_released_and_not_in_progress() const noexcept
  {
    return released_ && !in_progress_ && !done_;
  }
  bool restart_after_estop_deactivation() const noexcept
  {
    return restart_after_estop_deactivation_;
  }
  /// Stop offering to run the sequence again until the next e-stop release. Used after a failed
  /// run so the main loop does not immediately retry at full speed.
  void mark_sequence_done() noexcept { done_.store( true ); }

private:
  // ----- helper structs -----
  struct ControllerCfg {
    bool activate{ true };
    bool specified{ false }; // true if the controller was specified in the parameters
  };

  // ----- callbacks -----
  void estopCb( const std_msgs::msg::Bool::SharedPtr msg );

  // ----- helpers -----
  bool loadAndActivateHardware( const std::string &name );
  bool loadController( const std::string &name );
  bool configureController( const std::string &name );
  bool replicateParamsToCM();
  /// @return true when every controller reached the state the configuration asks for.
  bool verifyFinalStates();
  /// Replace current_state with a fresh name → lifecycle state snapshot from the manager.
  void snapshotControllerStates( std::unordered_map<std::string, std::string> &current_state );
  /// @return true when every interface in hw_interfaces_ is reported active by the manager.
  ///         A failed query counts as "not active" - repeating the start sequence is idempotent,
  ///         so guessing wrong in that direction is the safe one.
  bool hardwareInterfacesStillActive();
  /// Run @p attempt until it succeeds, retry_delay_ apart, at most max_attempts_ times.
  /// @return false once the attempts are exhausted or the context shuts down.
  bool retryUntil( const std::string &what, const std::function<bool()> &attempt );
  /// Request the switch with "FORCE_AUTO" strictness: the controller manager expands the chain
  /// dependencies of to_activate, deactivates whatever blocks them along with everything
  /// depending on those, and applies the result in a single update iteration.
  bool switchControllersRequest( const std::vector<std::string> &to_activate,
                                 const std::vector<std::string> &to_deactivate );

  // ----- parameters -----
  std::vector<std::string> hw_interfaces_;
  std::vector<std::string> controllers_;
  std::unordered_map<std::string, ControllerCfg> controller_cfg_;
  double retry_delay_{ 5.0 };
  double start_delay_{ 0.0 };
  std::string estop_topic_;
  bool restart_after_estop_deactivation_{ true };
  // The manager serialises load/configure/switch behind one mutex and the first load of a plugin
  // type pays for the pluginlib scan, so a single call can take a good while on a loaded machine.
  // Timing out early here is expensive: the request is not cancelled, so the manager may still
  // carry it out while we go on to retry it.
  int service_call_timeout_ms_{ 20000 };
  int max_attempts_{ 10 };

  hector::ParameterSubscription retry_delay_param_sub_;
  hector::ParameterSubscription start_delay_param_sub_;
  hector::ParameterSubscription restart_after_estop_deactivation_param_sub_;
  hector::ParameterSubscription service_call_timeout_ms_param_sub_;
  hector::ParameterSubscription max_attempts_param_sub_;

  std::atomic<bool> in_progress_{ false };
  std::atomic<bool> done_{ false };
  std::atomic<bool> released_{ false };

  std::chrono::milliseconds serviceCallTimeout() const
  {
    return std::chrono::milliseconds( service_call_timeout_ms_ );
  }

  // ----- service clients -----
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr set_hw_state_client_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr list_hardware_ctrl_client_;
  rclcpp::AsyncParametersClient::SharedPtr cm_param_client_;
  // ----- subscription -----
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
};

} // namespace hector_controller_spawner

#endif // HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP
