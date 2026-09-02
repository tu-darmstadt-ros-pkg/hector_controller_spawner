#ifndef HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP
#define HECTOR_CONTROLLER_SPAWNER_HECTOR_CONTROLLER_SPAWNER_HPP

#include <optional>
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

/// @brief How a startup sequence ended, which decides whether it runs again.
enum class SequenceOutcome : uint8_t {
  Completed, ///< The controller phase ran and left the robot in the requested state.
  Failed,    ///< It ran and did not, and running the same sequence again cannot change that.
  Abandoned, ///< The attempt stopped early. Nothing was decided, so it has to run again.
};

/**
 *  @brief  Multispawner waits for an (optional) e‑stop, then loads & activates
 *          hardware interfaces followed by controllers .
 *
 *  The sequence blocks on service calls, so it must run on its own thread while an executor spins
 *  the node. Without that the e-stop subscription is only served between calls, and the sequence
 *  brings hardware up against a press it has not been told about yet.
 */
class MultiSpawner final : public rclcpp::Node
{
public:
  using ControllerGroup = std::vector<std::string>; // group of controllers to activate together
  struct ControllerChainInfo {
    std::vector<std::string> required_controllers; // controllers that must be started before this one
    std::vector<std::string> upper_controllers;    // controllers that can be started after this one
  };
  explicit MultiSpawner();
  explicit MultiSpawner( const rclcpp::NodeOptions &options );
  void initialize();
  SequenceOutcome start_sequence();
  bool is_tracking_estop() const noexcept { return !estop_topic_.empty(); }
  /// @brief Whether the e-stop allows a sequence and one is still wanted.
  bool ready_to_start() const noexcept { return released_ && run_requested_; }
  bool restart_after_estop_deactivation() const noexcept
  {
    return restart_after_estop_deactivation_;
  }
  /// @brief Whether any startup sequence ended without the robot in the requested state. The
  /// process exit code carries this: best-effort switches report ok whatever they dropped, so
  /// nothing else distinguishes a spawner that worked from one that quietly did nothing.
  bool sequence_failed() const noexcept { return sequence_failed_; }

private:
  // ----- helper structs -----
  struct ControllerCfg {
    bool activate{ true };
    bool specified{ false }; // true if the controller was specified in the parameters
  };

  // ----- callbacks -----
  void estopCb( const std_msgs::msg::Bool::SharedPtr msg );

  // ----- helpers -----
  /// @brief Whether the e-stop is engaged again, which makes going on with a startup sequence
  /// work against whatever drove the hardware down.
  bool abortRequested() const;
  bool loadAndActivateHardware( const std::string &name );
  bool loadController( const std::string &name );
  bool configureController( const std::string &name );
  bool replicateParamsToCM();
  /// @brief Whether the controllers ended up in the state the configuration asked for, or nothing
  /// if the manager did not answer. Not knowing is not the same as knowing they are wrong.
  std::optional<bool> verifyFinalStates();
  /// @brief Replaces current_state with a fresh listing. False means the manager did not answer,
  /// and the caller must not carry on against what it has.
  bool refreshControllerState( std::unordered_map<std::string, std::string> &current_state,
                               controller_manager_msgs::srv::ListControllers_Response &resp_out );
  void parseControllerInfo( const controller_manager_msgs::srv::ListControllers_Response &resp,
                            std::unordered_map<std::string, std::string> &current_state );
  void checkRequiredControllersActive( const std::string &controller_name );
  bool
  validateDesiredControllerState( const controller_manager_msgs::srv::ListControllers_Response &resp );
  bool activateControllers( const std::unordered_map<std::string, std::string> &current_state );
  bool
  deactivateAllActiveControllers( const std::unordered_map<std::string, std::string> &current_state );
  bool loadControllerGroup( const std::vector<std::string> &to_activate,
                            const std::vector<std::string> &to_deactivate );
  bool switchControllersRequest( const std::vector<std::string> &to_activate,
                                 const std::vector<std::string> &to_deactivate );

  // ----- parameters -----
  std::vector<std::string> hw_interfaces_;
  std::vector<std::string> controllers_;
  std::vector<std::string> controller_param_files_;
  std::unordered_map<std::string, ControllerCfg> controller_cfg_;
  // std::vector<ControllerGroup> controller_groups_;
  std::unordered_map<std::string, ControllerChainInfo> controller_info_;
  // Written by the parameter callbacks, which run on the executor thread while the sequence runs
  // on its own. Atomic so a value changing mid-sequence is a race the language defines.
  std::atomic<double> retry_delay_{ 5.0 };
  double start_delay_{ 0.0 }; // read once, during initialize()
  std::string estop_topic_;
  std::atomic<bool> restart_after_estop_deactivation_{ true };
  std::atomic<bool> load_groups_one_by_one_{ true };
  std::atomic<int> service_call_timeout_ms_{ 5000 };

  /// The plain values the reconfigurable-parameter helper owns and writes. Mirrored into the
  /// atomics above by its update callback, which is the only thing that reads them.
  double retry_delay_value_{ 5.0 };
  bool restart_after_estop_deactivation_value_{ true };
  bool load_groups_one_by_one_value_{ true };
  int service_call_timeout_ms_value_{ 5000 };

  hector::ParameterSubscription retry_delay_param_sub_;
  hector::ParameterSubscription start_delay_param_sub_;
  hector::ParameterSubscription restart_after_estop_deactivation_param_sub_;
  hector::ParameterSubscription load_groups_one_by_one_param_sub_;
  hector::ParameterSubscription service_call_timeout_ms_param_sub_;

  /// Cleared when a sequence starts and set again by an e-stop release, so a release that lands
  /// while a sequence runs still asks for the next one.
  std::atomic<bool> run_requested_{ true };
  std::atomic<bool> released_{ false };
  std::atomic<bool> sequence_failed_{ false };
  /// Whether a sequence has ever finished and verified its controller phase. The restart shortcut
  /// needs this: hardware left active by an attempt that was abandoned before the controllers is
  /// not a robot that has nothing left to do.
  bool controllers_started_{ false };

  /// Attempts abandoned back to back. A failure that repeats is not a busy manager, and retrying
  /// it forever leaves whatever waits for this process waiting for good. Reset by an e-stop
  /// release, which runs on the executor thread.
  std::atomic<int> consecutive_abandoned_{ 0 };
  static constexpr int MAX_CONSECUTIVE_ABANDONED = 10;

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
