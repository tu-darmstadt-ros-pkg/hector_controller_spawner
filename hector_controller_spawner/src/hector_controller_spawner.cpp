#include "hector_controller_spawner/hector_controller_spawner.hpp"
#include <functional>
#include <unordered_set>

namespace hector_controller_spawner
{

using namespace std::chrono_literals;

MultiSpawner::MultiSpawner() : MultiSpawner( rclcpp::NodeOptions() ) { }

MultiSpawner::MultiSpawner( const rclcpp::NodeOptions &options )
    : Node( "multi_controller_spawner", options )
{
}
void MultiSpawner::initialize()
{
  // 1) Declare & fetch parameters
  hw_interfaces_ = this->declare_parameter<std::vector<std::string>>( "hardware_interfaces",
                                                                      std::vector<std::string>() );
  controllers_ =
      this->declare_parameter<std::vector<std::string>>( "controllers", std::vector<std::string>() );
  retry_delay_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "retry_delay", std::ref( retry_delay_ ), "Retry delay in seconds",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );
  start_delay_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "start_delay", std::ref( start_delay_ ), "Start delay in seconds",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0; } ) );
  estop_topic_ = this->declare_parameter<std::string>( "estop_topic", "" );
  restart_after_estop_deactivation_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "restart_after_estop_deactivation",
      std::ref( restart_after_estop_deactivation_ ), "Restart after e-stop deactivation" );
  service_call_timeout_ms_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "service_call_timeout_ms", std::ref( service_call_timeout_ms_ ),
      "Service call timeout in milliseconds",
      hector::ParameterOptions<int>().onValidate( []( const auto &value ) { return value > 0; } ) );
  for ( const auto &ctrl : controllers_ ) {
    ControllerCfg cfg;
    cfg.activate = this->declare_parameter<bool>( ctrl + ".activate", true );
    cfg.specified = true;
    controller_cfg_[ctrl] = cfg;
  }

  // output parameters for debugging
  std::stringstream ss;
  ss << "Parameters:\n";
  ss << "  hardware_interfaces: " << hw_interfaces_.size() << "\n";
  for ( const auto &hw : hw_interfaces_ ) { ss << "    - " << hw << "\n"; }
  ss << "  controllers: " << controllers_.size() << "\n";
  for ( const auto &ctrl : controllers_ ) {
    ss << "    - " << ctrl << " (activate: " << controller_cfg_.at( ctrl ).activate << ")\n";
  }
  ss << "  retry_delay: " << retry_delay_ << " seconds\n";
  ss << "  estop_topic: '" << estop_topic_ << "'\n";
  RCLCPP_DEBUG( get_logger(), "%s", ss.str().c_str() );

  if ( start_delay_ > 0.0 ) {
    RCLCPP_INFO( get_logger(), "Delaying start sequence by %.1f seconds...", start_delay_ );
    const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>( start_delay_ ) );
    rclcpp::sleep_for( delay );
  }

  // 2) Create service clients
  set_hw_state_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state" );
  load_ctrl_client_ = this->create_client<controller_manager_msgs::srv::LoadController>(
      "controller_manager/load_controller" );
  switch_ctrl_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "controller_manager/switch_controller" );
  list_ctrl_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "controller_manager/list_controllers" );
  configure_ctrl_client_ = this->create_client<controller_manager_msgs::srv::ConfigureController>(
      "controller_manager/configure_controller" );
  list_hardware_ctrl_client_ =
      this->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          "controller_manager/list_hardware_components" );
  cm_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>( shared_from_this(), "controller_manager" );

  // 3) Handle e‑stop logic
  if ( estop_topic_.empty() ) {
    released_ = true;
  } else {
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        estop_topic_, rclcpp::QoS( 1 ).transient_local(),
        std::bind( &MultiSpawner::estopCb, this, std::placeholders::_1 ) );
    RCLCPP_INFO( get_logger(), "Waiting for e‑stop topic '%s' to become false…",
                 estop_topic_.c_str() );
  }
}

void MultiSpawner::estopCb( const std_msgs::msg::Bool::SharedPtr msg )
{
  if ( !in_progress_ && !msg->data ) {
    RCLCPP_INFO( get_logger(), "E‑stop released — commencing startup sequence." );
    done_ = false;
  }
  released_ = !msg->data;
}

void MultiSpawner::start_sequence( bool initial_init )
{
  in_progress_ = true;

  if ( start_delay_ > 0.0 ) {
    RCLCPP_INFO( get_logger(), "Delaying start sequence by %.1f seconds...", start_delay_ );
    const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>( start_delay_ ) );
    rclcpp::sleep_for( delay );
  }

  const auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>( retry_delay_ ) );

  // ===== Controller Manager Availability =================================
  while ( rclcpp::ok() && !list_ctrl_client_->wait_for_service( sleep_ns ) ) {
    RCLCPP_WARN( get_logger(), "Controller Manager is not yet available %.1fs", retry_delay_ );
  }

  if ( initial_init ) {
    // ===== Copy Parameters ==================================================
    // replicateParamsToCM();

  } else {
    // ===== Restart Necessary ?  ==============================================
    // test if hardware interfaces are available -> if not redo start sequence
    auto list_hw_fut = list_hardware_ctrl_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>() );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), list_hw_fut, serviceCallTimeout() ) !=
         rclcpp::FutureReturnCode::SUCCESS ) {
      RCLCPP_WARN( get_logger(), "Failed to list hardware components" );
    }
    auto list_hw_resp = list_hw_fut.get();
    size_t active_hw_interfaces = 0;
    for ( const auto &hw : list_hw_resp->component ) {
      if ( hw.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
           std::find( hw_interfaces_.begin(), hw_interfaces_.end(), hw.name ) !=
               hw_interfaces_.end() ) {
        active_hw_interfaces++;
      }
    }
    if ( active_hw_interfaces == hw_interfaces_.size() ) {
      RCLCPP_INFO( get_logger(),
                   "Hardware interfaces are still available after e-stop deactivation. No need to "
                   "redo hw interface and controller start sequence." );
      in_progress_ = false;
      done_ = true;
      return;
    }
    RCLCPP_INFO( get_logger(), "Hardware Interface must be reactivated after estop deactivation" );
  }

  // ===== Hardware =========================================================
  for ( const auto &hw : hw_interfaces_ ) {
    while ( rclcpp::ok() ) {
      if ( loadAndActivateHardware( hw ) ) {
        RCLCPP_DEBUG( get_logger(), "Hardware '%s' is active.", hw.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Hardware '%s' failed – retrying in %.1fs", hw.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  // ===== Controllers ======================================================
  // 0) Snapshot current controller states once -----------------------------
  std::unordered_map<std::string, std::string> current_state; // name → state string
  snapshotControllerStates( current_state );

  // 1) Load and configure every requested controller the manager does not know yet ---------
  std::vector<std::string> to_load;
  for ( const auto &name : controllers_ ) {
    if ( current_state.find( name ) == current_state.end() )
      to_load.push_back( name );
  }

  for ( const auto &name : to_load ) {
    while ( rclcpp::ok() ) {
      if ( loadController( name ) ) {
        RCLCPP_INFO( get_logger(), "Controller '%s' loaded.", name.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Failed to load '%s' – retrying in %.1fs", name.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  // A controller has to reach 'inactive' before it can be activated - the switch below does not
  // configure controllers, it only activates and deactivates them. Freshly loaded controllers are
  // unconfigured, and so is one that a previous run loaded but failed to configure.
  std::vector<std::string> to_configure = to_load;
  for ( const auto &name : controllers_ ) {
    const auto it = current_state.find( name );
    if ( it != current_state.end() && it->second == "unconfigured" )
      to_configure.push_back( name );
  }

  for ( const auto &name : to_configure ) {
    while ( rclcpp::ok() ) {
      if ( configureController( name ) ) {
        RCLCPP_INFO( get_logger(), "Controller '%s' configured.", name.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Failed to configure '%s' – retrying in %.1fs", name.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }

  if ( !to_configure.empty() )
    snapshotControllerStates( current_state );

  // 2) Hand the desired state to the controller manager in a single switch -----------------
  // "FORCE_AUTO" pulls in the chain dependencies of the requested controllers, rejects a set
  // that claims the same command interface twice, and deactivates every active controller that
  // blocks the activation together with everything depending on it.
  std::vector<std::string> to_activate;
  std::vector<std::string> to_deactivate;

  for ( const auto &name : controllers_ ) {
    if ( controller_cfg_.at( name ).activate ) {
      to_activate.push_back( name );
      continue;
    }
    // Only active controllers can be deactivated. A controller that is requested inactive but is
    // needed by the chain of a requested one stays active - the manager reports it.
    const auto it = current_state.find( name );
    if ( it != current_state.end() && it->second == "active" )
      to_deactivate.push_back( name );
  }

  // Controllers this spawner does not manage must not keep claiming the hardware.
  for ( const auto &[name, state] : current_state ) {
    if ( state == "active" &&
         std::find( controllers_.begin(), controllers_.end(), name ) == controllers_.end() )
      to_deactivate.push_back( name );
  }

  RCLCPP_INFO( get_logger(), "Switching controllers – activate: [%s], deactivate: [%s]",
               vecToString( to_activate ).c_str(), vecToString( to_deactivate ).c_str() );

  for ( int attempt = 1; rclcpp::ok(); ++attempt ) {
    if ( switchControllersRequest( to_activate, to_deactivate ) )
      break;
    if ( attempt >= switch_retries_ ) {
      RCLCPP_ERROR( get_logger(), "Controller switch failed after %d attempts.", attempt );
      break;
    }
    RCLCPP_WARN( get_logger(), "Controller switch failed – retrying in %.1fs", retry_delay_ );
    rclcpp::sleep_for( sleep_ns );
  }

  // ===== Done =============================================================
  verifyFinalStates();
  RCLCPP_INFO( get_logger(), " Multi Controller Spawner complete – shutting down." );
  done_.store( true );
  in_progress_ = false;
}

bool MultiSpawner::switchControllersRequest( const std::vector<std::string> &to_activate,
                                             const std::vector<std::string> &to_deactivate )
{
  if ( !switch_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_ERROR( get_logger(), "switch_controller service unavailable" );
    return false;
  }
  const auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = to_activate;
  req->deactivate_controllers = to_deactivate;
  // The controller manager resolves chain dependencies and resource conflicts itself and applies
  // the resolved request atomically - a controller that cannot be switched fails the whole switch.
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO;
  req->timeout = rclcpp::Duration::from_seconds( 5.0 );

  auto fut = switch_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_ERROR( get_logger(), "switch_controller call did not complete" );
    return false;
  }
  const auto resp = fut.get();
  if ( !resp->ok )
    RCLCPP_ERROR( get_logger(), "switch_controller failed: %s", resp->message.c_str() );
  return resp->ok;
}

void MultiSpawner::snapshotControllerStates( std::unordered_map<std::string, std::string> &current_state )
{
  // Left untouched when the query fails, so a failed refresh does not silently degrade an earlier
  // snapshot into "no controller is loaded".
  if ( !list_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_WARN( get_logger(), "list_controllers service unavailable" );
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto fut = list_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_WARN( get_logger(), "Failed to list controllers" );
    return;
  }
  const auto resp = fut.get();
  current_state.clear();
  for ( const auto &c : resp->controller ) { current_state[c.name] = c.state; }
}

bool MultiSpawner::loadAndActivateHardware( const std::string &name )
{
  if ( !set_hw_state_client_->wait_for_service( std::chrono::seconds( 3 ) ) ) {
    RCLCPP_WARN( get_logger(), "[MultiControllerSpawner] Service %s not available yet.",
                 set_hw_state_client_->get_service_name() );
    return false;
  }
  // 2) Activate
  const auto act_req =
      std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  act_req->name = name;
  act_req->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  act_req->target_state.label = "active";
  auto act_future = set_hw_state_client_->async_send_request( act_req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), act_future, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    return false;
  }
  return act_future.get()->ok;
}

bool MultiSpawner::loadController( const std::string &name )
{
  if ( !load_ctrl_client_->wait_for_service( 2s ) )
    return false;

  const auto req = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  req->name = name;
  auto fut = load_ctrl_client_->async_send_request( req );
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

bool MultiSpawner::configureController( const std::string &name )
{
  if ( !configure_ctrl_client_->wait_for_service( 2s ) )
    return false;
  const auto req = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  req->name = name;
  auto fut = configure_ctrl_client_->async_send_request( req );
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

void MultiSpawner::verifyFinalStates()
{
  static const char *GREEN = "\033[32m";
  static const char *RED = "\033[31m";
  static const char *RESET = "\033[0m";

  if ( !list_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_WARN( get_logger(),
                 "Cannot verify final controller states – list_controllers unavailable." );
    return;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto fut = list_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_WARN( get_logger(), "Failed to query controller states for final verification." );
    return;
  }

  std::unordered_map<std::string, std::string> state;
  const auto resp = fut.get();
  for ( const auto &c : resp->controller ) state[c.name] = c.state;

  // Controllers an active controller chains to. The manager keeps these running even when the
  // config asks for them to be inactive, so they are not reported as a failure.
  std::unordered_set<std::string> required_by_active;
  for ( const auto &c : resp->controller ) {
    if ( c.state != "active" )
      continue;
    for ( const auto &conn : c.chain_connections ) { required_by_active.insert( conn.name ); }
  }

  size_t ok_cnt = 0, fail_cnt = 0;
  std::stringstream report;
  report << "\nFinal controller states:\n";

  for ( const auto &name : controllers_ ) {
    std::string current = state.count( name ) ? state.at( name ) : "missing";
    const bool should_be_active = controller_cfg_[name].activate;
    const bool success =
        ( should_be_active && ( current == "active" ) ) ||
        ( !should_be_active && ( current == "inactive" || current == "configured" ) );

    if ( success ) {
      ++ok_cnt;
      report << "  " << GREEN << "✔ " << name << " → " << current << RESET << "\n";
    } else if ( !should_be_active && current == "active" && required_by_active.count( name ) ) {
      ++ok_cnt;
      report << "  " << GREEN << "(✔) " << name << " → " << current
             << " (kept active - required by an active controller)" << RESET << "\n";
    } else if ( !controller_cfg_[name].specified ) {
      ++ok_cnt;
      report << "  " << GREEN << "(✔) " << name << " → " << current
             << " (desired state unspecified in config)" << RESET << "\n";
    } else {
      ++fail_cnt;
      report << "  " << RED << "✘ " << name << " → " << current << RESET << "\n";
    }
  }
  report << ( ( fail_cnt > 0 ) ? RED : GREEN ) << "Summary: " << ok_cnt << " OK / " << fail_cnt
         << " failed." << RESET;
  if ( fail_cnt == 0 )
    RCLCPP_INFO( get_logger(), "%s", report.str().c_str() );
  else
    RCLCPP_WARN( get_logger(), "%s", report.str().c_str() );
}
} // namespace hector_controller_spawner
