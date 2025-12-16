#include "hector_controller_spawner/hector_controller_spawner.hpp"
#include <functional>
#include <unordered_set>

namespace hector_controller_spawner
{

using namespace std::chrono_literals;

MultiSpawner::MultiSpawner() : Node( "multi_controller_spawner" ) { }
void MultiSpawner::initialize()
{
  // 1) Declare & fetch parameters
  hw_interfaces_ = this->declare_parameter<std::vector<std::string>>( "hardware_interfaces",
                                                                      std::vector<std::string>() );
  controllers_ =
      this->declare_parameter<std::vector<std::string>>( "controllers", std::vector<std::string>() );
  retry_delay_ = this->declare_parameter<double>( "retry_delay", 5.0 );
  start_delay_ = this->declare_parameter<double>( "start_delay", 0.0 );
  estop_topic_ = this->declare_parameter<std::string>( "estop_topic", "" );
  restart_after_estop_deactivation_ =
      this->declare_parameter<bool>( "restart_after_estop_deactivation", true );
  load_groups_one_by_one_ = this->declare_parameter<bool>( "load_groups_one_by_one", true );
  int src_call_timeout_ms = this->declare_parameter<int>( "service_call_timeout_ms", 5000 );
  srv_call_timeout_ = std::chrono::milliseconds( src_call_timeout_ms );
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
    if ( rclcpp::spin_until_future_complete( shared_from_this(), list_hw_fut, srv_call_timeout_ ) !=
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

  if ( list_ctrl_client_->wait_for_service( 2s ) ) {
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_ctrl_client_->async_send_request( req );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
         rclcpp::FutureReturnCode::SUCCESS ) {
      auto resp = fut.get();
      parseControllerInfo( *resp, current_state );
    }
  }

  // 1) Decide what to load / activate / deactivate -------------------------
  std::vector<std::string> to_load;
  std::vector<std::string> to_activate;
  std::vector<std::string> to_deactivate;

  // a) Pass 1 – deal with requested controllers
  for ( const auto &name : controllers_ ) {
    const auto cfg = controller_cfg_.at( name );
    const auto it = current_state.find( name );

    const bool present = ( it != current_state.end() );
    const bool active = present && ( it->second == "active" );

    if ( !present )
      to_load.push_back( name );

    if ( cfg.activate ) {
      if ( !active )
        to_activate.push_back( name );
    } else // requested inactive
    {
      if ( active )
        to_deactivate.push_back( name );
    }
  }

  // b) Pass 2 – any other active controllers that should be shut down?
  for ( const auto &[name, state] : current_state ) {
    if ( state == "active" ) {
      // if not in our list *or* listed but with activate=false we already handled
      if ( std::find( controllers_.begin(), controllers_.end(), name ) == controllers_.end() )
        to_deactivate.push_back( name );
    }
  }

  // 2) Load missing controllers (one service call per controller) ----------
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

  // 2.5) Configure missing controllers
  for ( const auto &name : to_load ) {
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

  // 2.5) Re-request current state -> chained info only after configuring available----------------
  if ( !to_load.empty() && list_ctrl_client_->wait_for_service( 2s ) ) {
    current_state.clear();
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_ctrl_client_->async_send_request( req );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
         rclcpp::FutureReturnCode::SUCCESS ) {
      auto resp = fut.get();
      parseControllerInfo( *resp, current_state );
    }
  }

  // 4) Activate / deactivate controllers in groups ------------------------
  // deactivate all controllers (started by the spawner)
  // robuster to first deactivate and then re-activate in their respective groups
  deactivateAllActiveControllers( current_state );

  // update current state & check validity of desired state
  if ( list_ctrl_client_->wait_for_service( 2s ) ) {
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_ctrl_client_->async_send_request( req );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
         rclcpp::FutureReturnCode::SUCCESS ) {
      auto resp = fut.get();
      parseControllerInfo( *resp, current_state );
      if ( !validateDesiredControllerState( *resp ) ) {
        RCLCPP_ERROR( get_logger(),
                      "Desired controller state is not valid – aborting startup sequence." );
        in_progress_ = false;
        return;
      }
    }
  }
  // 5) activate controllers that are requested
  activateControllers( current_state );
  // ===== Done =============================================================
  verifyFinalStates();
  RCLCPP_INFO( get_logger(), " Multi Controller Spawner complete – shutting down." );
  done_.store( true );
  in_progress_ = false;
}

void MultiSpawner::checkRequiredControllersActive( const std::string &controller_name )
{
  for ( const auto &req : controller_info_.at( controller_name ).required_controllers ) {
    if ( !controller_cfg_.at( req ).activate ) {
      if ( controller_cfg_.at( req ).specified )
        RCLCPP_WARN( get_logger(),
                     "Controller '%s' requires controller '%s' to be active, but it is not "
                     "requested to be active. Auto activating it.",
                     controller_name.c_str(), req.c_str() );
      controller_cfg_.at( req ).activate = true;
    }
    // recursively check down the chain
    checkRequiredControllersActive( req );
  }
}
bool MultiSpawner::validateDesiredControllerState(
    const controller_manager_msgs::srv::ListControllers_Response &resp )
{
  // check whether the desired controller state is possible
  // all lower controllers in a chain must also be activated if an upper controller is activated
  for ( const auto &[name, config] : controller_cfg_ ) {
    if ( config.activate ) {
      checkRequiredControllersActive( name );
    }
  }
  // Interface check: no controllers that should be activated can share the same claimed command interfaces
  std::unordered_map<std::string, std::string> req_interface_owners; // interface -> controller name
  for ( const auto &c : resp.controller ) {
    if ( !controller_cfg_.at( c.name ).activate ) {
      // only care about controllers that should be active
      continue;
    }
    for ( const auto &req_inf : c.required_command_interfaces ) {
      if ( req_interface_owners.find( req_inf ) == req_interface_owners.end() ) {
        req_interface_owners[req_inf] = c.name;
      } else {
        // already owned
        const auto &owner = req_interface_owners[req_inf];
        RCLCPP_ERROR( get_logger(),
                      "Controllers '%s' and '%s' both require command interface '%s' and are both "
                      "requested to be active. This is not possible.",
                      owner.c_str(), c.name.c_str(), req_inf.c_str() );
        return false;
      }
    }
  }
  return true;
}

bool MultiSpawner::activateControllers( const std::unordered_map<std::string, std::string> &current_state )
{

  std::vector<std::string> to_activate;
  std::unordered_set<std::string> added;
  std::function<void( const std::string & )> add_with_dependencies = [&]( const std::string &name ) {
    if ( added.find( name ) != added.end() ) {
      return;
    }
    const auto info_it = controller_info_.find( name );
    if ( info_it != controller_info_.end() ) {
      for ( const auto &req : info_it->second.required_controllers ) {
        add_with_dependencies( req );
      }
    }
    added.insert( name );
    if ( current_state.count( name ) == 0 || current_state.at( name ) != "active" )
      to_activate.push_back( name );
  };
  for ( const auto &[name, info] : controller_info_ ) {
    if ( controller_cfg_[name].activate ) {
      add_with_dependencies( name );
    }
  }
  const bool success = loadControllerGroup( to_activate, {} );
  return success;
}

bool MultiSpawner::deactivateAllActiveControllers(
    const std::unordered_map<std::string, std::string> &current_state )
{
  std::vector<std::string> to_deactivate;
  std::unordered_set<std::string> added;
  std::function<void( const std::string & )> add_with_dependents = [&]( const std::string &name ) {
    if ( added.find( name ) != added.end() ) {
      return;
    }
    const auto info_it = controller_info_.find( name );
    if ( info_it != controller_info_.end() ) {
      for ( const auto &upper : info_it->second.upper_controllers ) {
        add_with_dependents( upper );
      }
    }
    added.insert( name );
    if ( current_state.at( name ) == "active" )
      to_deactivate.push_back( name );
  };
  for ( const auto &[name, info] : controller_info_ ) { add_with_dependents( name ); }
  // reverse to deactivate from top to bottom
  std::reverse( to_deactivate.begin(), to_deactivate.end() );
  bool success = loadControllerGroup( {}, to_deactivate );
  return success;
}

bool MultiSpawner::loadControllerGroup( const std::vector<std::string> &to_activate,
                                        const std::vector<std::string> &to_deactivate )
{
  if ( load_groups_one_by_one_ ) {
    // deactivate in reverse order
    for ( auto it = to_deactivate.rbegin(); it != to_deactivate.rend(); ++it ) {
      const auto &ctrl = *it;
      if ( !switchControllersRequest( {}, { ctrl } ) ) {
        RCLCPP_ERROR( get_logger(), "Deactivated controller: %s", ctrl.c_str() );
        return false;
      }
      RCLCPP_DEBUG( get_logger(), "Deactivated controller: %s", ctrl.c_str() );
    }
    // activate in order (in respect to normal dependencies)
    for ( const auto &ctrl : to_activate ) {
      if ( !switchControllersRequest( { ctrl }, {} ) ) {
        RCLCPP_ERROR( get_logger(), "Failed to activate controller '%s'", ctrl.c_str() );
        return false;
      }
      RCLCPP_DEBUG( get_logger(), "Activated controller: %s", ctrl.c_str() );
    }
    return true;
  }
  return switchControllersRequest( to_activate, to_deactivate );
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
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->timeout = rclcpp::Duration::from_seconds( 5.0 );

  auto fut = switch_ctrl_client_->async_send_request( req );
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

void MultiSpawner::parseControllerInfo(
    const controller_manager_msgs::srv::ListControllers_Response &resp,
    std::unordered_map<std::string, std::string> &current_state )
{
  // save snapshot of states
  for ( const auto &c : resp.controller ) { current_state[c.name] = c.state; }

  // parse controller chain info
  for ( const auto &c : resp.controller ) {
    controller_info_[c.name] = ControllerChainInfo();
    for ( const auto &conn : c.chain_connections ) {
      controller_info_[c.name].required_controllers.push_back( conn.name );
    }
  }

  // get first upper direction
  for ( auto &[name, info] : controller_info_ ) {
    for ( const auto &conn_name : info.required_controllers ) {
      controller_info_[conn_name].upper_controllers.push_back( name );
    }
  }
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
  if ( rclcpp::spin_until_future_complete( shared_from_this(), act_future, srv_call_timeout_ ) !=
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
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
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
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) ==
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
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, srv_call_timeout_ ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_WARN( get_logger(), "Failed to query controller states for final verification." );
    return;
  }

  std::unordered_map<std::string, std::string> state;
  const auto resp = fut.get();
  for ( const auto &c : resp->controller ) state[c.name] = c.state;

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

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  auto node = std::make_shared<hector_controller_spawner::MultiSpawner>();
  node->initialize();
  bool initial_init = true;
  using namespace std::chrono_literals;
  while ( rclcpp::ok() ) {
    rclcpp::spin_some( node );
    if ( node->estop_released_and_not_in_progress() ) {
      node->start_sequence( initial_init ); // safe – not inside another callback
      if ( !node->is_tracking_estop() || !node->restart_after_estop_deactivation() )
        break;
      initial_init = false;
    }
    std::this_thread::sleep_for( 50ms );
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
