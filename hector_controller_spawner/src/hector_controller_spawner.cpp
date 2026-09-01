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
  controller_param_files_ = this->declare_parameter<std::vector<std::string>>(
      "controller_param_files", std::vector<std::string>() );
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
  load_groups_one_by_one_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "load_groups_one_by_one", std::ref( load_groups_one_by_one_ ),
      "Load controller groups one by one" );
  service_call_timeout_ms_param_sub_ = hector::createReconfigurableParameter(
      shared_from_this(), "service_call_timeout_ms", std::ref( service_call_timeout_ms_ ),
      "Service call timeout in milliseconds",
      hector::ParameterOptions<int>().onValidate( []( const auto &value ) { return value > 0; } ) );
  // An override under a name nothing declares is dropped without a word, so a configuration that
  // sets the timeout under the old name runs on the default and reads as if it had not.
  for ( const auto &override : this->get_node_options().parameter_overrides() ) {
    if ( override.get_name() == "srv_call_timeout_ms" ) {
      RCLCPP_ERROR( get_logger(),
                    "Parameter 'srv_call_timeout_ms' does not exist and is being ignored. The "
                    "service call timeout is 'service_call_timeout_ms' (an integer, milliseconds); "
                    "running with %d ms.",
                    service_call_timeout_ms_ );
    }
  }
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

bool MultiSpawner::abortRequested() const
{
  // Only when the e-stop is tracked: without a topic, released_ is set once and never changes.
  return is_tracking_estop() && !released_;
}

void MultiSpawner::estopCb( const std_msgs::msg::Bool::SharedPtr msg )
{
  const bool released = !msg->data;
  const bool was_released = released_.exchange( released );
  if ( released == was_released ) {
    // The e-stop board publishes its state, not its changes, so most messages say what the last
    // one said. Acting on every one of them restarts the sequence forever.
    return;
  }

  if ( !released ) {
    // A press that lands while a sequence is running has to survive it: the sequence is bringing
    // hardware up against an e-stop that is now engaged, and the release that follows is the only
    // thing that would have restarted it.
    estop_pressed_during_sequence_ = in_progress_.load();
    RCLCPP_INFO( get_logger(), "E-stop engaged." );
    return;
  }

  RCLCPP_INFO( get_logger(), "E‑stop released — commencing startup sequence." );
  if ( !in_progress_ ) {
    done_ = false;
  }
}

void MultiSpawner::start_sequence( bool initial_init )
{
  in_progress_ = true;
  estop_pressed_during_sequence_ = false;

  const auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>( retry_delay_ ) );

  // ===== Controller Manager Availability =================================
  while ( rclcpp::ok() && !list_ctrl_client_->wait_for_service( sleep_ns ) ) {
    RCLCPP_WARN( get_logger(), "Controller Manager is not yet available %.1fs", retry_delay_ );
  }

  if ( initial_init ) {
    // ===== Copy Parameters ==================================================
    if ( !replicateParamsToCM() ) {
      RCLCPP_ERROR( get_logger(), "Failed to replicate controller parameters to controller_manager "
                                  "- controllers may fail to initialize." );
    }

  } else {
    // ===== Restart Necessary ?  ==============================================
    // test if hardware interfaces are available -> if not redo start sequence
    auto list_hw_fut = list_hardware_ctrl_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>() );
    if ( rclcpp::spin_until_future_complete( shared_from_this(), list_hw_fut, serviceCallTimeout() ) !=
         rclcpp::FutureReturnCode::SUCCESS ) {
      // Nothing spins this node outside the main loop, so the answer to a call that timed out can
      // never arrive: get() would block here for good. The attempt is abandoned instead, leaving
      // done_ clear so the main loop starts a new one.
      RCLCPP_WARN( get_logger(),
                   "Failed to list hardware components. Retrying the startup sequence in %.1fs.",
                   retry_delay_ );
      list_hardware_ctrl_client_->remove_pending_request( list_hw_fut );
      in_progress_ = false;
      rclcpp::sleep_for( sleep_ns );
      return;
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
      if ( abortRequested() ) {
        RCLCPP_WARN( get_logger(),
                     "E-stop engaged while activating hardware '%s'. Abandoning this startup "
                     "sequence.",
                     hw.c_str() );
        in_progress_ = false;
        return;
      }
      if ( loadAndActivateHardware( hw ) ) {
        RCLCPP_DEBUG( get_logger(), "Hardware '%s' is active.", hw.c_str() );
        break;
      }
      RCLCPP_WARN( get_logger(), "Hardware '%s' failed – retrying in %.1fs", hw.c_str(),
                   retry_delay_ );
      rclcpp::sleep_for( sleep_ns );
    }
  }
  if ( abortRequested() ) {
    RCLCPP_WARN( get_logger(),
                 "E-stop engaged before the controllers were started. Abandoning this startup "
                 "sequence." );
    in_progress_ = false;
    return;
  }

  // ===== Controllers ======================================================
  // 0) Snapshot current controller states once -----------------------------
  std::unordered_map<std::string, std::string> current_state; // name → state string

  controller_manager_msgs::srv::ListControllers_Response listing;
  if ( !refreshControllerState( current_state, listing ) ) {
    // Proceeding on an empty snapshot would treat every controller as missing and try to load
    // controllers the manager already holds, which it refuses, forever.
    RCLCPP_WARN( get_logger(),
                 "Could not read the controller states. Retrying the startup sequence in %.1fs.",
                 retry_delay_ );
    in_progress_ = false;
    rclcpp::sleep_for( sleep_ns );
    return;
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
      if ( abortRequested() ) {
        RCLCPP_WARN( get_logger(),
                     "E-stop engaged while trying to load '%s'. Abandoning this startup sequence.",
                     name.c_str() );
        in_progress_ = false;
        return;
      }
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
      if ( abortRequested() ) {
        RCLCPP_WARN(
            get_logger(),
            "E-stop engaged while trying to configure '%s'. Abandoning this startup sequence.",
            name.c_str() );
        in_progress_ = false;
        return;
      }
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
  if ( !to_load.empty() && !refreshControllerState( current_state, listing ) ) {
    RCLCPP_WARN( get_logger(),
                 "Could not re-read the controller states after configuring. Retrying the startup "
                 "sequence in %.1fs.",
                 retry_delay_ );
    in_progress_ = false;
    rclcpp::sleep_for( sleep_ns );
    return;
  }

  // 4) Activate / deactivate controllers in groups ------------------------
  // deactivate all controllers (started by the spawner)
  // robuster to first deactivate and then re-activate in their respective groups
  if ( !deactivateAllActiveControllers( current_state ) ) {
    RCLCPP_ERROR( get_logger(),
                  "Failed to deactivate the running controllers before re-activating them." );
    sequence_failed_ = true;
  }

  // update current state & check validity of desired state
  if ( !refreshControllerState( current_state, listing ) ) {
    RCLCPP_WARN( get_logger(),
                 "Could not re-read the controller states before activating. Retrying the startup "
                 "sequence in %.1fs.",
                 retry_delay_ );
    in_progress_ = false;
    rclcpp::sleep_for( sleep_ns );
    return;
  }
  if ( !validateDesiredControllerState( listing ) ) {
    // Every controller was deactivated above, so this leaves the robot with none. Retrying cannot
    // change a configuration that asks two controllers for the same interface, so the sequence
    // does not pretend it is done and the process reports the failure when it ends.
    RCLCPP_ERROR( get_logger(),
                  "Desired controller state is not valid - aborting startup sequence. No "
                  "controllers are active." );
    sequence_failed_ = true;
    in_progress_ = false;
    return;
  }
  // 5) activate controllers that are requested
  if ( !activateControllers( current_state ) ) {
    RCLCPP_ERROR( get_logger(), "Failed to activate the requested controllers." );
    sequence_failed_ = true;
  }
  // ===== Done =============================================================
  if ( !verifyFinalStates() ) {
    RCLCPP_ERROR( get_logger(),
                  "The controllers are not in the requested state at the end of the startup "
                  "sequence." );
    sequence_failed_ = true;
  }
  RCLCPP_INFO( get_logger(), " Multi Controller Spawner complete – shutting down." );
  done_.store( true );
  in_progress_ = false;
  // A press and release that both landed while this ran left released_ true with done_ now set,
  // and the depth-1 subscription has nothing left to redeliver. Without this the robot stays down
  // after an e-stop cycle nobody saw.
  if ( estop_pressed_during_sequence_ && released_ ) {
    RCLCPP_INFO( get_logger(),
                 "The e-stop was engaged and released while the sequence ran. Starting it again." );
    estop_pressed_during_sequence_ = false;
    done_ = false;
  }
}

bool MultiSpawner::refreshControllerState(
    std::unordered_map<std::string, std::string> &current_state,
    controller_manager_msgs::srv::ListControllers_Response &resp_out )
{
  if ( !list_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_WARN( get_logger(), "list_controllers service unavailable" );
    return false;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto fut = list_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    // Nothing spins this node outside the main loop, so a call that timed out is never answered.
    RCLCPP_WARN( get_logger(), "Failed to list controllers" );
    list_ctrl_client_->remove_pending_request( fut );
    return false;
  }
  auto resp = fut.get();
  if ( !resp )
    return false;
  // Replaced rather than merged: a controller the manager no longer holds must not linger here as
  // a state that decides what gets switched.
  current_state.clear();
  parseControllerInfo( *resp, current_state );
  resp_out = *resp;
  return true;
}

void MultiSpawner::checkRequiredControllersActive( const std::string &controller_name )
{
  const auto info_it = controller_info_.find( controller_name );
  if ( info_it == controller_info_.end() )
    return;
  for ( const auto &req : info_it->second.required_controllers ) {
    // The chain comes from the controller manager, so a required controller can be one this
    // spawner's configuration never named. Nothing here is entitled to activate it.
    const auto cfg_it = controller_cfg_.find( req );
    if ( cfg_it == controller_cfg_.end() ) {
      RCLCPP_WARN( get_logger(),
                   "Controller '%s' requires controller '%s', which is not in this spawner's "
                   "configuration. Leaving it alone.",
                   controller_name.c_str(), req.c_str() );
      continue;
    }
    if ( !cfg_it->second.activate ) {
      if ( cfg_it->second.specified )
        RCLCPP_WARN( get_logger(),
                     "Controller '%s' requires controller '%s' to be active, but it is not "
                     "requested to be active. Auto activating it.",
                     controller_name.c_str(), req.c_str() );
      cfg_it->second.activate = true;
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
    // The manager lists every controller it holds, including ones loaded by another actor. Those
    // are not this spawner's to reason about, and asking its configuration about them throws.
    const auto cfg_it = controller_cfg_.find( c.name );
    if ( cfg_it == controller_cfg_.end() || !cfg_it->second.activate ) {
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
    // Subscripting would enter a controller this spawner was never told about with activate=true,
    // and then start it.
    const auto cfg_it = controller_cfg_.find( name );
    if ( cfg_it != controller_cfg_.end() && cfg_it->second.activate ) {
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
    // A controller the chain info still remembers can be gone from the manager, so what it is
    // doing now is only what this listing says.
    const auto state_it = current_state.find( name );
    if ( state_it != current_state.end() && state_it->second == "active" )
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
  return rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) ==
             rclcpp::FutureReturnCode::SUCCESS &&
         fut.get()->ok;
}

void MultiSpawner::parseControllerInfo(
    const controller_manager_msgs::srv::ListControllers_Response &resp,
    std::unordered_map<std::string, std::string> &current_state )
{
  // save snapshot of states
  for ( const auto &c : resp.controller ) { current_state[c.name] = c.state; }

  // Rebuilt, not added to: a controller the manager no longer holds must not survive here as a
  // name later passed to a switch, or looked up in a state map that has never heard of it.
  controller_info_.clear();

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
  if ( rclcpp::spin_until_future_complete( shared_from_this(), act_future, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    return false;
  }
  return act_future.get()->ok;
}

bool MultiSpawner::replicateParamsToCM()
{
  // controller_manager does not pass its own --params-file down to the controller nodes
  // it spawns, so e.g. a JointTrajectoryController would come up with an empty 'joints'.
  // Set '<controller>.params_file' instead, which controller_manager forwards as
  // --params-file when loading the controller (same as `ros2 control spawner --param-file`).
  if ( controller_param_files_.empty() )
    return true;

  if ( !cm_param_client_->wait_for_service( 2s ) ) {
    RCLCPP_ERROR( get_logger(), "controller_manager parameter service unavailable" );
    return false;
  }

  std::vector<rclcpp::Parameter> to_set;
  for ( const auto &name : controllers_ )
    to_set.emplace_back( name + ".params_file", controller_param_files_ );

  auto fut = cm_param_client_->set_parameters( to_set );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_ERROR( get_logger(), "Timed out setting controller parameters on controller_manager" );
    return false;
  }

  bool success = true;
  for ( const auto &result : fut.get() ) {
    if ( !result.successful ) {
      RCLCPP_ERROR( get_logger(), "controller_manager rejected parameter: %s", result.reason.c_str() );
      success = false;
    }
  }
  return success;
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

bool MultiSpawner::verifyFinalStates()
{
  static const char *GREEN = "\033[32m";
  static const char *RED = "\033[31m";
  static const char *RESET = "\033[0m";

  if ( !list_ctrl_client_->wait_for_service( 2s ) ) {
    RCLCPP_WARN( get_logger(),
                 "Cannot verify final controller states – list_controllers unavailable." );
    return false;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto fut = list_ctrl_client_->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( shared_from_this(), fut, serviceCallTimeout() ) !=
       rclcpp::FutureReturnCode::SUCCESS ) {
    RCLCPP_WARN( get_logger(), "Failed to query controller states for final verification." );
    list_ctrl_client_->remove_pending_request( fut );
    return false;
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
  if ( fail_cnt == 0 ) {
    RCLCPP_INFO( get_logger(), "%s", report.str().c_str() );
    return true;
  }
  // The switch requests are best effort, so a controller the manager quietly dropped reports no
  // error of its own. This listing is the only place that says whether the robot is actually in
  // the state that was asked for.
  RCLCPP_ERROR( get_logger(), "%s", report.str().c_str() );
  return false;
}
} // namespace hector_controller_spawner
