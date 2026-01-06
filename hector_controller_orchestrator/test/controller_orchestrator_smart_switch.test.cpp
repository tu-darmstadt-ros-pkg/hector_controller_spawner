#include <gtest/gtest.h>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <hector_controller_spawner/hector_controller_spawner.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <errno.h>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using controller_manager_msgs::msg::ControllerManagerActivity;
using controller_manager_msgs::srv::ListControllers;
using hector_testing_utils::HectorTestFixture;

using namespace std::chrono_literals;

namespace
{

constexpr int kSchedPriority = 50;

std::string load_file( const std::string &path )
{
  std::ifstream stream( path );
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string lifecycle_state_label( const lifecycle_msgs::msg::State &state )
{
  if ( !state.label.empty() ) {
    return state.label;
  }
  switch ( state.id ) {
  case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    return "active";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    return "inactive";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
    return "unconfigured";
  case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
    return "finalized";
  default:
    return "unknown";
  }
}

std::unordered_map<std::string, std::string> states_from_activity( const ControllerManagerActivity &msg )
{
  std::unordered_map<std::string, std::string> states;
  states.reserve( msg.controllers.size() );
  for ( const auto &controller : msg.controllers ) {
    states[controller.name] = lifecycle_state_label( controller.state );
  }
  return states;
}

std::unordered_map<std::string, std::string> states_from_list( const ListControllers::Response &resp )
{
  std::unordered_map<std::string, std::string> states;
  states.reserve( resp.controller.size() );
  for ( const auto &controller : resp.controller ) { states[controller.name] = controller.state; }
  return states;
}

bool overlaps( const std::vector<std::string> &left, const std::unordered_set<std::string> &right )
{
  for ( const auto &value : left ) {
    if ( right.count( value ) != 0U ) {
      return true;
    }
  }
  return false;
}

std::unordered_set<std::string>
compute_expected_deactivation( const ListControllers::Response &resp,
                               const std::vector<std::string> &requested )
{
  std::unordered_map<std::string, std::vector<std::string>> controller_resources;
  std::unordered_map<std::string, std::vector<std::string>> chain_connections;
  std::vector<std::string> active_controllers;

  controller_resources.reserve( resp.controller.size() );
  chain_connections.reserve( resp.controller.size() );

  for ( const auto &ctrl : resp.controller ) {
    controller_resources[ctrl.name] = std::vector<std::string>(
        ctrl.required_command_interfaces.begin(), ctrl.required_command_interfaces.end() );
    if ( ctrl.state == "active" ) {
      active_controllers.push_back( ctrl.name );
    }
    for ( const auto &chain : ctrl.chain_connections ) {
      chain_connections[ctrl.name].push_back( chain.name );
    }
  }

  std::vector<std::string> to_activate = requested;
  bool added = true;
  while ( added ) {
    added = false;
    for ( size_t i = 0; i < to_activate.size(); ++i ) {
      const auto &name = to_activate[i];
      for ( const auto &linked : chain_connections[name] ) {
        if ( std::find( to_activate.begin(), to_activate.end(), linked ) == to_activate.end() ) {
          to_activate.push_back( linked );
          added = true;
        }
      }
    }
  }

  std::unordered_set<std::string> needed_resources;
  for ( const auto &name : to_activate ) {
    const auto &resources = controller_resources[name];
    needed_resources.insert( resources.begin(), resources.end() );
  }

  std::unordered_set<std::string> to_deactivate;
  for ( const auto &active_name : active_controllers ) {
    if ( overlaps( controller_resources[active_name], needed_resources ) ) {
      to_deactivate.insert( active_name );
    }
  }

  added = true;
  while ( added ) {
    added = false;
    for ( const auto &active_name : active_controllers ) {
      for ( const auto &linked : chain_connections[active_name] ) {
        if ( to_deactivate.count( linked ) != 0U && to_deactivate.count( active_name ) == 0U ) {
          to_deactivate.insert( active_name );
          added = true;
        }
      }
    }
  }

  for ( const auto &name : to_activate ) { to_deactivate.erase( name ); }

  return to_deactivate;
}

} // namespace

class ControllerOrchestratorFixture : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    HectorTestFixture::SetUp();

    controllers_yaml_ = std::string( TEST_CONFIG_DIR ) + "/controllers.yaml";
    spawner_yaml_ = std::string( TEST_CONFIG_DIR ) + "/controller_spawner.yaml";
    urdf_path_ = std::string( TEST_CONFIG_DIR ) + "/athena.urdf";

    const std::string urdf = load_file( urdf_path_ );
    ASSERT_FALSE( urdf.empty() );

    auto yaml_options = hector_testing_utils::node_options_from_yaml( controllers_yaml_ );
    auto cm_options = controller_manager::get_cm_node_options();
    cm_options.arguments( yaml_options.arguments() );
    cm_options.automatically_declare_parameters_from_overrides( true );

    cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
        cm_executor_, urdf, false, "controller_manager", "", cm_options );

    cm_executor_->add_node( controller_manager_ );

    const bool use_sim_time = controller_manager_->get_parameter_or( "use_sim_time", false );
    const bool has_realtime = realtime_tools::has_realtime_kernel();
    const bool lock_memory =
        controller_manager_->get_parameter_or<bool>( "lock_memory", has_realtime );
    if ( lock_memory ) {
      const auto lock_result = realtime_tools::lock_memory();
      if ( !lock_result.first ) {
        RCLCPP_WARN( controller_manager_->get_logger(), "Unable to lock the memory: '%s'",
                     lock_result.second.c_str() );
      }
    }

    RCLCPP_INFO( controller_manager_->get_logger(), "update rate is %d Hz",
                 controller_manager_->get_update_rate() );
    const bool manage_overruns =
        controller_manager_->get_parameter_or<bool>( "overruns.manage", true );
    RCLCPP_INFO( controller_manager_->get_logger(), "Overruns handling is : %s",
                 manage_overruns ? "enabled" : "disabled" );
    const int thread_priority =
        controller_manager_->get_parameter_or<int>( "thread_priority", kSchedPriority );
    RCLCPP_INFO( controller_manager_->get_logger(),
                 "Spawning %s RT thread with scheduler priority: %d",
                 controller_manager_->get_name(), thread_priority );

    cm_running_ = true;
    cm_spin_thread_ = std::thread( [this]() { cm_executor_->spin(); } );

    start_update_loop( use_sim_time, manage_overruns, thread_priority );

    activity_sub_ = tester_node_->create_test_subscription<ControllerManagerActivity>(
        "/controller_manager/activity" );
    list_client_ =
        tester_node_->create_test_client<ListControllers>( "/controller_manager/list_controllers" );

    ASSERT_TRUE( list_client_->wait_for_service( *executor_, 10s ) );

    orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
        tester_node_, "/controller_manager" );

    ASSERT_TRUE( start_spawner_node() );

    ASSERT_TRUE( wait_for_activity( expected_initial_states(), 30s ) );
  }

  void TearDown() override
  {
    stop_spawner_node();
    stop_controller_manager();
    HectorTestFixture::TearDown();
  }

  std::unordered_map<std::string, std::string> expected_initial_states() const
  {
    return {
        { "joint_state_broadcaster", "active" },   { "flipper_velocity_controller", "active" },
        { "vel_to_pos_controller", "active" },     { "gripper_trajectory_controller", "active" },
        { "arm_trajectory_controller", "active" }, { "flipper_trajectory_controller", "inactive" },
    };
  }

  bool wait_for_activity( const std::unordered_map<std::string, std::string> &expected,
                          std::chrono::nanoseconds timeout )
  {
    return activity_sub_->wait_for_message(
        *executor_, timeout, [&expected]( const ControllerManagerActivity &msg ) {
          const auto states = states_from_activity( msg );
          for ( const auto &pair : expected ) {
            auto it = states.find( pair.first );
            if ( it == states.end() || it->second != pair.second ) {
              return false;
            }
          }
          return true;
        } );
  }

  ListControllers::Response::SharedPtr list_controllers()
  {
    auto request = std::make_shared<ListControllers::Request>();
    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 10s;
    options.response_timeout = 10s;
    return hector_testing_utils::call_service<ListControllers>( list_client_->get(), request,
                                                                *executor_, options );
  }

  void start_update_loop( bool use_sim_time, bool manage_overruns, int thread_priority )
  {
    cm_update_thread_ = std::thread( [this, use_sim_time, manage_overruns, thread_priority]() {
      rclcpp::Parameter cpu_affinity_param;
      if ( controller_manager_->get_parameter( "cpu_affinity", cpu_affinity_param ) ) {
        std::vector<int> cpus;
        if ( cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ) {
          cpus = { static_cast<int>( cpu_affinity_param.as_int() ) };
        } else if ( cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY ) {
          const auto cpu_affinity_param_array = cpu_affinity_param.as_integer_array();
          for ( const auto cpu : cpu_affinity_param_array ) {
            cpus.push_back( static_cast<int>( cpu ) );
          }
        }
        const auto affinity_result = realtime_tools::set_current_thread_affinity( cpus );
        if ( !affinity_result.first ) {
          RCLCPP_WARN( controller_manager_->get_logger(), "Unable to set the CPU affinity : '%s'",
                       affinity_result.second.c_str() );
        }
      }

      if ( !realtime_tools::configure_sched_fifo( thread_priority ) ) {
        RCLCPP_WARN(
            controller_manager_->get_logger(),
            "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details on how to enable realtime scheduling.",
            errno, std::strerror( errno ) );
      } else {
        RCLCPP_INFO( controller_manager_->get_logger(),
                     "Successful set up FIFO RT scheduling policy with priority %i.",
                     thread_priority );
      }

      controller_manager_->get_clock()->wait_until_started();
      controller_manager_->get_clock()->sleep_for(
          rclcpp::Duration::from_seconds( 1.0 / controller_manager_->get_update_rate() ) );

      const auto period =
          std::chrono::nanoseconds( 1'000'000'000 / controller_manager_->get_update_rate() );
      rclcpp::Time previous_time = controller_manager_->get_trigger_clock()->now();
      std::this_thread::sleep_for( period );

      std::chrono::steady_clock::time_point next_iteration_time{ std::chrono::steady_clock::now() };

      while ( cm_running_ && rclcpp::ok() ) {
        const auto current_time = controller_manager_->get_trigger_clock()->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        controller_manager_->read( current_time, measured_period );
        controller_manager_->update( current_time, measured_period );
        controller_manager_->write( current_time, measured_period );

        if ( use_sim_time ) {
          controller_manager_->get_clock()->sleep_until( current_time + period );
        } else {
          next_iteration_time += period;
          const auto time_now = std::chrono::steady_clock::now();
          if ( manage_overruns && next_iteration_time < time_now ) {
            const double time_diff =
                static_cast<double>( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         time_now - next_iteration_time )
                                         .count() ) /
                1.e6;
            const double cm_period =
                1.e3 / static_cast<double>( controller_manager_->get_update_rate() );
            const int overrun_count = static_cast<int>( std::ceil( time_diff / cm_period ) );
            RCLCPP_WARN_THROTTLE(
                controller_manager_->get_logger(), *controller_manager_->get_clock(), 1000,
                "Overrun detected! The controller manager missed its desired rate of %d Hz. The "
                "loop took %f ms (missed cycles : %d).",
                controller_manager_->get_update_rate(), time_diff + cm_period, overrun_count + 1 );
            next_iteration_time += ( overrun_count * period );
          }
          std::this_thread::sleep_until( next_iteration_time );
        }
      }
    } );
  }

  bool start_spawner_node()
  {
    auto spawner_options = hector_testing_utils::node_options_from_yaml( spawner_yaml_ );
    spawner_options.automatically_declare_parameters_from_overrides( false );
    spawner_node_ = std::make_shared<hector_controller_spawner::MultiSpawner>( spawner_options );
    spawner_node_->initialize();
    spawner_node_->start_sequence( true );
    executor_->add_node( spawner_node_ );
    return true;
  }

  void stop_spawner_node()
  {
    if ( !spawner_node_ ) {
      return;
    }
    spawner_node_.reset();
  }

  void stop_controller_manager()
  {
    cm_running_ = false;
    if ( cm_update_thread_.joinable() ) {
      cm_update_thread_.join();
    }
    if ( cm_executor_ ) {
      cm_executor_->cancel();
    }
    if ( cm_spin_thread_.joinable() ) {
      cm_spin_thread_.join();
    }
    controller_manager_.reset();
    cm_executor_.reset();
  }

  std::string controllers_yaml_;
  std::string spawner_yaml_;
  std::string urdf_path_;

  std::atomic<bool> cm_running_{ false };
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> cm_executor_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::thread cm_spin_thread_;
  std::thread cm_update_thread_;

  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::shared_ptr<hector_controller_spawner::MultiSpawner> spawner_node_;
  std::shared_ptr<hector_testing_utils::TestSubscription<ControllerManagerActivity>> activity_sub_;
  std::shared_ptr<hector_testing_utils::TestClient<ListControllers>> list_client_;
};

TEST_F( ControllerOrchestratorFixture, SmartSwitchSync )
{
  const std::vector<std::string> requested = { "flipper_trajectory_controller" };
  const auto before_resp = list_controllers();
  ASSERT_NE( before_resp, nullptr );

  const auto expected_deactivate = compute_expected_deactivation( *before_resp, requested );

  activity_sub_->reset();
  std::atomic<bool> switch_done{ false };
  bool switch_success = false;
  std::thread switch_thread( [this, &requested, &switch_done, &switch_success]() {
    auto to_activate = requested;
    switch_success = orchestrator_->smartSwitchController( to_activate, 10, true );
    switch_done = true;
  } );
  ASSERT_TRUE( wait_for_activity( { { "flipper_trajectory_controller", "active" } }, 20s ) );
  ASSERT_TRUE( executor_->spin_until( [&switch_done]() { return switch_done.load(); }, 20s ) );
  switch_thread.join();
  ASSERT_TRUE( switch_success );

  const auto after_resp = list_controllers();
  ASSERT_NE( after_resp, nullptr );

  const auto before_states = states_from_list( *before_resp );
  const auto after_states = states_from_list( *after_resp );

  EXPECT_EQ( after_states.at( "flipper_trajectory_controller" ), "active" );

  for ( const auto &name : expected_deactivate ) {
    auto it = after_states.find( name );
    if ( it != after_states.end() ) {
      EXPECT_NE( it->second, "active" );
    }
  }

  for ( const auto &pair : before_states ) {
    if ( pair.second == "active" && expected_deactivate.count( pair.first ) == 0U &&
         std::find( requested.begin(), requested.end(), pair.first ) == requested.end() ) {
      EXPECT_EQ( after_states.at( pair.first ), "active" );
    }
  }
}

TEST_F( ControllerOrchestratorFixture, SmartSwitchAsync )
{
  const std::vector<std::string> first = { "flipper_trajectory_controller" };
  activity_sub_->reset();
  std::atomic<bool> initial_done{ false };
  bool initial_success = false;
  std::thread initial_thread( [this, &first, &initial_done, &initial_success]() {
    auto to_activate = first;
    initial_success = orchestrator_->smartSwitchController( to_activate, 10, true );
    initial_done = true;
  } );
  ASSERT_TRUE( wait_for_activity( { { "flipper_trajectory_controller", "active" } }, 20s ) );
  ASSERT_TRUE( executor_->spin_until( [&initial_done]() { return initial_done.load(); }, 20s ) );
  initial_thread.join();
  ASSERT_TRUE( initial_success );

  const std::vector<std::string> requested = { "flipper_velocity_controller" };
  const auto before_resp = list_controllers();
  ASSERT_NE( before_resp, nullptr );
  const auto expected_deactivate = compute_expected_deactivation( *before_resp, requested );

  std::atomic<bool> callback_done{ false };
  bool async_success = false;
  std::string async_message;

  activity_sub_->reset();
  std::thread async_thread( [this, &requested, &callback_done, &async_success, &async_message]() {
    orchestrator_->smartSwitchControllerAsync(
        requested,
        [&callback_done, &async_success, &async_message]( bool success, const std::string &message ) {
          async_success = success;
          async_message = message;
          callback_done = true;
        },
        true );
  } );

  ASSERT_TRUE( wait_for_activity( { { "flipper_velocity_controller", "active" } }, 20s ) );
  ASSERT_TRUE( executor_->spin_until( [&callback_done]() { return callback_done.load(); }, 20s ) );
  async_thread.join();
  ASSERT_TRUE( async_success ) << async_message;

  const auto after_resp = list_controllers();
  ASSERT_NE( after_resp, nullptr );

  const auto before_states = states_from_list( *before_resp );
  const auto after_states = states_from_list( *after_resp );

  EXPECT_EQ( after_states.at( "flipper_velocity_controller" ), "active" );

  for ( const auto &name : expected_deactivate ) {
    auto it = after_states.find( name );
    if ( it != after_states.end() ) {
      EXPECT_NE( it->second, "active" );
    }
  }

  for ( const auto &pair : before_states ) {
    if ( pair.second == "active" && expected_deactivate.count( pair.first ) == 0U &&
         std::find( requested.begin(), requested.end(), pair.first ) == requested.end() ) {
      EXPECT_EQ( after_states.at( pair.first ), "active" );
    }
  }
}
