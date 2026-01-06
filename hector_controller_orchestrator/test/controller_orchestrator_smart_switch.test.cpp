#include <gtest/gtest.h>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <hardware_interface/introspection.hpp>
#include <hector_controller_spawner/hector_controller_spawner.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

#include "controller_orchestrator_test_helpers.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <errno.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using controller_manager_msgs::msg::ControllerManagerActivity;
using controller_manager_msgs::srv::ListControllers;
using controller_orchestrator_test::compute_expected_deactivation;
using controller_orchestrator_test::load_file;
using controller_orchestrator_test::spin_while_executing;
using controller_orchestrator_test::states_from_activity;
using controller_orchestrator_test::states_from_list;
using hector_testing_utils::HectorTestFixture;

using namespace std::chrono_literals;

namespace
{

constexpr int kSchedPriority = 50;

} // namespace

class ControllerOrchestratorFixtureBase : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    HectorTestFixture::SetUp();

    const auto config_files = get_config_files();
    controllers_yaml_ = config_files.first;
    spawner_yaml_ = config_files.second;
    urdf_path_ = std::string( TEST_CONFIG_DIR ) + "/athena.urdf";

    const std::string urdf = load_file( urdf_path_ );
    ASSERT_FALSE( urdf.empty() );

    auto yaml_options = hector_testing_utils::node_options_from_yaml( controllers_yaml_ );
    auto cm_options = controller_manager::get_cm_node_options();
    cm_options.arguments( yaml_options.arguments() );
    cm_options.automatically_declare_parameters_from_overrides( true );

    // Initialize pal_statistics registries before ControllerManager registers interfaces.
    INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY( tester_node_,
                                                    hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
                                                    hardware_interface::DEFAULT_REGISTRY_KEY );
    INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY( tester_node_,
                                                    hardware_interface::CM_STATISTICS_TOPIC,
                                                    hardware_interface::CM_STATISTICS_KEY );

    cm_executor_ = create_cm_executor();
    ASSERT_NE( cm_executor_, nullptr );
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

    // wait until the multi-spawner has loaded and started the controllers
    // if this fails, likely the spawner node failed to not the orchestrator
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

  bool wait_for_list_states( const std::unordered_map<std::string, std::string> &expected,
                             std::chrono::nanoseconds timeout )
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while ( std::chrono::steady_clock::now() < deadline ) {
      const auto resp = list_controllers();
      if ( resp != nullptr ) {
        const auto states = states_from_list( *resp );
        bool matches = true;
        for ( const auto &pair : expected ) {
          auto it = states.find( pair.first );
          if ( it == states.end() || it->second != pair.second ) {
            matches = false;
            break;
          }
        }
        if ( matches ) {
          return true;
        }
      }
      executor_->spin_some();
      std::this_thread::sleep_for( 50ms );
    }
    // print final states for debugging
    const auto resp = list_controllers();
    if ( resp != nullptr ) {
      const auto states = states_from_list( *resp );
      RCLCPP_INFO( tester_node_->get_logger(), "Final controller states:" );
      for ( const auto &pair : states ) {
        RCLCPP_INFO( tester_node_->get_logger(), "  %s: %s", pair.first.c_str(), pair.second.c_str() );
      }
    }
    return false;
  }

  int get_number_of_active_controllers()
  {
    const auto resp = list_controllers();
    if ( resp == nullptr ) {
      return 0;
    }
    int count = 0;
    for ( const auto &ctrl : resp->controller ) {
      if ( ctrl.state == "active" ) {
        ++count;
      }
    }
    return count;
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
  std::shared_ptr<rclcpp::Executor> cm_executor_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::thread cm_spin_thread_;
  std::thread cm_update_thread_;

  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::shared_ptr<hector_controller_spawner::MultiSpawner> spawner_node_;
  std::shared_ptr<hector_testing_utils::TestSubscription<ControllerManagerActivity>> activity_sub_;
  std::shared_ptr<hector_testing_utils::TestClient<ListControllers>> list_client_;

  virtual std::shared_ptr<rclcpp::Executor> create_cm_executor() = 0;
  virtual std::pair<std::string, std::string> get_config_files() const = 0;
};

// Configuration traits for normal config
struct NormalConfig {
  static std::pair<std::string, std::string> get_config_files()
  {
    return { std::string( TEST_CONFIG_DIR ) + "/controllers.yaml",
             std::string( TEST_CONFIG_DIR ) + "/controller_spawner.yaml" };
  }
  static const char *name() { return "Normal"; }
};

// Configuration traits for multiple chained config
struct MultipleChainedConfig {
  static std::pair<std::string, std::string> get_config_files()
  {
    return { std::string( TEST_CONFIG_DIR ) + "/controllers_multiple_chained.yaml",
             std::string( TEST_CONFIG_DIR ) + "/controller_spawner_multiple_chained.yaml" };
  }
  static const char *name() { return "MultipleChained"; }
};

// Base template fixture combining executor type and config type
template<typename ExecutorType, typename ConfigType>
class ControllerOrchestratorFixtureTemplate : public ControllerOrchestratorFixtureBase
{
protected:
  std::shared_ptr<rclcpp::Executor> create_cm_executor() override
  {
    return ExecutorType::create_executor();
  }

  std::pair<std::string, std::string> get_config_files() const override
  {
    return ConfigType::get_config_files();
  }
};

// Executor type traits
struct MultiThreadedExecutorType {
  static std::shared_ptr<rclcpp::Executor> create_executor()
  {
    return std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }
  static const char *name() { return "MultiThreaded"; }
};

struct SingleThreadedExecutorType {
  static std::shared_ptr<rclcpp::Executor> create_executor()
  {
    return std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }
  static const char *name() { return "SingleThreaded"; }
};

// ---------------------------------------------------------------------------
// Test parameter combinations (Executor × Config)
// ---------------------------------------------------------------------------
struct NormalMultiThreaded {
  using ExecutorType = MultiThreadedExecutorType;
  using ConfigType = NormalConfig;
};

struct NormalSingleThreaded {
  using ExecutorType = SingleThreadedExecutorType;
  using ConfigType = NormalConfig;
};

struct MultipleChainedMultiThreaded {
  using ExecutorType = MultiThreadedExecutorType;
  using ConfigType = MultipleChainedConfig;
};

struct MultipleChainedSingleThreaded {
  using ExecutorType = SingleThreadedExecutorType;
  using ConfigType = MultipleChainedConfig;
};

// Typed fixture that wires executor/config combos into the common base
template<typename Param>
class ControllerOrchestratorTypedFixture
    : public ControllerOrchestratorFixtureTemplate<typename Param::ExecutorType, typename Param::ConfigType>
{
protected:
  using Base =
      ControllerOrchestratorFixtureTemplate<typename Param::ExecutorType, typename Param::ConfigType>;
  using Base::activity_sub_;
  using Base::controller_manager_;
  using Base::executor_;
  using Base::get_number_of_active_controllers;
  using Base::list_controllers;
  using Base::orchestrator_;
  using Base::spawner_node_;
  using Base::wait_for_activity;
  using Base::wait_for_list_states;
};

// Pretty type names for gtest
struct ControllerOrchestratorTestName {
  template<typename T>
  static std::string GetName( int )
  {
    return std::string( T::ConfigType::name() ) + "_" + T::ExecutorType::name();
  }
};

using ControllerOrchestratorTestTypes =
    ::testing::Types<NormalMultiThreaded, NormalSingleThreaded, MultipleChainedMultiThreaded,
                     MultipleChainedSingleThreaded>;

TYPED_TEST_SUITE( ControllerOrchestratorTypedFixture, ControllerOrchestratorTestTypes,
                  ControllerOrchestratorTestName );

TYPED_TEST( ControllerOrchestratorTypedFixture, SmartSwitchSync )
{
  const std::vector<std::string> requested = { "flipper_trajectory_controller" };
  const auto before_resp = this->list_controllers();
  ASSERT_NE( before_resp, nullptr );

  const auto expected_deactivate = compute_expected_deactivation( *before_resp, requested );

  this->activity_sub_->reset();
  std::atomic<bool> switch_done{ false };
  bool switch_success = false;
  std::thread switch_thread( [this, &requested, &switch_done, &switch_success]() {
    auto to_activate = requested;
    switch_success = this->orchestrator_->smartSwitchController( to_activate, 10, true );
    switch_done = true;
  } );
  ASSERT_TRUE( this->wait_for_activity( { { "flipper_trajectory_controller", "active" } }, 20s ) );
  ASSERT_TRUE( this->executor_->spin_until( [&switch_done]() { return switch_done.load(); }, 20s ) );
  switch_thread.join();
  ASSERT_TRUE( switch_success );

  const auto after_resp = this->list_controllers();
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

TYPED_TEST( ControllerOrchestratorTypedFixture, SmartSwitchAsyncSingleThreadedExecutor )
{
  const std::vector<std::string> requested = { "flipper_trajectory_controller" };

  this->activity_sub_->reset();
  std::atomic<bool> callback_done{ false };
  bool async_success = false;
  std::string async_message;

  std::thread async_thread( [this, &requested, &callback_done, &async_success, &async_message]() {
    this->orchestrator_->smartSwitchControllerAsync(
        requested,
        [&callback_done, &async_success, &async_message]( bool success, const std::string &message ) {
          async_success = success;
          async_message = message;
          callback_done = true;
        },
        true );
  } );

  ASSERT_TRUE( this->wait_for_activity( { { "flipper_trajectory_controller", "active" } }, 20s ) );
  ASSERT_TRUE(
      this->executor_->spin_until( [&callback_done]() { return callback_done.load(); }, 20s ) );
  async_thread.join();
  ASSERT_TRUE( async_success ) << async_message;
}

TYPED_TEST( ControllerOrchestratorTypedFixture, SmartSwitchAsync )
{
  const std::vector<std::string> first = { "flipper_trajectory_controller" };
  this->activity_sub_->reset();
  std::atomic<bool> initial_done{ false };
  bool initial_success = false;
  std::thread initial_thread( [this, &first, &initial_done, &initial_success]() {
    auto to_activate = first;
    initial_success = this->orchestrator_->smartSwitchController( to_activate, 10, true );
    initial_done = true;
  } );
  ASSERT_TRUE( this->wait_for_activity( { { "flipper_trajectory_controller", "active" } }, 20s ) );
  ASSERT_TRUE( this->executor_->spin_until( [&initial_done]() { return initial_done.load(); }, 20s ) );
  initial_thread.join();
  ASSERT_TRUE( initial_success );

  const std::vector<std::string> requested = { "flipper_velocity_controller" };
  const auto before_resp = this->list_controllers();
  ASSERT_NE( before_resp, nullptr );
  const auto expected_deactivate = compute_expected_deactivation( *before_resp, requested );

  std::atomic<bool> callback_done{ false };
  bool async_success = false;
  std::string async_message;

  this->activity_sub_->reset();
  std::thread async_thread( [this, &requested, &callback_done, &async_success, &async_message]() {
    this->orchestrator_->smartSwitchControllerAsync(
        requested,
        [&callback_done, &async_success, &async_message]( bool success, const std::string &message ) {
          async_success = success;
          async_message = message;
          callback_done = true;
        },
        true );
  } );

  ASSERT_TRUE( this->wait_for_activity( { { "flipper_velocity_controller", "active" } }, 20s ) );
  ASSERT_TRUE(
      this->executor_->spin_until( [&callback_done]() { return callback_done.load(); }, 20s ) );
  async_thread.join();
  ASSERT_TRUE( async_success ) << async_message;

  const auto after_resp = this->list_controllers();
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

TYPED_TEST( ControllerOrchestratorTypedFixture, RefreshControllerStates )
{
  ASSERT_TRUE( spin_while_executing(
      *this->executor_, [this]() { return this->orchestrator_->refreshControllerStates( 10 ); } ) );
}

TYPED_TEST( ControllerOrchestratorTypedFixture, ActivateDeactivateControllers )
{
  const std::string controller = "gripper_trajectory_controller";

  const auto before_resp = this->list_controllers();
  ASSERT_NE( before_resp, nullptr );
  const auto before_states = states_from_list( *before_resp );
  ASSERT_EQ( before_states.at( controller ), "active" );

  ASSERT_TRUE( spin_while_executing( *this->executor_, [this, &controller]() {
    return this->orchestrator_->deactivateControllers( { controller }, 10 );
  } ) );
  ASSERT_TRUE( this->wait_for_list_states( { { controller, "inactive" } }, 20s ) );

  const auto after_deactivate = this->list_controllers();
  ASSERT_NE( after_deactivate, nullptr );
  const auto after_states = states_from_list( *after_deactivate );
  EXPECT_EQ( after_states.at( controller ), "inactive" );

  ASSERT_TRUE( spin_while_executing( *this->executor_, [this, &controller]() {
    return this->orchestrator_->activateControllers( { controller }, 10 );
  } ) );
  ASSERT_TRUE( this->wait_for_list_states( { { controller, "active" } }, 20s ) );
}

TYPED_TEST( ControllerOrchestratorTypedFixture, GetActiveControllerOfHardwareInterface )
{
  bool is_multiple_chained_config =
      ( std::string( TypeParam::ConfigType::name() ) == "MultipleChained" );
  const auto flipper_controllers = spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->getActiveControllerOfHardwareInterface( "athena_flipper_interface",
                                                                        10 );
  } );
  // at startup, there should be a controller chain of two controllers active on the flipper interface
  EXPECT_NE( std::find( flipper_controllers.begin(), flipper_controllers.end(),
                        "flipper_velocity_controller" ),
             flipper_controllers.end() );
  EXPECT_NE(
      std::find( flipper_controllers.begin(), flipper_controllers.end(), "vel_to_pos_controller" ),
      flipper_controllers.end() );
  // make sure there are no false positives
  int flp_ctrls = is_multiple_chained_config ? 3 : 2;
  EXPECT_EQ( flipper_controllers.size(), flp_ctrls );

  const auto arm_controllers = spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->getActiveControllerOfHardwareInterface( "athena_arm_interface", 10 );
  } );
  // at startup, there should be two controllers active on the arm interface (not a chain)
  EXPECT_NE( std::find( arm_controllers.begin(), arm_controllers.end(), "arm_trajectory_controller" ),
             arm_controllers.end() );
  EXPECT_NE(
      std::find( arm_controllers.begin(), arm_controllers.end(), "gripper_trajectory_controller" ),
      arm_controllers.end() );
  if ( is_multiple_chained_config ) {
    EXPECT_NE( std::find( arm_controllers.begin(), arm_controllers.end(),
                          "arm_safety_position_controller" ),
               arm_controllers.end() );
  }
  int arm_ctrls = is_multiple_chained_config ? 3 : 2;
  EXPECT_EQ( arm_controllers.size(), arm_ctrls );

  const auto unknown_controllers = spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->getActiveControllerOfHardwareInterface( "unknown_interface", 10 );
  } );
  EXPECT_TRUE( unknown_controllers.empty() );
}

TYPED_TEST( ControllerOrchestratorTypedFixture, UnloadControllersOfJoint )
{
  bool is_multiple_chained_config =
      ( std::string( TypeParam::ConfigType::name() ) == "MultipleChained" );
  // case: unknown joint → no-op
  ASSERT_TRUE( spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->unloadControllersOfJoint( "unknown_joint", 10 );
  } ) );
  // case: joint with active controller → deactivate controller
  ASSERT_TRUE( spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->unloadControllersOfJoint( "gripper_servo_joint", 10 );
  } ) );
  ASSERT_TRUE(
      this->wait_for_list_states( { { "gripper_trajectory_controller", "inactive" } }, 20s ) );
  // case: controller chain → deactivate all controllers in chain
  // get number of currently active controllers in general
  const int active_before = this->get_number_of_active_controllers();
  ASSERT_GT( active_before, 0 );
  ASSERT_TRUE( spin_while_executing( *this->executor_, [this]() {
    return this->orchestrator_->unloadControllersOfJoint( "flipper_fl_joint", 10 );
  } ) );
  ASSERT_TRUE( this->wait_for_list_states(
      { { "flipper_velocity_controller", "inactive" }, { "vel_to_pos_controller", "inactive" } },
      20s ) );
  const int active_after = this->get_number_of_active_controllers();
  int flipper_ctrls = is_multiple_chained_config ? 3 : 2;
  EXPECT_EQ( active_before - active_after,
             flipper_ctrls ); // two controllers should have been deactivated, not more
}
