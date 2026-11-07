#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>

using controller_manager_msgs::msg::ControllerManagerActivity;
using controller_manager_msgs::msg::ControllerState;
using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::SwitchController;
using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

namespace
{
constexpr char kController[] = "arm_controller";
constexpr char kOtherController[] = "gripper_controller";

bool waitUntil( const std::function<bool()> &condition, std::chrono::milliseconds timeout )
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while ( std::chrono::steady_clock::now() < end ) {
    if ( condition() )
      return true;
    std::this_thread::sleep_for( 20ms );
  }
  return condition();
}

/// Every answer one async request got, in order.
struct Answers {
  std::mutex mutex;
  std::vector<std::pair<bool, std::string>> received;

  std::function<void( bool, const std::string & )> callback()
  {
    return [this]( bool success, const std::string &message ) {
      std::lock_guard<std::mutex> lock( mutex );
      received.emplace_back( success, message );
    };
  }
  size_t count()
  {
    std::lock_guard<std::mutex> lock( mutex );
    return received.size();
  }
  std::pair<bool, std::string> first()
  {
    std::lock_guard<std::mutex> lock( mutex );
    return received.front();
  }
};
} // namespace

/**
 * A stand-in controller manager. It records every switch request and answers with reply_, or not
 * at all, lists listing_, and publishes ~/activity as the real one does. It declares the
 * capabilities of hector_controller_manager unless a test says otherwise.
 */
class FakeControllerManagerTest : public HectorTestFixture
{
protected:
  /// Whether switch_controller answers at all. One that does not is a manager that died.
  virtual bool switchAnswers() const { return true; }
  /// Whether the manager declares the capabilities parameter with force_auto.
  virtual bool declaresForceAuto() const { return true; }

  void SetUp() override
  {
    HectorTestFixture::SetUp();

    // Named as the manager, so its parameter services are where the orchestrator asks
    fake_cm_node_ = std::make_shared<rclcpp::Node>( "fake_cm" );
    if ( declaresForceAuto() )
      fake_cm_node_->declare_parameter( "capabilities", std::vector<std::string>{ "force_auto" } );
    list_service_ = fake_cm_node_->create_service<ListControllers>(
        "/fake_cm/list_controllers", [this]( const ListControllers::Request::SharedPtr,
                                             ListControllers::Response::SharedPtr response ) {
          std::lock_guard<std::mutex> lock( mutex_ );
          ++listings_;
          response->controller = listing_;
        } );
    // The deferred form, so a request can go unanswered without blocking an executor thread
    switch_service_ = fake_cm_node_->create_service<SwitchController>(
        "/fake_cm/switch_controller",
        [this]( const std::shared_ptr<rclcpp::Service<SwitchController>> service,
                const std::shared_ptr<rmw_request_id_t> header,
                const SwitchController::Request::SharedPtr request ) {
          SwitchController::Response response;
          {
            std::lock_guard<std::mutex> lock( mutex_ );
            requests_.push_back( *request );
            response = reply_;
          }
          if ( switchAnswers() )
            service->send_response( *header, response );
        } );
    activity_publisher_ = fake_cm_node_->create_publisher<ControllerManagerActivity>(
        "/fake_cm/activity", rclcpp::QoS( 1 ).reliable().transient_local() );
    reply_.ok = true;

    // Its own executor, so the orchestrator's responses are delivered by the fixture's executor
    fake_cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_cm_executor_->add_node( fake_cm_node_ );
    fake_cm_thread_ = std::thread( [this]() { fake_cm_executor_->spin(); } );

    orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>( tester_node_,
                                                                                       "/fake_cm" );
  }

  void TearDown() override
  {
    orchestrator_.reset();
    fake_cm_executor_->cancel();
    if ( fake_cm_thread_.joinable() )
      fake_cm_thread_.join();
    switch_service_.reset();
    list_service_.reset();
    activity_publisher_.reset();
    fake_cm_node_.reset();
    fake_cm_executor_.reset();
    HectorTestFixture::TearDown();
  }

  /// The orchestrator refuses outright while its client has not discovered the service, which
  /// would pass a test for the wrong reason.
  void waitForService()
  {
    auto client =
        tester_node_->create_test_client<SwitchController>( "/fake_cm/switch_controller" );
    ASSERT_TRUE( client->wait_for_service( *executor_, 10s ) );
  }

  void publishActivity( const std::vector<std::pair<std::string, uint8_t>> &controllers )
  {
    ControllerManagerActivity message;
    for ( const auto &[name, id] : controllers ) {
      controller_manager_msgs::msg::NamedLifecycleState state;
      state.name = name;
      state.state.id = id;
      message.controllers.push_back( state );
    }
    activity_publisher_->publish( message );
  }

  std::vector<SwitchController::Request> requests()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return requests_;
  }

  void reply( bool ok, const std::string &message )
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    reply_.ok = ok;
    reply_.message = message;
  }

  void list( std::vector<ControllerState> controllers )
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    listing_ = std::move( controllers );
  }

  int listings()
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    return listings_;
  }

  std::shared_ptr<rclcpp::Node> fake_cm_node_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::Publisher<ControllerManagerActivity>::SharedPtr activity_publisher_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> fake_cm_executor_;
  std::thread fake_cm_thread_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::mutex mutex_;
  std::vector<SwitchController::Request> requests_;
  SwitchController::Response reply_;
  std::vector<ControllerState> listing_;
  int listings_{ 0 };
};

/// Never answers switch_controller, as when the manager dies with a request in flight.
class StalledSwitchTest : public FakeControllerManagerTest
{
protected:
  bool switchAnswers() const override { return false; }
};

/// Declares no capabilities, as the stock ros2_control controller manager.
class StockControllerManagerTest : public FakeControllerManagerTest
{
protected:
  bool declaresForceAuto() const override { return false; }
};

ControllerState controller( const std::string &name, const std::string &state,
                            std::vector<std::string> command_interfaces,
                            std::vector<std::string> reference_interfaces = {} )
{
  ControllerState controller;
  controller.name = name;
  controller.state = state;
  controller.required_command_interfaces = std::move( command_interfaces );
  controller.reference_interfaces = std::move( reference_interfaces );
  controller.is_chainable = !controller.reference_interfaces.empty();
  return controller;
}

constexpr uint8_t kActive = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
constexpr uint8_t kInactive = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

// The controller manager resolves providers and conflicts, so a switch is exactly one request
TEST_F( FakeControllerManagerTest, ASwitchIsOneForceAutoRequest )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  EXPECT_TRUE( orchestrator_->smartSwitchController( { kController, kOtherController }, 3 ) );

  const auto sent = requests();
  ASSERT_EQ( sent.size(), 1u );
  EXPECT_EQ( sent[0].activate_controllers,
             ( std::vector<std::string>{ kController, kOtherController } ) );
  EXPECT_TRUE( sent[0].deactivate_controllers.empty() );
  EXPECT_EQ( sent[0].strictness, SwitchController::Request::FORCE_AUTO );
  // What is left of the timeout after reading the capabilities
  EXPECT_LE( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 3s ) );
  EXPECT_GT( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 2500ms ) );
  EXPECT_EQ( listings(), 0 );
}

TEST_F( FakeControllerManagerTest, AnAsyncSwitchIsOneForceAutoRequestAnsweredOnce )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { kController }, answers.callback(), 2 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_TRUE( answers.first().first ) << answers.first().second;

  const auto sent = requests();
  ASSERT_EQ( sent.size(), 1u );
  EXPECT_EQ( sent[0].activate_controllers, std::vector<std::string>{ kController } );
  EXPECT_EQ( sent[0].strictness, SwitchController::Request::FORCE_AUTO );
  EXPECT_LE( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 2s ) );
  EXPECT_GT( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 1500ms ) );

  // Past the deadline, which must not answer a second time
  std::this_thread::sleep_for( 3500ms );
  EXPECT_EQ( answers.count(), 1u );
}

TEST_F( FakeControllerManagerTest, ARefusedSwitchFailsWithTheManagersReason )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  reply( false, "arm_torque_off_controller is protected" );

  EXPECT_FALSE( orchestrator_->smartSwitchController( { kController }, 3 ) );

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { kController }, answers.callback(), 3 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_FALSE( answers.first().first );
  EXPECT_EQ( answers.first().second, "arm_torque_off_controller is protected" );
  EXPECT_EQ( requests().size(), 2u );
}

TEST_F( FakeControllerManagerTest, AnEmptySwitchSendsNothing )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  EXPECT_TRUE( orchestrator_->smartSwitchController( {}, 3 ) );
  Answers answers;
  orchestrator_->smartSwitchControllerAsync( {}, answers.callback(), 3 );
  ASSERT_EQ( answers.count(), 1u );
  EXPECT_TRUE( answers.first().first );
  std::this_thread::sleep_for( 200ms );
  EXPECT_TRUE( requests().empty() );
}

TEST_F( StalledSwitchTest, AnUnansweredSwitchFailsAtItsDeadline )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { kController }, answers.callback(), 1 );

  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) )
      << "the request was never answered although the controller manager did not answer";
  EXPECT_FALSE( answers.first().first );
  EXPECT_EQ( requests().size(), 1u );

  std::this_thread::sleep_for( 2500ms );
  EXPECT_EQ( answers.count(), 1u );
}

TEST_F( StalledSwitchTest, ABlockingSwitchGivesUpAfterItsTimeout )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE( orchestrator_->smartSwitchController( { kController }, 1 ) );
  EXPECT_LT( std::chrono::steady_clock::now() - start, 3s );
}

TEST_F( StalledSwitchTest, DestructionWaitsForACallbackInProgress )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  std::atomic<bool> entered{ false };
  std::atomic<bool> finished{ false };
  orchestrator_->smartSwitchControllerAsync(
      { kController },
      [&]( bool, const std::string & ) {
        entered = true;
        std::this_thread::sleep_for( 500ms );
        finished = true;
      },
      1 );
  ASSERT_TRUE( waitUntil( [&] { return entered.load(); }, 5000ms ) );

  orchestrator_.reset();
  EXPECT_TRUE( finished.load() ) << "the orchestrator was destroyed while its callback still ran";
}

TEST_F( StalledSwitchTest, AnOrchestratorCanBeDestroyedFromItsOwnCallback )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();

  std::atomic<bool> destroyed{ false };
  orchestrator_->smartSwitchControllerAsync(
      { kController },
      [&]( bool, const std::string & ) {
        orchestrator_.reset();
        destroyed = true;
      },
      1 );
  EXPECT_TRUE( waitUntil( [&] { return destroyed.load(); }, 5000ms ) )
      << "destroying the orchestrator from its own callback did not return";
}

// Without a controller manager the callback still runs, once, with the reason
TEST_F( FakeControllerManagerTest, AnAsyncSwitchWithoutAControllerManagerFailsOnce )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  controller_orchestrator::ControllerOrchestrator orphan( tester_node_, "/no_cm" );

  Answers answers;
  orphan.smartSwitchControllerAsync( { kController }, answers.callback(), 1 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_FALSE( answers.first().first );
  EXPECT_NE( answers.first().second.find( "not available" ), std::string::npos )
      << answers.first().second;

  // Past the deadline, which must not answer a second time
  std::this_thread::sleep_for( 2500ms );
  EXPECT_EQ( answers.count(), 1u );
}

// A switch requested right after start-up waits for the service within its own timeout
TEST_F( FakeControllerManagerTest, AnAsyncSwitchWaitsForALateControllerManager )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  // Unique, so a service of an earlier run still in the discovery cache cannot answer for it
  const std::string manager = "/late_cm_" + std::to_string( getpid() );
  controller_orchestrator::ControllerOrchestrator orchestrator( tester_node_, manager );

  Answers answers;
  orchestrator.smartSwitchControllerAsync( { kController }, answers.callback(), 3 );
  std::this_thread::sleep_for( 500ms );
  std::atomic<int> requests{ 0 };
  std::atomic<int64_t> request_timeout_ns{ 0 };
  const auto late_node = std::make_shared<rclcpp::Node>( manager.substr( 1 ) );
  late_node->declare_parameter( "capabilities", std::vector<std::string>{ "force_auto" } );
  const auto late_list = late_node->create_service<ListControllers>(
      "~/list_controllers",
      []( const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr ) { } );
  const auto late_service = late_node->create_service<SwitchController>(
      "~/switch_controller", [&]( const SwitchController::Request::SharedPtr request,
                                  SwitchController::Response::SharedPtr response ) {
        request_timeout_ns = rclcpp::Duration( request->timeout ).nanoseconds();
        requests++;
        response->ok = true;
      } );
  fake_cm_executor_->add_node( late_node );

  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_TRUE( answers.first().first ) << answers.first().second;
  EXPECT_EQ( requests.load(), 1 );
  EXPECT_LT( request_timeout_ns.load(), rclcpp::Duration( 2500ms ).nanoseconds() )
      << "the wait is part of the timeout";
  EXPECT_GT( request_timeout_ns.load(), 0 );
  fake_cm_executor_->remove_node( late_node );
}

TEST_F( FakeControllerManagerTest, AnOrchestratorNotOwnedByASharedPtrAnswers )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  controller_orchestrator::ControllerOrchestrator orchestrator( tester_node_, "/fake_cm" );
  // Blocking first, because it waits for its client to discover the service
  ASSERT_TRUE( orchestrator.smartSwitchController( { kController }, 3 ) );

  Answers answers;
  orchestrator.smartSwitchControllerAsync( { kController }, answers.callback() );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_TRUE( answers.first().first ) << answers.first().second;
}

TEST_F( FakeControllerManagerTest, ANodeOwningItsOrchestratorIsReleased )
{
  struct OwningNode : rclcpp::Node {
    OwningNode() : rclcpp::Node( "orchestrator_owner" ) { }
    std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator;
  };
  auto owner = std::make_shared<OwningNode>();
  owner->orchestrator =
      std::make_shared<controller_orchestrator::ControllerOrchestrator>( owner, "/fake_cm" );
  const std::weak_ptr<OwningNode> weak_owner = owner;

  owner.reset();
  EXPECT_TRUE( weak_owner.expired() ) << "the orchestrator keeps the node that owns it alive";
}

TEST_F( FakeControllerManagerTest, ControllersAreActiveAsTheLatestActivitySays )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );

  publishActivity( { { kController, kActive }, { kOtherController, kInactive } } );
  ASSERT_TRUE( waitUntil( [&] { return orchestrator_->isControllerActive( kController ); }, 5000ms ) );
  EXPECT_TRUE( orchestrator_->areControllersActive( { kController } ) );
  EXPECT_FALSE( orchestrator_->areControllersActive( { kController, kOtherController } ) );
  EXPECT_FALSE( orchestrator_->areControllersActive( { "unknown_controller" } ) );

  publishActivity( { { kController, kInactive }, { kOtherController, kActive } } );
  ASSERT_TRUE(
      waitUntil( [&] { return orchestrator_->isControllerActive( kOtherController ); }, 5000ms ) );
  EXPECT_FALSE( orchestrator_->areControllersActive( { kController } ) );
}

// The manager latches its activity, so an orchestrator built later knows the states at once
TEST_F( FakeControllerManagerTest, AnOrchestratorBuiltAfterTheActivityKnowsTheStates )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  publishActivity( { { kController, kActive } } );

  controller_orchestrator::ControllerOrchestrator late( tester_node_, "/fake_cm" );
  EXPECT_TRUE( waitUntil( [&] { return late.areControllersActive( { kController } ); }, 5000ms ) );
}

// Stock ros2_control runs FORCE_AUTO as BEST_EFFORT, so the orchestrator plans the chain and the
// conflicts itself and sends them as one STRICT request
TEST_F( StockControllerManagerTest, AChainReplacesTheChainOnItsJointInOneStrictSwitch )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  list( { controller( "old_head", "active", { "old_safety/joint_1/position" } ),
          controller( "old_safety", "active", { "joint_1/position" }, { "joint_1/position" } ),
          controller( "arm_trajectory", "inactive", { "arm_safety/joint_1/position" } ),
          controller( "arm_safety", "inactive", { "joint_1/position" }, { "joint_1/position" } ),
          controller( "flipper", "active", { "joint_2/position" } ) } );

  EXPECT_TRUE( orchestrator_->smartSwitchController( { "arm_trajectory" }, 3 ) );

  const auto sent = requests();
  ASSERT_EQ( sent.size(), 1u );
  EXPECT_EQ( sent[0].strictness, SwitchController::Request::STRICT );
  EXPECT_EQ( sent[0].activate_controllers,
             ( std::vector<std::string>{ "arm_trajectory", "arm_safety" } ) );
  EXPECT_EQ( sent[0].deactivate_controllers,
             ( std::vector<std::string>{ "old_head", "old_safety" } ) );
  EXPECT_GT( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 2500ms ) );
}

// A conflict on a reference interface stops the other head, and what reads its state with it
TEST_F( StockControllerManagerTest, AConflictOnAReferenceStopsTheOtherHeadAndItsPreceders )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  auto head_a = controller( "head_a", "active", { "safety/joint_1/position" } );
  head_a.exported_state_interfaces = { "joint_1/estimate" };
  head_a.is_chainable = true;
  auto watcher = controller( "watcher", "active", {} );
  watcher.required_state_interfaces = { "head_a/joint_1/estimate" };
  list( { controller( "safety", "active", { "joint_1/position" }, { "joint_1/position" } ), head_a,
          watcher, controller( "head_b", "inactive", { "safety/joint_1/position" } ) } );

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { "head_b" }, answers.callback(), 3 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_TRUE( answers.first().first ) << answers.first().second;

  const auto sent = requests();
  ASSERT_EQ( sent.size(), 1u );
  EXPECT_EQ( sent[0].strictness, SwitchController::Request::STRICT );
  EXPECT_EQ( sent[0].activate_controllers, std::vector<std::string>{ "head_b" } );
  EXPECT_EQ( sent[0].deactivate_controllers, ( std::vector<std::string>{ "head_a", "watcher" } ) );
}

TEST_F( StockControllerManagerTest, AFailedActivationIsAFailure )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  list( { controller( kController, "inactive", { "joint_1/position" } ) } );
  reply( false, "Could not activate controller : 'arm_controller'" );

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { kController }, answers.callback(), 3 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) );
  EXPECT_FALSE( answers.first().first );
  EXPECT_EQ( answers.first().second, "Could not activate controller : 'arm_controller'" );
  EXPECT_FALSE( orchestrator_->smartSwitchController( { kController }, 3 ) );
}

// The stock manager drops such a controller from a switch and still answers ok
TEST_F( StockControllerManagerTest, AControllerThatCannotBeActivatedFailsWithoutASwitch )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  list( { controller( kController, "inactive", { "joint_1/position" } ),
          controller( kOtherController, "unconfigured", {} ) } );

  EXPECT_FALSE( orchestrator_->smartSwitchController( { kController, kOtherController }, 3 ) );
  EXPECT_FALSE( orchestrator_->smartSwitchController( { "unknown_controller" }, 3 ) );
  EXPECT_TRUE( requests().empty() );
}

TEST_F( StockControllerManagerTest, ActiveControllersNeedNoSwitch )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  list( { controller( kController, "active", { "joint_1/position" } ) } );

  EXPECT_TRUE( orchestrator_->smartSwitchController( { kController }, 3 ) );
  EXPECT_TRUE( requests().empty() );
}

// A restarted manager may be another one, so its capabilities are read again
TEST_F( FakeControllerManagerTest, ARestartedControllerManagerIsAskedAgain )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForService();
  publishActivity( { { kController, kActive } } );
  ASSERT_TRUE( waitUntil( [&] { return orchestrator_->isControllerActive( kController ); }, 5000ms ) );
  ASSERT_TRUE( orchestrator_->smartSwitchController( { kController }, 3 ) );
  ASSERT_EQ( requests().back().strictness, SwitchController::Request::FORCE_AUTO );

  fake_cm_node_->set_parameter( rclcpp::Parameter( "capabilities", std::vector<std::string>{} ) );
  list( { controller( kController, "inactive", { "joint_1/position" } ) } );
  // Unchanged until the manager is seen to restart
  ASSERT_TRUE( orchestrator_->smartSwitchController( { kController }, 3 ) );
  ASSERT_EQ( requests().back().strictness, SwitchController::Request::FORCE_AUTO );

  activity_publisher_ = fake_cm_node_->create_publisher<ControllerManagerActivity>(
      "/fake_cm/activity", rclcpp::QoS( 1 ).reliable().transient_local() );
  publishActivity( { { kController, kInactive } } );
  ASSERT_TRUE(
      waitUntil( [&] { return !orchestrator_->isControllerActive( kController ); }, 5000ms ) );

  ASSERT_TRUE( orchestrator_->smartSwitchController( { kController }, 3 ) );
  EXPECT_EQ( requests().back().strictness, SwitchController::Request::STRICT );
}
