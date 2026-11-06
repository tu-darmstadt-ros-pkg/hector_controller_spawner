#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>

using controller_manager_msgs::msg::ControllerManagerActivity;
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
 * at all, and publishes ~/activity as the real one does.
 */
class FakeControllerManagerTest : public HectorTestFixture
{
protected:
  /// Whether switch_controller answers at all. One that does not is a manager that died.
  virtual bool switchAnswers() const { return true; }

  void SetUp() override
  {
    HectorTestFixture::SetUp();

    fake_cm_node_ = std::make_shared<rclcpp::Node>( "fake_controller_manager" );
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

  std::shared_ptr<rclcpp::Node> fake_cm_node_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  rclcpp::Publisher<ControllerManagerActivity>::SharedPtr activity_publisher_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> fake_cm_executor_;
  std::thread fake_cm_thread_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::mutex mutex_;
  std::vector<SwitchController::Request> requests_;
  SwitchController::Response reply_;
};

/// Never answers switch_controller, as when the manager dies with a request in flight.
class StalledSwitchTest : public FakeControllerManagerTest
{
protected:
  bool switchAnswers() const override { return false; }
};

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
  EXPECT_EQ( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 3s ) );
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
  EXPECT_EQ( rclcpp::Duration( sent[0].timeout ), rclcpp::Duration( 2s ) );

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
