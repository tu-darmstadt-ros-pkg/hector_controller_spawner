#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>

using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::ListHardwareComponents;
using controller_manager_msgs::srv::SwitchController;
using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

namespace
{
constexpr char kControllerName[] = "test_controller";
constexpr char kComponentName[] = "test_component";
constexpr auto kListDelay = 3s;
} // namespace

/**
 * A stand-in controller manager offering one inactive controller. Subclasses choose how it
 * misbehaves.
 */
class FakeControllerManagerTest : public HectorTestFixture
{
protected:
  /// How long list_controllers takes to answer.
  virtual std::chrono::seconds listDelay() const { return 0s; }
  /// How long list_hardware_components takes to answer.
  virtual std::chrono::seconds hardwareListDelay() const { return 0s; }
  /// Whether switch_controller answers at all. One that does not is a manager that died mid-chain.
  virtual bool switchAnswers() const { return true; }

  void SetUp() override
  {
    HectorTestFixture::SetUp();

    fake_cm_node_ = std::make_shared<rclcpp::Node>( "fake_controller_manager" );
    list_service_ = fake_cm_node_->create_service<ListControllers>(
        "/fake_cm/list_controllers", [this]( const ListControllers::Request::SharedPtr,
                                             ListControllers::Response::SharedPtr response ) {
          std::this_thread::sleep_for( listDelay() );
          controller_manager_msgs::msg::ControllerState controller;
          controller.name = kControllerName;
          controller.state = "inactive";
          response->controller.push_back( controller );
        } );
    hardware_service_ = fake_cm_node_->create_service<ListHardwareComponents>(
        "/fake_cm/list_hardware_components",
        [this]( const ListHardwareComponents::Request::SharedPtr,
                ListHardwareComponents::Response::SharedPtr response ) {
          std::this_thread::sleep_for( hardwareListDelay() );
          controller_manager_msgs::msg::HardwareInterface interface;
          interface.name = "test_joint/position";
          controller_manager_msgs::msg::HardwareComponentState component;
          component.name = kComponentName;
          component.command_interfaces.push_back( interface );
          response->component.push_back( component );
        } );
    // The deferred form, so a request can go unanswered without blocking an executor thread.
    switch_service_ = fake_cm_node_->create_service<SwitchController>(
        "/fake_cm/switch_controller",
        [this]( const std::shared_ptr<rclcpp::Service<SwitchController>> service,
                const std::shared_ptr<rmw_request_id_t> header,
                const SwitchController::Request::SharedPtr ) {
          switch_requests_++;
          if ( !switchAnswers() )
            return;
          SwitchController::Response response;
          response.ok = true;
          service->send_response( *header, response );
        } );

    // Its own multi-threaded executor: the delayed handler must not stall the switch service, and
    // the orchestrator's responses are delivered by the fixture's executor, not this one.
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
    list_service_.reset();
    hardware_service_.reset();
    switch_service_.reset();
    fake_cm_node_.reset();
    fake_cm_executor_.reset();
    HectorTestFixture::TearDown();
  }

  std::shared_ptr<rclcpp::Node> fake_cm_node_;
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::Service<ListHardwareComponents>::SharedPtr hardware_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> fake_cm_executor_;
  std::thread fake_cm_thread_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::atomic<int> switch_requests_{ 0 };

  /// The orchestrator refuses outright while its clients have not discovered the services, which
  /// would pass a test for the wrong reason.
  void waitForServices()
  {
    auto list_client =
        tester_node_->create_test_client<ListControllers>( "/fake_cm/list_controllers" );
    auto hardware_client = tester_node_->create_test_client<ListHardwareComponents>(
        "/fake_cm/list_hardware_components" );
    auto switch_client =
        tester_node_->create_test_client<SwitchController>( "/fake_cm/switch_controller" );
    ASSERT_TRUE( list_client->wait_for_service( *executor_, 10s ) );
    ASSERT_TRUE( hardware_client->wait_for_service( *executor_, 10s ) );
    ASSERT_TRUE( switch_client->wait_for_service( *executor_, 10s ) );
  }
};

/**
 * Answers list_controllers only long after the caller has given up. The synchronous switch times
 * out first, which is what the real manager does while it holds its resource lock.
 */
class SlowControllerManagerTest : public FakeControllerManagerTest
{
protected:
  std::chrono::seconds listDelay() const override { return kListDelay; }
};

/// Also takes a second for list_hardware_components, so a query of both services outlives a
/// timeout applied to each of them separately.
class SlowListingsTest : public SlowControllerManagerTest
{
protected:
  std::chrono::seconds hardwareListDelay() const override { return 1s; }
};

/// Never answers switch_controller, as when the manager dies with a chain in flight.
class StalledSwitchTest : public FakeControllerManagerTest
{
protected:
  bool switchAnswers() const override { return false; }
};

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

TEST_F( SlowControllerManagerTest, AChainAbandonedByItsCallerDoesNotSwitchLater )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  std::vector<std::string> activate = { kControllerName };
  EXPECT_FALSE( orchestrator_->smartSwitchController( activate, 1 ) )
      << "the switch was expected to time out waiting for the listing";
  EXPECT_EQ( switch_requests_.load(), 0 ) << "a switch went out before the caller gave up";

  // Long enough for the delayed listing to arrive and for the chain it would start to run.
  std::this_thread::sleep_for( kListDelay + 2s );

  EXPECT_EQ( switch_requests_.load(), 0 )
      << "the listing arrived after the caller gave up and the chain switched controllers anyway";
}

TEST_F( StalledSwitchTest, AStepLeftUnansweredFailsTheRequestAtItsDeadline )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  Answers answers;
  orchestrator_->smartSwitchControllerAsync( { kControllerName }, answers.callback(), 1 );

  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 5000ms ) )
      << "the request was never answered although its switch step went unanswered";
  EXPECT_EQ( switch_requests_.load(), 1 ) << "the chain did not reach its switch step";
  const auto [success, message] = answers.first();
  EXPECT_FALSE( success );
  EXPECT_NE( message.find( std::string( "Activating " ) + kControllerName ), std::string::npos )
      << "the answer does not name the step that stalled: " << message;

  // Several more deadlines' worth: the timer re-fires, and must not answer a second time.
  std::this_thread::sleep_for( 2500ms );
  EXPECT_EQ( answers.count(), 1u );
}

TEST_F( StalledSwitchTest, AStalledRequestIsAnsweredWhenANewerOneArrives )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  // A deadline far beyond the test, so only the newer request can be what answers the first.
  Answers first;
  orchestrator_->smartSwitchControllerAsync( { kControllerName }, first.callback(), 60 );
  ASSERT_TRUE( waitUntil( [&] { return switch_requests_.load() == 1; }, 5000ms ) )
      << "the first chain did not reach its switch step";
  EXPECT_EQ( first.count(), 0u );

  Answers second;
  orchestrator_->smartSwitchControllerAsync( { kControllerName }, second.callback(), 60 );

  ASSERT_TRUE( waitUntil( [&] { return first.count() > 0; }, 2000ms ) )
      << "the stalled request was left waiting after a newer one took the controllers over";
  const auto [success, message] = first.first();
  EXPECT_FALSE( success );
  EXPECT_NE( message.find( "superseded" ), std::string::npos ) << message;
  EXPECT_EQ( first.count(), 1u );
}

TEST_F( StalledSwitchTest, DestructionWaitsForACallbackInProgress )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  std::atomic<bool> entered{ false };
  std::atomic<bool> finished{ false };
  orchestrator_->smartSwitchControllerAsync(
      { kControllerName },
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
  waitForServices();

  std::atomic<bool> destroyed{ false };
  orchestrator_->smartSwitchControllerAsync(
      { kControllerName },
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
  controller_orchestrator::ControllerOrchestrator orchestrator( tester_node_, "/fake_cm" );
  // Blocking first, because it waits for its client to discover the service.
  ASSERT_TRUE( orchestrator.refreshControllerStates( 10 ) );

  Answers answers;
  orchestrator.refreshControllerStatesAsync( answers.callback() );
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

TEST_F( SlowListingsTest, AQueryOfTwoServicesKeepsToItsTimeout )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  const auto start = std::chrono::steady_clock::now();
  const auto result = orchestrator_->getActiveControllerOfHardwareInterface( kComponentName, 2 );
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE( result.has_value() );
  EXPECT_LT( elapsed, 2500ms ) << "the timeout applied to each service call, not to the query";
}

TEST_F( SlowControllerManagerTest, AnAsyncRefreshIsAnsweredAtItsTimeout )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  waitForServices();

  Answers answers;
  orchestrator_->refreshControllerStatesAsync( answers.callback(), 1 );
  ASSERT_TRUE( waitUntil( [&] { return answers.count() > 0; }, 2000ms ) )
      << "the refresh was not answered at its timeout";
  EXPECT_FALSE( answers.first().first );

  // The listing still arrives, and must not answer a second time.
  std::this_thread::sleep_for( kListDelay );
  EXPECT_EQ( answers.count(), 1u );
}
