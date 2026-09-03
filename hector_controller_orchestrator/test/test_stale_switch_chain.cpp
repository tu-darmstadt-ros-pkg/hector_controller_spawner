#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>

using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::SwitchController;
using hector_testing_utils::HectorTestFixture;
using namespace std::chrono_literals;

namespace
{
constexpr char kControllerName[] = "test_controller";
constexpr auto kListDelay = 3s;
} // namespace

/**
 * A controller manager that answers list_controllers only long after the caller has given up.
 * The synchronous switch times out first, which is what the real manager does while it holds its
 * resource lock.
 */
class SlowControllerManagerTest : public HectorTestFixture
{
protected:
  void SetUp() override
  {
    HectorTestFixture::SetUp();

    fake_cm_node_ = std::make_shared<rclcpp::Node>( "fake_controller_manager" );
    list_service_ = fake_cm_node_->create_service<ListControllers>(
        "/fake_cm/list_controllers", [this]( const ListControllers::Request::SharedPtr,
                                             ListControllers::Response::SharedPtr response ) {
          std::this_thread::sleep_for( kListDelay );
          controller_manager_msgs::msg::ControllerState controller;
          controller.name = kControllerName;
          controller.state = "inactive";
          response->controller.push_back( controller );
        } );
    switch_service_ = fake_cm_node_->create_service<SwitchController>(
        "/fake_cm/switch_controller", [this]( const SwitchController::Request::SharedPtr,
                                              SwitchController::Response::SharedPtr response ) {
          switch_requests_++;
          response->ok = true;
        } );

    // Its own multi-threaded executor: the delayed handler must not stall the switch service, and
    // the orchestrator's responses are delivered by the fixture's executor, not this one.
    fake_cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    fake_cm_executor_->add_node( fake_cm_node_ );
    fake_cm_thread_ = std::thread( [this]() { fake_cm_executor_->spin(); } );

    orchestrator_ =
        controller_orchestrator::ControllerOrchestrator::create( tester_node_, "/fake_cm" );
  }

  void TearDown() override
  {
    orchestrator_.reset();
    fake_cm_executor_->cancel();
    if ( fake_cm_thread_.joinable() )
      fake_cm_thread_.join();
    list_service_.reset();
    switch_service_.reset();
    fake_cm_node_.reset();
    fake_cm_executor_.reset();
    HectorTestFixture::TearDown();
  }

  std::shared_ptr<rclcpp::Node> fake_cm_node_;
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> fake_cm_executor_;
  std::thread fake_cm_thread_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::atomic<int> switch_requests_{ 0 };
};

TEST_F( SlowControllerManagerTest, AChainAbandonedByItsCallerDoesNotSwitchLater )
{
  hector_testing_utils::TestExecutor::ScopedSpinner spinner( *executor_ );
  // The orchestrator refuses outright while its clients have not discovered the services, which
  // would pass this test for the wrong reason.
  auto list_client =
      tester_node_->create_test_client<ListControllers>( "/fake_cm/list_controllers" );
  auto switch_client =
      tester_node_->create_test_client<SwitchController>( "/fake_cm/switch_controller" );
  ASSERT_TRUE( list_client->wait_for_service( *executor_, 10s ) );
  ASSERT_TRUE( switch_client->wait_for_service( *executor_, 10s ) );

  std::vector<std::string> activate = { kControllerName };
  EXPECT_FALSE( orchestrator_->smartSwitchController( activate, 1 ) )
      << "the switch was expected to time out waiting for the listing";
  EXPECT_EQ( switch_requests_.load(), 0 ) << "a switch went out before the caller gave up";

  // Long enough for the delayed listing to arrive and for the chain it would start to run.
  std::this_thread::sleep_for( kListDelay + 2s );

  EXPECT_EQ( switch_requests_.load(), 0 )
      << "the listing arrived after the caller gave up and the chain switched controllers anyway";
}
