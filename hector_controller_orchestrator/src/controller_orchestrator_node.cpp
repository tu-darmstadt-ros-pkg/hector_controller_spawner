#include <memory>
#include <string>
#include <vector>

#include "controller_orchestrator/controller_orchestrator.hpp"
#include <rclcpp/rclcpp.hpp>

// This node is purely for testing the ControllerOrchestrator class quickly.

int main( int argc, char **argv )
{
  // 1) Initialize ROS 2
  rclcpp::init( argc, argv );

  // 2) Create a ROS 2 node
  auto node = rclcpp::Node::make_shared( "test_smart_switch_node" );

  // 3) Instantiate the ControllerOrchestrator, pointing to your controller_manager
  //    (replace "/athena/controller_manager" if your namespace is different)
  auto orchestrator = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node, "/athena/controller_manager" );

  // 4) Set up a one‐shot timer to invoke smartSwitchController after ~1 second
  rclcpp::TimerBase::SharedPtr timer =
      node->create_wall_timer( std::chrono::seconds( 1 ), [orchestrator, node]() {
        static bool called = false;
        if ( called ) {
          return; // Only run once
        }
        called = true;

        // deactivate controllers of athena_flipper_interface
        auto to_deactivate =
            orchestrator->getActiveControllerOfHardwareInterface( "athena_flipper_interface", 10 );
        // print the controllers to deactivate
        RCLCPP_INFO( node->get_logger(), "[Test] Controllers to deactivate: [%zu]",
                     to_deactivate.size() );
        for ( const auto &controller : to_deactivate ) {
          RCLCPP_INFO( node->get_logger(), "  - %s", controller.c_str() );
        }
        if ( !orchestrator->deactivateControllers( to_deactivate, 10 ) ) {
          RCLCPP_ERROR( node->get_logger(), "[Test] Failed to deactivate controllers" );
        } else {
          RCLCPP_INFO( node->get_logger(), "[Test] Successfully deactivated controllers" );
        }
      } );

  // 5) Create a MultiThreadedExecutor with at least 2 threads.
  //    This lets one thread block on future.wait_for(...) while another
  //    thread handles incoming service callbacks.
  rclcpp::executors::MultiThreadedExecutor executor( rclcpp::ExecutorOptions(),
                                                     2 // two threads minimum
  );
  executor.add_node( node );

  // 6) Spin until shutdown (Ctrl+C). The timer will fire once after ~1 second.
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
