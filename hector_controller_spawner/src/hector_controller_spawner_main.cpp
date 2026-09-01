#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "hector_controller_spawner/hector_controller_spawner.hpp"

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
      node->start_sequence( initial_init ); // safe - not inside another callback
      if ( !node->is_tracking_estop() || !node->restart_after_estop_deactivation() )
        break;
      initial_init = false;
    }
    std::this_thread::sleep_for( 50ms );
  }

  // A startup sequence that could not put the robot into the requested state says so here. The
  // switch requests are best effort and report ok whatever the manager dropped, so without this
  // the process exits successfully having activated nothing.
  const bool failed = node->sequence_failed();
  node.reset();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
