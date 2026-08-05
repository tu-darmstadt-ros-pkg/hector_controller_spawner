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
  bool success = true;
  using namespace std::chrono_literals;
  while ( rclcpp::ok() ) {
    rclcpp::spin_some( node );
    if ( node->estop_released_and_not_in_progress() ) {
      success = node->start_sequence( initial_init ); // safe - not inside another callback
      if ( !node->is_tracking_estop() || !node->restart_after_estop_deactivation() )
        break;
      // When tracking the e-stop we stay alive and let the next release retry, but a failed run
      // must not be retried back-to-back at loop speed.
      if ( !success ) {
        RCLCPP_ERROR( node->get_logger(),
                      "Start sequence failed – waiting for the next e-stop release to retry." );
        node->mark_sequence_done();
      }
      initial_init = false;
    }
    std::this_thread::sleep_for( 50ms );
  }

  // A sequence cut short by SIGINT is a shutdown, not a failure of the spawner.
  const bool interrupted = !rclcpp::ok();

  node.reset();
  rclcpp::shutdown();
  return ( success || interrupted ) ? 0 : 1;
}
