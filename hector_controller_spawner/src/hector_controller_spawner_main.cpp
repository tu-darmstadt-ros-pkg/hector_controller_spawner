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

  // The sequence blocks on service calls and on waits for a manager that is not there yet. Spinning
  // the node from another thread is what keeps the e-stop subscription served across those waits:
  // otherwise a press that lands during one is only seen after the next hardware activation has
  // already gone out.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node( node );
  std::thread spin_thread( [&executor]() {
    try {
      executor.spin();
    } catch ( const std::exception &e ) {
      // A shutdown that lands mid-wait surfaces here, as does anything a callback throws. Letting
      // it escape the thread ends the process through terminate() instead of the exit path below.
      RCLCPP_ERROR( rclcpp::get_logger( "multi_controller_spawner" ), "Executor stopped: %s",
                    e.what() );
    }
  } );

  using namespace std::chrono_literals;
  using hector_controller_spawner::SequenceOutcome;
  while ( rclcpp::ok() ) {
    if ( node->ready_to_start() ) {
      const SequenceOutcome outcome = node->start_sequence();
      // An abandoned attempt decided nothing, so it is run again whether or not an e-stop is
      // tracked. The sequence has already waited out its retry delay.
      if ( outcome != SequenceOutcome::Abandoned &&
           ( !node->is_tracking_estop() || !node->restart_after_estop_deactivation() ) )
        break;
      continue;
    }
    std::this_thread::sleep_for( 50ms );
  }

  // A startup sequence that could not put the robot into the requested state says so here. The
  // switch requests are best effort and report ok whatever the manager dropped, so without this
  // the process exits successfully having activated nothing.
  const bool failed = node->sequence_failed();
  executor.cancel();
  if ( spin_thread.joinable() )
    spin_thread.join();
  node.reset();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
