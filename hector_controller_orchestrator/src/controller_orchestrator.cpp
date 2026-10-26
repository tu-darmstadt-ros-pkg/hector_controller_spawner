#include "controller_orchestrator/controller_orchestrator.hpp"

#include "controller_graph.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <functional>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace controller_orchestrator
{

namespace
{

using Clock = std::chrono::steady_clock;
using Callback = ControllerOrchestrator::Callback;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;
using ControllerManagerActivity = controller_manager_msgs::msg::ControllerManagerActivity;

/// How long each step of a chain started by the blocking smartSwitchController may take.
constexpr int kSyncStepTimeoutS = 5;

Clock::time_point deadlineIn( int timeout_s )
{
  return Clock::now() + std::chrono::seconds( timeout_s );
}

Clock::duration remaining( Clock::time_point deadline )
{
  return std::max( deadline - Clock::now(), Clock::duration::zero() );
}

/**
 * @brief Calls a service and waits for the response until @p deadline.
 *
 * Never plain get(). The response is delivered by an executor thread of this node, so a caller that
 * is itself a callback occupying the last free thread waits for something that can no longer
 * happen.
 *
 * @return The response, or nullptr after logging why there is none.
 */
template<typename ServiceT>
typename ServiceT::Response::SharedPtr
callService( rclcpp::Client<ServiceT> &client, typename ServiceT::Request::SharedPtr request,
             Clock::time_point deadline, const rclcpp::Logger &logger )
{
  if ( !client.wait_for_service( remaining( deadline ) ) ) {
    RCLCPP_ERROR( logger, "%s is not available.", client.get_service_name() );
    return nullptr;
  }
  auto future = client.async_send_request( std::move( request ) );
  if ( future.wait_for( remaining( deadline ) ) != std::future_status::ready ) {
    // The client keeps an abandoned request forever unless it is removed.
    client.remove_pending_request( future );
    RCLCPP_ERROR( logger,
                  "%s did not answer in time. If this recurs, the calling executor has no thread "
                  "left to deliver the response on.",
                  client.get_service_name() );
    return nullptr;
  }
  auto response = future.get();
  if ( !response ) {
    RCLCPP_ERROR( logger, "%s returned a null response.", client.get_service_name() );
  }
  return response;
}

/// Removes a pending request of @p client. Weak, because the request's callback owns the holder.
template<typename ClientT>
std::function<void()> dropRequest( const std::shared_ptr<ClientT> &client, int64_t request_id )
{
  return [weak_client = std::weak_ptr<ClientT>( client ), request_id]() {
    if ( const auto c = weak_client.lock() )
      c->remove_pending_request( request_id );
  };
}

std::string lifecycleStateLabel( const lifecycle_msgs::msg::State &state )
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

/// Lets the destructor wait for the user callbacks in progress and refuse every later one.
class CallbackGate
{
public:
  /// Runs @p callback unless the gate is closed.
  void run( const std::function<void()> &callback )
  {
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      if ( closed_ )
        return;
      running_.push_back( std::this_thread::get_id() );
    }
    struct Leave {
      CallbackGate &gate;
      ~Leave() { gate.leave(); }
    } leave{ *this };
    callback();
  }

  /// Closes the gate and waits for the callbacks running on other threads. One running on this
  /// thread is destroying the orchestrator from inside itself and cannot be waited for.
  void close()
  {
    std::unique_lock<std::mutex> lock( mutex_ );
    closed_ = true;
    const auto self = std::this_thread::get_id();
    idle_.wait( lock, [this, self] {
      return std::all_of( running_.begin(), running_.end(),
                          [self]( const std::thread::id &id ) { return id == self; } );
    } );
  }

private:
  void leave()
  {
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      running_.erase( std::find( running_.begin(), running_.end(), std::this_thread::get_id() ) );
    }
    idle_.notify_all();
  }

  std::mutex mutex_;
  std::condition_variable idle_;
  bool closed_{ false };
  std::vector<std::thread::id> running_;
};

/// Answers its caller exactly once, whichever comes first: the request finishing, failing, being
/// superseded, or a step outliving its deadline. All four can race on a multi-threaded executor.
struct PendingCall {
  PendingCall( Callback callback, std::chrono::seconds step_timeout )
      : callback( std::move( callback ) ), step_timeout( step_timeout ),
        deadline( Clock::now() + step_timeout )
  {
  }

  /// Takes the timer that enforces the deadline.
  void setTimer( rclcpp::TimerBase::SharedPtr deadline_timer )
  {
    std::lock_guard<std::mutex> lock( mutex );
    timer = std::move( deadline_timer );
    if ( answered )
      timer->cancel();
  }

  /// Starts a step and its deadline. False once the call has been answered, and then nothing more
  /// may be sent for it.
  bool beginStep( const std::string &description )
  {
    std::lock_guard<std::mutex> lock( mutex );
    if ( answered )
      return false;
    step = description;
    deadline = Clock::now() + step_timeout;
    drop_pending = nullptr;
    if ( timer )
      timer->reset();
    return true;
  }

  /// Stores how to cancel the request just sent. If the call was answered while it was being
  /// sent, cancels it right away.
  void track( std::function<void()> drop )
  {
    std::unique_lock<std::mutex> lock( mutex );
    if ( answered ) {
      lock.unlock();
      drop();
      return;
    }
    drop_pending = std::move( drop );
  }

  /// Whether a response for the current step may still advance the call.
  bool open()
  {
    std::lock_guard<std::mutex> lock( mutex );
    return !answered;
  }

  void succeeded( const std::string &controller, bool activation )
  {
    std::lock_guard<std::mutex> lock( mutex );
    ( activation ? activated : deactivated ).push_back( controller );
  }

  /// Marks the call answered without calling back, and cancels its deadline and pending request.
  /// False if it was answered already.
  bool close()
  {
    std::function<void()> drop;
    {
      std::lock_guard<std::mutex> lock( mutex );
      if ( answered )
        return false;
      answered = true;
      drop = std::move( drop_pending );
      if ( timer )
        timer->cancel();
    }
    if ( drop )
      drop();
    return true;
  }

  /// Calls back unless the call was answered already.
  void answer( bool success, const std::string &message )
  {
    if ( close() )
      callback( success, message );
  }

  /// Fails the call if its current step is past the deadline. Checked against the time and not
  /// only the timer firing, since a firing already queued when a step re-armed it belongs to the
  /// step before.
  void expireIfDue()
  {
    std::string message;
    {
      std::lock_guard<std::mutex> lock( mutex );
      if ( answered || Clock::now() < deadline )
        return;
      message = step + " did not answer within " + std::to_string( step_timeout.count() ) + "s.";
      if ( !deactivated.empty() )
        message += " Deactivated: " + vecToString( deactivated ) + ".";
      if ( !activated.empty() )
        message += " Activated: " + vecToString( activated ) + ".";
    }
    answer( false, message );
  }

  const Callback callback;
  const std::chrono::seconds step_timeout;

  std::mutex mutex;
  rclcpp::TimerBase::SharedPtr timer;
  bool answered{ false };
  std::string step;
  Clock::time_point deadline;
  std::function<void()> drop_pending;
  std::vector<std::string> deactivated;
  std::vector<std::string> activated;
};

struct SwitchStep {
  std::string controller;
  bool activate;
};

} // namespace

/// The orchestrator's state. Callbacks hold it weakly and lock it for their whole body.
struct ControllerOrchestrator::Core : std::enable_shared_from_this<Core> {
  Core( const rclcpp::Node::SharedPtr &node, const std::string &controller_manager_name )
      : controller_manager_name_( controller_manager_name ), logger_( node->get_logger() ),
        node_base_( node->get_node_base_interface() ),
        node_timers_( node->get_node_timers_interface() )
  {
    // Two groups, because the clients and the subscription need opposite things. The clients need
    // reentrancy: a blocking call parks a thread of the group waiting for a response another thread
    // of the same group must deliver. The subscription needs the opposite. It replaces the cache
    // wholesale, so dispatching it to several threads at once only makes them queue on the mutex.
    client_callback_group_ = node->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
    cache_callback_group_ =
        node->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );

    list_controllers_client_ =
        node->create_client<ListControllers>( controller_manager_name_ + "/list_controllers",
                                              rclcpp::ServicesQoS(), client_callback_group_ );
    switch_controller_client_ =
        node->create_client<SwitchController>( controller_manager_name_ + "/switch_controller",
                                               rclcpp::ServicesQoS(), client_callback_group_ );
    list_hardware_components_client_ = node->create_client<ListHardwareComponents>(
        controller_manager_name_ + "/list_hardware_components", rclcpp::ServicesQoS(),
        client_callback_group_ );
  }

  /// Separate from the constructor, where weak_from_this() is still empty.
  void subscribeToActivity( const rclcpp::Node::SharedPtr &node );
  /// Closes the gate and stops the chain in flight.
  void shutdown();
  /// Wraps a user callback so it runs through the gate.
  Callback guarded( Callback callback );

  /// Starts a smart switch. Returns its chain, or nullptr if it was answered without one.
  std::shared_ptr<PendingCall> startSwitch( const std::vector<std::string> &activate_controllers,
                                            const Callback &callback, int step_timeout_s );
  void continueSwitch( const std::shared_ptr<PendingCall> &chain,
                       const std::vector<std::string> &requested,
                       const ListControllers::Response::SharedPtr &listing );
  /// Sends step @p index of @p steps, each step sent once the previous one succeeded.
  void sendStep( const std::shared_ptr<PendingCall> &chain,
                 const std::shared_ptr<const std::vector<SwitchStep>> &steps, size_t index );
  void refreshAsync( const Callback &callback, int timeout_s );
  void armDeadline( const std::shared_ptr<PendingCall> &call );

  bool switchControllers( const std::vector<std::string> &activate,
                          const std::vector<std::string> &deactivate, Clock::time_point deadline );
  void updateFromList( const ListControllers::Response &response );
  /// Takes the new map by value so the caller builds it outside the lock, and no reader observes a
  /// half-rebuilt cache.
  void replaceStates( std::unordered_map<std::string, std::string> states );
  /// @pre states_mutex_ is held. std::shared_lock is not recursive, so the public accessors share
  /// this instead of calling each other.
  bool isActiveLocked( const std::string &controller_name ) const;

  const std::string controller_manager_name_;
  const rclcpp::Logger logger_;
  // Interfaces rather than the node, so that a node owning its orchestrator is still released.
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;

  rclcpp::CallbackGroup::SharedPtr cache_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<ListHardwareComponents>::SharedPtr list_hardware_components_client_;
  rclcpp::Subscription<ControllerManagerActivity>::SharedPtr activity_subscription_;

  CallbackGate gate_;

  /// Shared, because reads dominate: areControllersActive() is on the e-stop path and asks about
  /// several controllers at once, while writes happen only when the activity topic ticks.
  std::shared_mutex states_mutex_;
  std::unordered_map<std::string, std::string> controller_states_;

  /// Guards current_chain_, the latest switch request. Replacing it answers the previous one.
  std::mutex chain_mutex_;
  std::shared_ptr<PendingCall> current_chain_;
};

void ControllerOrchestrator::Core::subscribeToActivity( const rclcpp::Node::SharedPtr &node )
{
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = cache_callback_group_;
  activity_subscription_ = node->create_subscription<ControllerManagerActivity>(
      // Transient local to match the manager, which latches this topic and publishes it only
      // when something changes. A volatile subscription discards the retained sample, so an
      // orchestrator built after bring-up would hold an empty cache until the next switch.
      controller_manager_name_ + "/activity", rclcpp::QoS( 10 ).transient_local(),
      [weak = weak_from_this()]( ControllerManagerActivity::ConstSharedPtr msg ) {
        const auto core = weak.lock();
        if ( !core || !msg ) {
          return;
        }
        std::unordered_map<std::string, std::string> states;
        states.reserve( msg->controllers.size() );
        for ( const auto &controller : msg->controllers ) {
          states.emplace( controller.name, lifecycleStateLabel( controller.state ) );
        }
        core->replaceStates( std::move( states ) );
      },
      subscription_options );
}

void ControllerOrchestrator::Core::shutdown()
{
  gate_.close();
  std::shared_ptr<PendingCall> chain;
  {
    std::lock_guard<std::mutex> lock( chain_mutex_ );
    chain = std::move( current_chain_ );
  }
  if ( chain )
    chain->close();
}

Callback ControllerOrchestrator::Core::guarded( Callback callback )
{
  return [weak = weak_from_this(), callback = std::move( callback )]( bool success,
                                                                      const std::string &message ) {
    if ( const auto core = weak.lock() )
      core->gate_.run( [&] { callback( success, message ); } );
  };
}

std::shared_ptr<PendingCall>
ControllerOrchestrator::Core::startSwitch( const std::vector<std::string> &activate_controllers,
                                           const Callback &callback, int step_timeout_s )
{
  // A callback answered here may destroy the orchestrator.
  const auto self = shared_from_this();
  if ( activate_controllers.empty() ) {
    callback( true, "No controllers requested" );
    return nullptr;
  }

  // No shortcut on the cache here. It is fed by the activity topic and by list responses with no
  // ordering between them, and a chain in flight is about to change the very states it holds, so
  // "already active" can be a claim about a moment that has passed. The listing below is one round
  // trip and answers the same question about now.
  if ( !list_controllers_client_->service_is_ready() ) {
    callback( false, "list_controllers service not ready" );
    return nullptr;
  }

  const auto chain =
      std::make_shared<PendingCall>( callback, std::chrono::seconds( step_timeout_s ) );
  std::shared_ptr<PendingCall> superseded;
  {
    std::lock_guard<std::mutex> lock( chain_mutex_ );
    superseded = std::exchange( current_chain_, chain );
  }
  if ( superseded ) {
    superseded->answer( false, "superseded by a newer switch request" );
  }
  armDeadline( chain );
  if ( !chain->beginStep( "list_controllers" ) )
    return chain;
  const auto request = list_controllers_client_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [weak = weak_from_this(), chain,
       activate_controllers]( rclcpp::Client<ListControllers>::SharedFuture future ) {
        const auto core = weak.lock();
        if ( !core || !chain->open() )
          return;
        core->continueSwitch( chain, activate_controllers, future.get() );
      } );
  chain->track( dropRequest( list_controllers_client_, request.request_id ) );
  return chain;
}

void ControllerOrchestrator::Core::continueSwitch( const std::shared_ptr<PendingCall> &chain,
                                                   const std::vector<std::string> &requested,
                                                   const ListControllers::Response::SharedPtr &listing )
{
  if ( !listing ) {
    chain->answer( false, "list_controllers returned null response" );
    return;
  }
  updateFromList( *listing );

  const auto plan = planSwitch( requested, listing->controller, logger_ );
  if ( !plan ) {
    chain->answer( false, "No switch satisfies the requested controllers" );
    return;
  }
  if ( plan->activate.empty() && plan->deactivate.empty() ) {
    chain->answer( true, "No controllers to activate or deactivate" );
    return;
  }
  if ( !switch_controller_client_->service_is_ready() ) {
    chain->answer( false, "switch_controller service not ready" );
    return;
  }

  // Controllers chained to others stop before them and start after them.
  auto steps = std::make_shared<std::vector<SwitchStep>>();
  for ( const auto &name : plan->deactivate ) steps->push_back( { name, false } );
  for ( auto it = plan->activate.rbegin(); it != plan->activate.rend(); ++it )
    steps->push_back( { *it, true } );
  sendStep( chain, steps, 0 );
}

void ControllerOrchestrator::Core::sendStep( const std::shared_ptr<PendingCall> &chain,
                                             const std::shared_ptr<const std::vector<SwitchStep>> &steps,
                                             size_t index )
{
  if ( index >= steps->size() ) {
    chain->answer( true, "Activation chain complete" );
    return;
  }
  const SwitchStep &step = ( *steps )[index];
  if ( !chain->beginStep( ( step.activate ? "Activating " : "Deactivating " ) + step.controller ) )
    return;
  auto req = std::make_shared<SwitchController::Request>();
  ( step.activate ? req->activate_controllers : req->deactivate_controllers ) = { step.controller };
  // Best effort, deliberately. The plan comes from one listing while the chain sends a request per
  // controller, so by a later step the snapshot can be seconds old. Strict refuses a controller
  // that has since become active, and refusing aborts the chain partway, leaving the joint with
  // the deactivations applied and none of the activations.
  req->strictness = SwitchController::Request::BEST_EFFORT;
  const auto request = switch_controller_client_->async_send_request(
      req, [weak = weak_from_this(), chain, steps,
            index]( rclcpp::Client<SwitchController>::SharedFuture future ) {
        const auto core = weak.lock();
        if ( !core || !chain->open() )
          return;
        const SwitchStep &step = ( *steps )[index];
        const auto response = future.get();
        if ( !response || !response->ok ) {
          chain->answer( false, ( step.activate ? "Failed to activate " : "Failed to deactivate " ) +
                                    step.controller );
          return;
        }
        chain->succeeded( step.controller, step.activate );
        core->sendStep( chain, steps, index + 1 );
      } );
  chain->track( dropRequest( switch_controller_client_, request.request_id ) );
}

void ControllerOrchestrator::Core::refreshAsync( const Callback &callback, int timeout_s )
{
  // A callback answered here may destroy the orchestrator.
  const auto self = shared_from_this();
  if ( !list_controllers_client_->service_is_ready() ) {
    callback( false, "list_controllers service not ready" );
    return;
  }
  const auto call = std::make_shared<PendingCall>( callback, std::chrono::seconds( timeout_s ) );
  armDeadline( call );
  call->beginStep( "list_controllers" );
  const auto request = list_controllers_client_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [weak = weak_from_this(), call]( rclcpp::Client<ListControllers>::SharedFuture future ) {
        const auto core = weak.lock();
        if ( !core || !call->open() )
          return;
        const auto listing = future.get();
        if ( !listing ) {
          call->answer( false, "list_controllers returned null response" );
          return;
        }
        core->updateFromList( *listing );
        call->answer( true, "Refresh successful" );
      } );
  call->track( dropRequest( list_controllers_client_, request.request_id ) );
}

void ControllerOrchestrator::Core::armDeadline( const std::shared_ptr<PendingCall> &call )
{
  // Re-armed by every step. Weak, because the call owns its timer.
  call->setTimer( rclcpp::create_wall_timer(
      call->step_timeout,
      [weak_call = std::weak_ptr<PendingCall>( call )]() {
        if ( const auto c = weak_call.lock() )
          c->expireIfDue();
      },
      client_callback_group_, node_base_.get(), node_timers_.get() ) );
}

bool ControllerOrchestrator::Core::switchControllers( const std::vector<std::string> &activate,
                                                      const std::vector<std::string> &deactivate,
                                                      Clock::time_point deadline )
{
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = SwitchController::Request::BEST_EFFORT;
  const auto response = callService( *switch_controller_client_, request, deadline, logger_ );
  if ( !response )
    return false;
  if ( !response->ok ) {
    RCLCPP_ERROR( logger_, "The controller manager refused the switch: %s",
                  response->message.c_str() );
    return false;
  }
  return true;
}

void ControllerOrchestrator::Core::updateFromList( const ListControllers::Response &response )
{
  std::unordered_map<std::string, std::string> states;
  states.reserve( response.controller.size() );
  for ( const auto &ctrl : response.controller ) { states.emplace( ctrl.name, ctrl.state ); }
  replaceStates( std::move( states ) );
}

void ControllerOrchestrator::Core::replaceStates( std::unordered_map<std::string, std::string> states )
{
  const std::lock_guard lock( states_mutex_ );
  controller_states_ = std::move( states );
}

bool ControllerOrchestrator::Core::isActiveLocked( const std::string &controller_name ) const
{
  const auto it = controller_states_.find( controller_name );
  return it != controller_states_.end() && it->second == "active";
}

ControllerOrchestrator::ControllerOrchestrator( const rclcpp::Node::SharedPtr &node,
                                                const std::string &controller_manager_name )
    : core_( std::make_shared<Core>( node, controller_manager_name ) )
{
  core_->subscribeToActivity( node );
}

ControllerOrchestrator::~ControllerOrchestrator() { core_->shutdown(); }

bool ControllerOrchestrator::smartSwitchController( const std::vector<std::string> &activate_controllers,
                                                    int timeout_s ) const
{
  auto promise = std::make_shared<std::promise<std::pair<bool, std::string>>>();
  auto future = promise->get_future();
  const auto chain = core_->startSwitch(
      activate_controllers,
      [promise]( bool success, const std::string &message ) {
        promise->set_value( { success, message } );
      },
      kSyncStepTimeoutS );

  if ( future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout ) {
    // Stops the chain, so a listing that arrives after the caller gave up switches nothing.
    if ( chain )
      chain->answer( false, "the caller stopped waiting" );
    RCLCPP_ERROR( core_->logger_,
                  "[ControllerOrchestrator] Synchronous Switch Timed Out after %d seconds. "
                  "Likely Cause: The ROS Executor is not spinning to process the response.",
                  timeout_s );
    return false;
  }

  const auto [success, message] = future.get();
  if ( !success ) {
    RCLCPP_ERROR( core_->logger_, "[ControllerOrchestrator] Switch failed: %s", message.c_str() );
  }
  return success;
}

void ControllerOrchestrator::smartSwitchControllerAsync(
    const std::vector<std::string> &activate_controllers, const Callback &callback,
    int step_timeout_s ) const
{
  core_->startSwitch( activate_controllers, core_->guarded( callback ), step_timeout_s );
}

std::optional<std::vector<std::string>>
ControllerOrchestrator::getActiveControllerOfHardwareInterface( const std::string &hardware_interface,
                                                                int timeout_s ) const
{
  const auto deadline = deadlineIn( timeout_s );
  const auto components =
      callService( *core_->list_hardware_components_client_,
                   std::make_shared<ListHardwareComponents::Request>(), deadline, core_->logger_ );
  if ( !components )
    return std::nullopt;
  const auto component =
      std::find_if( components->component.begin(), components->component.end(),
                    [&]( const auto &comp ) { return comp.name == hardware_interface; } );
  if ( component == components->component.end() ) {
    RCLCPP_ERROR( core_->logger_, "No hardware component named '%s' is loaded.",
                  hardware_interface.c_str() );
    return std::nullopt;
  }
  std::unordered_set<std::string> interfaces;
  for ( const auto &interface : component->command_interfaces ) interfaces.insert( interface.name );
  if ( interfaces.empty() )
    return std::vector<std::string>{}; // nothing to claim, so nothing can be active on it

  const auto listing =
      callService( *core_->list_controllers_client_, std::make_shared<ListControllers::Request>(),
                   deadline, core_->logger_ );
  if ( !listing )
    return std::nullopt;
  return activeClaimersOf( listing->controller, interfaces );
}

bool ControllerOrchestrator::deactivateControllers(
    const std::vector<std::string> &controllers_to_deactivate, int timeout_s ) const
{
  return core_->switchControllers( {}, controllers_to_deactivate, deadlineIn( timeout_s ) );
}

bool ControllerOrchestrator::activateControllers(
    const std::vector<std::string> &controllers_to_activate, int timeout_s ) const
{
  return core_->switchControllers( controllers_to_activate, {}, deadlineIn( timeout_s ) );
}

bool ControllerOrchestrator::deactivateControllersOfJoint( const std::string &joint_name,
                                                           int timeout_s ) const
{
  const auto deadline = deadlineIn( timeout_s );
  const auto listing =
      callService( *core_->list_controllers_client_, std::make_shared<ListControllers::Request>(),
                   deadline, core_->logger_ );
  if ( !listing )
    return false;
  const auto order = deactivationOrderForJoint( listing->controller, joint_name, core_->logger_ );
  if ( !order )
    return false;
  if ( order->empty() )
    return true;
  return core_->switchControllers( {}, *order, deadline );
}

bool ControllerOrchestrator::refreshControllerStates( int timeout_s ) const
{
  const auto listing =
      callService( *core_->list_controllers_client_, std::make_shared<ListControllers::Request>(),
                   deadlineIn( timeout_s ), core_->logger_ );
  if ( !listing )
    return false;
  core_->updateFromList( *listing );
  return true;
}

void ControllerOrchestrator::refreshControllerStatesAsync( const Callback &callback,
                                                           int timeout_s ) const
{
  core_->refreshAsync( core_->guarded( callback ), timeout_s );
}

bool ControllerOrchestrator::isControllerActive( const std::string &controller_name ) const
{
  const std::shared_lock lock( core_->states_mutex_ );
  return core_->isActiveLocked( controller_name );
}

bool ControllerOrchestrator::areControllersActive( const std::vector<std::string> &controller_names ) const
{
  const std::shared_lock lock( core_->states_mutex_ );
  return std::all_of( controller_names.begin(), controller_names.end(),
                      [this]( const std::string &name ) { return core_->isActiveLocked( name ); } );
}

} // namespace controller_orchestrator
