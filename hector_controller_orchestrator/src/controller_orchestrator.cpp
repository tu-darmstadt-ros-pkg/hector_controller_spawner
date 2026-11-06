#include "controller_orchestrator/controller_orchestrator.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_set>
#include <utility>

namespace controller_orchestrator
{

namespace
{

using Callback = ControllerOrchestrator::Callback;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using ControllerManagerActivity = controller_manager_msgs::msg::ControllerManagerActivity;

/// The controller manager answers within a switch's timeout plus 500 ms, so a caller waits this
/// much longer before it gives up on an answer.
constexpr std::chrono::seconds kAnswerMargin{ 1 };

std::shared_ptr<SwitchController::Request> forceAutoRequest( const std::vector<std::string> &activate,
                                                             int timeout_s )
{
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->strictness = SwitchController::Request::FORCE_AUTO;
  request->timeout = rclcpp::Duration( std::chrono::seconds( timeout_s ) );
  return request;
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

/// Answers its caller exactly once: the response or the deadline, which can race.
struct PendingSwitch {
  explicit PendingSwitch( Callback callback ) : callback( std::move( callback ) ) { }

  void answer( bool success, const std::string &message )
  {
    {
      std::lock_guard<std::mutex> lock( mutex );
      if ( answered )
        return;
      answered = true;
      if ( timer )
        timer->cancel();
    }
    callback( success, message );
  }

  const Callback callback;
  std::mutex mutex;
  bool answered{ false };
  rclcpp::TimerBase::SharedPtr timer;
};

} // namespace

/// The orchestrator's state. Callbacks hold it weakly and lock it for their whole body.
struct ControllerOrchestrator::Core : std::enable_shared_from_this<Core> {
  Core( const rclcpp::Node::SharedPtr &node, const std::string &controller_manager_name )
      : controller_manager_name_( controller_manager_name ), logger_( node->get_logger() ),
        node_base_( node->get_node_base_interface() ),
        node_timers_( node->get_node_timers_interface() )
  {
    // Reentrant, because a blocking call parks a thread of the group waiting for a response
    // another thread of the same group must deliver
    client_callback_group_ = node->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
    activity_callback_group_ =
        node->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
    switch_controller_client_ =
        node->create_client<SwitchController>( controller_manager_name_ + "/switch_controller",
                                               rclcpp::ServicesQoS(), client_callback_group_ );
  }

  /// Separate from the constructor, where weak_from_this() is still empty.
  void subscribeToActivity( const rclcpp::Node::SharedPtr &node );
  void switchAsync( const std::vector<std::string> &activate, const Callback &callback,
                    int timeout_s );
  /// Wraps a user callback so it runs through the gate.
  Callback guarded( Callback callback );
  bool isActiveLocked( const std::string &controller_name ) const;

  const std::string controller_manager_name_;
  const rclcpp::Logger logger_;
  // Interfaces rather than the node, so that a node owning its orchestrator is still released.
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr activity_callback_group_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Subscription<ControllerManagerActivity>::SharedPtr activity_subscription_;

  CallbackGate gate_;

  mutable std::shared_mutex active_mutex_;
  std::unordered_set<std::string> active_controllers_;
};

void ControllerOrchestrator::Core::subscribeToActivity( const rclcpp::Node::SharedPtr &node )
{
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = activity_callback_group_;
  activity_subscription_ = node->create_subscription<ControllerManagerActivity>(
      // Transient local, because the manager latches this topic and publishes only on a change
      controller_manager_name_ + "/activity", rclcpp::QoS( 1 ).reliable().transient_local(),
      [weak = weak_from_this()]( ControllerManagerActivity::ConstSharedPtr msg ) {
        const auto core = weak.lock();
        if ( !core )
          return;
        std::unordered_set<std::string> active;
        for ( const auto &controller : msg->controllers ) {
          if ( controller.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE )
            active.insert( controller.name );
        }
        const std::lock_guard lock( core->active_mutex_ );
        core->active_controllers_ = std::move( active );
      },
      subscription_options );
}

Callback ControllerOrchestrator::Core::guarded( Callback callback )
{
  return [weak = weak_from_this(), callback = std::move( callback )]( bool success,
                                                                      const std::string &message ) {
    if ( const auto core = weak.lock() )
      core->gate_.run( [&] { callback( success, message ); } );
  };
}

void ControllerOrchestrator::Core::switchAsync( const std::vector<std::string> &activate,
                                                const Callback &callback, int timeout_s )
{
  if ( activate.empty() ) {
    callback( true, "No controllers requested" );
    return;
  }
  if ( !switch_controller_client_->service_is_ready() ) {
    callback( false,
              switch_controller_client_->get_service_name() + std::string( " is not available" ) );
    return;
  }
  const auto pending = std::make_shared<PendingSwitch>( callback );
  const auto request = switch_controller_client_->async_send_request(
      forceAutoRequest( activate, timeout_s ),
      [pending]( rclcpp::Client<SwitchController>::SharedFuture future ) {
        const auto response = future.get();
        pending->answer( response->ok, response->message );
      } );
  const auto deadline = std::chrono::seconds( timeout_s ) + kAnswerMargin;
  auto timer = rclcpp::create_wall_timer(
      deadline,
      [weak_pending = std::weak_ptr<PendingSwitch>( pending ),
       weak_client = std::weak_ptr<rclcpp::Client<SwitchController>>( switch_controller_client_ ),
       request_id = request.request_id, deadline]() {
        // Taken first: the pending request owns it, and removing the request releases that
        const auto p = weak_pending.lock();
        if ( const auto client = weak_client.lock() )
          client->remove_pending_request( request_id );
        if ( p )
          p->answer( false, "The controller manager did not answer within " +
                                std::to_string( deadline.count() ) + " s" );
      },
      client_callback_group_, node_base_.get(), node_timers_.get() );
  std::lock_guard<std::mutex> lock( pending->mutex );
  if ( pending->answered )
    timer->cancel();
  pending->timer = std::move( timer );
}

bool ControllerOrchestrator::Core::isActiveLocked( const std::string &controller_name ) const
{
  return active_controllers_.count( controller_name ) != 0;
}

ControllerOrchestrator::ControllerOrchestrator( const rclcpp::Node::SharedPtr &node,
                                                const std::string &controller_manager_name )
    : core_( std::make_shared<Core>( node, controller_manager_name ) )
{
  core_->subscribeToActivity( node );
}

ControllerOrchestrator::~ControllerOrchestrator() { core_->gate_.close(); }

bool ControllerOrchestrator::smartSwitchController( const std::vector<std::string> &activate_controllers,
                                                    int timeout_s ) const
{
  if ( activate_controllers.empty() )
    return true;
  auto &client = *core_->switch_controller_client_;
  const auto wait = std::chrono::seconds( timeout_s ) + kAnswerMargin;
  if ( !client.wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( core_->logger_, "%s is not available.", client.get_service_name() );
    return false;
  }
  auto future = client.async_send_request( forceAutoRequest( activate_controllers, timeout_s ) );
  // Never plain get(): the response comes through an executor thread this caller may occupy
  if ( future.wait_for( wait ) != std::future_status::ready ) {
    client.remove_pending_request( future );
    RCLCPP_ERROR( core_->logger_,
                  "%s did not answer in time. If this recurs, the calling executor has no thread "
                  "left to deliver the response on.",
                  client.get_service_name() );
    return false;
  }
  const auto response = future.get();
  if ( !response->ok ) {
    RCLCPP_ERROR( core_->logger_, "The controller manager refused the switch: %s",
                  response->message.c_str() );
  }
  return response->ok;
}

void ControllerOrchestrator::smartSwitchControllerAsync(
    const std::vector<std::string> &activate_controllers, const Callback &callback, int timeout_s ) const
{
  core_->switchAsync( activate_controllers, core_->guarded( callback ), timeout_s );
}

bool ControllerOrchestrator::isControllerActive( const std::string &controller_name ) const
{
  const std::shared_lock lock( core_->active_mutex_ );
  return core_->isActiveLocked( controller_name );
}

bool ControllerOrchestrator::areControllersActive( const std::vector<std::string> &controller_names ) const
{
  const std::shared_lock lock( core_->active_mutex_ );
  return std::all_of( controller_names.begin(), controller_names.end(),
                      [this]( const std::string &name ) { return core_->isActiveLocked( name ); } );
}

} // namespace controller_orchestrator
