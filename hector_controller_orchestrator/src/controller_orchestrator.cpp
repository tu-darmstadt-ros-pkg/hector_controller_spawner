#include "controller_orchestrator/controller_orchestrator.hpp"

#include "switch_plan.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <future>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <shared_mutex>
#include <thread>
#include <unordered_set>
#include <utility>

namespace controller_orchestrator
{

namespace
{

using Clock = std::chrono::steady_clock;
using Callback = ControllerOrchestrator::Callback;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using GetParameters = rcl_interfaces::srv::GetParameters;
using ControllerManagerActivity = controller_manager_msgs::msg::ControllerManagerActivity;
using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

/// The controller manager answers within a switch's timeout plus 500 ms, so a caller waits this
/// much longer before it gives up on an answer.
constexpr std::chrono::seconds kAnswerMargin{ 1 };

/// How often an async switch checks whether the services have been discovered.
constexpr std::chrono::milliseconds kDiscoveryPoll{ 50 };

/// Declared by a controller manager that resolves providers and conflicts of a FORCE_AUTO switch.
constexpr char kCapabilitiesParameter[] = "capabilities";
constexpr char kForceAuto[] = "force_auto";

std::shared_ptr<SwitchController::Request> switchRequest( std::vector<std::string> activate,
                                                          std::vector<std::string> deactivate,
                                                          int strictness,
                                                          const rclcpp::Duration &timeout )
{
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = std::move( activate );
  request->deactivate_controllers = std::move( deactivate );
  request->strictness = strictness;
  request->timeout = timeout;
  return request;
}

bool declaresForceAuto( const GetParameters::Response &response )
{
  if ( response.values.size() != 1 ||
       response.values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY )
    return false;
  const auto &capabilities = response.values[0].string_array_value;
  return std::find( capabilities.begin(), capabilities.end(), kForceAuto ) != capabilities.end();
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

/// Answers its caller exactly once, with the last response or the deadline, which can race.
struct PendingSwitch {
  PendingSwitch( Callback callback, Clock::time_point end )
      : callback( std::move( callback ) ), end( end )
  {
  }

  void answer( bool success, const std::string &message )
  {
    std::function<void()> drop;
    {
      std::lock_guard<std::mutex> lock( mutex );
      if ( answered )
        return;
      answered = true;
      if ( deadline )
        deadline->cancel();
      stopDiscoveryLocked();
      drop = std::move( drop_request );
    }
    if ( drop )
      drop();
    callback( success, message );
  }

  bool open()
  {
    std::lock_guard<std::mutex> lock( mutex );
    return !answered;
  }

  void stopDiscoveryLocked()
  {
    if ( discovery )
      discovery->cancel();
    discovery.reset();
  }

  /// What is left of the switch's timeout, so the switch still ends at its deadline.
  rclcpp::Duration left() const
  {
    return rclcpp::Duration( std::max( end - Clock::now(), Clock::duration::zero() ) );
  }

  const Callback callback;
  const Clock::time_point end;
  std::mutex mutex;
  bool answered{ false };
  bool sent{ false };
  /// Removes the request in flight, which the client keeps forever otherwise.
  std::function<void()> drop_request;
  rclcpp::TimerBase::SharedPtr deadline;
  /// Waits for the services and owns this switch until the first request is sent.
  rclcpp::TimerBase::SharedPtr discovery;
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
    list_controllers_client_ =
        node->create_client<ListControllers>( controller_manager_name_ + "/list_controllers",
                                              rclcpp::ServicesQoS(), client_callback_group_ );
    get_parameters_client_ =
        node->create_client<GetParameters>( controller_manager_name_ + "/get_parameters",
                                            rclcpp::ServicesQoS(), client_callback_group_ );
  }

  /// Separate from the constructor, where weak_from_this() is still empty.
  void subscribeToActivity( const rclcpp::Node::SharedPtr &node );
  /// Starts a switch. Returns it, or nullptr if it was answered at once.
  std::shared_ptr<PendingSwitch> switchAsync( const std::vector<std::string> &activate,
                                              const Callback &callback, int timeout_s );
  bool servicesReady() const;
  void start( const std::shared_ptr<PendingSwitch> &pending,
              const std::vector<std::string> &activate );
  void dispatch( const std::shared_ptr<PendingSwitch> &pending,
                 const std::vector<std::string> &activate, bool force_auto );
  /// Sends a request of @p pending and hands its response on, unless @p pending is answered.
  template<typename ServiceT>
  void send( const std::shared_ptr<PendingSwitch> &pending,
             const std::shared_ptr<rclcpp::Client<ServiceT>> &client,
             typename ServiceT::Request::SharedPtr request,
             std::function<void( const typename ServiceT::Response & )> on_response );
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
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<GetParameters>::SharedPtr get_parameters_client_;
  rclcpp::Subscription<ControllerManagerActivity>::SharedPtr activity_subscription_;

  CallbackGate gate_;

  mutable std::shared_mutex active_mutex_;
  std::unordered_set<std::string> active_controllers_;

  /// Whether the controller manager declares FORCE_AUTO. Unknown until read, and again once a
  /// restarted manager publishes its activity from a new publisher.
  std::mutex mode_mutex_;
  std::optional<bool> force_auto_;
  std::optional<PublisherGid> activity_publisher_;
};

void ControllerOrchestrator::Core::subscribeToActivity( const rclcpp::Node::SharedPtr &node )
{
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = activity_callback_group_;
  activity_subscription_ = node->create_subscription<ControllerManagerActivity>(
      // Transient local, because the manager latches this topic and publishes only on a change
      controller_manager_name_ + "/activity", rclcpp::QoS( 1 ).reliable().transient_local(),
      [weak = weak_from_this()]( ControllerManagerActivity::ConstSharedPtr msg,
                                 const rclcpp::MessageInfo &info ) {
        const auto core = weak.lock();
        if ( !core )
          return;
        std::unordered_set<std::string> active;
        for ( const auto &controller : msg->controllers ) {
          if ( controller.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE )
            active.insert( controller.name );
        }
        {
          const std::lock_guard lock( core->active_mutex_ );
          core->active_controllers_ = std::move( active );
        }
        PublisherGid publisher;
        std::copy_n( info.get_rmw_message_info().publisher_gid.data, publisher.size(),
                     publisher.begin() );
        const std::lock_guard lock( core->mode_mutex_ );
        if ( core->activity_publisher_ && *core->activity_publisher_ != publisher )
          core->force_auto_.reset();
        core->activity_publisher_ = publisher;
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

bool ControllerOrchestrator::Core::servicesReady() const
{
  return switch_controller_client_->service_is_ready() &&
         list_controllers_client_->service_is_ready() && get_parameters_client_->service_is_ready();
}

std::shared_ptr<PendingSwitch>
ControllerOrchestrator::Core::switchAsync( const std::vector<std::string> &activate,
                                           const Callback &callback, int timeout_s )
{
  if ( activate.empty() ) {
    callback( true, "No controllers requested" );
    return nullptr;
  }
  const auto timeout = std::chrono::seconds( timeout_s );
  const auto pending = std::make_shared<PendingSwitch>( callback, Clock::now() + timeout );
  const auto deadline = timeout + kAnswerMargin;
  auto deadline_timer = rclcpp::create_wall_timer(
      deadline,
      [weak_pending = std::weak_ptr<PendingSwitch>( pending ), name = controller_manager_name_,
       deadline]() {
        // Weak, because the pending request owns it, and removing the request releases that
        const auto p = weak_pending.lock();
        if ( !p )
          return;
        bool sent;
        {
          std::lock_guard<std::mutex> lock( p->mutex );
          sent = p->sent;
        }
        p->answer( false, sent ? name + " did not answer within " +
                                     std::to_string( deadline.count() ) + " s"
                               : name + " is not available" );
      },
      client_callback_group_, node_base_.get(), node_timers_.get() );
  {
    std::lock_guard<std::mutex> lock( pending->mutex );
    pending->deadline = std::move( deadline_timer );
  }
  if ( servicesReady() ) {
    start( pending, activate );
    return pending;
  }
  // Polled, because rclcpp has no asynchronous wait for a service
  auto discovery = rclcpp::create_wall_timer(
      kDiscoveryPoll,
      [weak_core = weak_from_this(), pending, activate]() {
        const auto core = weak_core.lock();
        if ( !core || !core->servicesReady() || pending->left() == rclcpp::Duration( 0, 0 ) )
          return;
        {
          std::lock_guard<std::mutex> lock( pending->mutex );
          if ( pending->answered || !pending->discovery )
            return;
          pending->stopDiscoveryLocked();
        }
        core->start( pending, activate );
      },
      client_callback_group_, node_base_.get(), node_timers_.get() );
  std::lock_guard<std::mutex> lock( pending->mutex );
  if ( !pending->answered )
    pending->discovery = std::move( discovery );
  else
    discovery->cancel();
  return pending;
}

void ControllerOrchestrator::Core::start( const std::shared_ptr<PendingSwitch> &pending,
                                          const std::vector<std::string> &activate )
{
  std::optional<bool> force_auto;
  {
    const std::lock_guard lock( mode_mutex_ );
    force_auto = force_auto_;
  }
  if ( force_auto ) {
    dispatch( pending, activate, *force_auto );
    return;
  }
  auto request = std::make_shared<GetParameters::Request>();
  request->names = { kCapabilitiesParameter };
  send<GetParameters>(
      pending, get_parameters_client_, request,
      [weak = weak_from_this(), pending, activate]( const GetParameters::Response &response ) {
        const auto core = weak.lock();
        if ( !core )
          return;
        const bool force_auto = declaresForceAuto( response );
        RCLCPP_INFO( core->logger_, "%s %s", core->controller_manager_name_.c_str(),
                     force_auto ? "declares force_auto, so a switch is one "
                                  "FORCE_AUTO request"
                                : "does not declare force_auto, so the "
                                  "orchestrator plans each switch itself" );
        {
          const std::lock_guard lock( core->mode_mutex_ );
          core->force_auto_ = force_auto;
        }
        core->dispatch( pending, activate, force_auto );
      } );
}

void ControllerOrchestrator::Core::dispatch( const std::shared_ptr<PendingSwitch> &pending,
                                             const std::vector<std::string> &activate,
                                             bool force_auto )
{
  const auto answer = [pending]( const SwitchController::Response &response ) {
    pending->answer( response.ok, response.message );
  };
  if ( force_auto ) {
    send<SwitchController>(
        pending, switch_controller_client_,
        switchRequest( activate, {}, SwitchController::Request::FORCE_AUTO, pending->left() ),
        answer );
    return;
  }
  send<ListControllers>(
      pending, list_controllers_client_, std::make_shared<ListControllers::Request>(),
      [weak = weak_from_this(), pending, activate, answer]( const ListControllers::Response &listing ) {
        const auto core = weak.lock();
        if ( !core )
          return;
        auto plan = planSwitch( activate, listing.controller );
        if ( !plan.error.empty() ) {
          pending->answer( false, plan.error );
          return;
        }
        if ( plan.activate.empty() && plan.deactivate.empty() ) {
          pending->answer( true, "The requested controllers are already active" );
          return;
        }
        // Strict, so a manager that changed since the listing refuses instead of switching part
        core->send<SwitchController>(
            pending, core->switch_controller_client_,
            switchRequest( std::move( plan.activate ), std::move( plan.deactivate ),
                           SwitchController::Request::STRICT, pending->left() ),
            answer );
      } );
}

template<typename ServiceT>
void ControllerOrchestrator::Core::send(
    const std::shared_ptr<PendingSwitch> &pending,
    const std::shared_ptr<rclcpp::Client<ServiceT>> &client,
    typename ServiceT::Request::SharedPtr request,
    std::function<void( const typename ServiceT::Response & )> on_response )
{
  // Held while sending, so the deadline either finds nothing to drop or this request
  std::lock_guard<std::mutex> lock( pending->mutex );
  if ( pending->answered )
    return;
  pending->sent = true;
  const auto request_id =
      client
          ->async_send_request( std::move( request ),
                                [pending, on_response = std::move( on_response )](
                                    typename rclcpp::Client<ServiceT>::SharedFuture future ) {
                                  if ( pending->open() )
                                    on_response( *future.get() );
                                } )
          .request_id;
  pending->drop_request = [weak_client = std::weak_ptr<rclcpp::Client<ServiceT>>( client ),
                           request_id]() {
    if ( const auto c = weak_client.lock() )
      c->remove_pending_request( request_id );
  };
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
  auto promise = std::make_shared<std::promise<std::pair<bool, std::string>>>();
  auto future = promise->get_future();
  const auto pending = core_->switchAsync(
      activate_controllers,
      [promise]( bool success, const std::string &message ) {
        promise->set_value( { success, message } );
      },
      timeout_s );
  // The deadline answers unless the executor is not spinning, so this waits a little longer
  if ( future.wait_for( std::chrono::seconds( timeout_s ) + 2 * kAnswerMargin ) !=
       std::future_status::ready ) {
    pending->answer( false, "the caller stopped waiting" );
    RCLCPP_ERROR( core_->logger_,
                  "The switch did not finish in time. If this recurs, the calling executor has no "
                  "thread left to deliver the responses on." );
    return false;
  }
  const auto [success, message] = future.get();
  if ( !success )
    RCLCPP_ERROR( core_->logger_, "The switch failed: %s", message.c_str() );
  return success;
}

void ControllerOrchestrator::smartSwitchControllerAsync(
    const std::vector<std::string> &activate_controllers, const Callback &callback, int timeout_s ) const
{
  // An answer can overtake this call, and its callback may destroy the orchestrator
  const auto core = core_;
  core->switchAsync( activate_controllers, core->guarded( callback ), timeout_s );
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
