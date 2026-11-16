// Copyright 2025, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2025-01-17
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/trajectory_until_node.hpp>
#include <functional>
#include <rclcpp/exceptions.hpp>

namespace ur_robot_driver
{

using TCAction = ur_msgs::action::ToolContact;
using GoalHandleTrajectoryUntil = rclcpp_action::ServerGoalHandle<ur_msgs::action::FollowJointTrajectoryUntil>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using TrajectoryResult = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult;

using namespace std::chrono_literals;

TrajectoryUntilNode::TrajectoryUntilNode(const rclcpp::NodeOptions& options)
  : Node("TrajectoryUntilNode", options)
  , server_goal_handle_(nullptr)
  , current_trajectory_goal_handle_(nullptr)
  , current_until_goal_handle_(nullptr)
  , trajectory_accepted_(false)
  , until_accepted_(false)
{
  prealloc_res_ = std::make_shared<TrajectoryUntilAction::Result>(TrajectoryUntilAction::Result());
  prealloc_fb_ = std::make_shared<TrajectoryUntilAction::Feedback>(TrajectoryUntilAction::Feedback());

  // Different callback groups for the server and the clients, so their callbacks can run concurrently.
  server_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clients_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  this->declare_parameter("motion_controller", "scaled_joint_trajectory_controller");
  std::string motion_controller = this->get_parameter("motion_controller").as_string();

  // Initialize a trajectory action client, to a generic action that does not exist. This is remapped via ros-args when
  // launching the node
  trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                                                  motion_controller + "/follow_joint_"
                                                                                                      "trajectory",
                                                                                  clients_callback_group);

  // Create action server to advertise the "~/execute" action
  action_server_ = rclcpp_action::create_server<TrajectoryUntilAction>(
      this, "~/execute",
      std::bind(&TrajectoryUntilNode::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryUntilNode::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&TrajectoryUntilNode::goal_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), server_callback_group);
}

TrajectoryUntilNode::~TrajectoryUntilNode()
{
}

// Assign the correct type of action client to the variant
bool TrajectoryUntilNode::assign_until_action_client(std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
{
  switch (goal->until_type) {
    case TrajectoryUntilAction::Goal::TOOL_CONTACT: {
      // PATCH(lpreol #81): build the client OUTSIDE the lock (create_client does graph work), then
      // publish it under the lock — cancel_until_handle() reads this variant from the clients group.
      auto client = rclcpp_action::create_client<TCAction>(this,
                                                          "/tool_contact_controller/detect_tool_contact",
                                                          clients_callback_group);
      std::lock_guard<std::mutex> lock(mutex_child_goals_);
      until_action_client_variant = client;
      break;
    }

    default:
      RCLCPP_ERROR(this->get_logger(), "Received unknown until-type.");
      return false;
  }

  return true;
}

// Check if the node is capable of accepting a new action goal right now.
rclcpp_action::GoalResponse TrajectoryUntilNode::goal_received_callback(
    const rclcpp_action::GoalUUID& /* uuid */, std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
{
  {
    std::lock_guard<std::mutex> lock(mutex_server_goal_);  // PATCH(lpreol #77)
    if (server_goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Node is currently busy, rejecting action goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!assign_until_action_client(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Until type not defined, double check the types in the action "
                                     "definition.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory action server not available.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!std::visit([](const auto& client) { return client->wait_for_action_server(std::chrono::seconds(1)); },
                  until_action_client_variant)) {
    RCLCPP_ERROR(this->get_logger(), "Until action server not available.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check until action server, send action goal to until-controller and wait for it to be accepted.
  if (std::holds_alternative<TCClient>(until_action_client_variant)) {
    send_until_goal<TCAction, TCClient>(goal);
  } else {
    throw std::runtime_error("Until type not implemented. This should not happen.");
  }

  // PATCH(lpreol #36): predicated waits on the CORRECT condition variables.
  // Upstream waits without a predicate (a notify that beats the wait is lost →
  // full 1 s burn) and the trajectory wait blocks on cv_until_, which nothing
  // on the trajectory path ever notifies (→ unconditional 1 s burn per goal).
  // The 1 s timeouts remain as backstops only.
  bool until_ok;
  {
    std::unique_lock<std::mutex> until_lock(mutex_until);
    until_ok = cv_until_.wait_for(until_lock, 1s, [this] { return until_accepted_.load(); });
  }
  if (!until_ok) {
    // PATCH(lpreol #81): upstream returns REJECT here with the until goal still in flight. If it is
    // accepted a moment later nothing tracks it and nothing cancels it — tool-contact stays ARMED on
    // the robot, and the next goal's assign_until_action_client() replaces the client that owned it.
    // The trajectory timeout below always did call reset_node(); this one never did. reset_node()
    // also disarms expecting_until_goal_, so a late acceptance cancels itself.
    RCLCPP_ERROR(this->get_logger(), "Until condition was not accepted within 1 s — rejecting, and cancelling the "
                                     "until goal so tool contact cannot be left armed.");
    reset_node();
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Send action goal to trajectory controller and wait for it to be accepted. If it is not accepted within 1 seconds,
  // it is assumed to be rejected.
  send_trajectory_goal(goal);
  bool trajectory_ok;
  {
    std::unique_lock<std::mutex> traj_lock(mutex_trajectory);
    trajectory_ok = cv_trajectory_.wait_for(traj_lock, 1s, [this] { return trajectory_accepted_.load(); });
  }
  if (!trajectory_ok) {
    // PATCH(lpreol #81): reset_node() moved OUT of the mutex_trajectory scope — it now takes
    // mutex_child_goals_, and baking a lock-order dependency into a path that does not need one is
    // how the next deadlock gets written.
    reset_node();
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Save the server goal handle for later use.
void TrajectoryUntilNode::goal_accepted_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(mutex_server_goal_);  // PATCH(lpreol #77)
    server_goal_handle_ = goal_handle;
  }
  RCLCPP_INFO(this->get_logger(), "Accepted goal");
  return;
}

// If the trajectory_until action is canceled, it should cancel the until and trajectory actions, if any are running.
rclcpp_action::CancelResponse
TrajectoryUntilNode::goal_cancelled_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> /* goal_handle */)
{
  RCLCPP_INFO(this->get_logger(), "Cancel received. Cancelling trajectory-until action.");
  // PATCH(lpreol #77): async_cancel_goal THROWS exceptions::UnknownGoalHandleError (which derives
  // std::invalid_argument, NOT RCLError) when the goal is unknown or has already reached a terminal
  // state — and the client erases the goal as soon as its result callback has run. Racing result
  // delivery here is normal, and an uncaught throw would abort the process: the very failure mode
  // this file is being fixed for.
  // PATCH(lpreol #81): this runs on the SERVER callback group while the response and result
  // callbacks write these handles from the CLIENTS group. Reading them unsynchronised was UB.
  // Snapshot under the lock, cancel outside it; clearing is left to reset_node().
  rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr until_handle;
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_child_goals_);
    until_handle = current_until_goal_handle_;
    traj_handle = current_trajectory_goal_handle_;
  }
  if (until_handle) {
    cancel_until_handle(until_handle);
  }
  try {
    if (traj_handle) {
      trajectory_action_client_->async_cancel_goal(traj_handle);
    }
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(this->get_logger(), "Trajectory goal was already finished when we tried to cancel it: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

// Send the trajectory goal, using the relevant fields supplied from the action server
void TrajectoryUntilNode::send_trajectory_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
{
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = goal->trajectory;
  goal_msg.goal_time_tolerance = goal->goal_time_tolerance;
  goal_msg.goal_tolerance = goal->goal_tolerance;
  goal_msg.path_tolerance = goal->path_tolerance;

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TrajectoryUntilNode::trajectory_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&TrajectoryUntilNode::trajectory_result_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&TrajectoryUntilNode::trajectory_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
}

// Send the until goal, using the relevant fields supplied from the action server.
template <class ActionType, class ClientType>
void TrajectoryUntilNode::send_until_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> /* goal */)
{
  // Setup goal
  auto goal_msg = typename ActionType::Goal();
  auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TrajectoryUntilNode::until_response_callback<ActionType>, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&TrajectoryUntilNode::until_result_callback<ActionType>, this, std::placeholders::_1);

  // PATCH(lpreol #81): arm BEFORE sending and under the lock, so an acceptance racing back cannot
  // be mistaken for one belonging to an abandoned goal.
  {
    std::lock_guard<std::mutex> lock(mutex_child_goals_);
    expecting_until_goal_ = true;
  }

  // Send goal
  std::get<ClientType>(until_action_client_variant)->async_send_goal(goal_msg, send_goal_options);
}

void TrajectoryUntilNode::trajectory_response_callback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr& goal_handle)
{
  {
    std::lock_guard<std::mutex> guard(mutex_trajectory);
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Trajectory accepted by server.");
      std::lock_guard<std::mutex> child_guard(mutex_child_goals_);  // PATCH(lpreol #81)
      current_trajectory_goal_handle_ = goal_handle;
      trajectory_accepted_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Trajectory rejected by server.");
    }
  }
  cv_trajectory_.notify_one();
}

template <class T>
void TrajectoryUntilNode::until_response_callback(
    const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr& goal_handle)
{
  // PATCH(lpreol #81): an acceptance can arrive AFTER we gave up waiting for it. Storing it then
  // orphans a live tool-contact goal — nothing tracks it, nothing cancels it, tool contact stays
  // armed. ⭐ The test and the store MUST be one critical section: checking the flag, then storing
  // outside the lock, loses to a concurrent reset_node() and additionally poisons until_accepted_
  // for the NEXT goal — strictly worse than the bug it closes, because the state then lies.
  bool unwanted = false;
  {
    std::lock_guard<std::mutex> guard(mutex_until);
    if (goal_handle) {
      std::lock_guard<std::mutex> child_guard(mutex_child_goals_);
      if (expecting_until_goal_) {
        RCLCPP_INFO(this->get_logger(), "Until condition accepted by server.");
        current_until_goal_handle_ = goal_handle;
        until_accepted_ = true;
      } else {
        unwanted = true;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Until condition was rejected by server.");
    }
  }
  cv_until_.notify_one();
  if (unwanted) {
    RCLCPP_WARN(this->get_logger(), "Until condition accepted after this node gave up waiting for it — cancelling "
                                    "it rather than leaving tool contact armed.");
    cancel_until_handle(goal_handle);
  }
}

// Just forward feedback from trajectory controller. No feedback from until controllers so far.
void TrajectoryUntilNode::trajectory_feedback_callback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr& /* goal_handle */,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
  // PATCH(lpreol #77): copy the handle under the lock instead of the original double-check, which
  // could still be nulled by reset_node() between the test and the publish.
  std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> handle;
  {
    std::lock_guard<std::mutex> lock(mutex_server_goal_);
    handle = server_goal_handle_;
  }
  if (handle) {
    prealloc_fb_->actual = feedback->actual;
    prealloc_fb_->desired = feedback->desired;
    prealloc_fb_->error = feedback->error;
    prealloc_fb_->header = feedback->header;
    prealloc_fb_->joint_names = feedback->joint_names;
    handle->publish_feedback(prealloc_fb_);
  }
}

// When a result is received from either trajectory or until condition, report it back to user
namespace
{
// PATCH(lpreol #77): result codes are logged as names, not integers — the whole point is that a
// human reads them out of a container log after a rare failure.
const char* describe(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

// PATCH(lpreol #81) — claim a child result for the goal we are actually serving.
//
// Neither result callback used to ask whose result it was. Both cleared the current child handle
// and forwarded to report_goal(), which terminalises whatever goal is current — and the healthy
// path leaves a cancelled child behind on EVERY press, whose CANCELED result is still in flight.
// It is normally harmless because the handle is already null, but goal_received_callback only
// rejects while server_goal_handle_ is non-null, so once reset_node() clears it the NEXT goal can
// be accepted on the server callback group — which does run concurrently with this one.
//
// ⭐ The comparison and the clear happen in ONE critical section. An earlier version passed the
// member in and read it three times; reset_node() clearing it between the null test and
// current->get_goal_id() is a null dereference on the hot result path.
template <typename HandlePtr>
bool TrajectoryUntilNode::claim_child_result(HandlePtr& member, const rclcpp_action::GoalUUID& goal_id,
                                             const char* which)
{
  std::string current_id;
  {
    std::lock_guard<std::mutex> lock(mutex_child_goals_);
    if (member && member->get_goal_id() == goal_id) {
      member.reset();
      return true;
    }
    if (member) {
      current_id = rclcpp_action::to_string(member->get_goal_id());
    }
  }
  if (current_id.empty()) {
    // The ORDINARY case, once per press — no goal is being served, so there is nothing to protect.
    // At WARN this fired on every single press and buried the signal worth reading.
    RCLCPP_DEBUG(this->get_logger(), "Dropping a %s result for goal %s: no %s goal is being served.", which,
                 rclcpp_action::to_string(goal_id).c_str(), which);
    return false;
  }
  RCLCPP_WARN(this->get_logger(),
              "Ignoring a %s result for goal %s: this node is currently serving %s. A late result from an "
              "already-finished goal must not terminalise the current one.",
              which, rclcpp_action::to_string(goal_id).c_str(), current_id.c_str());
  return false;
}

void TrajectoryUntilNode::trajectory_result_callback(const TrajectoryResult& result)
{
  // PATCH(lpreol #77): the result code and goal id are the two facts that were missing when this
  // node aborted in production, and they cost nothing to log once per goal.
  RCLCPP_INFO(this->get_logger(), "Trajectory result received: code=%s goal=%s.", describe(result.code),
              rclcpp_action::to_string(result.goal_id).c_str());
  if (!claim_child_result(current_trajectory_goal_handle_, result.goal_id, "trajectory")) {
    return;
  }
  report_goal(result);
}

template <class T>
void TrajectoryUntilNode::until_result_callback(
    const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& result)
{
  // PATCH(lpreol #77): see trajectory_result_callback. This is the callback that was logging bare
  // "Until result received." microseconds before the process died, with no way to tell afterwards
  // which goal it belonged to or what code it carried.
  RCLCPP_INFO(this->get_logger(), "Until result received: code=%s goal=%s.", describe(result.code),
              rclcpp_action::to_string(result.goal_id).c_str());
  if (!claim_child_result(current_until_goal_handle_, result.goal_id, "until")) {
    return;
  }
  report_goal(result);
}

// PATCH(lpreol #77) — claim the server goal for terminalisation, exactly once.
//
// ⛔ NOT because the two result callbacks race: they share one MutuallyExclusive group and are
// strictly serialised. The original bare `if (server_goal_handle_)` was in fact deterministically
// safe on the normal contact path — reset_node() nulls the pointer on the same serialised thread
// that later processes the cancelled child's CANCELED result, so the null is ordered before it.
//
// The claim earns its place for the OTHER two reasons. It makes single-terminalisation explicit
// even under a sequential double result, and it keeps the loser off prealloc_res_ while the winner's
// reset_node() reallocates it. What actually kept the crash alive was terminate_goal()'s absence:
// a child result carrying CANCELED, forwarded to a parent still EXECUTING, is an illegal transition
// that rcl_action rejects by throwing — uncaught, aborting a process the whole row depends on.
// (returns GoalHandleTrajectoryUntil, the file-scope alias — the class's own TrajectoryUntilAction
//  typedef is private, so it cannot be named in a return type ahead of the TrajectoryUntilNode:: scope)
std::shared_ptr<GoalHandleTrajectoryUntil> TrajectoryUntilNode::claim_server_goal()
{
  std::lock_guard<std::mutex> lock(mutex_server_goal_);
  auto handle = server_goal_handle_;
  server_goal_handle_ = nullptr;
  return handle;
}

// PATCH(lpreol #77) — terminalise legally, and never take the process down if we get it wrong.
//
// rcl_action only permits EXECUTING → CANCELING → CANCELED, so canceled() is legal only once the
// server goal has itself accepted a cancel request. A CANCELED result from a CHILD action says
// nothing about the parent's state — the child is routinely cancelled by reset_node() on the normal
// success path — so it must not be forwarded blindly. Anything still EXECUTING is aborted instead,
// which is the honest report: the goal did not complete.
//
// The try/catch is the backstop. A terminal transition that rcl_action rejects should fail THIS
// goal, not kill a node the whole row depends on.
void TrajectoryUntilNode::terminate_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>>& handle, GoalOutcome outcome)
{
  try {
    switch (outcome) {
      case GoalOutcome::kSucceed:
        handle->succeed(prealloc_res_);
        break;
      case GoalOutcome::kAbort:
        handle->abort(prealloc_res_);
        break;
      case GoalOutcome::kCancel:
        if (handle->is_canceling()) {
          handle->canceled(prealloc_res_);
        } else {
          RCLCPP_WARN(this->get_logger(), "A child action reported CANCELED but this goal was never cancelled; "
                                          "aborting it instead of forcing an illegal terminal transition.");
          handle->abort(prealloc_res_);
        }
        break;
    }
  } catch (const rclcpp::exceptions::RCLError& e) {
    RCLCPP_ERROR(this->get_logger(), "Could not terminalise the trajectory-until goal: %s. The goal is abandoned; "
                                     "the node stays up.",
                 e.what());
  }
}

void TrajectoryUntilNode::report_goal(TrajectoryResult result)
{
  // PATCH(lpreol #77): see claim_server_goal(). Losing the claim means another callback is already
  // terminalising this goal and will reset the node — returning here keeps us off prealloc_res_
  // while it is being reallocated underneath us.
  auto handle = claim_server_goal();
  if (!handle) {
    return;
  }
  prealloc_res_->until_condition_result = TrajectoryUntilAction::Result::NOT_TRIGGERED;
  prealloc_res_->error_code = result.result->error_code;
  prealloc_res_->error_string = result.result->error_string;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      terminate_goal(handle, GoalOutcome::kSucceed);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      prealloc_res_->error_string += " Trajectory action was aborted. Aborting goal.";
      terminate_goal(handle, GoalOutcome::kAbort);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      prealloc_res_->error_string += " Trajectory action was canceled.";
      terminate_goal(handle, GoalOutcome::kCancel);
      break;

    default:
      prealloc_res_->error_string += " Unknown result code received from trajectory action, this should not happen. "
                                     "Aborting goal.";
      terminate_goal(handle, GoalOutcome::kAbort);
      break;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "%s", prealloc_res_->error_string.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Trajectory finished successfully, did not trigger until condition.");
  }
  reset_node();
}

template <typename UntilResult>
void TrajectoryUntilNode::report_goal(UntilResult result)
{
  // PATCH(lpreol #77): this is the overload that crashed the node in production — the tool-contact
  // result arriving as CANCELED while the server goal was still EXECUTING. See claim_server_goal().
  auto handle = claim_server_goal();
  if (!handle) {
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      prealloc_res_->error_code = TrajectoryUntilAction::Result::SUCCESSFUL;
      prealloc_res_->until_condition_result = TrajectoryUntilAction::Result::TRIGGERED;
      prealloc_res_->error_string += "Trajectory finished successfully by triggering until condition.";
      terminate_goal(handle, GoalOutcome::kSucceed);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      prealloc_res_->error_string += "Until action was aborted. Aborting goal.";
      terminate_goal(handle, GoalOutcome::kAbort);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      prealloc_res_->error_string += "Until action was canceled.";
      terminate_goal(handle, GoalOutcome::kCancel);
      break;

    default:
      prealloc_res_->error_string += "Unknown result code received from until action, this should not happen. "
                                     "Aborting "
                                     "goal.";
      terminate_goal(handle, GoalOutcome::kAbort);

      break;
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "%s", prealloc_res_->error_string.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Trajectory finished by triggering until condition.");
  }
  reset_node();
}

// Reset all variables and cancel running actions, if any.
void TrajectoryUntilNode::reset_node()
{
  // PATCH(lpreol #77): async_cancel_goal THROWS exceptions::UnknownGoalHandleError (which derives
  // std::invalid_argument, NOT RCLError) when the goal is unknown or has already reached a terminal
  // state — and the client erases the goal as soon as its result callback has run. Racing result
  // delivery here is normal, and an uncaught throw would abort the process: the very failure mode
  // this file is being fixed for.
  // PATCH(lpreol #81): snapshot, clear and DISARM in one critical section; cancel outside it.
  // Disarming inside the lock is what closes the check-then-act hole in until_response_callback:
  // a response that saw expecting_until_goal_ true has already stored its handle under this same
  // lock, so it is in the snapshot and gets cancelled here.
  rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr until_handle;
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_child_goals_);
    until_handle = current_until_goal_handle_;
    traj_handle = current_trajectory_goal_handle_;
    current_until_goal_handle_ = nullptr;
    current_trajectory_goal_handle_ = nullptr;
    expecting_until_goal_ = false;
  }
  if (until_handle) {
    cancel_until_handle(until_handle);
  }
  // async_cancel_goal THROWS UnknownGoalHandleError (derives std::invalid_argument, NOT RCLError)
  // when the goal is unknown or already terminal, and the client erases a goal as soon as its
  // result callback has run — so racing result delivery here is normal, not exceptional.
  try {
    if (traj_handle) {
      trajectory_action_client_->async_cancel_goal(traj_handle);
    }
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(this->get_logger(), "Trajectory goal was already finished when we tried to cancel it: %s", e.what());
  }
  trajectory_accepted_ = false;
  until_accepted_ = false;
  prealloc_res_ = std::make_shared<TrajectoryUntilAction::Result>(TrajectoryUntilAction::Result());
  prealloc_fb_ = std::make_shared<TrajectoryUntilAction::Feedback>(TrajectoryUntilAction::Feedback());

  {
    std::lock_guard<std::mutex> lock(mutex_server_goal_);  // PATCH(lpreol #77)
    server_goal_handle_ = nullptr;
  }
}

// Cancel the action contained in the variant, regardless of what type it is.
// PATCH(lpreol #81) — cancel a SPECIFIC until goal.
//
// Takes the handle explicitly so a caller cannot re-read the member after releasing the lock, and
// copies the client out of the variant under the lock so the actual rclcpp call happens with no
// lock of ours held. Throwing is normal here: async_cancel_goal raises UnknownGoalHandleError once
// the goal is terminal, and the client drops a goal the moment its result callback has run.
void TrajectoryUntilNode::cancel_until_handle(
    const rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr& handle)
{
  if (!handle) {
    return;
  }
  TCClient client;
  {
    std::lock_guard<std::mutex> lock(mutex_child_goals_);
    client = std::get<TCClient>(until_action_client_variant);
  }
  if (!client) {
    return;
  }
  try {
    client->async_cancel_goal(handle);
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(this->get_logger(), "Until goal was already finished when we tried to cancel it: %s", e.what());
  }
}

}  // namespace ur_robot_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_robot_driver::TrajectoryUntilNode>();
  // Use multithreaded executor because we have two callback groups
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
