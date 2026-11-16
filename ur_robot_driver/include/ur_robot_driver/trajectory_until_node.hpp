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

#ifndef UR_ROBOT_DRIVER__TRAJECTORY_UNTIL_NODE_HPP_
#define UR_ROBOT_DRIVER__TRAJECTORY_UNTIL_NODE_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/duration.hpp>
#include <ur_msgs/action/tool_contact.hpp>
#include <ur_msgs/action/follow_joint_trajectory_until.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ur_robot_driver
{
class TrajectoryUntilNode : public rclcpp::Node
{
public:
  explicit TrajectoryUntilNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TrajectoryUntilNode();

private:
  using TrajectoryUntilAction = ur_msgs::action::FollowJointTrajectoryUntil;

  rclcpp_action::Server<TrajectoryUntilAction>::SharedPtr action_server_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;

  // Add new until types here, when available
  using TCClient = rclcpp_action::Client<ur_msgs::action::ToolContact>::SharedPtr;
  std::variant<TCClient> until_action_client_variant;

  rclcpp::CallbackGroup::SharedPtr server_callback_group;
  rclcpp::CallbackGroup::SharedPtr clients_callback_group;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> server_goal_handle_;

  template <class ActionType, class ClientType>
  void send_until_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> goal);

  // PATCH(lpreol #103): the arm-after-lead-in path. See the header comment and the definitions.
  rclcpp_action::GoalResponse accept_and_arm_later(std::shared_ptr<const TrajectoryUntilAction::Goal> goal,
                                                   double lead_in_s);
  void arm_tool_contact(uint64_t generation);
  void fail_arming(const std::string& error_string);
  bool arm_after_lead_in_{ false };
  double arm_lead_margin_s_{ 0.02 };
  double arm_accept_timeout_s_{ 0.05 };
  // Server callback group only (goal_received / goal_accepted / goal_cancelled / the arm timer share
  // that MutuallyExclusive group, so these need no lock).
  rclcpp::TimerBase::SharedPtr arm_timer_;
  std::chrono::steady_clock::time_point arm_trajectory_sent_at_;
  double arm_lead_in_s_{ 0.0 };
  // GUARDED BY mutex_child_goals_: an arming still owed to the goal being served, which goal that is,
  // and whether its parent was accepted. reset_node() clears arm_pending_ in the same critical
  // section that disarms expecting_until_goal_, so an arming that has not happened yet can never
  // happen after the goal ended.
  bool arm_pending_{ false };
  bool arm_parent_accepted_{ false };
  bool arm_cancel_requested_{ false };
  uint64_t arm_generation_{ 0 };

  template <class T>
  void until_response_callback(const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr& goal_handle);

  template <class T>
  void until_result_callback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& result);

  void cancel_until_goal();

  void trajectory_response_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle);
  void trajectory_feedback_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);
  void trajectory_result_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result);

  bool assign_until_action_client(std::shared_ptr<const TrajectoryUntilAction::Goal> goal);

  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const TrajectoryUntilAction::Goal> goal);
  void
  goal_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> goal_handle);
  rclcpp_action::CancelResponse
  goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> goal_handle);

  void send_trajectory_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> goal);

  void report_goal(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult result);

  template <typename UntilResult>
  void report_goal(UntilResult result);

  void reset_node();

  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
      current_trajectory_goal_handle_;
  rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr current_until_goal_handle_;

  std::shared_ptr<TrajectoryUntilAction::Result> prealloc_res_;

  std::shared_ptr<TrajectoryUntilAction::Feedback> prealloc_fb_;

  std::atomic<bool> trajectory_accepted_;
  std::atomic<bool> until_accepted_;
  // PATCH(lpreol #103): the until goal was explicitly refused. Only the arm-after-lead-in path waits
  // on it (so a refusal fails closed at once, not at its deadline); reset per goal in reset_node().
  std::atomic<bool> until_rejected_{ false };

  std::condition_variable cv_until_;
  std::condition_variable cv_trajectory_;
  std::mutex mutex_until;
  std::mutex mutex_trajectory;

  // PATCH(lpreol #77): serialises access to server_goal_handle_ ACROSS THE ONE REAL CONCURRENCY
  // BOUNDARY in this node — the server callback group (goal_received / goal_accepted /
  // goal_cancelled) against the clients group (results / feedback). ⛔ It is NOT the two result
  // callbacks racing each other: both clients live in the same MutuallyExclusive group, so those
  // two are strictly serialised and cannot both be in report_goal(). Held only for the claim/clear,
  // never across reset_node() or an rclcpp_action call, so it cannot deadlock.
  // ⚠️ This mutex is not what prevents the production crash — terminate_goal() is; it closes a
  // latent race. The child handles and the client variant are covered SEPARATELY, by
  // mutex_child_goals_ below (#81) — they were an open hole when this comment was first written.
  std::mutex mutex_server_goal_;

  // PATCH(lpreol #77): take ownership of the server goal handle, or nullptr if another callback got
  // there first. The FIRST result terminalises the goal; every later one is a genuine no-op.
  std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> claim_server_goal();

  // PATCH(lpreol #81): THE lock for the child goal state — current_until_goal_handle_,
  // current_trajectory_goal_handle_, until_action_client_variant and expecting_until_goal_.
  // ⚠️ ONE DOCUMENTED EXEMPTION: goal_received_callback reads the variant unlocked (the
  // wait_for_action_server visit, the holds_alternative check, and send_until_goal's async_send_goal).
  // Those run on the SERVER group, which is also the only writer (assign_until_action_client), and
  // that group is MutuallyExclusive — so they are same-thread, not a race. Anything reaching the
  // variant from the CLIENTS group must take this lock.
  // goal_cancelled_callback reads these from the SERVER callback group while the response and
  // result callbacks write them from the CLIENTS group, which is the one boundary in this node
  // where two callbacks genuinely run at once. Order: mutex_until / mutex_trajectory (outer) →
  // this (inner). ⭐ Never held across an rclcpp_action call: snapshot under it, act outside.
  std::mutex mutex_child_goals_;

  // PATCH(lpreol #81): is an until goal still wanted? Plain bool, GUARDED BY mutex_child_goals_ —
  // an atomic would not help, because the bug is check-then-act, not torn reads.
  bool expecting_until_goal_{ false };

  // PATCH(lpreol #81): claim a child result for the goal we are actually serving. Compares the id
  // and clears the member in ONE critical section; returns false for a stale or unexpected result.
  template <typename HandlePtr>
  bool claim_child_result(HandlePtr& member, const rclcpp_action::GoalUUID& goal_id, const char* which);

  // PATCH(lpreol #81): cancel a specific until goal. Takes the handle explicitly so callers cannot
  // re-read the member after releasing the lock.
  void cancel_until_handle(const rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr& handle);

  // PATCH(lpreol #77): the terminal transition, made legal and non-fatal. See the definition.
  enum class GoalOutcome
  {
    kSucceed,
    kAbort,
    kCancel
  };
  void terminate_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>>& handle,
                      GoalOutcome outcome);
};

}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__TRAJECTORY_UNTIL_NODE_HPP_
