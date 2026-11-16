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

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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
  // ⚠️ This mutex is not what prevents the production crash — terminate_goal() is. It closes a
  // latent race, and it does NOT yet cover current_until_goal_handle_ /
  // current_trajectory_goal_handle_ / until_action_client_variant, which goal_cancelled_callback
  // reads across the same boundary. Same defect class, still open.
  std::mutex mutex_server_goal_;

  // PATCH(lpreol #77): take ownership of the server goal handle, or nullptr if another callback got
  // there first. The FIRST result terminalises the goal; every later one is a genuine no-op.
  std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryUntilAction>> claim_server_goal();

  // PATCH(lpreol #77): drop a child result that belongs to an already-finished goal. See definition.
  template <typename HandlePtr>
  bool is_current_child_goal(HandlePtr current, const rclcpp_action::GoalUUID& goal_id, const char* which);

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
