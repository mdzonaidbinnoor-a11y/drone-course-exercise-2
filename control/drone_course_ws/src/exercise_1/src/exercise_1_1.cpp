// Copyright 2026 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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


#include <exercise_1/exercise_1_1.hpp>

namespace drone_course
{

DroneCourseExercise1_1::DroneCourseExercise1_1(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  // QoS profiles
  rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable();
  rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  drone_course_msgs::msg::Point point_1;
  point_1.x = 4.0;
  point_1.y = 0.0;
  point_1.z = 2.1;

  drone_course_msgs::msg::Point point_2;
  point_2.x = 11.0;
  point_2.y = 0.0;
  point_2.z = 2.1;

  drone_course_msgs::msg::Point point_3;
  point_3.x = 11.0;
  point_3.y = 5.0;
  point_3.z = 2.1;

  drone_course_msgs::msg::Point point_4;
  point_4.x = 4.0;
  point_4.y = 5.0;
  point_4.z = 2.1;

  path_ = {point_1, point_2, point_3, point_4};

  // Suscribers

  // TODO(Exercise 1.1): Suscribe to localization pose
  // Suscribe to topic: /drone0/self_localization/pose
  // Message type: geometry_msgs::msg::PoseStamped
  // Bind suscription to callback: state_subscription_callback
  // Use best effort QoS

  // pose_sub_ = ...

  // Publishers

  // TODO(Exercise 1.1): Publish position motion reference commands
  // Publish to topic: /drone0/motion_reference/pose
  // Message type: geometry_msgs::msg::PoseStamped
  // Use reliable QoS

  // pose_pub_ = ...

  // Timers
  double timer_freq = 100.0;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0f / timer_freq),
    std::bind(&DroneCourseExercise1_1::timer_callback, this),
    callback_group_);

  dt_ = 1.0f / timer_freq;

  RCLCPP_INFO(this->get_logger(), "DroneCourseExercise1_1 initialized\n");
}

DroneCourseExercise1_1::~DroneCourseExercise1_1() {}

void DroneCourseExercise1_1::timer_callback()
{
  // Desired position reference
  drone_course_msgs::msg::Point position_ref = path_[path_index_];

  // Generate motion reference command
  geometry_msgs::msg::PoseStamped position_msg;
  position_msg.header.stamp = this->get_clock()->now();
  position_msg.header.frame_id = "earth";
  position_msg.pose.position.x = position_ref.x;
  position_msg.pose.position.y = position_ref.y;
  position_msg.pose.position.z = position_ref.z;
  position_msg.pose.orientation.w = 1.0;  // Neutral orientation
  position_msg.pose.orientation.x = 0.0;
  position_msg.pose.orientation.y = 0.0;
  position_msg.pose.orientation.z = 0.0;

  // Read current drone state
  double current_x = state_pose_.pose.position.x;
  double current_y = state_pose_.pose.position.y;
  double current_z = state_pose_.pose.position.z;

  double position_error_x = position_ref.x - current_x;
  double position_error_y = position_ref.y - current_y;
  double position_error_z = position_ref.z - current_z;
  double position_error_norm = std::sqrt(
    position_error_x * position_error_x + position_error_y * position_error_y + position_error_z *
    position_error_z);

  if (pose_pub_) {
    // TODO(Exercise 1.1): Publish the genereted position motion reference command
  }

  if (position_error_norm < 0.2) {
    RCLCPP_INFO(this->get_logger(), "Drone has reach the target position");
    // TODO(Exercise 1.1): Update path index to complete the circuit
  }
}

void DroneCourseExercise1_1::state_subscription_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  state_pose_ = *msg;
}
}  // namespace drone_course
