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


#include <exercise_3/exercise_3.hpp>

namespace drone_course
{

DroneCourseExercise3::DroneCourseExercise3(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  // QoS profiles
  rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable();
  rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Suscribers

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone0/self_localization/pose", best_effort_qos,
    std::bind(&DroneCourseExercise3::state_subscription_callback, this, std::placeholders::_1));

  // Publishers

  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/drone0/motion_reference/twist", reliable_qos);

  // Services clients

  control_mode_service_client_ = this->create_client<as2_msgs::srv::SetControlMode>(
    "/drone0/controller/set_control_mode", rmw_qos_profile_services_default, callback_group_);

  path_service_client_ = this->create_client<drone_course_msgs::srv::RequestPath>(
    "/request_path", rmw_qos_profile_services_default, callback_group_);

  // Timers
  double timer_freq = 100.0;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0f / timer_freq),
    std::bind(&DroneCourseExercise3::timer_callback, this));

  dt_ = 1.0f / timer_freq;

  RCLCPP_INFO(this->get_logger(), "DroneCourseExercise2 initialized\n");
}

DroneCourseExercise3::~DroneCourseExercise3() {}

void DroneCourseExercise3::control_mode_service_callback(
  rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    as2_msgs::srv::SetControlMode::Response::SharedPtr response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Control mode set successfully");
      control_mode_set_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set control mode");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void DroneCourseExercise3::path_service_callback(
  rclcpp::Client<drone_course_msgs::srv::RequestPath>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    drone_course_msgs::srv::RequestPath::Response::SharedPtr response = future.get();
    if (response->path.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Received path from service");
      path_ = response->path;
      path_received_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Could not retrieve path from service");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}


void DroneCourseExercise3::timer_callback()
{
  if (!path_received_) {
    RCLCPP_INFO(this->get_logger(), "Requesting path from service...");
    path_service_request_ = std::make_shared<drone_course_msgs::srv::RequestPath::Request>();

    path_service_client_->async_send_request(
      path_service_request_, std::bind(
        &DroneCourseExercise3::path_service_callback, this, std::placeholders::_1));

  }

  // Check if control mode is set, if not, call service
  if (!control_mode_set_) {
    RCLCPP_INFO(this->get_logger(), "Calling control mode service...");
    as2_msgs::srv::SetControlMode::Request::SharedPtr control_mode_request =
      std::make_shared<as2_msgs::srv::SetControlMode::Request>();
    control_mode_request->control_mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
    control_mode_request->control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
    control_mode_request->control_mode.reference_frame =
      as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

    control_mode_service_client_->async_send_request(
      control_mode_request, std::bind(
        &DroneCourseExercise3::control_mode_service_callback, this, std::placeholders::_1));
  }

  // TODO (Exercise 3):
  // Modify reference generation to follow a smooth trajectory
  // Desired position reference
  drone_course_msgs::msg::Point position_ref;
  if (path_.size() > 0) {
    position_ref = path_[path_index_];
  } else {
    position_ref.x = 0.0;
    position_ref.y = 0.0;
    position_ref.z = 1.0;
  }


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

  // Tune PID controller gains
  double Kp = 1.0;

  // Compute PID control action
  double velocity_x = Kp * position_error_x;
  double velocity_y = Kp * position_error_y;
  double velocity_z = Kp * position_error_z;

  // Store current error for next iteration
  prev_error_[0] = position_error_x;
  prev_error_[1] = position_error_y;
  prev_error_[2] = position_error_z;

  // Generate motion reference command
  geometry_msgs::msg::TwistStamped velocity_msg;
  velocity_msg.header.stamp = this->get_clock()->now();
  velocity_msg.header.frame_id = "earth";
  velocity_msg.twist.linear.x = velocity_x;
  velocity_msg.twist.linear.y = velocity_y;
  velocity_msg.twist.linear.z = velocity_z;
  velocity_msg.twist.angular.x = 0.0;
  velocity_msg.twist.angular.y = 0.0;
  velocity_msg.twist.angular.z = 0.0;

  if (vel_pub_) {
    vel_pub_->publish(velocity_msg);
  }

  if (position_error_norm < 0.2) {
    RCLCPP_INFO(this->get_logger(), "Drone has reach the target position");
    if (path_.size() > 0) {
      path_index_ = (path_index_ + 1) % (path_.size());
    }
  }
}

void DroneCourseExercise3::state_subscription_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  state_pose_ = *msg;
}
}  // namespace drone_course
