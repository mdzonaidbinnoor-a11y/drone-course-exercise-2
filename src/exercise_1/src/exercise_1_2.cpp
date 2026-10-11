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


#include <exercise_1/exercise_1_2.hpp>

namespace drone_course
{

DroneCourseExercise1_2::DroneCourseExercise1_2(
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
    std::bind(&DroneCourseExercise1_2::state_subscription_callback, this, std::placeholders::_1));

  // Publishers

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/drone0/motion_reference/pose", reliable_qos);

  // Services clients

  // Control Mode Service Client
  control_mode_service_client_ = this->create_client<as2_msgs::srv::SetControlMode>(
    "/drone0/controller/set_control_mode", rmw_qos_profile_services_default, callback_group_);

  // Path Service Client
  // TODO(Exercise 1.2): Create service client to get the path
  // Create service client for: /request_path
  // Service type: drone_course_msgs/srv/RequestPath
  // path_service_client_ = ...

  // Timers
  double timer_freq = 100.0;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0f / timer_freq),
    std::bind(&DroneCourseExercise1_2::timer_callback, this),
    callback_group_);

  dt_ = 1.0f / timer_freq;

  RCLCPP_INFO(this->get_logger(), "DroneCourseExercise1_2 initialized\n");
}

DroneCourseExercise1_2::~DroneCourseExercise1_2() {}

void DroneCourseExercise1_2::control_mode_service_callback(
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

void DroneCourseExercise1_2::path_service_callback(
  rclcpp::Client<drone_course_msgs::srv::RequestPath>::SharedFuture future)
{
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    // TODO (Exercise 1.2)
    // Process the service response here
    RCLCPP_INFO(this->get_logger(), "Service response is ready to be processed!");
  }
}

void DroneCourseExercise1_2::timer_callback()
{
  if (!path_received_) {
    RCLCPP_INFO(this->get_logger(), "Requesting path from service...");
    // (TODO) Exercise 1.2: Send the request and process the path message
    path_service_request_ = std::make_shared<drone_course_msgs::srv::RequestPath::Request>();

    // Uncomment this section to send a request to the path
    // path_service_client_->async_send_request(path_service_request, std::bind(
    //   &DroneCourseExercise1_2::path_service_callback, this, std::placeholders::_1));

  }

  // Check if control mode is set, if not, call service
  if (!control_mode_set_) {
    RCLCPP_INFO(this->get_logger(), "Calling control mode service...");
    as2_msgs::srv::SetControlMode::Request::SharedPtr control_mode_request =
      std::make_shared<as2_msgs::srv::SetControlMode::Request>();
    control_mode_request->control_mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
    control_mode_request->control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_request->control_mode.reference_frame =
      as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

    control_mode_service_client_->async_send_request(
      control_mode_request, std::bind(
        &DroneCourseExercise1_2::control_mode_service_callback, this, std::placeholders::_1));
  }

  drone_course_msgs::msg::Point position_ref;
  // Desired position reference
  if (path_received_) {
    position_ref = path_[path_index_];
  } else {
    position_ref.x = 0.0;
    position_ref.y = 0.0;
    position_ref.z = 1.0;
  }

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
    pose_pub_->publish(position_msg);
  }

  if (position_error_norm < 0.2) {
    RCLCPP_INFO(this->get_logger(), "Drone has reach the target position");
    if (path_.size() > 0) {
      path_index_ = (path_index_ + 1) % path_.size();
    }
  }
}

void DroneCourseExercise1_2::state_subscription_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  state_pose_ = *msg;
}
}  // namespace drone_course
